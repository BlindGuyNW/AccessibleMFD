// AutopilotController.cpp - Launch autopilot implementation

#include "AutopilotController.h"
#include "Controls.h"
#include "Formatting.h"
#include "OrbitalCalc.h"
#include <stdio.h>
#include <math.h>

// Global instance
AutopilotController g_autopilotController;

// Debug logging for XR SCRAM tuning (set to 0 to disable)
#define XR_DEBUG_LOG 0
static const double XR_DEBUG_INTERVAL = 5.0;    // Debug log interval (seconds)
#if XR_DEBUG_LOG
static double s_lastDebugTime = 0;
#endif

// Configuration constants
static const double COUNTDOWN_START = 3.0;      // T-3 seconds before launch
static const double LIFTOFF_DURATION = 10.0;    // Vertical climb duration
static const double STATUS_INTERVAL = 15.0;     // Status update interval
static const double MIN_THROTTLE = 0.5;         // Minimum throttle during maxQ
static const double PROGRADE_ALT = 20000.0;     // Altitude to enable prograde hold
static const double MIN_PROGRADE_PATH_ANGLE = 10.0;  // Min flight path angle (deg) for prograde hold
// CIRC_LEAD_TIME removed - circularization now manual
static const double LOW_FUEL_THRESHOLD = 0.05;  // 5% fuel = abort
static const double DYNP_DANGER  = 60000.0;     // 60 kPa — aggressive climb correction
static const double DYNP_MAIN_CUT = 80000.0;    // 80 kPa — force main engine reduction
static const double DYNP_SCRAM_CUT = 90000.0;   // 90 kPa — emergency SCRAM throttle-back
static const double MIN_HEADING_SPEED = 50.0;    // Min horizontal speed (m/s) for heading control
static const double MAX_HEADING_BANK = 22.0;       // Max bank angle (deg) for heading correction
static const double HEADING_BANK_THRESHOLD = 5.0;  // Heading error (deg) above which banking begins

// Departure turn constants
static const double DEPARTURE_TURN_PITCH = 18.0;          // Hold pitch (deg) during turn
static const double DEPARTURE_TURN_MAX_BANK = 35.0;       // Max bank angle (deg) for turn
static const double DEPARTURE_TURN_ENTRY_HEADING = 10.0;  // Heading error (deg) to trigger departure turn
static const double DEPARTURE_TURN_EXIT_HEADING = 5.0;    // Heading error (deg) to exit departure turn
static const double DEPARTURE_TURN_MAX_ALT = 5000.0;      // Force exit to pitchover (m)

// Default configuration for Delta Glider
static const LaunchConfig DEFAULT_CONFIG = {
    200000.0,         // targetAlt: 200 km
    PI05,             // launchAzimuth: 90 degrees (east)
    1000.0,           // pitchOverAlt: 1 km
    50000.0,          // pitchEndAlt: 50 km
    45.0 * RAD,       // targetPitch: 45 degrees
    15000.0,          // maxDynPressure: 15 kPa
    false             // isXR
};

AutopilotController::AutopilotController()
    : m_state(AP_IDLE)
    , m_launchTime(0)
    , m_met(0)
    , m_lastStatusTime(0)
    , m_lastCountdown(0)
    , m_peakScramThrust(0)
    , m_mainRampDown(1.0)
{
    m_config = DEFAULT_CONFIG;
    m_statusBuf[0] = '\0';
}

bool AutopilotController::StartLaunch(double targetAltKm, double azimuthDeg) {
    if (IsActive()) {
        printf("Autopilot already active. Use 'launch abort' first.\n");
        return false;
    }

    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel selected.\n");
        return false;
    }

    // Validate altitude range (80 km to 1000 km)
    if (targetAltKm < 80 || targetAltKm > 1000) {
        printf("Target altitude must be 80-1000 km.\n");
        return false;
    }

    // Validate azimuth range (0-360 degrees)
    if (azimuthDeg < 0 || azimuthDeg > 360) {
        printf("Azimuth must be 0-360 degrees.\n");
        return false;
    }

    // Check if on the ground (altitude < 100m and low speed)
    double alt = v->GetAltitude();
    double spd = v->GetAirspeed();
    if (alt > 100 || spd > 10) {
        printf("Launch autopilot requires starting on the ground.\n");
        return false;
    }

    // Check if vessel has main engines
    double maxThrust = v->GetMaxThrust(ENGINE_MAIN);
    if (maxThrust <= 0) {
        printf("Launch autopilot requires main engines.\n");
        return false;
    }

    // Check fuel
    double fuel = v->GetFuelMass();
    double maxFuel = v->GetMaxFuelMass();
    if (maxFuel > 0 && fuel / maxFuel < 0.5) {
        printf("Warning: Low fuel (%.0f%%). May not achieve target orbit.\n",
               (fuel / maxFuel) * 100);
    }

    // Configure for target altitude and azimuth
    m_config = DEFAULT_CONFIG;
    m_config.targetAlt = targetAltKm * 1000.0;
    m_config.launchAzimuth = azimuthDeg * RAD;

    // Adjust pitch program based on target altitude
    // Higher orbits need shallower pitch angles
    if (targetAltKm > 300) {
        m_config.targetPitch = 40.0 * RAD;
    } else if (targetAltKm < 150) {
        m_config.targetPitch = 50.0 * RAD;
    }

    // Detect XR vessel
#ifdef HAS_XRVESSELCTRL
    {
        XRVesselCtrl* xr = GetXRVessel(true);
        m_config.isXR = (xr != nullptr);
        if (m_config.isXR) {
            XRDoorState apuState = xr->GetDoorState(XRDoorID::XRD_APU);
            if (apuState != XRDoorState::XRDS_Open)
                printf("WARNING: APU is not running! Start APU before launch.\n");
            m_config.maxDynPressure = 35000.0;  // 35 kPa initial limit for XR
        }
    }
#endif

    // Reset XR tracking state
    m_peakScramThrust = 0;
    m_mainRampDown = 1.0;
#if XR_DEBUG_LOG
    s_lastDebugTime = 0;
#endif

    // Arm autopilot
    m_state = AP_PRELAUNCH;
    m_launchTime = oapiGetSimTime() + COUNTDOWN_START;
    m_met = -COUNTDOWN_START;
    m_lastStatusTime = 0;
    m_lastCountdown = (int)COUNTDOWN_START + 1;

    printf("Launch autopilot armed. Target: %.0f km, Azimuth: %.0f deg\n", targetAltKm, azimuthDeg);
    printf("T-%.0f...\n", COUNTDOWN_START);

    return true;
}

void AutopilotController::Abort() {
    if (!IsActive()) {
        printf("Autopilot not active.\n");
        return;
    }

    VESSEL* v = oapiGetFocusInterface();
    if (v) {
        SafeShutdown(v);
    }

    m_state = AP_ABORTED;
    printf("ABORT - Autopilot disengaged. Engines cut.\n");
}

void AutopilotController::Reset() {
    m_state = AP_IDLE;
    m_met = 0;
    m_launchTime = 0;
    m_lastStatusTime = 0;
    m_lastCountdown = 0;
}

void AutopilotController::Update(double simt, double simdt) {
    if (!IsActive()) return;

    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        Abort();
        return;
    }

    // Update mission elapsed time
    m_met = simt - m_launchTime;

    // Check for fuel exhaustion during powered flight
    if (m_state == AP_LIFTOFF || m_state == AP_DEPARTURE_TURN || m_state == AP_PITCHOVER ||
        m_state == AP_XR_CLIMB || m_state == AP_XR_SCRAM || m_state == AP_XR_FINAL) {
        if (CheckFuelExhaustion(v)) {
            printf("ABORT - Fuel exhausted.\n");
            Abort();
            return;
        }
    }

    // Run state machine
    UpdateStateMachine(v, simt, simdt);

    // Execute guidance and control
    if (m_state == AP_LIFTOFF || m_state == AP_DEPARTURE_TURN || m_state == AP_PITCHOVER ||
        m_state == AP_XR_CLIMB || m_state == AP_XR_SCRAM || m_state == AP_XR_FINAL) {
        ExecuteGuidance(v, simdt);
    }

    // Periodic status updates
    if (UpdateStatus(v, simt)) {
        printf("\n%s\n> ", m_statusBuf);
        fflush(stdout);
    }
}

// Compute heading error in degrees (positive = need to turn right)
// Returns 0 if horizontal speed is too low for meaningful heading
static double ComputeHeadingError(VESSEL* v, double targetAzimuthRad) {
    VECTOR3 horizVel;
    v->GetAirspeedVector(FRAME_HORIZON, horizVel);
    double hSpeed = sqrt(horizVel.x * horizVel.x + horizVel.z * horizVel.z);
    if (hSpeed < MIN_HEADING_SPEED) return 0.0;
    double currentHeading = atan2(horizVel.x, horizVel.z);  // 0=north, PI/2=east
    return normangle(targetAzimuthRad - currentHeading) * DEG;
}

void AutopilotController::UpdateStateMachine(VESSEL* v, double simt, double simdt) {
    switch (m_state) {
        case AP_PRELAUNCH: {
            // Countdown - use member variable to track which seconds have been announced
            int countdown = (int)ceil(-m_met);
            if (countdown < m_lastCountdown && countdown > 0) {
                printf("T-%d...\n", countdown);
                m_lastCountdown = countdown;
            }
            if (m_met >= 0.0) {
                // LIFTOFF!
                printf("LIFTOFF! Rolling...\n");
                v->SetWheelbrakeLevel(0);  // Release brakes
                v->SetEngineLevel(ENGINE_MAIN, 1.0);
                // Don't engage any autopilot yet - let vessel roll and rotate naturally
                m_state = AP_LIFTOFF;
            }
            break;
        }

        case AP_LIFTOFF: {
            double alt = v->GetAltitude();
            double spd = v->GetAirspeed();
            double pitch = v->GetPitch() * DEG;
            double bank = v->GetBank() * DEG;

            // Get vertical speed
            VECTOR3 horizVel;
            v->GetAirspeedVector(FRAME_HORIZON, horizVel);
            double vspd = horizVel.y;

            // Sanity check: if after 30 seconds we still have very low speed, we're stuck
            if (m_met >= 30.0 && spd < 50.0) {
                printf("ABORT - Not accelerating. Check brakes.\n");
                Abort();
                return;
            }

            // ROLL CONTROL - Keep wings level with rate damping
            VECTOR3 angvelLO;
            v->GetAngularVel(angvelLO);
            double rollRateLO = angvelLO.z * DEG;

            double aileronCmd = bank * 0.15 - rollRateLO * 0.5;
            if (aileronCmd > 1.0) aileronCmd = 1.0;
            if (aileronCmd < -1.0) aileronCmd = -1.0;
            v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);

            // Also use RCS for roll - same convention with damping
            double rcsRoll = bank * 0.05 - rollRateLO * 0.15;
            if (rcsRoll > 0.5) rcsRoll = 0.5;
            if (rcsRoll < -0.5) rcsRoll = -0.5;
            v->SetAttitudeRotLevel(2, rcsRoll);

            // PITCH CONTROL
            // CRITICAL: In Orbiter, NEGATIVE elevator = nose UP (pull back on stick)
            //           POSITIVE elevator = nose DOWN (push forward)
            double elevatorCmd = 0.0;
            double rcsCmd = 0.0;  // RCS pitch command for backup

            // Ground altitude is ~2.5m, so use 10m as "clearly airborne" threshold
            bool onGround = (alt < 10.0);

            if (onGround) {
                // GROUND PHASE: Wait for rotation speed, then pull up
                if (spd >= 90.0) {
                    // At or near rotation speed - pull up
                    // TRY POSITIVE - maybe positive IS nose up after all!
                    elevatorCmd = 0.6;
                    static bool rotationAnnounced = false;
                    if (!rotationAnnounced && spd >= 100.0) {
                        printf("Rotating at %.0f m/s, elev=%.1f\n", spd, elevatorCmd);
                        rotationAnnounced = true;
                    }
                }
            } else {
                // AIRBORNE PHASE
                // At low altitude, just hold steady 12° pitch - don't get fancy
                // Let the engines do the work of accelerating
                double targetPitch;

                if (alt < 500.0) {
                    // LOW ALTITUDE - simple constant pitch, no stall recovery nonsense
                    // Just hold 12° and let engines accelerate us
                    targetPitch = 12.0;
                } else {
                    // HIGHER ALTITUDE - can vary pitch with speed
                    if (spd < 150.0) {
                        targetPitch = 10.0;
                    } else if (spd < 200.0) {
                        targetPitch = 15.0;
                    } else if (spd < 250.0) {
                        targetPitch = 20.0;
                    } else {
                        targetPitch = 25.0;
                    }
                }

                double pitchError = targetPitch - pitch;
                // POSITIVE elevator = nose up (trying this)
                elevatorCmd = pitchError * 0.08;
                rcsCmd = pitchError * 0.02;

                // Clamp
                if (elevatorCmd > 0.5) elevatorCmd = 0.5;
                if (elevatorCmd < -0.3) elevatorCmd = -0.3;
                if (rcsCmd > 0.3) rcsCmd = 0.3;
                if (rcsCmd < -0.2) rcsCmd = -0.2;
            }

            v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, elevatorCmd);
            v->SetAttitudeRotLevel(0, rcsCmd);  // axis 0 = pitch

            // HEADING CONTROL - light correction during liftoff (no banking)
            {
                double headingError = ComputeHeadingError(v, m_config.launchAzimuth);
                double rudderCmd = 0.0;
                double rcsYaw = 0.0;
                if (fabs(headingError) > 1.0) {    // 1.0 deg dead band
                    rudderCmd = headingError * 0.015;
                    if (rudderCmd > 0.3) rudderCmd = 0.3;
                    if (rudderCmd < -0.3) rudderCmd = -0.3;
                    rcsYaw = headingError * 0.008;
                    if (rcsYaw > 0.2) rcsYaw = 0.2;
                    if (rcsYaw < -0.2) rcsYaw = -0.2;
                }
                v->SetControlSurfaceLevel(AIRCTRL_RUDDER, rudderCmd);
                v->SetAttitudeRotLevel(1, rcsYaw);
            }

            // Transition once we have altitude AND good climb
            if (alt > 1000.0 && vspd > 20.0) {
                double headingError = ComputeHeadingError(v, m_config.launchAzimuth);
                if (fabs(headingError) > DEPARTURE_TURN_ENTRY_HEADING) {
                    printf("Heading error %.0f deg — departure turn.\n", headingError);
                    m_state = AP_DEPARTURE_TURN;
                } else if (m_config.isXR) {
                    printf("On heading. Starting XR climb profile.\n");
                    printf("** Retract gear (G) **\n");
                    m_state = AP_XR_CLIMB;
                } else {
                    printf("On heading. Starting gravity turn.\n");
                    m_state = AP_PITCHOVER;
                }
            }
            break;
        }

        case AP_DEPARTURE_TURN: {
            double headingError = ComputeHeadingError(v, m_config.launchAzimuth);
            double depAlt = v->GetAltitude();
            bool turnDone = (fabs(headingError) < DEPARTURE_TURN_EXIT_HEADING);
            bool altLimit = (depAlt > DEPARTURE_TURN_MAX_ALT);
            if (turnDone || altLimit) {
                if (turnDone)
                    printf("On heading (err=%.1f deg).", headingError);
                else
                    printf("Altitude limit (heading err=%.0f deg).", headingError);

                if (m_config.isXR) {
                    printf(" Starting XR climb profile.\n");
                    printf("** Retract gear (G) **\n");
                    m_state = AP_XR_CLIMB;
                } else {
                    printf(" Starting gravity turn.\n");
                    m_state = AP_PITCHOVER;
                }
            }
            break;
        }

        case AP_PITCHOVER:
            // Follow pitch program until MECO
            if (CheckMECOCondition(v)) {
                v->SetEngineLevel(ENGINE_MAIN, 0.0);
                v->ActivateNavmode(NAVMODE_PROGRADE);
                m_state = AP_COMPLETE;

                // Get orbital info for guidance
                OBJHANDLE hRef = v->GetGravityRef();
                ELEMENTS el;
                ORBITPARAM prm;
                if (v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) {
                    double refSize = oapiGetSize(hRef);
                    double apAlt = (prm.ApD - refSize) / 1000.0;
                    double peAlt = (prm.PeD - refSize) / 1000.0;

                    char apTBuf[32];
                    FormatTime(prm.ApT, apTBuf, sizeof(apTBuf));

                    printf("\nMECO - Main engine cutoff.\n");
                    printf("Apoapsis: %.1f km | Periapsis: %.1f km\n", apAlt, peAlt);
                    printf("Time to apoapsis: %s\n", apTBuf);
                    printf("\nFor circularization:\n");
                    printf("  1. Coast to apoapsis (prograde hold active)\n");
                    printf("  2. When ApT < 30s, throttle up and burn prograde\n");
                    printf("  3. Cut engines when periapsis reaches %.0f km\n",
                           m_config.targetAlt / 1000.0);
                    printf("  Use 'orbit' command to monitor progress.\n");
                } else {
                    printf("\nMECO - Main engine cutoff. Prograde hold active.\n");
                }
            }
            break;

        case AP_XR_CLIMB: {
            // Steep climb, then level out at ~24km and build speed before SCRAM
            double xrAlt = v->GetAltitude();
            double xrMach = v->GetMachNumber();

            // Get vertical speed
            VECTOR3 xrHv;
            v->GetAirspeedVector(FRAME_HORIZON, xrHv);
            double xrVspd = xrHv.y;

            // Transition: at altitude, at speed, AND leveled out
            if (xrAlt > 24000.0 && xrMach > 3.5 && xrVspd < 50.0) {
                printf("At %.0f km, Mach %.1f. Engaging SCRAM.\n", xrAlt / 1000.0, xrMach);
                printf("** Open SCRAM doors (Ctrl-G) **\n");
                m_peakScramThrust = 0;
                m_mainRampDown = 1.0;
                m_state = AP_XR_SCRAM;
            }
            break;
        }

        case AP_XR_SCRAM: {
#ifdef HAS_XRVESSELCTRL
            // Monitor SCRAM engines for cutoff conditions
            double scramAlt = v->GetAltitude();
            double scramMach = v->GetMachNumber();

            XRVesselCtrl* xr = GetXRVessel(true);
            if (!xr) {
                // Lost XR interface — fall through to final
                printf("SCRAM cutoff — lost XR interface.\n");
                m_state = AP_XR_FINAL;
                break;
            }

            // Read SCRAM engine state
            XREngineStateRead left = {}, right = {};
            bool haveLeft = xr->GetEngineState(XREngineID::XRE_ScramLeft, left);
            bool haveRight = xr->GetEngineState(XREngineID::XRE_ScramRight, right);

            double combinedThrust = 0;
            if (haveLeft) combinedThrust += left.Thrust;
            if (haveRight) combinedThrust += right.Thrust;

            if (combinedThrust > m_peakScramThrust)
                m_peakScramThrust = combinedThrust;

            // Check SCRAM cutoff conditions
            const char* cutoffReason = nullptr;

            if (scramAlt > 65000.0) {
                cutoffReason = "Altitude limit (65 km).";
            } else if (m_peakScramThrust > 0 && combinedThrust < m_peakScramThrust * 0.30 && scramAlt > 45000.0) {
                cutoffReason = "SCRAM thrust degraded.";
            } else if ((haveLeft && left.DiffuserTemp > 7500.0) || (haveRight && right.DiffuserTemp > 7500.0)) {
                cutoffReason = "Diffuser temp limit.";
            }

            // Check SCRAM fuel reserve
            if (!cutoffReason) {
                PROPELLANT_HANDLE hScramTank = nullptr;
                // Find SCRAM fuel tank — check propellant resources for one that isn't main/RCS
                int nTanks = v->GetPropellantCount();
                for (int i = 0; i < nTanks && !hScramTank; i++) {
                    PROPELLANT_HANDLE hP = v->GetPropellantHandleByIndex(i);
                    // SCRAM tank is typically the 2nd tank (index 1)
                    if (i == 1) hScramTank = hP;  // Best guess for XR vessels
                }
                if (hScramTank) {
                    double scramFuel = v->GetPropellantMass(hScramTank);
                    double scramMax = v->GetPropellantMaxMass(hScramTank);
                    if (scramMax > 0 && scramFuel / scramMax < 0.03) {
                        cutoffReason = "SCRAM fuel reserve.";
                    }
                }
            }

            if (cutoffReason) {
                printf("SCRAM cutoff at %.0f km, Mach %.1f. %s\n", scramAlt / 1000.0, scramMach, cutoffReason);
                printf("** Close SCRAM doors (Ctrl-G) **\n");

                // Shut down SCRAM engines
                if (haveLeft) {
                    XREngineStateRead st = left;
                    st.ThrottleLevel = 0.0;
                    xr->SetEngineState(XREngineID::XRE_ScramLeft, st);
                }
                if (haveRight) {
                    XREngineStateRead st = right;
                    st.ThrottleLevel = 0.0;
                    xr->SetEngineState(XREngineID::XRE_ScramRight, st);
                }

                // Ensure main engines are at full for final push
                v->SetEngineLevel(ENGINE_MAIN, 1.0);
                m_state = AP_XR_FINAL;
            }
#endif // HAS_XRVESSELCTRL
            break;
        }

        case AP_XR_FINAL:
            // Burn mains to raise apoapsis to target, then coast
            if (CheckMECOCondition(v)) {
                v->SetEngineLevel(ENGINE_MAIN, 0.0);
                v->ActivateNavmode(NAVMODE_PROGRADE);
                m_state = AP_COAST;

                OBJHANDLE hRef = v->GetGravityRef();
                ELEMENTS el;
                ORBITPARAM prm;
                if (v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) {
                    double refSize = oapiGetSize(hRef);
                    double apAlt = (prm.ApD - refSize) / 1000.0;
                    double peAlt = (prm.PeD - refSize) / 1000.0;

                    char apTBuf[32];
                    FormatTime(prm.ApT, apTBuf, sizeof(apTBuf));

                    printf("\nMECO - Apoapsis at %.1f km. Coasting.\n", apAlt);
                    printf("Periapsis: %.1f km | Time to apoapsis: %s\n", peAlt, apTBuf);
                    printf("** Deploy radiator (Alt-R) **\n");
                    printf("** Shut down APU (Ctrl-A) **\n");
                    printf("Circularization burn will start near apoapsis.\n");
                } else {
                    printf("\nMECO - Coasting to apoapsis. Prograde hold active.\n");
                }
            }
            break;

        case AP_COAST: {
            // Coast to apoapsis, then begin circularization burn
            OBJHANDLE hRef = v->GetGravityRef();
            if (hRef) {
                ELEMENTS el;
                ORBITPARAM prm;
                if (v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) {
                    double refSize = oapiGetSize(hRef);
                    double apAlt = (prm.ApD - refSize) / 1000.0;

                    // Start burn 30s before apoapsis, or immediately if
                    // apoapsis time is very short (already near it)
                    if (prm.ApT < 30.0 && prm.ApT > 0.0) {
                        printf("\nApproaching apoapsis (%.0fs). Starting circularization burn.\n", prm.ApT);
                        printf("Burning prograde to raise periapsis to %.0f km.\n",
                               m_config.targetAlt / 1000.0);
                        v->SetEngineLevel(ENGINE_MAIN, 1.0);
                        m_state = AP_CIRCULARIZE;
                    }
                }
            }
            break;
        }

        case AP_CIRCULARIZE: {
            // Burn prograde until periapsis reaches target
            OBJHANDLE hRef = v->GetGravityRef();
            if (hRef) {
                double peDist;
                v->GetPeDist(peDist);
                double refSize = oapiGetSize(hRef);
                double peAlt = peDist - refSize;

                if (peAlt >= m_config.targetAlt * 0.95) {
                    // Periapsis within 95% of target — done
                    v->SetEngineLevel(ENGINE_MAIN, 0.0);
                    m_state = AP_COMPLETE;

                    double apDist;
                    v->GetApDist(apDist);
                    double apAlt = (apDist - refSize) / 1000.0;

                    printf("\nCircularization complete!\n");
                    printf("Apoapsis: %.1f km | Periapsis: %.1f km\n",
                           apAlt, peAlt / 1000.0);
                    printf("** Deploy radiator (Alt-R) if not already deployed **\n");
                }
            }
            break;
        }

        default:
            break;
    }
}

double AutopilotController::CalculateTargetPitch(double altitude) const {
    // Gravity turn pitch profile (returns degrees)
    // Steep early climb, gradual flattening toward orbital insertion
    double altKm = altitude / 1000.0;

    if (altKm < 1.0)  return 80.0;
    if (altKm < 5.0)  return 80.0 - (altKm - 1.0) * 7.5;     // 80 -> 50
    if (altKm < 10.0) return 50.0 - (altKm - 5.0) * 2.0;     // 50 -> 40
    if (altKm < 20.0) return 40.0 - (altKm - 10.0) * 2.0;    // 40 -> 20
    double p = 20.0 - (altKm - 20.0) * 0.5;                   // 20 -> 5 at 50km
    return (p < 5.0) ? 5.0 : p;
}

double AutopilotController::CalculateThrottle(VESSEL* v) const {
    double dynPressure = v->GetDynPressure();

    // Reduce throttle if exceeding max Q
    if (dynPressure > m_config.maxDynPressure) {
        // Linear reduction - at 2x max Q, throttle to 50%
        double excess = dynPressure / m_config.maxDynPressure;
        return max(MIN_THROTTLE, 1.0 / excess);
    }

    return 1.0;
}

bool AutopilotController::CheckMECOCondition(VESSEL* v) const {
    // Cut engines when apoapsis reaches target altitude
    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) return false;

    double apDist;
    v->GetApDist(apDist);
    double refSize = oapiGetSize(hRef);
    double apAlt = apDist - refSize;

    // Cut at target apoapsis - don't wait for atmosphere exit
    // (waiting caused overshoot when apoapsis reached target while still in atmosphere)
    return (apAlt >= m_config.targetAlt);
}

bool AutopilotController::CheckFuelExhaustion(VESSEL* v) const {
    double fuel = v->GetFuelMass();
    double maxFuel = v->GetMaxFuelMass();

    // Check if fuel is critically low (less than 5%)
    if (maxFuel > 0 && fuel / maxFuel < LOW_FUEL_THRESHOLD) {
        return true;
    }
    return false;
}

void AutopilotController::ExecuteGuidance(VESSEL* v, double simdt) {
    double alt = v->GetAltitude();

    switch (m_state) {
        case AP_LIFTOFF:
            // Hold vertical, full throttle
            v->SetEngineLevel(ENGINE_MAIN, 1.0);
            break;

        case AP_DEPARTURE_TURN: {
            v->SetEngineLevel(ENGINE_MAIN, CalculateThrottle(v));

            double bank = v->GetBank() * DEG;
            double pitch = v->GetPitch() * DEG;
            VECTOR3 angvel;
            v->GetAngularVel(angvel);
            double rollRate = angvel.z * DEG;

            double headingError = ComputeHeadingError(v, m_config.launchAzimuth);

            // PITCH — hold moderate pitch for efficient turning
            double pitchError = DEPARTURE_TURN_PITCH - pitch;
            double elevatorCmd = pitchError * 0.1;
            if (elevatorCmd > 0.6) elevatorCmd = 0.6;
            if (elevatorCmd < -0.3) elevatorCmd = -0.3;
            double rcsPitch = pitchError * 0.05;
            if (rcsPitch > 0.5) rcsPitch = 0.5;
            if (rcsPitch < -0.3) rcsPitch = -0.3;

            // ROLL — bank proportional to heading error
            // Orbiter convention: positive GetBank() = left bank, so negate
            double targetBank = -headingError * (DEPARTURE_TURN_MAX_BANK / 30.0);  // Full bank at 30 deg error
            if (targetBank > DEPARTURE_TURN_MAX_BANK) targetBank = DEPARTURE_TURN_MAX_BANK;
            if (targetBank < -DEPARTURE_TURN_MAX_BANK) targetBank = -DEPARTURE_TURN_MAX_BANK;
            double bankError = bank - targetBank;
            double aileronCmd = bankError * 0.15 - rollRate * 0.5;
            if (aileronCmd > 1.0) aileronCmd = 1.0;
            if (aileronCmd < -1.0) aileronCmd = -1.0;
            double rcsRoll = bankError * 0.05 - rollRate * 0.15;
            if (rcsRoll > 0.5) rcsRoll = 0.5;
            if (rcsRoll < -0.5) rcsRoll = -0.5;

            // RUDDER/YAW — assist the turn
            double rudderCmd = 0.0;
            double rcsYaw = 0.0;
            if (fabs(headingError) > 0.5) {
                rudderCmd = headingError * 0.02;
                if (rudderCmd > 0.5) rudderCmd = 0.5;
                if (rudderCmd < -0.5) rudderCmd = -0.5;
                rcsYaw = headingError * 0.01;
                if (rcsYaw > 0.3) rcsYaw = 0.3;
                if (rcsYaw < -0.3) rcsYaw = -0.3;
            }

            v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);
            v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, elevatorCmd);
            v->SetControlSurfaceLevel(AIRCTRL_RUDDER, rudderCmd);
            v->SetAttitudeRotLevel(0, rcsPitch);
            v->SetAttitudeRotLevel(1, rcsYaw);
            v->SetAttitudeRotLevel(2, rcsRoll);
            break;
        }

        case AP_PITCHOVER: {
            double throttle = CalculateThrottle(v);
            v->SetEngineLevel(ENGINE_MAIN, throttle);

            // Decide: active pitch control vs prograde hold
            // Once prograde is engaged, commit to it (flight path naturally
            // flattens toward orbit — don't flip back to active control)
            bool useActiveControl = (alt < PROGRADE_ALT);
            if (!useActiveControl && !v->GetNavmodeState(NAVMODE_PROGRADE)) {
                // Prograde not yet engaged — check flight path angle
                VECTOR3 hv;
                v->GetAirspeedVector(FRAME_HORIZON, hv);
                double hSpd = sqrt(hv.x * hv.x + hv.z * hv.z);
                double pathAngle = atan2(hv.y, hSpd) * DEG;
                if (pathAngle < MIN_PROGRADE_PATH_ANGLE) {
                    useActiveControl = true;  // Flight path too flat, keep active control
                }
            }
            if (useActiveControl) {
                double bank = v->GetBank() * DEG;

                // Roll rate for damping (positive = rolling left in Orbiter convention)
                VECTOR3 angvel;
                v->GetAngularVel(angvel);
                double rollRate = angvel.z * DEG;  // deg/s

                // === ROLL — default wings-level with rate damping ===
                double aileronCmd = bank * 0.15 - rollRate * 0.5;
                if (aileronCmd > 1.0) aileronCmd = 1.0;
                if (aileronCmd < -1.0) aileronCmd = -1.0;
                double rcsRoll = bank * 0.05 - rollRate * 0.15;
                if (rcsRoll > 0.5) rcsRoll = 0.5;
                if (rcsRoll < -0.5) rcsRoll = -0.5;

                // === PITCH — gravity turn profile ===
                double targetPitch = CalculateTargetPitch(alt);
                double pitchError = targetPitch - (v->GetPitch() * DEG);
                double elevatorCmd = pitchError * 0.1;
                if (elevatorCmd > 0.6) elevatorCmd = 0.6;
                if (elevatorCmd < -0.3) elevatorCmd = -0.3;
                double rcsPitch = pitchError * 0.05;
                if (rcsPitch > 0.5) rcsPitch = 0.5;
                if (rcsPitch < -0.3) rcsPitch = -0.3;

                // === HEADING — steer toward target azimuth ===
                double headingError = ComputeHeadingError(v, m_config.launchAzimuth);
                double rudderCmd = 0.0;
                double rcsYaw = 0.0;

                // Below 10km: reduce heading authority to prioritize pitch
                double headingScale = 1.0;
                double maxBank = MAX_HEADING_BANK;
                if (alt < 5000.0) {
                    headingScale = 0.3;
                    maxBank = 0.0;       // No banking during steep climb
                } else if (alt < 10000.0) {
                    double blend = (alt - 5000.0) / 5000.0;
                    headingScale = 0.3 + 0.7 * blend;
                    maxBank = MAX_HEADING_BANK * blend;
                }

                if (fabs(headingError) > 0.5) {    // 0.5 deg dead band
                    // Positive command = turn right; positive error = need right
                    rudderCmd = headingError * 0.02 * headingScale;
                    if (rudderCmd > 0.5) rudderCmd = 0.5;
                    if (rudderCmd < -0.5) rudderCmd = -0.5;
                    rcsYaw = headingError * 0.01 * headingScale;
                    if (rcsYaw > 0.3) rcsYaw = 0.3;
                    if (rcsYaw < -0.3) rcsYaw = -0.3;

                    // Bank into turn for large errors (>5 deg)
                    if (fabs(headingError) > HEADING_BANK_THRESHOLD && maxBank > 0.0) {
                        double bankScale = (fabs(headingError) - HEADING_BANK_THRESHOLD) / 20.0;
                        if (bankScale > 1.0) bankScale = 1.0;
                        // Positive error -> turn right -> negative bank (Orbiter: positive bank = left)
                        double targetBank = (headingError > 0 ? -1.0 : 1.0) * maxBank * bankScale;
                        // Override roll: (bank - targetBank) pattern with rate damping
                        double bankError = bank - targetBank;
                        aileronCmd = bankError * 0.15 - rollRate * 0.5;
                        if (aileronCmd > 1.0) aileronCmd = 1.0;
                        if (aileronCmd < -1.0) aileronCmd = -1.0;
                        rcsRoll = bankError * 0.05 - rollRate * 0.15;
                        if (rcsRoll > 0.5) rcsRoll = 0.5;
                        if (rcsRoll < -0.5) rcsRoll = -0.5;
                    }
                }

                // === APPLY ALL ===
                v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);
                v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, elevatorCmd);
                v->SetControlSurfaceLevel(AIRCTRL_RUDDER, rudderCmd);
                v->SetAttitudeRotLevel(0, rcsPitch);
                v->SetAttitudeRotLevel(1, rcsYaw);
                v->SetAttitudeRotLevel(2, rcsRoll);
            } else {
                // Above 20km: switch to prograde hold, but KEEP roll control!
                v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
                v->SetAttitudeRotLevel(0, 0.0);  // Release RCS pitch - prograde handles it

                // KEEP ROLL CONTROL - prograde doesn't stabilize roll!
                double bank = v->GetBank() * DEG;
                VECTOR3 angvel2;
                v->GetAngularVel(angvel2);
                double rollRate2 = angvel2.z * DEG;

                double aileronCmd = bank * 0.15 - rollRate2 * 0.5;
                if (aileronCmd > 1.0) aileronCmd = 1.0;
                if (aileronCmd < -1.0) aileronCmd = -1.0;
                v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);

                double rcsRoll = bank * 0.1 - rollRate2 * 0.2;
                if (rcsRoll > 0.5) rcsRoll = 0.5;
                if (rcsRoll < -0.5) rcsRoll = -0.5;
                v->SetAttitudeRotLevel(2, rcsRoll);

                // Zero out heading control - prograde handles yaw
                v->SetControlSurfaceLevel(AIRCTRL_RUDDER, 0.0);
                v->SetAttitudeRotLevel(1, 0.0);

                if (!v->GetNavmodeState(NAVMODE_PROGRADE)) {
                    printf("Engaging prograde hold at %.0f km.\n", alt / 1000.0);
                    v->ActivateNavmode(NAVMODE_PROGRADE);
                }
            }
            break;
        }

        case AP_XR_CLIMB: {
            // Phase 1 (alt < 20km): steep ~70 deg climb on full mains
            // Phase 2 (alt >= 20km): level out, hold altitude, build speed to Mach 3.5
            v->SetEngineLevel(ENGINE_MAIN, CalculateThrottle(v));

            double pitch = v->GetPitch() * DEG;
            double bank = v->GetBank() * DEG;
            VECTOR3 angvel;
            v->GetAngularVel(angvel);
            double rollRate = angvel.z * DEG;

            double altKm = alt / 1000.0;
            double targetPitch;

            if (altKm < 12.0) {
                // Steep initial climb
                if (altKm < 5.0)
                    targetPitch = 70.0;
                else
                    targetPitch = 70.0 - (altKm - 5.0) * (15.0 / 7.0);  // 70 -> 55 at 12km
            } else if (altKm < 24.0) {
                // Gradual taper to level: 55 -> 5 by 24km
                // Spread over 12km for smooth transition to SCRAM altitude
                targetPitch = 55.0 - (altKm - 12.0) * (50.0 / 12.0);  // 55 -> 5
            } else {
                // Level-out: vspd control to hold ~24km
                VECTOR3 hv;
                v->GetAirspeedVector(FRAME_HORIZON, hv);
                double vspd = hv.y;

                double targetVspd = 0.0;  // Hold altitude — SCRAM handles the climb
                double vspdError = targetVspd - vspd;
                targetPitch = vspdError * 0.08;
                if (targetPitch > 15.0) targetPitch = 15.0;
                if (targetPitch < -10.0) targetPitch = -10.0;
            }

            double pitchError = targetPitch - pitch;
            double elevatorCmd = pitchError * 0.1;
            if (elevatorCmd > 0.6) elevatorCmd = 0.6;
            if (elevatorCmd < -0.3) elevatorCmd = -0.3;
            double rcsPitch = pitchError * 0.05;
            if (rcsPitch > 0.5) rcsPitch = 0.5;
            if (rcsPitch < -0.3) rcsPitch = -0.3;

            // Wings level with rate damping
            double aileronCmd = bank * 0.15 - rollRate * 0.5;
            if (aileronCmd > 1.0) aileronCmd = 1.0;
            if (aileronCmd < -1.0) aileronCmd = -1.0;
            double rcsRoll = bank * 0.05 - rollRate * 0.15;
            if (rcsRoll > 0.5) rcsRoll = 0.5;
            if (rcsRoll < -0.5) rcsRoll = -0.5;

            // Heading maintenance
            double headingError = ComputeHeadingError(v, m_config.launchAzimuth);
            double rudderCmd = 0.0;
            double rcsYaw = 0.0;
            if (fabs(headingError) > 1.0) {
                rudderCmd = headingError * 0.02;
                if (rudderCmd > 0.5) rudderCmd = 0.5;
                if (rudderCmd < -0.5) rudderCmd = -0.5;
                rcsYaw = headingError * 0.01;
                if (rcsYaw > 0.3) rcsYaw = 0.3;
                if (rcsYaw < -0.3) rcsYaw = -0.3;
            }

            v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);
            v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, elevatorCmd);
            v->SetControlSurfaceLevel(AIRCTRL_RUDDER, rudderCmd);
            v->SetAttitudeRotLevel(0, rcsPitch);
            v->SetAttitudeRotLevel(1, rcsYaw);
            v->SetAttitudeRotLevel(2, rcsRoll);

#if XR_DEBUG_LOG
            {
                double dbgSimt = oapiGetSimTime();
                if (dbgSimt - s_lastDebugTime >= XR_DEBUG_INTERVAL) {
                    s_lastDebugTime = dbgSimt;
                    VECTOR3 dbgHv;
                    v->GetAirspeedVector(FRAME_HORIZON, dbgHv);
                    printf("[CLIMB DBG] M%.1f | Alt:%.0fkm | VS:%+.0f | Pitch:%.1f | TgtP:%.1f | DynP:%.0fkPa\n",
                           v->GetMachNumber(), alt / 1000.0, dbgHv.y, pitch, targetPitch,
                           v->GetDynPressure() / 1000.0);
                }
            }
#endif
            break;
        }

        case AP_XR_SCRAM: {
            // Hoist shared variables so engine block can use dynP for safety
            VECTOR3 scramHv;
            v->GetAirspeedVector(FRAME_HORIZON, scramHv);
            double vspd = scramHv.y;
            double altKmS = alt / 1000.0;
            double dynP = v->GetDynPressure();

#ifdef HAS_XRVESSELCTRL
            // SCRAM cruise per XR flight manual (steps 8-9, 13-14):
            // 1. SCRAM at full throttle throughout
            // 2. Gradually shut down main engines (they waste rocket fuel)
            // 3. Control climb rate via trim to track DECLINING dynP schedule:
            //    ~35 kPa at 24km -> ~10 kPa at 40km -> ~4 kPa at 55km+
            XRVesselCtrl* xr = GetXRVessel(true);
            if (xr) {
                // Ensure SCRAM engines are at full throttle
                XREngineStateRead left = {}, right = {};
                bool haveLeft = xr->GetEngineState(XREngineID::XRE_ScramLeft, left);
                bool haveRight = xr->GetEngineState(XREngineID::XRE_ScramRight, right);

                if (haveLeft && left.ThrottleLevel < 1.0) {
                    left.ThrottleLevel = 1.0;
                    xr->SetEngineState(XREngineID::XRE_ScramLeft, left);
                }
                if (haveRight && right.ThrottleLevel < 1.0) {
                    right.ThrottleLevel = 1.0;
                    xr->SetEngineState(XREngineID::XRE_ScramRight, right);
                }

                // Main engine ramp-down: begin immediately, accelerate once SCRAM
                // produces meaningful thrust. Every second of mains wastes rocket
                // fuel needed for AP_XR_FINAL.
                double combinedScram = 0;
                if (haveLeft) combinedScram += left.Thrust;
                if (haveRight) combinedScram += right.Thrust;

                if (combinedScram > 30000.0) {
                    // SCRAM producing > 30 kN — aggressive ramp-down
                    m_mainRampDown -= 0.08 * simdt;  // ~12.5s to zero
                } else {
                    // Begin ramp-down immediately at moderate rate
                    m_mainRampDown -= 0.02 * simdt;  // ~50s to zero
                }

                // Safety restore: only if SCRAM has truly failed at low altitude
                if (combinedScram < 5000.0 && altKmS < 30.0) {
                    if (m_mainRampDown < 0.3) {
                        m_mainRampDown += 0.1 * simdt;
                    }
                }

                // Hard dynP safety: force-cap mains if pressure dangerously high
                if (dynP > DYNP_MAIN_CUT) {
                    double safeMain = 1.0 - (dynP - DYNP_MAIN_CUT) / (100000.0 - DYNP_MAIN_CUT);
                    if (safeMain < 0.0) safeMain = 0.0;
                    if (m_mainRampDown > safeMain) m_mainRampDown = safeMain;
                }

                if (m_mainRampDown < 0.0) m_mainRampDown = 0.0;
                if (m_mainRampDown > 1.0) m_mainRampDown = 1.0;
                v->SetEngineLevel(ENGINE_MAIN, m_mainRampDown);

                // Emergency SCRAM throttle-back: last-resort survival measure.
                // Overrides the full-throttle set above if dynP is critically high.
                if (dynP > DYNP_SCRAM_CUT) {
                    double scramScale = 1.0 - (dynP - DYNP_SCRAM_CUT) / (120000.0 - DYNP_SCRAM_CUT);
                    if (scramScale < 0.3) scramScale = 0.3;  // Never below 30%
                    if (haveLeft) {
                        left.ThrottleLevel = scramScale;
                        xr->SetEngineState(XREngineID::XRE_ScramLeft, left);
                    }
                    if (haveRight) {
                        right.ThrottleLevel = scramScale;
                        xr->SetEngineState(XREngineID::XRE_ScramRight, right);
                    }
                }
            }
#endif

            // DynP-based climb control: target a DECLINING dynP ceiling.
            // XR SCRAM limit is 8000K diffuser temp (auto-throttled by vessel).
            // Keep moderate dynP for good SCRAM air intake; don't starve the engines.

            // Target dynamic pressure schedule (altitude-based)
            double targetDynP;
            if (altKmS < 30.0)
                targetDynP = 30000.0;                                   // 30 kPa
            else if (altKmS < 40.0)
                targetDynP = 30000.0 - (altKmS - 30.0) * 2000.0;      // 30 -> 10 kPa
            else if (altKmS < 55.0)
                targetDynP = 10000.0 - (altKmS - 40.0) * 400.0;       // 10 -> 4 kPa
            else
                targetDynP = 4000.0;                                    // 4 kPa

            // Baseline VS: steady climb, tapering at altitude
            double baseVspd;
            if (altKmS < 35.0)
                baseVspd = 120.0;
            else if (altKmS < 55.0)
                baseVspd = 120.0 - (altKmS - 35.0) * 4.0;  // 120 -> 40
            else
                baseVspd = 40.0;

            // Asymmetric dynP ceiling: being below target is fine, above means climb harder
            double targetVspd = baseVspd;
            double dynPExcess = dynP - targetDynP;
            if (dynPExcess > 0.0) {
                targetVspd = baseVspd + (dynPExcess / targetDynP) * 200.0;
                // Danger zone: additional boost to escape dense air
                if (dynP > DYNP_DANGER) {
                    double dangerFrac = (dynP - DYNP_DANGER) / (100000.0 - DYNP_DANGER);
                    if (dangerFrac > 1.0) dangerFrac = 1.0;
                    targetVspd += dangerFrac * 200.0;
                }
            }
            if (targetVspd < 50.0) targetVspd = 50.0;    // Always climb meaningfully
            if (targetVspd > 500.0) targetVspd = 500.0;   // Sanity cap

            // Elevator trim: proportional to VS error, dynamically smoothed
            double vspdError = targetVspd - vspd;
            double desiredTrim = vspdError * 0.002;
            if (desiredTrim > 0.5) desiredTrim = 0.5;
            if (desiredTrim < -0.3) desiredTrim = -0.3;
            double currentTrim = v->GetControlSurfaceLevel(AIRCTRL_ELEVATORTRIM);

            // Dynamic smoothing: faster response when dynP is above ceiling
            double trimSmoothing = 0.05;
            if (dynPExcess > 0.0) {
                trimSmoothing = 0.05 + (dynPExcess / targetDynP) * 0.35;
                if (trimSmoothing > 0.4) trimSmoothing = 0.4;
            }
            if (dynP > DYNP_DANGER)
                trimSmoothing = 0.6;  // Near-instant response in danger zone

            double newTrim = currentTrim + (desiredTrim - currentTrim) * trimSmoothing;
            v->SetControlSurfaceLevel(AIRCTRL_ELEVATORTRIM, newTrim);

            // Zero momentary elevator — trim does the work
            v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
            v->SetAttitudeRotLevel(0, 0.0);

            double pitch = v->GetPitch() * DEG;
            double bank = v->GetBank() * DEG;
            VECTOR3 angvel;
            v->GetAngularVel(angvel);
            double rollRate = angvel.z * DEG;

            // Wings level
            double aileronCmd = bank * 0.15 - rollRate * 0.5;
            if (aileronCmd > 1.0) aileronCmd = 1.0;
            if (aileronCmd < -1.0) aileronCmd = -1.0;
            double rcsRoll = bank * 0.05 - rollRate * 0.15;
            if (rcsRoll > 0.5) rcsRoll = 0.5;
            if (rcsRoll < -0.5) rcsRoll = -0.5;

            // Heading maintenance
            double headingError = ComputeHeadingError(v, m_config.launchAzimuth);
            double rudderCmd = 0.0;
            double rcsYaw = 0.0;
            if (fabs(headingError) > 1.0) {
                rudderCmd = headingError * 0.02;
                if (rudderCmd > 0.5) rudderCmd = 0.5;
                if (rudderCmd < -0.5) rudderCmd = -0.5;
                rcsYaw = headingError * 0.01;
                if (rcsYaw > 0.3) rcsYaw = 0.3;
                if (rcsYaw < -0.3) rcsYaw = -0.3;
            }

            v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);
            v->SetControlSurfaceLevel(AIRCTRL_RUDDER, rudderCmd);
            v->SetAttitudeRotLevel(1, rcsYaw);
            v->SetAttitudeRotLevel(2, rcsRoll);

#if XR_DEBUG_LOG
            {
                double dbgSimt = oapiGetSimTime();
                if (dbgSimt - s_lastDebugTime >= XR_DEBUG_INTERVAL) {
                    s_lastDebugTime = dbgSimt;
                    double dbgScramN = 0;
                    double dbgDiffT = 0;
#ifdef HAS_XRVESSELCTRL
                    XRVesselCtrl* dbgXr = GetXRVessel(true);
                    if (dbgXr) {
                        XREngineStateRead dbgL = {}, dbgR = {};
                        if (dbgXr->GetEngineState(XREngineID::XRE_ScramLeft, dbgL)) {
                            dbgScramN += dbgL.Thrust;
                            if (dbgL.DiffuserTemp > dbgDiffT) dbgDiffT = dbgL.DiffuserTemp;
                        }
                        if (dbgXr->GetEngineState(XREngineID::XRE_ScramRight, dbgR)) {
                            dbgScramN += dbgR.Thrust;
                            if (dbgR.DiffuserTemp > dbgDiffT) dbgDiffT = dbgR.DiffuserTemp;
                        }
                    }
#endif
                    printf("[SCRAM DBG] M%.1f | Alt:%.0fkm | VS:%+.0f/%+.0f | Pitch:%.1f | Trim:%.3f(s%.2f) | Main:%.0f%% | SCRAM:%.0fkN | DynP:%.0f/%.0fkPa | Diff:%.0fK\n",
                           v->GetMachNumber(), alt / 1000.0, vspd, targetVspd, pitch,
                           v->GetControlSurfaceLevel(AIRCTRL_ELEVATORTRIM), trimSmoothing,
                           m_mainRampDown * 100.0, dbgScramN / 1000.0,
                           dynP / 1000.0, targetDynP / 1000.0, dbgDiffT);
                }
            }
#endif
            break;
        }

        case AP_XR_FINAL: {
            // Burn mains at ~10 deg pitch to raise apoapsis to target
            v->SetEngineLevel(ENGINE_MAIN, 1.0);

            double pitch = v->GetPitch() * DEG;
            double bank = v->GetBank() * DEG;
            VECTOR3 angvel;
            v->GetAngularVel(angvel);
            double rollRate = angvel.z * DEG;

            // Target pitch: 10 deg, reducing as apoapsis approaches target
            double targetPitch = 10.0;
            OBJHANDLE hRef = v->GetGravityRef();
            if (hRef) {
                double apDist;
                v->GetApDist(apDist);
                double refSize = oapiGetSize(hRef);
                double apAlt = apDist - refSize;
                double progress = apAlt / m_config.targetAlt;  // 0..1 toward target
                if (progress > 0.8)
                    targetPitch = 10.0 * (1.0 - (progress - 0.8) / 0.2);  // 10 -> 0 in last 20%
                if (targetPitch < 0.0) targetPitch = 0.0;
            }

            // Pitch/roll/heading control via RCS (above atmosphere, surfaces ineffective)
            {
                double pitchError = targetPitch - pitch;
                double rcsPitch = pitchError * 0.05;
                if (rcsPitch > 0.5) rcsPitch = 0.5;
                if (rcsPitch < -0.3) rcsPitch = -0.3;

                double rcsRoll = bank * 0.1 - rollRate * 0.2;
                if (rcsRoll > 0.5) rcsRoll = 0.5;
                if (rcsRoll < -0.5) rcsRoll = -0.5;

                v->SetAttitudeRotLevel(0, rcsPitch);
                v->SetAttitudeRotLevel(1, 0.0);
                v->SetAttitudeRotLevel(2, rcsRoll);

                // Clear aero surfaces (ineffective at altitude)
                v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
                v->SetControlSurfaceLevel(AIRCTRL_AILERON, 0.0);
                v->SetControlSurfaceLevel(AIRCTRL_RUDDER, 0.0);
            }
            break;
        }

        case AP_COAST:
        case AP_CIRCULARIZE: {
            // Prograde hold does attitude work; just ensure it's active
            if (!v->GetNavmodeState(NAVMODE_PROGRADE))
                v->ActivateNavmode(NAVMODE_PROGRADE);
            break;
        }

        default:
            break;
    }
}

void AutopilotController::SafeShutdown(VESSEL* v) {
    v->SetEngineLevel(ENGINE_MAIN, 0.0);
    v->SetEngineLevel(ENGINE_RETRO, 0.0);
    v->SetEngineLevel(ENGINE_HOVER, 0.0);
    v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
    v->SetControlSurfaceLevel(AIRCTRL_ELEVATORTRIM, 0.0);
    v->SetControlSurfaceLevel(AIRCTRL_AILERON, 0.0);
    v->SetControlSurfaceLevel(AIRCTRL_RUDDER, 0.0);
    v->SetAttitudeRotLevel(0, 0.0);  // Release RCS pitch
    v->SetAttitudeRotLevel(1, 0.0);  // Release RCS yaw
    v->SetAttitudeRotLevel(2, 0.0);  // Release RCS roll
    v->DeactivateNavmode(NAVMODE_PROGRADE);
    v->DeactivateNavmode(NAVMODE_KILLROT);
    v->DeactivateNavmode(NAVMODE_HLEVEL);

#ifdef HAS_XRVESSELCTRL
    if (m_config.isXR) {
        XRVesselCtrl* xr = GetXRVessel(true);
        if (xr) {
            XREngineStateRead state = {};
            if (xr->GetEngineState(XREngineID::XRE_ScramLeft, state)) {
                state.ThrottleLevel = 0.0;
                xr->SetEngineState(XREngineID::XRE_ScramLeft, state);
            }
            if (xr->GetEngineState(XREngineID::XRE_ScramRight, state)) {
                state.ThrottleLevel = 0.0;
                xr->SetEngineState(XREngineID::XRE_ScramRight, state);
            }
        }
    }
#endif
}

bool AutopilotController::UpdateStatus(VESSEL* v, double simt) {
    if (simt - m_lastStatusTime < STATUS_INTERVAL) {
        return false;
    }
    m_lastStatusTime = simt;

    char timeBuf[32];
    FormatTime(m_met, timeBuf, sizeof(timeBuf));

    double alt = v->GetAltitude();

    // XR phases get specialized status lines
    if (m_state == AP_XR_CLIMB) {
        double mach = v->GetMachNumber();
        double dynP = v->GetDynPressure() / 1000.0;  // kPa
        VECTOR3 climbStatusHv;
        v->GetAirspeedVector(FRAME_HORIZON, climbStatusHv);
        double climbVspd = climbStatusHv.y;
        snprintf(m_statusBuf, sizeof(m_statusBuf),
                 "[XR CLIMB] %s | Alt: %.0f km | M%.1f | VS: %+.0f m/s | DynP: %.0f kPa",
                 timeBuf, alt / 1000.0, mach, climbVspd, dynP);
    } else if (m_state == AP_XR_SCRAM) {
        double mach = v->GetMachNumber();
        double dynP = v->GetDynPressure() / 1000.0;
        VECTOR3 scramStatusHv;
        v->GetAirspeedVector(FRAME_HORIZON, scramStatusHv);
        double vspd = scramStatusHv.y;
        // Get combined SCRAM thrust for display
        double scramKN = 0;
#ifdef HAS_XRVESSELCTRL
        XRVesselCtrl* xr = GetXRVessel(true);
        if (xr) {
            XREngineStateRead left = {}, right = {};
            if (xr->GetEngineState(XREngineID::XRE_ScramLeft, left))
                scramKN += left.Thrust;
            if (xr->GetEngineState(XREngineID::XRE_ScramRight, right))
                scramKN += right.Thrust;
        }
#endif
        snprintf(m_statusBuf, sizeof(m_statusBuf),
                 "[XR SCRAM] %s | Alt: %.0f km | M%.1f | VS: %+.0f m/s | DynP: %.0f kPa | SCRAM: %.0f kN",
                 timeBuf, alt / 1000.0, mach, vspd, dynP, scramKN / 1000.0);
    } else if (m_state == AP_XR_FINAL) {
        OBJHANDLE hRef = v->GetGravityRef();
        double apDist = 0, peDist = 0;
        v->GetApDist(apDist);
        v->GetPeDist(peDist);
        double refSize = hRef ? oapiGetSize(hRef) : 0;
        snprintf(m_statusBuf, sizeof(m_statusBuf),
                 "[XR FINAL] %s | Alt: %.0f km | Ap: %.0f km | Pe: %.0f km",
                 timeBuf, alt / 1000.0, (apDist - refSize) / 1000.0, (peDist - refSize) / 1000.0);
    } else if (m_state == AP_COAST || m_state == AP_CIRCULARIZE) {
        OBJHANDLE hRef = v->GetGravityRef();
        double apDist = 0, peDist = 0;
        v->GetApDist(apDist);
        v->GetPeDist(peDist);
        double refSize = hRef ? oapiGetSize(hRef) : 0;
        ELEMENTS el;
        ORBITPARAM prm;
        char apTBuf[32] = "N/A";
        if (v->GetElements(hRef, el, &prm, 0, FRAME_ECL))
            FormatTime(prm.ApT, apTBuf, sizeof(apTBuf));
        const char* label = (m_state == AP_COAST) ? "COAST" : "CIRC";
        snprintf(m_statusBuf, sizeof(m_statusBuf),
                 "[%s] %s | Alt: %.0f km | Ap: %.0f km | Pe: %.0f km | ApT: %s",
                 label, timeBuf, alt / 1000.0,
                 (apDist - refSize) / 1000.0, (peDist - refSize) / 1000.0, apTBuf);
    } else {
        // Standard status for non-XR phases
        char altBuf[32], apBuf[32], peBuf[32];
        FormatDistance(alt, altBuf, sizeof(altBuf));

        OBJHANDLE hRef = v->GetGravityRef();
        double apDist = 0, peDist = 0;
        v->GetApDist(apDist);
        v->GetPeDist(peDist);
        double refSize = hRef ? oapiGetSize(hRef) : 0;

        FormatDistance(apDist - refSize, apBuf, sizeof(apBuf));
        FormatDistance(peDist - refSize, peBuf, sizeof(peBuf));

        snprintf(m_statusBuf, sizeof(m_statusBuf),
                 "[%s] %s | Alt: %s | Ap: %s | Pe: %s",
                 GetStateName(m_state), timeBuf, altBuf, apBuf, peBuf);
    }

    return true;
}

void AutopilotController::GetStatusText(char* buf, int bufLen) const {
    if (m_state == AP_IDLE) {
        snprintf(buf, bufLen, "Launch autopilot: Idle");
        return;
    }

    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        snprintf(buf, bufLen, "Launch autopilot: No vessel");
        return;
    }

    char timeBuf[32], altBuf[32];
    FormatTime(m_met > 0 ? m_met : 0, timeBuf, sizeof(timeBuf));
    FormatDistance(v->GetAltitude(), altBuf, sizeof(altBuf));

    snprintf(buf, bufLen, "%s | MET: %s | Alt: %s | Target: %.0f km",
             GetStateName(m_state), timeBuf, altBuf, m_config.targetAlt / 1000.0);
}

const char* AutopilotController::GetStateName(AutopilotState state) {
    switch (state) {
        case AP_IDLE:           return "IDLE";
        case AP_PRELAUNCH:      return "PRELAUNCH";
        case AP_LIFTOFF:        return "LIFTOFF";
        case AP_DEPARTURE_TURN: return "DEP TURN";
        case AP_PITCHOVER:      return "PITCHOVER";
        case AP_XR_CLIMB:       return "XR CLIMB";
        case AP_XR_SCRAM:       return "XR SCRAM";
        case AP_XR_FINAL:       return "XR FINAL";
        case AP_COAST:          return "COAST";
        case AP_CIRCULARIZE:    return "CIRC";
        case AP_COMPLETE:       return "COMPLETE";
        case AP_ABORTED:        return "ABORTED";
        default:                return "UNKNOWN";
    }
}
