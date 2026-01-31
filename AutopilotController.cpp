// AutopilotController.cpp - Launch autopilot implementation

#include "AutopilotController.h"
#include "Formatting.h"
#include "OrbitalCalc.h"
#include <stdio.h>
#include <math.h>

// Global instance
AutopilotController g_autopilotController;

// Configuration constants
static const double COUNTDOWN_START = 3.0;      // T-3 seconds before launch
static const double LIFTOFF_DURATION = 10.0;    // Vertical climb duration
static const double STATUS_INTERVAL = 15.0;     // Status update interval
static const double MIN_THROTTLE = 0.5;         // Minimum throttle during maxQ
static const double PROGRADE_ALT = 20000.0;     // Altitude to enable prograde hold
static const double MIN_PROGRADE_PATH_ANGLE = 10.0;  // Min flight path angle (deg) for prograde hold
// CIRC_LEAD_TIME removed - circularization now manual
static const double LOW_FUEL_THRESHOLD = 0.05;  // 5% fuel = abort
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
    15000.0           // maxDynPressure: 15 kPa
};

AutopilotController::AutopilotController()
    : m_state(AP_IDLE)
    , m_launchTime(0)
    , m_met(0)
    , m_lastStatusTime(0)
    , m_lastCountdown(0)
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
    if (m_state == AP_LIFTOFF || m_state == AP_DEPARTURE_TURN || m_state == AP_PITCHOVER) {
        if (CheckFuelExhaustion(v)) {
            printf("ABORT - Fuel exhausted.\n");
            Abort();
            return;
        }
    }

    // Run state machine
    UpdateStateMachine(v, simt, simdt);

    // Execute guidance and control
    if (m_state == AP_LIFTOFF || m_state == AP_DEPARTURE_TURN || m_state == AP_PITCHOVER) {
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
            if (fabs(headingError) < DEPARTURE_TURN_EXIT_HEADING) {
                printf("On heading (err=%.1f deg). Starting gravity turn.\n", headingError);
                m_state = AP_PITCHOVER;
            } else if (depAlt > DEPARTURE_TURN_MAX_ALT) {
                printf("Altitude limit — starting gravity turn (heading err=%.0f deg).\n", headingError);
                m_state = AP_PITCHOVER;
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

        case AP_COAST:
        case AP_CIRCULARIZE:
            // These states no longer used - circularization is manual
            // Keep cases to avoid compiler warning if enum values still exist
            break;

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

        case AP_COAST:
        case AP_CIRCULARIZE:
            // These states no longer used - circularization is manual
            break;

        default:
            break;
    }
}

void AutopilotController::SafeShutdown(VESSEL* v) {
    v->SetEngineLevel(ENGINE_MAIN, 0.0);
    v->SetEngineLevel(ENGINE_RETRO, 0.0);
    v->SetEngineLevel(ENGINE_HOVER, 0.0);
    v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
    v->SetControlSurfaceLevel(AIRCTRL_AILERON, 0.0);
    v->SetControlSurfaceLevel(AIRCTRL_RUDDER, 0.0);
    v->SetAttitudeRotLevel(0, 0.0);  // Release RCS pitch
    v->SetAttitudeRotLevel(1, 0.0);  // Release RCS yaw
    v->SetAttitudeRotLevel(2, 0.0);  // Release RCS roll
    v->DeactivateNavmode(NAVMODE_PROGRADE);
    v->DeactivateNavmode(NAVMODE_KILLROT);
    v->DeactivateNavmode(NAVMODE_HLEVEL);
}

bool AutopilotController::UpdateStatus(VESSEL* v, double simt) {
    if (simt - m_lastStatusTime < STATUS_INTERVAL) {
        return false;
    }
    m_lastStatusTime = simt;

    // Build status message
    char timeBuf[32], altBuf[32], apBuf[32], peBuf[32];
    FormatTime(m_met, timeBuf, sizeof(timeBuf));

    double alt = v->GetAltitude();
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
        case AP_IDLE:        return "IDLE";
        case AP_PRELAUNCH:   return "PRELAUNCH";
        case AP_LIFTOFF:        return "LIFTOFF";
        case AP_DEPARTURE_TURN: return "DEP TURN";
        case AP_PITCHOVER:      return "PITCHOVER";
        case AP_COAST:       return "COAST";
        case AP_CIRCULARIZE: return "CIRC";
        case AP_COMPLETE:    return "COMPLETE";
        case AP_ABORTED:     return "ABORTED";
        default:             return "UNKNOWN";
    }
}
