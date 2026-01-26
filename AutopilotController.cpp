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
static const double CIRC_LEAD_TIME = 30.0;      // Seconds before apoapsis to start circ burn
static const double LOW_FUEL_THRESHOLD = 0.05;  // 5% fuel = abort

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

bool AutopilotController::StartLaunch(double targetAltKm) {
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

    // Configure for target altitude
    m_config = DEFAULT_CONFIG;
    m_config.targetAlt = targetAltKm * 1000.0;

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

    printf("Launch autopilot armed. Target: %.0f km\n", targetAltKm);
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
    if (m_state == AP_LIFTOFF || m_state == AP_PITCHOVER || m_state == AP_CIRCULARIZE) {
        if (CheckFuelExhaustion(v)) {
            printf("ABORT - Fuel exhausted.\n");
            Abort();
            return;
        }
    }

    // Run state machine
    UpdateStateMachine(v, simt, simdt);

    // Execute guidance and control
    if (m_state >= AP_LIFTOFF && m_state <= AP_CIRCULARIZE) {
        ExecuteGuidance(v, simdt);
    }

    // Periodic status updates
    if (UpdateStatus(v, simt)) {
        printf("\n%s\n> ", m_statusBuf);
        fflush(stdout);
    }
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

            // ROLL CONTROL - Keep wings level
            // SIGN: In Orbiter, POSITIVE aileron = roll LEFT, NEGATIVE = roll RIGHT
            // So if bank is positive (rolled right), we need positive aileron (roll left)
            double aileronCmd = bank * 0.15;  // Same sign as bank to counter it
            if (aileronCmd > 1.0) aileronCmd = 1.0;
            if (aileronCmd < -1.0) aileronCmd = -1.0;
            v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);

            // Also use RCS for roll - same sign convention
            double rcsRoll = bank * 0.05;
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

            // VERBOSE DEBUG - every 2 seconds
            static double lastDebug = -10;
            if (m_met - lastDebug > 2.0) {
                printf("DEBUG: MET=%.0f Alt=%.1f Spd=%.0f Pitch=%.1f Bank=%.1f VS=%.1f Elev=%.2f OnGnd=%d\n",
                       m_met, alt, spd, pitch, bank, vspd, elevatorCmd, onGround ? 1 : 0);
                lastDebug = m_met;
            }

            // Transition to pitchover once we have altitude AND good climb
            if (alt > 1000.0 && vspd > 20.0) {
                printf("Climbing well. Alt: %.0f m, Speed: %.0f m/s, VS: %.0f m/s\n",
                       alt, spd, vspd);
                m_state = AP_PITCHOVER;
            }
            break;
        }

        case AP_PITCHOVER:
            // Follow pitch program until MECO
            if (CheckMECOCondition(v)) {
                printf("MECO - Main engine cutoff.\n");
                v->SetEngineLevel(ENGINE_MAIN, 0.0);
                v->ActivateNavmode(NAVMODE_PROGRADE);
                m_state = AP_COAST;
            }
            break;

        case AP_COAST:
            // Coast to apoapsis
            if (CheckCircularizationStart(v)) {
                printf("Circularization burn initiated.\n");
                v->SetEngineLevel(ENGINE_MAIN, 1.0);
                m_state = AP_CIRCULARIZE;
            }
            break;

        case AP_CIRCULARIZE:
            // Burn until orbit is circular
            if (CheckCircularizationEnd(v)) {
                printf("Circularization complete.\n");
                SafeShutdown(v);
                m_state = AP_COMPLETE;

                // Print final orbit info
                OBJHANDLE hRef = v->GetGravityRef();
                ELEMENTS el;
                ORBITPARAM prm;
                if (v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) {
                    double refSize = oapiGetSize(hRef);
                    printf("Orbit achieved: %.1f x %.1f km\n",
                           (prm.PeD - refSize) / 1000.0,
                           (prm.ApD - refSize) / 1000.0);
                }
            }
            break;

        default:
            break;
    }
}

double AutopilotController::CalculateTargetPitch(double altitude) const {
    // Pitch program: linear interpolation from vertical to target pitch
    if (altitude < m_config.pitchOverAlt) {
        return PI05;  // 90 degrees (vertical)
    }
    if (altitude > m_config.pitchEndAlt) {
        return m_config.targetPitch;
    }

    // Linear interpolation
    double fraction = (altitude - m_config.pitchOverAlt) /
                      (m_config.pitchEndAlt - m_config.pitchOverAlt);
    return PI05 - fraction * (PI05 - m_config.targetPitch);
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

    // Check if we're above atmosphere using actual pressure
    double atmPressure = v->GetAtmPressure();
    bool aboveAtmosphere = (atmPressure < 1.0);  // Less than 1 Pa

    return (apAlt >= m_config.targetAlt && aboveAtmosphere);
}

bool AutopilotController::CheckCircularizationStart(VESSEL* v) const {
    // Start circularization burn when approaching apoapsis
    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) return false;

    ELEMENTS el;
    ORBITPARAM prm;
    if (!v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) return false;

    // Calculate required delta-V for circularization
    double gm = GetGM(hRef);
    double vAtAp = sqrt(gm * (2.0 / prm.ApD - 1.0 / el.a));  // Velocity at apoapsis
    double vCirc = sqrt(gm / prm.ApD);  // Circular velocity at apoapsis altitude
    double deltaV = vCirc - vAtAp;

    // Calculate burn time based on actual thrust and mass
    double thrust = v->GetMaxThrust(ENGINE_MAIN);
    double mass = v->GetMass();
    if (thrust <= 0 || mass <= 0) return false;

    double accel = thrust / mass;
    double burnTime = deltaV / accel;

    // Start burn half the burn time before apoapsis, plus lead time for orientation
    return (prm.ApT > 0 && prm.ApT < burnTime / 2.0 + CIRC_LEAD_TIME);
}

bool AutopilotController::CheckCircularizationEnd(VESSEL* v) const {
    // End circularization when orbit is sufficiently circular
    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) return false;

    ELEMENTS el;
    ORBITPARAM prm;
    if (!v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) return false;

    double refSize = oapiGetSize(hRef);
    double peAlt = prm.PeD - refSize;

    // Consider circular when eccentricity is very low
    // Or when periapsis reaches ~95% of target
    return (el.e < 0.01 || peAlt > m_config.targetAlt * 0.95);
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

        case AP_PITCHOVER: {
            double throttle = CalculateThrottle(v);
            v->SetEngineLevel(ENGINE_MAIN, throttle);

            // Below 20km: actively control pitch and roll
            if (alt < PROGRADE_ALT) {
                // ROLL CONTROL - Keep wings level
                // SIGN: POSITIVE aileron = roll LEFT, so same sign as bank to counter it
                double bank = v->GetBank() * DEG;
                double aileronCmd = bank * 0.15;
                if (aileronCmd > 1.0) aileronCmd = 1.0;
                if (aileronCmd < -1.0) aileronCmd = -1.0;
                v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);

                // RCS roll assist - same sign convention
                double rcsRoll = bank * 0.05;
                if (rcsRoll > 0.5) rcsRoll = 0.5;
                if (rcsRoll < -0.5) rcsRoll = -0.5;
                v->SetAttitudeRotLevel(2, rcsRoll);

                // PITCH CONTROL - Gradually reduce pitch as we climb (gravity turn)
                // Target pitch: 25 deg at 1km, reducing to ~10 deg at 20km
                double targetPitch = 25.0 - (alt / 1000.0);  // Roughly 1 deg less per km
                if (targetPitch < 10.0) targetPitch = 10.0;

                double currentPitch = v->GetPitch() * DEG;
                double pitchError = targetPitch - currentPitch;

                // POSITIVE elevator = nose UP (trying this)
                double elevatorCmd = pitchError * 0.1;
                if (elevatorCmd > 0.6) elevatorCmd = 0.6;
                if (elevatorCmd < -0.3) elevatorCmd = -0.3;

                double rcsCmd = pitchError * 0.05;
                if (rcsCmd > 0.5) rcsCmd = 0.5;
                if (rcsCmd < -0.3) rcsCmd = -0.3;

                v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, elevatorCmd);
                v->SetAttitudeRotLevel(0, rcsCmd);  // RCS pitch assist
            } else {
                // Above 20km: switch to prograde hold, but KEEP roll control!
                v->SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
                v->SetAttitudeRotLevel(0, 0.0);  // Release RCS pitch - prograde handles it

                // KEEP ROLL CONTROL - prograde doesn't stabilize roll!
                double bank = v->GetBank() * DEG;
                double aileronCmd = bank * 0.15;
                if (aileronCmd > 1.0) aileronCmd = 1.0;
                if (aileronCmd < -1.0) aileronCmd = -1.0;
                v->SetControlSurfaceLevel(AIRCTRL_AILERON, aileronCmd);

                double rcsRoll = bank * 0.1;  // Stronger RCS roll at high altitude
                if (rcsRoll > 0.5) rcsRoll = 0.5;
                if (rcsRoll < -0.5) rcsRoll = -0.5;
                v->SetAttitudeRotLevel(2, rcsRoll);

                if (!v->GetNavmodeState(NAVMODE_PROGRADE)) {
                    printf("Engaging prograde hold at %.0f km.\n", alt / 1000.0);
                    v->ActivateNavmode(NAVMODE_PROGRADE);
                }
            }
            break;
        }

        case AP_COAST:
            // Prograde hold, engines off
            v->SetEngineLevel(ENGINE_MAIN, 0.0);
            if (!v->GetNavmodeState(NAVMODE_PROGRADE)) {
                v->ActivateNavmode(NAVMODE_PROGRADE);
            }
            break;

        case AP_CIRCULARIZE:
            // Prograde hold, full throttle
            v->SetEngineLevel(ENGINE_MAIN, 1.0);
            if (!v->GetNavmodeState(NAVMODE_PROGRADE)) {
                v->ActivateNavmode(NAVMODE_PROGRADE);
            }
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
        case AP_LIFTOFF:     return "LIFTOFF";
        case AP_PITCHOVER:   return "PITCHOVER";
        case AP_COAST:       return "COAST";
        case AP_CIRCULARIZE: return "CIRC";
        case AP_COMPLETE:    return "COMPLETE";
        case AP_ABORTED:     return "ABORTED";
        default:             return "UNKNOWN";
    }
}
