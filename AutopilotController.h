// AutopilotController.h - Launch autopilot state machine and guidance

#ifndef AUTOPILOT_CONTROLLER_H
#define AUTOPILOT_CONTROLLER_H

#include "orbitersdk.h"

// Launch autopilot phases
enum AutopilotState {
    AP_IDLE,           // Not active
    AP_PRELAUNCH,      // Armed, counting down
    AP_LIFTOFF,        // Vertical climb (first 10 seconds)
    AP_PITCHOVER,      // Gravity turn (pitch program)
    AP_COAST,          // Coasting to apoapsis
    AP_CIRCULARIZE,    // Circularization burn at apoapsis
    AP_COMPLETE,       // Orbit achieved
    AP_ABORTED         // User aborted
};

// Launch configuration
struct LaunchConfig {
    double targetAlt;         // Target orbit altitude (m above surface)
    double launchAzimuth;     // Launch heading (radians, PI/2 = east)
    double pitchOverAlt;      // Altitude to start pitch program (m)
    double pitchEndAlt;       // Altitude to end pitch program (m)
    double targetPitch;       // Final pitch at pitchEndAlt (radians)
    double maxDynPressure;    // Max Q limit for throttle reduction (Pa)
};

// Autopilot controller - manages guidance and control
class AutopilotController {
public:
    AutopilotController();

    // Start launch to target altitude (km)
    bool StartLaunch(double targetAltKm);

    // Abort launch immediately
    void Abort();

    // Called every simulation frame
    void Update(double simt, double simdt);

    // Reset on simulation end
    void Reset();

    // Status queries
    bool IsActive() const { return m_state != AP_IDLE && m_state != AP_COMPLETE && m_state != AP_ABORTED; }
    AutopilotState GetState() const { return m_state; }
    double GetMET() const { return m_met; }

    // Get status text for console display (thread-safe)
    void GetStatusText(char* buf, int bufLen) const;

    // Get state name for display
    static const char* GetStateName(AutopilotState state);

private:
    // State machine
    void UpdateStateMachine(VESSEL* v, double simt, double simdt);

    // Guidance calculations
    double CalculateTargetPitch(double altitude) const;
    double CalculateThrottle(VESSEL* v) const;
    bool CheckMECOCondition(VESSEL* v) const;
    bool CheckFuelExhaustion(VESSEL* v) const;

    // Control execution
    void ExecuteGuidance(VESSEL* v, double simdt);
    void SafeShutdown(VESSEL* v);

    // Periodic status update (returns true if status changed)
    bool UpdateStatus(VESSEL* v, double simt);

    AutopilotState m_state;
    LaunchConfig m_config;

    double m_launchTime;      // Simulation time at T-0
    double m_met;             // Mission Elapsed Time
    double m_lastStatusTime;  // Last status update time
    int m_lastCountdown;      // Last countdown second announced

    // Status string for thread-safe access
    mutable char m_statusBuf[256];
};

// Global autopilot controller instance
extern AutopilotController g_autopilotController;

#endif // AUTOPILOT_CONTROLLER_H
