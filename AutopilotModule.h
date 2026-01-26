// AutopilotModule.h - Orbiter module for per-frame autopilot updates

#ifndef AUTOPILOT_MODULE_H
#define AUTOPILOT_MODULE_H

#include "orbitersdk.h"

// Module class that receives per-frame callbacks from Orbiter
class AutopilotModule : public oapi::Module {
public:
    AutopilotModule(HINSTANCE hDLL);

    // Called at start of each simulation session
    void clbkSimulationStart(RenderMode mode) override;

    // Called at end of each simulation session
    void clbkSimulationEnd() override;

    // Called every simulation frame (before physics update)
    void clbkPreStep(double simt, double simdt, double mjd) override;
};

#endif // AUTOPILOT_MODULE_H
