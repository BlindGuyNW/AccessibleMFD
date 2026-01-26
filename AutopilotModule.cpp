// AutopilotModule.cpp - Orbiter module implementation for autopilot

#include "AutopilotModule.h"
#include "AutopilotController.h"

AutopilotModule::AutopilotModule(HINSTANCE hDLL)
    : oapi::Module(hDLL)
{
}

void AutopilotModule::clbkSimulationStart(RenderMode mode) {
    // Reset autopilot state at simulation start
    g_autopilotController.Reset();
}

void AutopilotModule::clbkSimulationEnd() {
    // Clean up autopilot state
    g_autopilotController.Reset();
}

void AutopilotModule::clbkPreStep(double simt, double simdt, double mjd) {
    // Update autopilot every frame
    g_autopilotController.Update(simt, simdt);
}
