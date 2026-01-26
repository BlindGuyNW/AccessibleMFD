// Autopilot.h - Launch command handlers

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

// Command handler for "launch" command
// Syntax:
//   launch           - Show status
//   launch <alt>     - Start launch to altitude (km)
//   launch abort     - Abort launch
void PrintLaunch(const char* arg);

#endif // AUTOPILOT_H
