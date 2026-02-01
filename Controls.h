// Controls.h - Control commands (autopilot, throttle, time warp)

#ifndef CONTROLS_H
#define CONTROLS_H

// Navigation autopilot control
void PrintNav(const char* arg);

// Throttle/engine control
void PrintThrottle(const char* arg);

// Time warp control
void PrintWarp(const char* arg);

// XR vessel resupply control
void PrintResupply(const char* arg);

// XR vessel fuel dump control
void PrintFuelDump(const char* arg);

// XR vessel cross-feed control
void PrintCrossFeed(const char* arg);

// XR vessel door control
void PrintDoors(const char* arg);

#ifdef HAS_XRVESSELCTRL
#include "XRVesselCtrl.h"
XRVesselCtrl* GetXRVessel(bool quiet = false);
const char* DoorStateStr(XRDoorState s);
const char* DoorName(XRDoorID id);
const char* SupplyLineName(XRSupplyLineID id);
void PrintSupplyLineStatus(XRVesselCtrl* xr, XRSupplyLineID id);
#endif

#endif // CONTROLS_H
