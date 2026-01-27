// OrbitalCalc.h - Orbital mechanics calculations

#ifndef ORBITALCALC_H
#define ORBITALCALC_H

#include "orbitersdk.h"

// Gravitational constant
extern const double G_CONST;  // m^3 kg^-1 s^-2

// Hohmann transfer data
struct HohmannTransfer {
    double departDV;     // m/s
    double arriveDV;     // m/s
    double totalDV;      // m/s
    double transferTime; // seconds
    bool valid;
};

// Phase angle calculation data
struct PhaseAngleData {
    double currentPhase;  // degrees
    double requiredPhase; // degrees
    double phaseDiff;     // degrees (positive = wait, negative = missed)
    double timeToWindow;  // seconds
    bool valid;
};

// Plane change calculation data
struct PlaneChangeData {
    double relInc;        // degrees
    double planeChangeDV; // m/s
    double nodeAngle;     // degrees from current position to AN
    bool valid;
};

// Rendezvous calculation data
struct RendezvousData {
    double distance;      // m
    double closureRate;   // m/s (positive = approaching)
    double relVelMag;     // m/s
    VECTOR3 relVel;       // local frame relative velocity
    double timeToClose;   // seconds at current rate
    bool valid;
};

// Plane alignment data
struct PlaneAlignData {
    // Vessel orbital plane (ecliptic frame)
    double vesselInc;      // Inclination [deg]
    double vesselLAN;      // Longitude of ascending node [deg]

    // Target orbital plane (ecliptic frame)
    double targetInc;      // Inclination [deg]
    double targetLAN;      // Longitude of ascending node [deg]

    // Relative plane data
    double relInc;         // Relative inclination [deg]

    // Node positions (angles from current position)
    double angleToAN;      // Angle to ascending node [deg]
    double angleToDN;      // Angle to descending node [deg]

    // Time to nodes
    double timeToAN;       // Time to ascending node [s]
    double timeToDN;       // Time to descending node [s]

    // Burn parameters
    bool burnAtAN;         // true = NML+ at AN, false = NML- at DN
    double burnDV;         // Required delta-V [m/s]
    double burnTime;       // Estimated burn duration [s]
    double timeToBurn;     // Time to start burn [s]

    bool valid;
};

enum AlignMode {
    ALIGN_AUTO,
    ALIGN_ORBIT,
    ALIGN_BALLISTIC,
    ALIGN_SURFACE
};

// Orbit intersection data
struct OrbitIntersect {
    bool exists;           // Whether orbits intersect
    double longitude1;     // True longitude of first intersection [rad]
    double longitude2;     // True longitude of second intersection [rad]
    double distance1;      // Distance from reference body at int 1 [m]
    double distance2;      // Distance from reference body at int 2 [m]
    double timeToInt1;     // Time to first intersection [s]
    double timeToInt2;     // Time to second intersection [s]
    double tgtTimeToInt1;  // Target time to first intersection [s]
    double tgtTimeToInt2;  // Target time to second intersection [s]
    VECTOR3 intPos1;       // Position vector at intersection 1
    VECTOR3 intPos2;       // Position vector at intersection 2
    bool valid;
};

// Orbit synchronization data
struct SyncData {
    int refMode;              // Reference axis mode
    double refLongitude;      // Reference axis longitude [rad]
    double shipTimeToRef[10]; // Ship time to ref for orbits 0-9 [s]
    double tgtTimeToRef[10];  // Target time to ref for orbits 0-9 [s]
    int bestShipOrbit;        // Best match orbit number for ship
    int bestTgtOrbit;         // Best match orbit number for target
    double minTimeDiff;       // Minimum time difference [s]
    bool valid;
};

// Sync reference modes
enum SyncRefMode {
    SYNC_INTERSECT1,    // First orbit intersection
    SYNC_INTERSECT2,    // Second orbit intersection
    SYNC_SHIP_PE,       // Ship periapsis
    SYNC_SHIP_AP,       // Ship apoapsis
    SYNC_TGT_PE,        // Target periapsis
    SYNC_TGT_AP         // Target apoapsis
};

// Get GM (gravitational parameter) for a body
double GetGM(OBJHANDLE hBody);

// Calculate Hohmann transfer between two circular orbits
// r1 = departure orbit radius, r2 = target orbit radius, gm = gravitational parameter
HohmannTransfer CalcHohmann(double r1, double r2, double gm);

// Calculate phase angle data between vessel and target
// Returns current phase, required phase for Hohmann, and time to window
PhaseAngleData CalcPhaseAngle(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef);

// Calculate plane change requirements
PlaneChangeData CalcPlaneChange(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef);

// Calculate rendezvous data for vessel targets
RendezvousData CalcRendezvous(VESSEL* v, OBJHANDLE hTarget);

// Calculate plane alignment data for orbital plane matching
PlaneAlignData CalcPlaneAlign(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef, AlignMode mode);

// Calculate orbit intersection points between vessel and target
OrbitIntersect CalcOrbitIntersection(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef);

// Calculate multi-orbit synchronization timing
SyncData CalcSync(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef, SyncRefMode mode);

// Calculate surface launch window for plane alignment
// Returns time in seconds until next launch window, or -1 if not applicable
double CalcSurfaceLaunchWindow(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef);

#endif // ORBITALCALC_H
