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

#endif // ORBITALCALC_H
