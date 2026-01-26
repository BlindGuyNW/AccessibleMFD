// Transfer.cpp - Transfer planner and help

#include "Transfer.h"
#include "OrbitalCalc.h"
#include "Formatting.h"
#include "orbitersdk.h"
#include <stdio.h>
#include <string.h>

// Module-local target state
static OBJHANDLE g_hTarget = NULL;
static bool g_bTargetIsVessel = false;

void PrintTarget(const char* arg) {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    // No argument - show current target
    if (!arg || arg[0] == '\0') {
        if (!g_hTarget) {
            printf("No target selected\n");
            printf("Use: tgt <name> to set, tgt list to see options\n");
            return;
        }

        char name[256];
        oapiGetObjectName(g_hTarget, name, 256);
        printf("Target: %s (%s)\n", name, g_bTargetIsVessel ? "vessel" : "celestial body");
        return;
    }

    // "list" - show available targets
    if (_stricmp(arg, "list") == 0) {
        printf("=== Celestial Bodies ===\n");
        int nBodies = oapiGetGbodyCount();
        for (int i = 0; i < nBodies; i++) {
            OBJHANDLE hBody = oapiGetGbodyByIndex(i);
            if (hBody) {
                char name[256];
                oapiGetObjectName(hBody, name, 256);
                printf("  %s\n", name);
            }
        }

        printf("=== Vessels ===\n");
        int nVessels = oapiGetVesselCount();
        char focusName[256];
        oapiGetObjectName(v->GetHandle(), focusName, 256);

        for (int i = 0; i < nVessels; i++) {
            OBJHANDLE hVessel = oapiGetVesselByIndex(i);
            if (hVessel) {
                char name[256];
                oapiGetObjectName(hVessel, name, 256);
                // Skip the focus vessel
                if (_stricmp(name, focusName) != 0) {
                    printf("  %s\n", name);
                }
            }
        }
        return;
    }

    // "clear" - clear target
    if (_stricmp(arg, "clear") == 0) {
        g_hTarget = NULL;
        g_bTargetIsVessel = false;
        printf("Target cleared\n");
        return;
    }

    // Try to find the target by name
    // Copy argument to non-const buffer (API requires char*)
    char targetName[256];
    strncpy(targetName, arg, 255);
    targetName[255] = '\0';

    // First try celestial body
    OBJHANDLE hBody = oapiGetGbodyByName(targetName);
    if (hBody) {
        g_hTarget = hBody;
        g_bTargetIsVessel = false;
        printf("Target: %s (celestial body)\n", targetName);
        return;
    }

    // Try vessel
    OBJHANDLE hVessel = oapiGetVesselByName(targetName);
    if (hVessel) {
        // Don't allow targeting self
        if (hVessel == v->GetHandle()) {
            printf("Cannot target own vessel\n");
            return;
        }
        g_hTarget = hVessel;
        g_bTargetIsVessel = true;
        printf("Target: %s (vessel)\n", targetName);
        return;
    }

    printf("Target not found: %s\n", arg);
    printf("Use 'tgt list' to see available targets\n");
}

void PrintHohmann() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    if (!g_hTarget) {
        printf("No target selected\n");
        return;
    }

    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) {
        printf("No reference body\n");
        return;
    }

    // Get vessel orbital elements
    ELEMENTS el;
    ORBITPARAM prm;
    if (!v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) {
        printf("Cannot get orbital elements\n");
        return;
    }

    // Check for escape trajectory
    if (el.e >= 1.0) {
        printf("Vessel on escape trajectory (e=%.3f)\n", el.e);
        printf("Hohmann transfer not applicable\n");
        return;
    }

    // Get current radius and target radius
    double gm = GetGM(hRef);
    double r1 = el.a * (1.0 - el.e * el.e) / (1.0 + el.e * cos(prm.TrA));

    // Get target radius relative to same reference
    VECTOR3 targetPos, refPos;
    oapiGetGlobalPos(g_hTarget, &targetPos);
    oapiGetGlobalPos(hRef, &refPos);
    double r2 = length(targetPos - refPos);

    // Calculate Hohmann transfer
    HohmannTransfer ht = CalcHohmann(r1, r2, gm);

    if (!ht.valid) {
        printf("Cannot calculate transfer\n");
        return;
    }

    char timeBuf[64];
    FormatTime(ht.transferTime, timeBuf, sizeof(timeBuf));

    printf("=== Hohmann Transfer ===\n");
    if (r2 > r1) {
        printf("  Departure DV: %.1f m/s (prograde)\n", ht.departDV);
        printf("  Arrival DV:   %.1f m/s (retrograde)\n", ht.arriveDV);
    } else {
        printf("  Departure DV: %.1f m/s (retrograde)\n", ht.departDV);
        printf("  Arrival DV:   %.1f m/s (prograde)\n", ht.arriveDV);
    }
    printf("  Total DV:     %.1f m/s\n", ht.totalDV);
    printf("  Transfer Time: %s\n", timeBuf);

    // Warn about elliptical orbit
    if (el.e > 0.05) {
        printf("  Note: Current orbit is elliptical (e=%.3f)\n", el.e);
        printf("        Values assume burn at current radius\n");
    }
}

void PrintPhase() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    if (!g_hTarget) {
        printf("No target selected\n");
        return;
    }

    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) {
        printf("No reference body\n");
        return;
    }

    PhaseAngleData pa = CalcPhaseAngle(v, g_hTarget, hRef);

    if (!pa.valid) {
        printf("Cannot calculate phase angle\n");
        printf("(Vessel may be on escape trajectory)\n");
        return;
    }

    printf("=== Phase Angle ===\n");
    printf("  Current:  %.1f deg\n", pa.currentPhase);
    printf("  Required: %.1f deg\n", pa.requiredPhase);
    printf("  Difference: %.1f deg\n", pa.phaseDiff);

    if (pa.timeToWindow > 0) {
        char timeBuf[64];
        FormatTime(pa.timeToWindow, timeBuf, sizeof(timeBuf));
        printf("  Time to Window: %s\n", timeBuf);
    } else if (pa.timeToWindow < 0) {
        printf("  Time to Window: N/A (same period)\n");
    }
}

void PrintPlane() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    if (!g_hTarget) {
        printf("No target selected\n");
        return;
    }

    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) {
        printf("No reference body\n");
        return;
    }

    PlaneChangeData pc = CalcPlaneChange(v, g_hTarget, hRef);

    if (!pc.valid) {
        printf("Cannot calculate plane change\n");
        return;
    }

    printf("=== Plane Change ===\n");
    printf("  Relative Inc: %.2f deg\n", pc.relInc);

    if (pc.relInc < 0.1) {
        printf("  Planes are coplanar - no correction needed\n");
    } else {
        printf("  Plane Change DV: %.1f m/s\n", pc.planeChangeDV);
        printf("  Angle to Node: %.1f deg\n", pc.nodeAngle);
    }
}

void PrintRendezvous() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    if (!g_hTarget) {
        printf("No target selected\n");
        return;
    }

    if (!g_bTargetIsVessel) {
        printf("Rendezvous data only available for vessel targets\n");
        return;
    }

    RendezvousData rd = CalcRendezvous(v, g_hTarget);

    if (!rd.valid) {
        printf("Cannot calculate rendezvous data\n");
        return;
    }

    char distBuf[64];
    FormatDistance(rd.distance, distBuf, sizeof(distBuf));

    printf("=== Rendezvous ===\n");
    printf("  Distance: %s\n", distBuf);
    printf("  Closure Rate: %.2f m/s", rd.closureRate);
    if (rd.closureRate > 0)
        printf(" (approaching)\n");
    else if (rd.closureRate < 0)
        printf(" (separating)\n");
    else
        printf("\n");

    printf("  Relative Vel: %.2f m/s\n", rd.relVelMag);

    if (rd.timeToClose > 0) {
        char timeBuf[64];
        FormatTime(rd.timeToClose, timeBuf, sizeof(timeBuf));
        printf("  Time to Close: %s\n", timeBuf);
    }

    printf("  Local Frame Vel:\n");
    printf("    Vx: %.2f m/s\n", rd.relVel.x);
    printf("    Vy: %.2f m/s\n", rd.relVel.y);
    printf("    Vz: %.2f m/s\n", rd.relVel.z);
}

void PrintTransfer(const char* arg) {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    if (!g_hTarget) {
        printf("No target selected\n");
        printf("Use: tgt <name> to set a target first\n");
        return;
    }

    // Get target name
    char targetName[256];
    oapiGetObjectName(g_hTarget, targetName, 256);

    // Subcommand handling
    if (arg && arg[0] != '\0') {
        if (_stricmp(arg, "hohmann") == 0) {
            PrintHohmann();
            return;
        } else if (_stricmp(arg, "phase") == 0) {
            PrintPhase();
            return;
        } else if (_stricmp(arg, "plane") == 0) {
            PrintPlane();
            return;
        } else if (_stricmp(arg, "ren") == 0 || _stricmp(arg, "rendezvous") == 0) {
            PrintRendezvous();
            return;
        } else {
            printf("Unknown transfer command: %s\n", arg);
            printf("Options: hohmann, phase, plane, ren\n");
            return;
        }
    }

    // Default: show full transfer summary
    printf("=== TRANSFER TO %s ===\n", targetName);

    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) {
        printf("No reference body\n");
        return;
    }

    // Get vessel orbital elements
    ELEMENTS el;
    ORBITPARAM prm;
    if (!v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) {
        printf("Cannot get orbital elements\n");
        return;
    }

    // Check for escape trajectory
    if (el.e >= 1.0) {
        printf("Vessel on escape trajectory (e=%.3f)\n", el.e);
        printf("Transfer calculations not applicable\n");
        return;
    }

    // Get radii
    double gm = GetGM(hRef);
    double r1 = el.a * (1.0 - el.e * el.e) / (1.0 + el.e * cos(prm.TrA));

    VECTOR3 targetPos, refPos;
    oapiGetGlobalPos(g_hTarget, &targetPos);
    oapiGetGlobalPos(hRef, &refPos);
    double r2 = length(targetPos - refPos);

    // Hohmann transfer
    HohmannTransfer ht = CalcHohmann(r1, r2, gm);
    if (ht.valid) {
        char timeBuf[64];
        FormatTime(ht.transferTime, timeBuf, sizeof(timeBuf));

        printf("\nHohmann Transfer:\n");
        if (r2 > r1) {
            printf("  Departure DV: %.1f m/s (prograde)\n", ht.departDV);
            printf("  Arrival DV:   %.1f m/s (retrograde)\n", ht.arriveDV);
        } else {
            printf("  Departure DV: %.1f m/s (retrograde)\n", ht.departDV);
            printf("  Arrival DV:   %.1f m/s (prograde)\n", ht.arriveDV);
        }
        printf("  Total DV:     %.1f m/s\n", ht.totalDV);
        printf("  Transfer Time: %s\n", timeBuf);
    }

    // Phase angle
    PhaseAngleData pa = CalcPhaseAngle(v, g_hTarget, hRef);
    if (pa.valid) {
        printf("\nPhase Angle:\n");
        printf("  Current:  %.1f deg\n", pa.currentPhase);
        printf("  Required: %.1f deg\n", pa.requiredPhase);
        if (pa.timeToWindow > 0) {
            char timeBuf[64];
            FormatTime(pa.timeToWindow, timeBuf, sizeof(timeBuf));
            printf("  Time to Window: %s\n", timeBuf);
        }
    }

    // Plane change
    PlaneChangeData pc = CalcPlaneChange(v, g_hTarget, hRef);
    if (pc.valid && pc.relInc > 0.1) {
        printf("\nPlane Change:\n");
        printf("  Relative Inc: %.2f deg\n", pc.relInc);
        printf("  Plane Change DV: %.1f m/s\n", pc.planeChangeDV);
    }

    // Rendezvous (vessel targets only)
    if (g_bTargetIsVessel) {
        RendezvousData rd = CalcRendezvous(v, g_hTarget);
        if (rd.valid) {
            char distBuf[64];
            FormatDistance(rd.distance, distBuf, sizeof(distBuf));

            printf("\nRendezvous:\n");
            printf("  Distance: %s\n", distBuf);
            printf("  Closure Rate: %.2f m/s\n", rd.closureRate);
        }
    }
}

void PrintHelp() {
    printf("=== Data Commands ===\n");
    printf("  v, vessel  - Vessel info\n");
    printf("  o, orbit   - Orbital data\n");
    printf("  f, flight  - Flight data\n");
    printf("  m, mfd     - MFD modes\n");
    printf("  d, dock    - Docking data\n");
    printf("  fuel       - Fuel status\n");
    printf("  map        - Position, altitude, ground track\n");
    printf("  map bases  - List bases with distance/bearing\n");
    printf("  a, all     - All data\n");
    printf("\n=== Transfer Planner ===\n");
    printf("  tgt [name] - Show/set target (body or vessel)\n");
    printf("  tgt list   - List bodies and vessels\n");
    printf("  tgt clear  - Clear target\n");
    printf("  tr         - Transfer summary\n");
    printf("  tr hohmann - Hohmann transfer delta-v\n");
    printf("  tr phase   - Phase angle to transfer window\n");
    printf("  tr plane   - Plane change requirements\n");
    printf("  tr ren     - Rendezvous data (vessel targets)\n");
    printf("\n=== Control Commands ===\n");
    printf("  na [mode]  - Autopilot (pro/retro/nml/anml/kill/level/halt/off)\n");
    printf("  th [n]     - Throttle 0-100 (or: th main/retro/hover n)\n");
    printf("  warp [n]   - Time warp (0.1=slow, 1=normal, 100000=max)\n");
    printf("\n=== System ===\n");
    printf("  ?, help    - This help\n");
    printf("  q, quit    - Close console\n");
}
