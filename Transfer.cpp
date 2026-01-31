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

    // Check if targeting the reference body itself
    if (g_hTarget == hRef) {
        printf("Cannot calculate transfer to current reference body\n");
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

    // Check if targeting the reference body itself
    if (g_hTarget == hRef) {
        printf("Cannot calculate phase angle to current reference body\n");
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

    // Check if targeting the reference body itself
    if (g_hTarget == hRef) {
        printf("Cannot calculate plane change to current reference body\n");
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
        } else if (_stricmp(arg, "int") == 0 || _stricmp(arg, "intersect") == 0) {
            PrintIntersect();
            return;
        } else {
            printf("Unknown transfer command: %s\n", arg);
            printf("Options: hohmann, phase, plane, ren, int\n");
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

    // Check if targeting the reference body itself
    if (g_hTarget == hRef) {
        printf("Cannot transfer to current reference body\n");
        printf("You are already orbiting %s\n", targetName);
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

void PrintIntersect() {
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

    // Check if targeting the reference body itself
    if (g_hTarget == hRef) {
        printf("Cannot calculate intersection with current reference body\n");
        return;
    }

    OrbitIntersect oi = CalcOrbitIntersection(v, g_hTarget, hRef);

    if (!oi.valid) {
        printf("Cannot calculate orbit intersection\n");
        printf("(Vessel may be on escape trajectory)\n");
        return;
    }

    char targetName[256];
    oapiGetObjectName(g_hTarget, targetName, 256);

    printf("=== ORBIT INTERSECTION ===\n");
    printf("Target: %s\n", targetName);

    if (!oi.exists) {
        printf("No intersection points found\n");
        return;
    }

    // Format distances and times
    char dist1[64], dist2[64];
    char time1[64], time2[64];
    char tgtTime1[64], tgtTime2[64];

    FormatDistance(oi.distance1, dist1, sizeof(dist1));
    FormatDistance(oi.distance2, dist2, sizeof(dist2));
    FormatTime(oi.timeToInt1, time1, sizeof(time1));
    FormatTime(oi.timeToInt2, time2, sizeof(time2));
    FormatTime(oi.tgtTimeToInt1, tgtTime1, sizeof(tgtTime1));
    FormatTime(oi.tgtTimeToInt2, tgtTime2, sizeof(tgtTime2));

    printf("\nIntersection 1:\n");
    printf("  Longitude: %.1f deg\n", oi.longitude1 * DEG);
    printf("  Distance:  %s\n", dist1);
    printf("  Ship TtI:  %s\n", time1);
    printf("  Tgt TtI:   %s\n", tgtTime1);
    double timeDiff1 = oi.timeToInt1 - oi.tgtTimeToInt1;
    printf("  DTi:       %.1f s (%s)\n", fabs(timeDiff1),
           timeDiff1 > 0 ? "ship arrives later" : "ship arrives first");

    printf("\nIntersection 2:\n");
    printf("  Longitude: %.1f deg\n", oi.longitude2 * DEG);
    printf("  Distance:  %s\n", dist2);
    printf("  Ship TtI:  %s\n", time2);
    printf("  Tgt TtI:   %s\n", tgtTime2);
    double timeDiff2 = oi.timeToInt2 - oi.tgtTimeToInt2;
    printf("  DTi:       %.1f s (%s)\n", fabs(timeDiff2),
           timeDiff2 > 0 ? "ship arrives later" : "ship arrives first");

    // Recommend closest timing
    if (fabs(timeDiff1) < fabs(timeDiff2)) {
        printf("\nBest match: Intersection 1 (DTi %.1f s)\n", fabs(timeDiff1));
    } else {
        printf("\nBest match: Intersection 2 (DTi %.1f s)\n", fabs(timeDiff2));
    }
}

void PrintSync(const char* arg) {
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

    // Check if targeting the reference body itself
    if (g_hTarget == hRef) {
        printf("Cannot sync with current reference body\n");
        return;
    }

    // Parse mode from argument
    SyncRefMode mode = SYNC_INTERSECT1;
    if (arg && arg[0] != '\0') {
        if (_stricmp(arg, "int1") == 0 || _stricmp(arg, "intersect1") == 0) {
            mode = SYNC_INTERSECT1;
        } else if (_stricmp(arg, "int2") == 0 || _stricmp(arg, "intersect2") == 0) {
            mode = SYNC_INTERSECT2;
        } else if (_stricmp(arg, "pe") == 0 || _stricmp(arg, "shippe") == 0) {
            mode = SYNC_SHIP_PE;
        } else if (_stricmp(arg, "ap") == 0 || _stricmp(arg, "shipap") == 0) {
            mode = SYNC_SHIP_AP;
        } else if (_stricmp(arg, "tgtpe") == 0) {
            mode = SYNC_TGT_PE;
        } else if (_stricmp(arg, "tgtap") == 0) {
            mode = SYNC_TGT_AP;
        } else {
            printf("Unknown sync mode: %s\n", arg);
            printf("Options: int1, int2, pe, ap, tgtpe, tgtap\n");
            return;
        }
    }

    SyncData sd = CalcSync(v, g_hTarget, hRef, mode);

    if (!sd.valid) {
        printf("Cannot calculate sync data\n");
        printf("(Vessel may be on escape trajectory)\n");
        return;
    }

    char targetName[256];
    oapiGetObjectName(g_hTarget, targetName, 256);

    // Mode name
    const char* modeName = "Intersect 1";
    switch (mode) {
        case SYNC_INTERSECT2: modeName = "Intersect 2"; break;
        case SYNC_SHIP_PE: modeName = "Ship Pe"; break;
        case SYNC_SHIP_AP: modeName = "Ship Ap"; break;
        case SYNC_TGT_PE: modeName = "Target Pe"; break;
        case SYNC_TGT_AP: modeName = "Target Ap"; break;
        default: break;
    }

    printf("=== ORBIT SYNC ===\n");
    printf("Target: %s\n", targetName);
    printf("Reference: %s (%.1f deg)\n", modeName, sd.refLongitude * DEG);

    printf("\n Orb   Ship TtRef    Tgt TtRef      DTmin\n");
    printf("----  -----------  -----------  ----------\n");

    for (int i = 0; i < 5; i++) {
        char shipTime[32], tgtTime[32];
        FormatTime(sd.shipTimeToRef[i], shipTime, sizeof(shipTime));
        FormatTime(sd.tgtTimeToRef[i], tgtTime, sizeof(tgtTime));

        // Find best target orbit for this ship orbit
        double bestDiff = 1e20;
        int bestTgt = 0;
        for (int j = 0; j < 10; j++) {
            double diff = fabs(sd.shipTimeToRef[i] - sd.tgtTimeToRef[j]);
            if (diff < bestDiff) {
                bestDiff = diff;
                bestTgt = j;
            }
        }

        char diffBuf[32];
        FormatTime(bestDiff, diffBuf, sizeof(diffBuf));

        // Mark best match
        char marker = (i == sd.bestShipOrbit) ? '*' : ' ';
        printf("%c %d   %11s  %11s  %10s\n", marker, i, shipTime, tgtTime, diffBuf);
    }

    printf("\nBest match: Ship orbit %d, Target orbit %d\n",
           sd.bestShipOrbit, sd.bestTgtOrbit);

    char minDiffBuf[64];
    FormatTime(sd.minTimeDiff, minDiffBuf, sizeof(minDiffBuf));
    printf("Minimum DTmin: %s\n", minDiffBuf);
}

void PrintAlign(const char* arg) {
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

    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) {
        printf("No reference body\n");
        return;
    }

    // Check if targeting the reference body itself
    if (g_hTarget == hRef) {
        printf("Cannot align to current reference body\n");
        return;
    }

    // Parse mode from argument
    AlignMode mode = ALIGN_AUTO;
    if (arg && arg[0] != '\0') {
        if (_stricmp(arg, "orbit") == 0) {
            mode = ALIGN_ORBIT;
        } else if (_stricmp(arg, "ballistic") == 0) {
            mode = ALIGN_BALLISTIC;
        } else if (_stricmp(arg, "surface") == 0) {
            mode = ALIGN_SURFACE;
        } else if (_stricmp(arg, "auto") != 0) {
            printf("Unknown align mode: %s\n", arg);
            printf("Options: auto, orbit, ballistic, surface\n");
            return;
        }
    }

    // Get target and reference names
    char targetName[256], refName[256];
    oapiGetObjectName(g_hTarget, targetName, 256);
    oapiGetObjectName(hRef, refName, 256);

    // Calculate alignment data
    PlaneAlignData pa = CalcPlaneAlign(v, g_hTarget, hRef, mode);

    if (!pa.valid) {
        printf("Cannot calculate plane alignment\n");
        printf("(Vessel may be on escape trajectory)\n");
        return;
    }

    // Determine mode name for display
    const char* modeName = "Orbit";
    if (mode == ALIGN_BALLISTIC) modeName = "Ballistic";
    else if (mode == ALIGN_SURFACE) modeName = "Surface";

    printf("=== PLANE ALIGNMENT ===\n");
    printf("Target: %s\n", targetName);
    printf("Reference: %s\n", refName);
    printf("Mode: %s\n", modeName);

    printf("\nVessel:  Inc %.2f deg  LAN %.2f deg\n", pa.vesselInc, pa.vesselLAN);
    printf("Target:  Inc %.2f deg  LAN %.2f deg\n", pa.targetInc, pa.targetLAN);

    printf("\nRelative Inc: %.2f deg\n", pa.relInc);

    if (pa.relInc < 0.01) {
        printf("\nPlanes are aligned - no correction needed\n");
        return;
    }

    // Format time to nodes
    char timeAN[64], timeDN[64];
    FormatTime(pa.timeToAN, timeAN, sizeof(timeAN));
    FormatTime(pa.timeToDN, timeDN, sizeof(timeDN));

    printf("\nAscending Node:  %.1f deg  TtAN %s\n", pa.angleToAN, timeAN);
    printf("Descending Node: %.1f deg  TtDN %s\n", pa.angleToDN, timeDN);

    // Burn recommendation: NML- at ascending node, NML+ at descending node
    printf("\nBurn at %s (%s)\n",
        pa.burnAtAN ? "AN" : "DN",
        pa.burnAtAN ? "NML-" : "NML+");

    printf("  dV: %.1f m/s\n", pa.burnDV);

    if (pa.burnTime > 0) {
        char burnTimeBuf[64], ttbBuf[64];
        FormatTime(pa.burnTime, burnTimeBuf, sizeof(burnTimeBuf));
        FormatTime(pa.timeToBurn, ttbBuf, sizeof(ttbBuf));
        printf("  Burn: %s\n", burnTimeBuf);
        printf("  TtB: %s\n", ttbBuf);
    }

    // Show surface launch window if in surface mode
    if (mode == ALIGN_SURFACE) {
        double launchWindow = CalcSurfaceLaunchWindow(v, g_hTarget, hRef);
        if (launchWindow >= 0) {
            char windowBuf[64];
            FormatTime(launchWindow, windowBuf, sizeof(windowBuf));
            printf("\n=== Surface Launch Window ===\n");
            if (launchWindow < 60) {
                printf("Launch window NOW!\n");
            } else {
                printf("Next window in: %s\n", windowBuf);
            }
            printf("(When launch site crosses target orbital plane)\n");
        }
    }
}

void PrintHelp() {
    printf("=== Data Commands ===\n");
    printf("  v, vessel  - Vessel info\n");
    printf("  o, orbit   - Orbital data\n");
    printf("  f, flight  - Flight data\n");
    printf("  sf, surface - Surface/atmosphere data\n");
    printf("    sf alt   - Altitude/speed/attitude details\n");
    printf("    sf atm   - Atmospheric conditions\n");
    printf("    sf forces - Lift/drag forces\n");
    printf("  m, mfd     - MFD modes and capture status\n");
    printf("    mfd l    - Read left MFD text\n");
    printf("    mfd r    - Read right MFD text\n");
    printf("    mfd <n>  - Read MFD slot n (0-11)\n");
    printf("  btn, buttons <l|r|n> - MFD button labels\n");
    printf("  press <l|r|n> <btn#> - Press MFD button (0-11)\n");
    printf("  cp, cockpit - Cockpit mode (glass/2D panel/virtual)\n");
    printf("  d, dock    - Docking guidance\n");
    printf("    d auto [n] - Auto-dock to target port n (default: IDS port)\n");
    printf("    d align  - Auto-align attitude only\n");
    printf("    d watch  - Continuous updates to clipboard\n");
    printf("    d ports  - List our ports and target ports\n");
    printf("  fu, fuel   - Fuel status (per tank, dV, flow)\n");
    printf("    fuel <n> - Detailed info for tank n\n");
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
    printf("  tr int     - Orbit intersection points and timing\n");
    printf("  sync       - Multi-orbit synchronization table\n");
    printf("    sync int1/int2 - Use orbit intersection as reference\n");
    printf("    sync pe/ap     - Use ship periapsis/apoapsis\n");
    printf("    sync tgtpe/tgtap - Use target periapsis/apoapsis\n");
    printf("  al, align  - Plane alignment (Inc, LAN, nodes, burn)\n");
    printf("    al orbit   - Orbit mode (osculating elements)\n");
    printf("    al ballistic - Ballistic mode (suborbital)\n");
    printf("    al surface - Surface mode (launch planning)\n");
    printf("\n=== Control Commands ===\n");
    printf("  na [mode]  - Autopilot (pro/retro/nml/anml/kill/level/halt/off)\n");
    printf("  th [n]     - Throttle 0-100 (or: th main/retro/hover n)\n");
    printf("  warp [n]   - Time warp (0.1=slow, 1=normal, 100000=max)\n");
    printf("\n=== Launch Autopilot ===\n");
    printf("  la, launch <alt>  - Launch to orbit (altitude in km)\n");
    printf("  la abort   - Abort launch autopilot\n");
    printf("  la         - Show launch status\n");
    printf("\n=== System ===\n");
    printf("  ?, help    - This help\n");
    printf("  q, quit    - Close console\n");
}
