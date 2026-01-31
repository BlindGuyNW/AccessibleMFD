// Queries.cpp - Data query functions for vessel, orbit, flight, etc.

#include "Queries.h"
#include "Formatting.h"
#include "MfdCapture.h"
#include <windows.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

// Copy text to Windows clipboard
static void CopyToClipboard(const char* text) {
    if (!OpenClipboard(NULL)) return;
    EmptyClipboard();

    HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, strlen(text) + 1);
    if (hMem) {
        char* pMem = (char*)GlobalLock(hMem);
        strcpy(pMem, text);
        GlobalUnlock(hMem);
        SetClipboardData(CF_TEXT, hMem);
    }

    CloseClipboard();
}

// Debug log file handle
static FILE* g_alignLog = NULL;

static void AlignLog(const char* fmt, ...) {
    if (!g_alignLog) return;
    va_list args;
    va_start(args, fmt);
    vfprintf(g_alignLog, fmt, args);
    va_end(args);
    fflush(g_alignLog);
}

// Calculate total angular error between dock orientations (returns degrees)
// Also outputs the rotation axis in ship coordinates for control
static double CalcDockAngularError(VESSEL* v, UINT refDock, OBJHANDLE hTarget, UINT tgtDock,
                                   VECTOR3* rotAxis) {
    DOCKHANDLE hOurDock = v->GetDockHandle(refDock);
    VESSEL* tgt = oapiGetVesselInterface(hTarget);
    DOCKHANDLE hTgtDock = tgt->GetDockHandle(tgtDock);

    // Get dock parameters (in vessel local coords)
    VECTOR3 ourPos, ourDir, ourRot;
    v->GetDockParams(hOurDock, ourPos, ourDir, ourRot);

    VECTOR3 tgtPos, tgtDir, tgtRot;
    tgt->GetDockParams(hTgtDock, tgtPos, tgtDir, tgtRot);

    AlignLog("--- CalcDockAngularError ---\n");
    AlignLog("ourDir (local): %.4f, %.4f, %.4f\n", ourDir.x, ourDir.y, ourDir.z);
    AlignLog("ourRot (local): %.4f, %.4f, %.4f\n", ourRot.x, ourRot.y, ourRot.z);
    AlignLog("tgtDir (local): %.4f, %.4f, %.4f\n", tgtDir.x, tgtDir.y, tgtDir.z);
    AlignLog("tgtRot (local): %.4f, %.4f, %.4f\n", tgtRot.x, tgtRot.y, tgtRot.z);

    // Get rotation matrices
    MATRIX3 ourRotMat, tgtRotMat;
    v->GetRotationMatrix(ourRotMat);
    tgt->GetRotationMatrix(tgtRotMat);

    // Target dock direction and up in global coords
    // We want to face OPPOSITE to target's direction
    VECTOR3 tgtDirGlobal = mul(tgtRotMat, tgtDir);
    VECTOR3 tgtUpGlobal = mul(tgtRotMat, tgtRot);
    VECTOR3 desiredDirGlobal = -tgtDirGlobal;  // We face opposite

    // Our dock direction and up in global coords
    VECTOR3 ourDirGlobal = mul(ourRotMat, ourDir);
    VECTOR3 ourUpGlobal = mul(ourRotMat, ourRot);

    AlignLog("ourDirGlobal: %.4f, %.4f, %.4f (len=%.4f)\n",
             ourDirGlobal.x, ourDirGlobal.y, ourDirGlobal.z, length(ourDirGlobal));
    AlignLog("desiredDirGlobal: %.4f, %.4f, %.4f (len=%.4f)\n",
             desiredDirGlobal.x, desiredDirGlobal.y, desiredDirGlobal.z, length(desiredDirGlobal));

    // Calculate rotation from our current orientation to desired
    // Using cross product to find rotation axis, dot product for angle

    VECTOR3 dirCross = crossp(ourDirGlobal, desiredDirGlobal);
    double dirDot = dotp(ourDirGlobal, desiredDirGlobal);
    double dirAngle = acos(max(-1.0, min(1.0, dirDot)));

    AlignLog("dirCross: %.4f, %.4f, %.4f\n", dirCross.x, dirCross.y, dirCross.z);
    AlignLog("dirDot: %.4f, dirAngle: %.2f deg\n", dirDot, dirAngle * DEG);

    // If directions are nearly opposite, pick an arbitrary perpendicular axis
    double dirCrossLen = length(dirCross);
    VECTOR3 dirAxis;
    if (dirCrossLen < 0.0001) {
        if (dirDot < 0) {
            // Nearly opposite - use our up as rotation axis
            dirAxis = ourUpGlobal;
            dirAngle = PI;
            AlignLog("Nearly opposite - using ourUpGlobal as axis\n");
        } else {
            // Nearly aligned
            dirAxis = _V(0, 1, 0);
            dirAngle = 0;
            AlignLog("Nearly aligned\n");
        }
    } else {
        dirAxis = dirCross / dirCrossLen;
    }

    AlignLog("dirAxis (global): %.4f, %.4f, %.4f\n", dirAxis.x, dirAxis.y, dirAxis.z);

    double totalError = dirAngle;
    double rollAngle = 0;
    double rollSign = 0;

    // Calculate roll error (angle between up vectors projected perpendicular to direction)
    // We need this for control even when direction isn't perfect
    {
        VECTOR3 ourUpProj = ourUpGlobal - desiredDirGlobal * dotp(ourUpGlobal, desiredDirGlobal);
        VECTOR3 tgtUpProj = tgtUpGlobal - desiredDirGlobal * dotp(tgtUpGlobal, desiredDirGlobal);
        double ourUpLen = length(ourUpProj);
        double tgtUpLen = length(tgtUpProj);

        if (ourUpLen > 0.001 && tgtUpLen > 0.001) {
            ourUpProj = ourUpProj / ourUpLen;
            tgtUpProj = tgtUpProj / tgtUpLen;
            double rollDot = dotp(ourUpProj, tgtUpProj);
            rollAngle = acos(max(-1.0, min(1.0, rollDot)));

            // Determine roll direction using cross product
            VECTOR3 rollCross = crossp(ourUpProj, tgtUpProj);
            rollSign = dotp(rollCross, desiredDirGlobal) > 0 ? 1.0 : -1.0;

            AlignLog("Roll: angle=%.2f deg, sign=%.0f\n", rollAngle * DEG, rollSign);
        }
    }

    // Total error is the larger of direction and roll errors
    totalError = max(dirAngle, rollAngle);

    // Transform rotation axis to ship coordinates for control output
    if (rotAxis) {
        // Direction correction axis in ship coords
        VECTOR3 dirAxisShip = tmul(ourRotMat, dirAxis);
        AlignLog("dirAxisShip (raw): %.4f, %.4f, %.4f\n",
                 dirAxisShip.x, dirAxisShip.y, dirAxisShip.z);

        // Start with direction correction
        *rotAxis = dirAxisShip * dirAngle;

        // Add roll correction around dock direction (Z axis in ship frame)
        // Only add significant roll correction when direction is reasonably close
        if (dirAngle < 0.35) {  // < 20 degrees - direction close enough to fix roll
            rotAxis->z += rollAngle * rollSign;
            AlignLog("Added roll correction: %.4f rad\n", rollAngle * rollSign);
        }

        AlignLog("rotAxis (combined): %.4f, %.4f, %.4f\n",
                 rotAxis->x, rotAxis->y, rotAxis->z);
    }

    AlignLog("totalError: %.2f deg\n\n", totalError * DEG);
    return totalError * DEG;
}

// Auto-align with docking port
static void DockAutoAlign(VESSEL* v, OBJHANDLE hTarget, UINT tgtDock) {
    // Open log file
    g_alignLog = fopen("align.log", "w");
    if (g_alignLog) {
        fprintf(g_alignLog, "=== Dock Auto-Align Log ===\n");
        fprintf(g_alignLog, "Target dock: %u\n\n", tgtDock);
    }

    printf("Auto-aligning with dock... Press Enter to abort.\n");
    printf("Debug log: align.log\n\n");

    const double ALIGNED_THRESHOLD = 2.0;  // degrees
    const double RATE_GAIN = 0.3;          // proportional gain (reduced)
    const double DAMP_GAIN = 2.0;          // damping gain for angular velocity
    int iteration = 0;

    while (!_kbhit()) {
        AlignLog("=== Iteration %d ===\n", iteration++);

        VECTOR3 rotAxis;
        double error = CalcDockAngularError(v, 0, hTarget, tgtDock, &rotAxis);

        // Get current angular velocity in ship coords
        VECTOR3 angVel;
        v->GetAngularVel(angVel);
        AlignLog("Angular velocity: %.4f, %.4f, %.4f rad/s\n", angVel.x, angVel.y, angVel.z);

        // Update display
        printf("\rError: %.1f deg   ", error);
        fflush(stdout);

        // Check if aligned and nearly stopped
        double angVelMag = length(angVel);
        if (error < ALIGNED_THRESHOLD && angVelMag < 0.01) {
            // Stop rotation
            v->SetAttitudeRotLevel(_V(0, 0, 0));
            AlignLog("ALIGNED! Stopping.\n");
            printf("\n\nALIGNED! Error: %.1f deg\n", error);
            break;
        }

        // Calculate control input with PD control
        // NEGATE rotAxis because cross product gives rotation FROM current TO desired,
        // but SetAttitudeRotLevel applies rotation TO the ship
        // Also add damping term to reduce angular velocity
        double px = max(-1.0, min(1.0, -rotAxis.x * RATE_GAIN - angVel.x * DAMP_GAIN));
        double py = max(-1.0, min(1.0, -rotAxis.y * RATE_GAIN - angVel.y * DAMP_GAIN));
        double pz = max(-1.0, min(1.0, -rotAxis.z * RATE_GAIN - angVel.z * DAMP_GAIN));

        AlignLog("Control output: px=%.3f, py=%.3f, pz=%.3f\n", px, py, pz);

        v->SetAttitudeRotLevel(_V(px, py, pz));

        Sleep(100);  // 10 Hz update rate
    }

    // Stop thrusters
    v->SetAttitudeRotLevel(_V(0, 0, 0));

    if (_kbhit()) {
        _getch();  // consume key
        AlignLog("Aborted by user.\n");
        printf("\n\nAborted.\n");
    }

    // Close log file
    if (g_alignLog) {
        fclose(g_alignLog);
        g_alignLog = NULL;
    }
}

const char* GetNavmodeName(int mode) {
    switch (mode) {
        case NAVMODE_KILLROT:    return "Kill Rotation";
        case NAVMODE_HLEVEL:     return "Hold Level";
        case NAVMODE_PROGRADE:   return "Prograde";
        case NAVMODE_RETROGRADE: return "Retrograde";
        case NAVMODE_NORMAL:     return "Normal";
        case NAVMODE_ANTINORMAL: return "Anti-Normal";
        case NAVMODE_HOLDALT:    return "Hold Altitude";
        default:                 return "Unknown";
    }
}

const char* GetEngineName(ENGINETYPE eng) {
    switch (eng) {
        case ENGINE_MAIN:  return "Main";
        case ENGINE_RETRO: return "Retro";
        case ENGINE_HOVER: return "Hover";
        default:           return "Unknown";
    }
}

const char* GetMFDModeName(int mode) {
    switch (mode) {
        case MFD_NONE:        return "Off";
        case MFD_ORBIT:       return "Orbit";
        case MFD_SURFACE:     return "Surface";
        case MFD_MAP:         return "Map";
        case MFD_HSI:         return "HSI";
        case MFD_LANDING:     return "Landing";
        case MFD_DOCKING:     return "Docking";
        case MFD_OPLANEALIGN: return "Align Planes";
        case MFD_OSYNC:       return "Sync Orbit";
        case MFD_TRANSFER:    return "Transfer";
        case MFD_COMMS:       return "Comms";
        default:              return (mode >= MFD_USERTYPE) ? "Custom" : "Unknown";
    }
}

void PrintCockpit() {
    int mode = oapiCockpitMode();
    const char* name;
    switch (mode) {
    case COCKPIT_GENERIC:  name = "Glass cockpit"; break;
    case COCKPIT_PANELS:   name = "2D panel"; break;
    case COCKPIT_VIRTUAL:  name = "Virtual cockpit (3D)"; break;
    default:               name = "Unknown"; break;
    }
    printf("Cockpit mode: %s\n", name);
    if (!oapiCameraInternal())
        printf("Note: External camera active\n");
}

void PrintVessel() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    printf("Vessel: %s\n", v->GetName());
    printf("Class: %s\n", v->GetClassName());

    OBJHANDLE hRef = v->GetGravityRef();
    if (hRef) {
        char name[256];
        oapiGetObjectName(hRef, name, 256);
        printf("Ref: %s\n", name);
    }
}

void PrintOrbit() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    OBJHANDLE hRef = v->GetGravityRef();
    if (!hRef) {
        printf("No reference body\n");
        return;
    }

    ELEMENTS el;
    ORBITPARAM prm;
    char buf[64];

    if (!v->GetElements(hRef, el, &prm, 0, FRAME_ECL)) {
        printf("Cannot get orbital elements\n");
        return;
    }

    double refSize = oapiGetSize(hRef);

    // Altitude
    double alt = v->GetAltitude();
    FormatDistance(alt, buf, sizeof(buf));
    printf("Alt: %s\n", buf);

    // Apoapsis
    if (el.e < 1.0) {
        FormatDistance(prm.ApD - refSize, buf, sizeof(buf));
        printf("ApA: %s\n", buf);
    } else {
        printf("ApA: N/A (escape)\n");
    }

    // Periapsis
    FormatDistance(prm.PeD - refSize, buf, sizeof(buf));
    printf("PeA: %s\n", buf);

    // Inclination
    printf("Inc: %.2f deg\n", el.i * DEG);

    // Eccentricity
    printf("Ecc: %.6f\n", el.e);

    // Period
    if (el.e < 1.0) {
        FormatTime(prm.T, buf, sizeof(buf));
        printf("Period: %s\n", buf);
    }

    // Time to Ap/Pe
    if (el.e < 1.0 && prm.ApT >= 0) {
        FormatTime(prm.ApT, buf, sizeof(buf));
        printf("T to Ap: %s\n", buf);
    }
    FormatTime(prm.PeT, buf, sizeof(buf));
    printf("T to Pe: %s\n", buf);
}

void PrintFlight() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    double speed = v->GetAirspeed();
    if (speed >= 1000)
        printf("Vel: %.2f km/s\n", speed / 1000);
    else
        printf("Vel: %.1f m/s\n", speed);

    double heading = posangle(v->GetYaw()) * DEG;
    printf("Hdg: %.1f deg\n", heading);

    printf("Pitch: %.1f deg\n", v->GetPitch() * DEG);
    printf("Bank: %.1f deg\n", v->GetBank() * DEG);
}

// Helper: Print captured MFD text entries sorted by (y, x) and grouped into lines
static void PrintMfdText(const MfdSlotData& slot) {
    if (slot.entryCount == 0) {
        printf("  (no text captured)\n");
        return;
    }

    // Sort entries by (y, x) using simple insertion sort (small N)
    MfdTextEntry sorted[64];
    int count = slot.entryCount;
    if (count > 64) count = 64;
    memcpy(sorted, slot.entries, count * sizeof(MfdTextEntry));

    for (int i = 1; i < count; i++) {
        MfdTextEntry tmp = sorted[i];
        int j = i - 1;
        while (j >= 0 && (sorted[j].y > tmp.y ||
               (sorted[j].y == tmp.y && sorted[j].x > tmp.x))) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = tmp;
    }

    // Group into lines by Y-proximity (8px tolerance)
    const int Y_TOLERANCE = 8;
    int lineY = sorted[0].y;
    bool firstOnLine = true;

    for (int i = 0; i < count; i++) {
        if (i > 0 && abs(sorted[i].y - lineY) > Y_TOLERANCE) {
            // New line
            printf("\n");
            lineY = sorted[i].y;
            firstOnLine = true;
        }

        if (!firstOnLine)
            printf("  ");  // space between fragments on same line
        printf("%s", sorted[i].text);
        firstOnLine = false;
    }
    printf("\n");
}

void PrintMFD(const char* arg) {
    // Show mode names for left/right
    printf("Left: %s\n", GetMFDModeName(oapiGetMFDMode(MFD_LEFT)));
    printf("Right: %s\n", GetMFDModeName(oapiGetMFDMode(MFD_RIGHT)));

    // No arg: just mode names plus capture status
    if (!arg || arg[0] == '\0') {
        DWORD age = MfdCaptureGetFrameAge();
        if (age == MAXDWORD)
            printf("Capture: not active\n");
        else if (age > 2000)
            printf("Capture: stale (%lu ms ago)\n", age);
        else
            printf("Capture: active (frame %lu ms ago)\n", age);
        return;
    }

    // Diagnostic subcommand
    if (_stricmp(arg, "diag") == 0) {
        MfdCaptureDiag();
        return;
    }

    // Determine which MFD slot to display
    int requestedIndex = -1;

    if (_stricmp(arg, "l") == 0 || _stricmp(arg, "left") == 0) {
        requestedIndex = MFD_LEFT;
    } else if (_stricmp(arg, "r") == 0 || _stricmp(arg, "right") == 0) {
        requestedIndex = MFD_RIGHT;
    } else {
        // Try numeric index
        char* endptr;
        long val = strtol(arg, &endptr, 10);
        if (*endptr == '\0' && val >= 0 && val < 12) {
            requestedIndex = (int)val;
        } else {
            printf("Usage: mfd [l|r|0-11]\n");
            return;
        }
    }

    // Check if the requested MFD is active
    int mode = oapiGetMFDMode(requestedIndex);
    if (mode == MFD_NONE) {
        printf("MFD %d is off\n", requestedIndex);
        return;
    }

    // Check frame age
    DWORD age = MfdCaptureGetFrameAge();
    if (age == MAXDWORD) {
        printf("No capture data available yet\n");
        return;
    }
    if (age > 2000) {
        printf("Warning: capture data is stale (%lu ms old)\n", age);
        printf("(MFDs may not render in external camera view)\n");
    }

    // Get frame data
    MfdFrameData frame;
    MfdCaptureGetFrame(&frame);

    // Find the requested MFD index in the frame
    for (int i = 0; i < frame.slotCount; i++) {
        if (frame.slots[i].mfdIndex == requestedIndex) {
            printf("\n=== MFD %d: %s ===\n", requestedIndex,
                   GetMFDModeName(frame.slots[i].modeId));
            PrintMfdText(frame.slots[i]);
            return;
        }
    }

    printf("MFD %d: %s (no captured text)\n", requestedIndex, GetMFDModeName(mode));
}

// Helper: Print dock port status list
static void PrintDockPorts(VESSEL* v) {
    // Our ports
    UINT nDocks = v->DockCount();
    printf("Our ports: %u\n", nDocks);
    if (nDocks == 0) {
        printf("  (none)\n");
    } else {
        for (UINT i = 0; i < nDocks; i++) {
            DOCKHANDLE hDock = v->GetDockHandle(i);
            OBJHANDLE hMate = v->GetDockStatus(hDock);
            if (hMate) {
                char mateName[256];
                oapiGetObjectName(hMate, mateName, 256);
                printf("  Port %u: Docked to %s\n", i, mateName);
            } else {
                printf("  Port %u: Free\n", i);
            }
        }
    }

    // Target ports (if NAV target exists)
    NAVHANDLE hNav = v->GetNavSource(0);
    if (!hNav) {
        printf("\nNo NAV target selected\n");
        return;
    }

    NAVDATA ndata;
    oapiGetNavData(hNav, &ndata);

    if (ndata.type != TRANSMITTER_IDS && ndata.type != TRANSMITTER_XPDR) {
        printf("\nNAV target is not a vessel\n");
        return;
    }

    OBJHANDLE hTarget = (ndata.type == TRANSMITTER_IDS)
        ? ndata.ids.hVessel : ndata.xpdr.hVessel;

    if (!hTarget) {
        printf("\nTarget vessel not found\n");
        return;
    }

    char tgtName[256];
    oapiGetObjectName(hTarget, tgtName, 256);
    VESSEL* tgt = oapiGetVesselInterface(hTarget);

    if (!tgt) {
        printf("\nCannot get target interface\n");
        return;
    }

    UINT tgtDocks = tgt->DockCount();
    printf("\nTarget %s ports: %u\n", tgtName, tgtDocks);

    // Which port is IDS pointing to?
    int idsDock = -1;
    if (ndata.type == TRANSMITTER_IDS && ndata.ids.hDock) {
        idsDock = tgt->GetDockIndex(ndata.ids.hDock);
    }

    for (UINT i = 0; i < tgtDocks; i++) {
        DOCKHANDLE hDock = tgt->GetDockHandle(i);
        OBJHANDLE hMate = tgt->GetDockStatus(hDock);
        const char* idsMarker = (i == (UINT)idsDock) ? " [IDS]" : "";
        if (hMate) {
            char mateName[256];
            oapiGetObjectName(hMate, mateName, 256);
            printf("  Port %u: Docked to %s%s\n", i, mateName, idsMarker);
        } else {
            printf("  Port %u: Free%s\n", i, idsMarker);
        }
    }
}

// Docking alignment data structure
struct DockAlignData {
    double dist;      // port-to-port distance
    double yaw;       // angular offset (deg), positive = target left
    double pitch;     // angular offset (deg), positive = target below
    double bank;      // roll offset (deg), positive = roll left needed
    double hDisp;     // horizontal offset (m), positive = target right
    double vDisp;     // vertical offset (m), positive = target above
    double vX;        // velocity in dock frame X (m/s), positive = drifting right
    double vY;        // velocity in dock frame Y (m/s), positive = drifting up
    double vZ;        // velocity in dock frame Z (m/s), positive = approaching
};

// Helper: Calculate alignment and velocity in dock frame
static bool CalcDockAlignment(VESSEL* v, UINT refDock, OBJHANDLE hTarget, UINT tgtDock,
                              DockAlignData* data) {
    DOCKHANDLE hOurDock = v->GetDockHandle(refDock);
    if (!hOurDock) return false;

    VESSEL* tgt = oapiGetVesselInterface(hTarget);
    if (!tgt) return false;

    DOCKHANDLE hTgtDock = tgt->GetDockHandle(tgtDock);
    if (!hTgtDock) return false;

    // Get our dock parameters (in vessel local coords)
    VECTOR3 ourPos, ourDir, ourRot;
    v->GetDockParams(hOurDock, ourPos, ourDir, ourRot);

    // Get target dock parameters
    VECTOR3 tgtPos, tgtDir, tgtRot;
    tgt->GetDockParams(hTgtDock, tgtPos, tgtDir, tgtRot);

    // Get global positions/rotations
    VECTOR3 ourGlobalPos, tgtGlobalPos;
    v->GetGlobalPos(ourGlobalPos);
    tgt->GetGlobalPos(tgtGlobalPos);

    MATRIX3 ourRotMat, tgtRotMat;
    v->GetRotationMatrix(ourRotMat);
    tgt->GetRotationMatrix(tgtRotMat);

    // Transform dock positions to global
    VECTOR3 ourDockGlobal = ourGlobalPos + mul(ourRotMat, ourPos);
    VECTOR3 tgtDockGlobal = tgtGlobalPos + mul(tgtRotMat, tgtPos);

    // Vector from our dock to target dock (global)
    VECTOR3 dockToTgt = tgtDockGlobal - ourDockGlobal;
    data->dist = length(dockToTgt);

    // Build dock approach frame: Z = approach, Y = up (rot), X = right
    VECTOR3 dockZ = ourDir;
    VECTOR3 dockY = ourRot;
    VECTOR3 dockX = crossp(dockY, dockZ);
    normalise(dockX);

    // Rotation matrix: vessel coords -> dock approach coords
    MATRIX3 dockFrame = _M(dockX.x, dockX.y, dockX.z,
                          dockY.x, dockY.y, dockY.z,
                          dockZ.x, dockZ.y, dockZ.z);

    // Position in dock frame
    // relLocal = vector from our dock to target dock, in vessel local coords
    // Just project onto dock frame axes - don't subtract ourPos (that was wrong)
    VECTOR3 relLocal = tmul(ourRotMat, dockToTgt);
    VECTOR3 relDock = mul(dockFrame, relLocal);

    // crossp(Y,Z) gives +X = RIGHT in Orbiter's left-handed system (standard formula)
    // No negations needed - the dock frame axes are correct as computed
    data->hDisp = relDock.x;   // positive = target right
    data->vDisp = relDock.y;   // positive = target above

    // Angular offset
    if (data->dist > 0.01) {
        VECTOR3 tgtDirDock = relDock;
        normalise(tgtDirDock);
        data->yaw = atan2(-tgtDirDock.x, tgtDirDock.z) * DEG;
        data->pitch = asin(tgtDirDock.y) * DEG;
    } else {
        data->yaw = 0;
        data->pitch = 0;
    }

    // Bank angle
    VECTOR3 tgtRotGlobal = mul(tgtRotMat, tgtRot);
    VECTOR3 tgtRotLocal = tmul(ourRotMat, tgtRotGlobal);
    VECTOR3 tgtRotDock = mul(dockFrame, tgtRotLocal);
    data->bank = atan2(-tgtRotDock.x, -tgtRotDock.y) * DEG;

    // Normalize bank to avoid ±180° oscillation - always use positive for ~180°
    if (data->bank < -175.0) data->bank += 360.0;  // -180 becomes +180

    // Velocity in dock frame (target's velocity relative to us)
    VECTOR3 relVel;
    v->GetRelativeVel(hTarget, relVel);
    VECTOR3 relVelLocal = tmul(ourRotMat, relVel);
    VECTOR3 relVelDock = mul(dockFrame, relVelLocal);
    // GetRelativeVel returns OUR velocity relative to target
    // Positive = we're moving in that direction relative to target
    data->vX = relVelDock.x;
    data->vY = relVelDock.y;
    data->vZ = relVelDock.z;

    // DEBUG: Log raw values to diagnose sign conventions
    static FILE* debugLog = nullptr;
    static int logCount = 0;
    if (!debugLog) {
        debugLog = fopen("velocity.log", "w");
    }
    if (debugLog && logCount < 50) {
        fprintf(debugLog, "=== Sample %d ===\n", logCount);
        fprintf(debugLog, "Position: hDisp=%.2f vDisp=%.2f dist=%.2f\n", data->hDisp, data->vDisp, data->dist);
        fprintf(debugLog, "Velocity: vX=%.3f vY=%.3f vZ=%.3f\n", data->vX, data->vY, data->vZ);
        fprintf(debugLog, "Raw relDock: x=%.2f y=%.2f z=%.2f\n", relDock.x, relDock.y, relDock.z);
        fprintf(debugLog, "Raw relVelDock: x=%.3f y=%.3f z=%.3f\n", relVelDock.x, relVelDock.y, relVelDock.z);
        fprintf(debugLog, "\n");
        fflush(debugLog);
        logCount++;
    }

    return true;
}

// Helper: Get next docking action based on priority
// Order: 1) Stop, 2) Rotate, 3) Translate, 4) Approach
static void GetNextDockAction(const DockAlignData& d, char* action, size_t actionLen,
                              char* key, size_t keyLen) {
    // Thresholds
    const double VEL_STOP = 0.3;     // must stop if above this (m/s)
    const double VEL_FINE = 0.1;     // fine velocity threshold
    const double ATT_BAD = 10.0;     // attitude needs fixing (deg)
    const double ATT_FINE = 5.0;     // fine attitude (deg) - relaxed since dock align uses different calc
    const double POS_LARGE = 2.0;    // position offset (m)
    const double POS_FINE = 0.5;     // fine position (m)

    // Calculate total lateral velocity (not counting approach axis)
    double lateralVel = sqrt(d.vX*d.vX + d.vY*d.vY);
    double totalVel = sqrt(d.vX*d.vX + d.vY*d.vY + d.vZ*d.vZ);

    // ============ PHASE 1: STOP (null velocity first) ============
    // Must stop before we can rotate or position accurately
    // Note: vX/vY/vZ are target's velocity relative to us
    // If vX > 0, target drifts right = we're moving left = need to thrust right (Num6) to stop
    // WRONG! If we're moving left, we need to thrust RIGHT to stop. Num6 thrusts right.
    // Actually: if vX > 0 (target moving right relative to us), we're moving LEFT.
    // To stop moving left, we thrust RIGHT (Num6). But wait, that's what the code said...
    // Let me reconsider: thrust RIGHT (Num6) adds rightward velocity to us.
    // If we're moving left (vX > 0 in target-relative terms), thrust right cancels it.
    // Hmm, but the user's log showed velocity building up, not canceling...
    //
    // Actually the REAL issue: vX is target's velocity relative to us.
    // If vX > 0, target is moving right relative to us = we're moving LEFT relative to target.
    // To STOP, we need to move RIGHT to match, so Num6.
    // But to CANCEL our leftward motion, we need to thrust RIGHT... that's Num6. Seems correct?
    //
    // Wait - I think the sign might be backwards. Let me reconsider from physics:
    // If I'm drifting LEFT, I have negative X velocity in my frame.
    // To stop, I thrust RIGHT (positive X), which is Num6.
    // But vX here is target's velocity relative to us, which is OPPOSITE of our velocity.
    // So if vX > 0 (target drifting right), our velocity is NEGATIVE (we're going left).
    // To stop going left, we thrust right = Num6. So vX > 0 -> Num6? That seems backwards!
    //
    // NO: to STOP we need to OPPOSE our motion. If we're going left (vX > 0 target-relative),
    // we thrust RIGHT to slow down. But Num6 adds rightward velocity...
    // To CANCEL leftward motion, thrust RIGHT = Num6. That matches.
    //
    // The problem is: the code says "STOP Vx +0.6, Num6" but Num6 adds MORE rightward velocity.
    // We want to REDUCE velocity, not add more in the same direction.
    //
    // I think the issue is: vX > 0 means target drifting right = we drifting left.
    // We want to stop drifting left = thrust right = Num6 to COUNTER our leftward drift.
    // But the current code does vX > 0 ? Num6 : Num4, which seems correct for stopping.
    //
    // Unless... the convention is different. Let me just flip the signs and see.

    // IMPORTANT: Only try to stop velocity if attitude is roughly correct!
    // Translation thrusters work in ship frame - if we're pointed 30° off,
    // "forward" thrust goes in the wrong direction and makes things worse.
    // Check attitude first - if bad, skip to rotation phase.
    double maxAttError = fmax(fabs(d.yaw), fabs(d.pitch));
    bool attitudeGoodEnough = (maxAttError < 15.0);  // Within 15 degrees

    if (attitudeGoodEnough && totalVel > VEL_STOP) {
        // Find largest velocity component and null it
        // vX/vY/vZ = target's relative velocity (target moving relative to us)
        // Since target is ~stationary, this reflects OUR motion (inverted):
        //   vX > 0 = target drifts right = WE move LEFT  -> thrust RIGHT (Num6) to stop
        //   vY > 0 = target drifts up    = WE move DOWN  -> thrust UP (Num8) to stop
        //   vZ > 0 = target drifts away  = WE move BACK  -> thrust FORWARD (Num9) to stop
        //   (vZ < 0 = approaching = thrust BACK Num3 to slow down)
        // vX/vY/vZ = OUR velocity relative to target (GetRelativeVel returns our velocity)
        // To STOP, thrust OPPOSITE to our velocity direction
        // Keys: Left=Num1, Right=Num3, Up=Num2, Down=Num8, Fwd=Num6, Back=Num9
        if (fabs(d.vX) >= fabs(d.vY) && fabs(d.vX) >= fabs(d.vZ)) {
            snprintf(action, actionLen, "STOP Vx %+.1f m/s (trans mode)", d.vX);
            snprintf(key, keyLen, d.vX > 0 ? "Num1" : "Num3");  // vX>0 = moving right, Num1 (left) to stop
        } else if (fabs(d.vY) >= fabs(d.vZ)) {
            snprintf(action, actionLen, "STOP Vy %+.1f m/s (trans mode)", d.vY);
            snprintf(key, keyLen, d.vY > 0 ? "Num8" : "Num2");  // vY>0 = moving up, Num8 (down) to stop
        } else {
            snprintf(action, actionLen, "STOP Vz %+.1f m/s (trans mode)", d.vZ);
            snprintf(key, keyLen, d.vZ > 0 ? "Num9" : "Num6");  // vZ>0 = approaching, Num9 (back) to slow
        }
        return;
    }

    // ============ PHASE 2: ROTATE (fix attitude) ============
    // NOTE: yaw/pitch here are angles TO TARGET POSITION, not orientation error.
    // After dock align, orientation is good but we may be offset from approach path.
    // Only suggest rotation if BOTH position offset is small AND angles are large,
    // which would indicate actual orientation error, not position offset.

    // If we're far from the approach path (large position offset), the yaw/pitch
    // are just telling us where the target is, not that we need to rotate.
    // Skip rotation phase and go straight to translation.
    double posOffset = sqrt(d.hDisp*d.hDisp + d.vDisp*d.vDisp);

    // Only consider rotation if position offset is small (< 3m)
    // Otherwise, translate first to get on approach path
    if (posOffset < 3.0) {
        double maxAtt = fmax(fabs(d.yaw), fmax(fabs(d.pitch), fabs(d.bank)));

        // Normalize bank - if near ±180°, it might just be a calculation artifact
        double bankErr = fabs(d.bank);
        if (bankErr > 170.0) bankErr = 180.0 - bankErr;  // Treat 178° as 2° error
        maxAtt = fmax(fabs(d.yaw), fmax(fabs(d.pitch), bankErr));

        if (maxAtt > 45.0) {
            snprintf(action, actionLen, "POINT AT DOCK: Y%+.0f P%+.0f R%+.0f",
                     d.yaw, d.pitch, d.bank);
            snprintf(key, keyLen, "use dock align");
            return;
        }

        // Rotation keys: YawL=Num1, YawR=Num3, PitchU=Num2, PitchD=Num8, BankL=Num4, BankR=Num6
        if (maxAtt > ATT_BAD) {
            if (fabs(d.yaw) > ATT_BAD) {
                snprintf(action, actionLen, "YAW %+.0f (rot mode)", d.yaw);
                snprintf(key, keyLen, d.yaw > 0 ? "Num1" : "Num3");
            } else if (fabs(d.pitch) > ATT_BAD) {
                snprintf(action, actionLen, "PITCH %+.0f (rot mode)", d.pitch);
                snprintf(key, keyLen, d.pitch > 0 ? "Num2" : "Num8");
            } else if (bankErr > ATT_BAD) {
                snprintf(action, actionLen, "ROLL %+.0f (rot mode)", d.bank);
                snprintf(key, keyLen, d.bank > 0 ? "Num4" : "Num6");
            }
            return;
        }

        if (maxAtt > ATT_FINE) {
            if (fabs(d.yaw) > ATT_FINE) {
                snprintf(action, actionLen, "Fine yaw %+.0f (rot)", d.yaw);
                snprintf(key, keyLen, d.yaw > 0 ? "Num1" : "Num3");
            } else if (fabs(d.pitch) > ATT_FINE) {
                snprintf(action, actionLen, "Fine pitch %+.0f (rot)", d.pitch);
                snprintf(key, keyLen, d.pitch > 0 ? "Num2" : "Num8");
            } else if (bankErr > ATT_FINE) {
                snprintf(action, actionLen, "Fine roll %+.0f (rot)", d.bank);
                snprintf(key, keyLen, d.bank > 0 ? "Num4" : "Num6");
            }
            return;
        }
    }

    // ============ PHASE 3: TRANSLATE (fix position) ============
    // Work on ONE axis at a time until done - don't switch back and forth!
    // Order: fix H first (until < 1m), then fix V (until < 1m)
    // Keys: Left=Num1, Right=Num3, Up=Num2, Down=Num8

    // Step 1: Fix H axis first (until |hDisp| < 1m)
    // SAME signs = moving toward zero (positive vel decreases positive pos)
    if (fabs(d.hDisp) > 1.0) {
        bool movingCorrectWay = (d.hDisp > 0 && d.vX > 0.05) || (d.hDisp < 0 && d.vX < -0.05);
        bool movingWrongWay = (d.hDisp > 0 && d.vX < -0.1) || (d.hDisp < 0 && d.vX > 0.1);

        if (movingWrongWay) {
            snprintf(action, actionLen, "STOP DRIFT H %+.1fm Vx%+.1f (trans)", d.hDisp, d.vX);
            snprintf(key, keyLen, d.vX > 0 ? "Num1" : "Num3");
        } else if (movingCorrectWay) {
            snprintf(action, actionLen, "COAST H %+.1fm Vx%+.1f (trans)", d.hDisp, d.vX);
            snprintf(key, keyLen, "wait");
        } else {
            // pos>0 needs pos vel (Num3=right), pos<0 needs neg vel (Num1=left)
            snprintf(action, actionLen, "MOVE H %+.1fm (trans mode)", d.hDisp);
            snprintf(key, keyLen, d.hDisp > 0 ? "Num3" : "Num1");
        }
        return;
    }

    // Step 2: Then fix V axis (until |vDisp| < 1m)
    // SAME signs = moving toward zero (positive vel decreases positive pos)
    if (fabs(d.vDisp) > 1.0) {
        bool movingCorrectWay = (d.vDisp > 0 && d.vY > 0.05) || (d.vDisp < 0 && d.vY < -0.05);
        bool movingWrongWay = (d.vDisp > 0 && d.vY < -0.1) || (d.vDisp < 0 && d.vY > 0.1);

        if (movingWrongWay) {
            snprintf(action, actionLen, "STOP DRIFT V %+.1fm Vy%+.1f (trans)", d.vDisp, d.vY);
            snprintf(key, keyLen, d.vY > 0 ? "Num8" : "Num2");
        } else if (movingCorrectWay) {
            snprintf(action, actionLen, "COAST V %+.1fm Vy%+.1f (trans)", d.vDisp, d.vY);
            snprintf(key, keyLen, "wait");
        } else {
            // pos>0 needs pos vel (Num2=up), pos<0 needs neg vel (Num8=down)
            snprintf(action, actionLen, "MOVE V %+.1fm (trans mode)", d.vDisp);
            snprintf(key, keyLen, d.vDisp > 0 ? "Num2" : "Num8");
        }
        return;
    }

    // Fine position - same logic but tighter thresholds
    // SAME signs = moving toward zero (positive vel decreases positive pos)
    // Keys: Left=Num1, Right=Num3, Up=Num2, Down=Num8
    if (fabs(d.hDisp) > POS_FINE || fabs(d.vDisp) > POS_FINE) {
        if (fabs(d.hDisp) >= fabs(d.vDisp)) {
            bool movingCorrectWay = (d.hDisp > 0 && d.vX > 0.02) || (d.hDisp < 0 && d.vX < -0.02);
            if (movingCorrectWay) {
                snprintf(action, actionLen, "COAST H %+.2fm (trans)", d.hDisp);
                snprintf(key, keyLen, "wait");
            } else {
                snprintf(action, actionLen, "Fine H %+.2fm (trans)", d.hDisp);
                snprintf(key, keyLen, d.hDisp > 0 ? "Num3" : "Num1");
            }
        } else {
            bool movingCorrectWay = (d.vDisp > 0 && d.vY > 0.02) || (d.vDisp < 0 && d.vY < -0.02);
            if (movingCorrectWay) {
                snprintf(action, actionLen, "COAST V %+.2fm (trans)", d.vDisp);
                snprintf(key, keyLen, "wait");
            } else {
                snprintf(action, actionLen, "Fine V %+.2fm (trans)", d.vDisp);
                snprintf(key, keyLen, d.vDisp > 0 ? "Num2" : "Num8");
            }
        }
        return;
    }

    // Null any remaining lateral velocity before approach
    // Keys: Left=Num1, Right=Num3, Up=Num2, Down=Num8
    if (lateralVel > VEL_FINE) {
        if (fabs(d.vX) >= fabs(d.vY)) {
            snprintf(action, actionLen, "Null Vx %+.2f (trans)", d.vX);
            snprintf(key, keyLen, d.vX > 0 ? "Num1" : "Num3");  // Oppose velocity to stop
        } else {
            snprintf(action, actionLen, "Null Vy %+.2f (trans)", d.vY);
            snprintf(key, keyLen, d.vY > 0 ? "Num8" : "Num2");  // Oppose velocity to stop
        }
        return;
    }

    // ============ PHASE 4: APPROACH ============
    // Keys: Forward=Num6, Back=Num9
    // vZ > 0 = approaching (moving forward), vZ < 0 = separating
    if (d.dist > 10.0) {
        double targetVz = 1.0;  // approach at 1 m/s (positive = forward)
        double vzError = d.vZ - targetVz;
        if (fabs(vzError) > 0.3) {
            snprintf(action, actionLen, "APPROACH %.0fm (trans)", d.dist);
            snprintf(key, keyLen, vzError > 0 ? "Num9" : "Num6");  // too fast->back, too slow->fwd
            return;
        }
    } else if (d.dist > 2.0) {
        double targetVz = 0.3;  // slow to 0.3 m/s
        double vzError = d.vZ - targetVz;
        if (fabs(vzError) > 0.1) {
            snprintf(action, actionLen, "SLOW %.1fm (trans)", d.dist);
            snprintf(key, keyLen, vzError > 0 ? "Num9" : "Num6");
            return;
        }
    } else {
        double targetVz = 0.1;  // creep at 0.1 m/s
        double vzError = d.vZ - targetVz;
        if (fabs(vzError) > 0.05) {
            snprintf(action, actionLen, "CREEP %.2fm (trans)", d.dist);
            snprintf(key, keyLen, vzError > 0 ? "Num9" : "Num6");
            return;
        }
    }

    // All aligned - just hold
    snprintf(action, actionLen, "HOLD closing %.2f m/s", -d.vZ);
    snprintf(key, keyLen, "---");
}

// Full auto-docking: align, translate, and approach
static void DockAutoApproach(VESSEL* v, OBJHANDLE hTarget, UINT tgtDock) {
    printf("Auto-docking... Press Enter to abort.\n\n");

    const double ALIGN_THRESHOLD = 10.0;   // degrees - acceptable alignment for approach
    const double POS_THRESHOLD = 0.5;      // meters - acceptable lateral position
    const double DOCK_DIST = 0.3;          // meters - docking trigger distance

    // Control gains
    const double ROT_GAIN = 0.3;
    const double ROT_DAMP = 2.0;
    const double TRANS_GAIN = 0.5;         // translation proportional gain
    const double TRANS_DAMP = 3.0;         // translation damping
    const double APPROACH_SPEED = 0.5;     // m/s approach speed
    const double CREEP_SPEED = 0.1;        // m/s final approach

    while (!_kbhit()) {
        // Get alignment data for position/velocity
        DockAlignData d;
        if (!CalcDockAlignment(v, 0, hTarget, tgtDock, &d)) {
            printf("\rCannot calculate alignment   ");
            Sleep(100);
            continue;
        }

        // Get rotation axis for attitude control (same as dock align)
        VECTOR3 rotAxis;
        double attError = CalcDockAngularError(v, 0, hTarget, tgtDock, &rotAxis);

        // Get angular velocity for damping
        VECTOR3 angVel;
        v->GetAngularVel(angVel);

        // Status line
        printf("\rDist:%.1fm H:%+.1f V:%+.1f Att:%.0f   ",
               d.dist, d.hDisp, d.vDisp, attError);
        fflush(stdout);

        // Check if docked
        if (d.dist < DOCK_DIST) {
            v->SetAttitudeRotLevel(_V(0, 0, 0));
            v->SetAttitudeLinLevel(_V(0, 0, 0));
            printf("\n\nDOCKED!\n");
            break;
        }

        // === ROTATION CONTROL ===
        // Use same approach as DockAutoAlign - rotAxis from CalcDockAngularError
        double rx = fmax(-1.0, fmin(1.0, -rotAxis.x * ROT_GAIN - angVel.x * ROT_DAMP));
        double ry = fmax(-1.0, fmin(1.0, -rotAxis.y * ROT_GAIN - angVel.y * ROT_DAMP));
        double rz = fmax(-1.0, fmin(1.0, -rotAxis.z * ROT_GAIN - angVel.z * ROT_DAMP));

        v->SetAttitudeRotLevel(_V(rx, ry, rz));

        // === TRANSLATION CONTROL ===
        // Only translate if reasonably aligned
        double tx = 0, ty = 0, tz = 0;

        if (attError < 30.0) {  // Only translate if within 30 degrees
            // Lateral translation: drive position errors to zero
            // Based on empirical data: positive velocity decreases positive position
            // So: hDisp > 0 needs positive vX, which needs positive tx
            double targetVx = d.hDisp * TRANS_GAIN;
            double targetVy = d.vDisp * TRANS_GAIN;

            // Limit target velocity
            targetVx = fmax(-0.5, fmin(0.5, targetVx));
            targetVy = fmax(-0.5, fmin(0.5, targetVy));

            // PD control: drive actual velocity toward target velocity
            tx = (targetVx - d.vX) * TRANS_DAMP;
            ty = (targetVy - d.vY) * TRANS_DAMP;

            // Approach: only if well-aligned and centered
            if (attError < ALIGN_THRESHOLD &&
                fabs(d.hDisp) < POS_THRESHOLD * 3 &&
                fabs(d.vDisp) < POS_THRESHOLD * 3) {

                double targetVz = (d.dist > 5.0) ? APPROACH_SPEED : CREEP_SPEED;
                tz = (targetVz - d.vZ) * TRANS_DAMP;
            } else {
                // Not aligned - hold position or back off if too close
                double targetVz = (d.dist < 3.0) ? -0.1 : 0.0;
                tz = (targetVz - d.vZ) * TRANS_DAMP;
            }
        }

        tx = fmax(-1.0, fmin(1.0, tx));
        ty = fmax(-1.0, fmin(1.0, ty));
        tz = fmax(-1.0, fmin(1.0, tz));

        v->SetAttitudeLinLevel(_V(tx, ty, tz));

        Sleep(100);  // 10 Hz update
    }

    // Stop all thrusters
    v->SetAttitudeRotLevel(_V(0, 0, 0));
    v->SetAttitudeLinLevel(_V(0, 0, 0));

    if (_kbhit()) {
        _getch();
        printf("\n\nAborted.\n");
    }
}

void PrintDock(const char* arg) {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    // "ports" subcommand - list all docking ports
    if (arg && _stricmp(arg, "ports") == 0) {
        PrintDockPorts(v);
        return;
    }

    // "watch" subcommand - continuous clipboard updates
    if (arg && _stricmp(arg, "watch") == 0) {
        printf("Watching dock alignment - copies to clipboard every 1.5s\n");
        printf("Press Enter to stop...\n\n");

        char lastClip[128] = "";

        while (!_kbhit()) {
            v = oapiGetFocusInterface();
            if (!v) {
                CopyToClipboard("No vessel");
                Sleep(1500);
                continue;
            }

            UINT nDocks = v->DockCount();
            NAVHANDLE hNav = v->GetNavSource(0);

            if (!hNav || nDocks == 0) {
                CopyToClipboard("No target or no dock");
                Sleep(1500);
                continue;
            }

            NAVDATA ndata;
            oapiGetNavData(hNav, &ndata);

            if (ndata.type != TRANSMITTER_IDS && ndata.type != TRANSMITTER_XPDR) {
                CopyToClipboard("Not a docking target");
                Sleep(1500);
                continue;
            }

            OBJHANDLE hTarget = (ndata.type == TRANSMITTER_IDS)
                ? ndata.ids.hVessel : ndata.xpdr.hVessel;

            if (!hTarget) {
                CopyToClipboard("Target not found");
                Sleep(1500);
                continue;
            }

            UINT tgtDock = 0;
            VESSEL* tgtVessel = oapiGetVesselInterface(hTarget);
            if (ndata.type == TRANSMITTER_IDS && tgtVessel && ndata.ids.hDock) {
                int idx = tgtVessel->GetDockIndex(ndata.ids.hDock);
                if (idx >= 0) tgtDock = (UINT)idx;
            }

            UINT tgtDocks = tgtVessel ? tgtVessel->DockCount() : 0;

            if (tgtDocks == 0) {
                CopyToClipboard("Target has no docks");
                Sleep(1500);
                continue;
            }

            DockAlignData align;
            if (!CalcDockAlignment(v, 0, hTarget, tgtDock, &align)) {
                CopyToClipboard("Cannot calc alignment");
                Sleep(1500);
                continue;
            }

            // Get action
            char action[64], key[16], clipText[128];
            GetNextDockAction(align, action, sizeof(action), key, sizeof(key));

            // Format: distance, relative speed, action, key
            double relSpeed = sqrt(align.vX*align.vX + align.vY*align.vY + align.vZ*align.vZ);
            snprintf(clipText, sizeof(clipText), "%.0fm %.1fm/s: %s, %s",
                     align.dist, relSpeed, action, key);

            // Only update clipboard if changed (reduces chatter)
            if (strcmp(clipText, lastClip) != 0) {
                CopyToClipboard(clipText);
                strcpy(lastClip, clipText);
                printf("%s\n", clipText);  // Also print for reference
            }

            Sleep(1500);
        }

        // Consume the key press
        _getch();
        printf("Watch stopped.\n");
        return;
    }

    // "align" subcommand - auto-align with target dock
    if (arg && _stricmp(arg, "align") == 0) {
        UINT nDocks = v->DockCount();
        if (nDocks == 0) {
            printf("No docking ports on this vessel\n");
            return;
        }

        NAVHANDLE hNav = v->GetNavSource(0);
        if (!hNav) {
            printf("No NAV target - tune to a docking port first\n");
            return;
        }

        NAVDATA ndata;
        oapiGetNavData(hNav, &ndata);

        if (ndata.type != TRANSMITTER_IDS && ndata.type != TRANSMITTER_XPDR) {
            printf("NAV target is not a docking transmitter\n");
            return;
        }

        OBJHANDLE hTarget = (ndata.type == TRANSMITTER_IDS)
            ? ndata.ids.hVessel : ndata.xpdr.hVessel;

        if (!hTarget) {
            printf("Target vessel not found\n");
            return;
        }

        VESSEL* tgtVessel = oapiGetVesselInterface(hTarget);
        if (!tgtVessel || tgtVessel->DockCount() == 0) {
            printf("Target has no docking ports\n");
            return;
        }

        // Get target dock port from IDS
        UINT tgtDock = 0;
        if (ndata.type == TRANSMITTER_IDS && ndata.ids.hDock) {
            int idx = tgtVessel->GetDockIndex(ndata.ids.hDock);
            if (idx >= 0) tgtDock = (UINT)idx;
        }

        char tgtName[256];
        oapiGetObjectName(hTarget, tgtName, 256);
        printf("Aligning with %s port %u...\n", tgtName, tgtDock);

        DockAutoAlign(v, hTarget, tgtDock);
        return;
    }

    // "auto" or "auto <n>" subcommand - full auto-docking (align + translate + approach)
    if (arg && (_stricmp(arg, "auto") == 0 || strncmp(arg, "auto ", 5) == 0)) {
        UINT nDocks = v->DockCount();
        if (nDocks == 0) {
            printf("No docking ports on this vessel\n");
            return;
        }

        NAVHANDLE hNav = v->GetNavSource(0);
        if (!hNav) {
            printf("No NAV target - tune to a docking port first\n");
            return;
        }

        NAVDATA ndata;
        oapiGetNavData(hNav, &ndata);

        if (ndata.type != TRANSMITTER_IDS && ndata.type != TRANSMITTER_XPDR) {
            printf("NAV target is not a docking transmitter\n");
            return;
        }

        OBJHANDLE hTarget = (ndata.type == TRANSMITTER_IDS)
            ? ndata.ids.hVessel : ndata.xpdr.hVessel;

        if (!hTarget) {
            printf("Target vessel not found\n");
            return;
        }

        VESSEL* tgtVessel = oapiGetVesselInterface(hTarget);
        if (!tgtVessel || tgtVessel->DockCount() == 0) {
            printf("Target has no docking ports\n");
            return;
        }

        // Parse optional target port number: "auto <n>"
        UINT tgtDock = 0;
        int specifiedPort = -1;
        if (strlen(arg) > 5) {
            specifiedPort = atoi(arg + 5);
            if (specifiedPort >= 0 && specifiedPort < (int)tgtVessel->DockCount()) {
                tgtDock = (UINT)specifiedPort;
            } else {
                printf("Invalid target port %d (target has %u ports)\n",
                       specifiedPort, tgtVessel->DockCount());
                return;
            }
        } else if (ndata.type == TRANSMITTER_IDS && ndata.ids.hDock) {
            // Use IDS port if no port specified
            int idx = tgtVessel->GetDockIndex(ndata.ids.hDock);
            if (idx >= 0) tgtDock = (UINT)idx;
        }

        char tgtName[256];
        oapiGetObjectName(hTarget, tgtName, 256);
        printf("Auto-docking with %s port %u...\n", tgtName, tgtDock);

        DockAutoApproach(v, hTarget, tgtDock);
        return;
    }

    // Check if arg specifies a reference dock port number
    int refDock = 0;  // default to port 0
    UINT nDocks = v->DockCount();

    if (arg && arg[0] != '\0') {
        char* endptr;
        long portNum = strtol(arg, &endptr, 10);
        if (*endptr == '\0' && portNum >= 0 && portNum < (long)nDocks) {
            refDock = (int)portNum;
        } else if (*endptr != '\0' && _stricmp(arg, "watch") != 0 &&
                   _stricmp(arg, "align") != 0 && _stricmp(arg, "auto") != 0 &&
                   strncmp(arg, "auto ", 5) != 0) {
            printf("Unknown dock command: %s\n", arg);
            printf("Options: ports, watch, align, auto [n]\n");
            return;
        }
    }

    // Show our dock ports summary
    if (nDocks > 0) {
        printf("Our ports: %u", nDocks);
        if (nDocks > 1) {
            printf(" (using port %d)", refDock);
        }
        // Check if our reference port is occupied
        DOCKHANDLE hOurDock = v->GetDockHandle(refDock);
        OBJHANDLE hOurMate = v->GetDockStatus(hOurDock);
        if (hOurMate) {
            char mateName[256];
            oapiGetObjectName(hOurMate, mateName, 256);
            printf(" [DOCKED to %s]", mateName);
        }
        printf("\n");
    }

    // Get NAV source for target
    NAVHANDLE hNav = v->GetNavSource(0);
    if (!hNav) {
        printf("NAV: No target\n");
        return;
    }

    char descr[128];
    oapiGetNavDescr(hNav, descr, 128);
    printf("NAV: %s\n", descr);

    NAVDATA ndata;
    oapiGetNavData(hNav, &ndata);

    if (ndata.type != TRANSMITTER_IDS && ndata.type != TRANSMITTER_XPDR) {
        printf("(Not a docking target)\n");
        return;
    }

    OBJHANDLE hTarget = (ndata.type == TRANSMITTER_IDS)
        ? ndata.ids.hVessel : ndata.xpdr.hVessel;

    if (!hTarget) {
        printf("Target vessel not found\n");
        return;
    }

    // Get target dock port from IDS
    UINT tgtDock = 0;
    VESSEL* tgtVessel = oapiGetVesselInterface(hTarget);
    if (ndata.type == TRANSMITTER_IDS && tgtVessel && ndata.ids.hDock) {
        int idx = tgtVessel->GetDockIndex(ndata.ids.hDock);
        if (idx >= 0) tgtDock = (UINT)idx;
    }

    char tgtName[256];
    oapiGetObjectName(hTarget, tgtName, 256);
    UINT tgtDocks = tgtVessel ? tgtVessel->DockCount() : 0;

    if (tgtDocks > 1) {
        printf("Target: %s port %u\n", tgtName, tgtDock);
    }

    // Basic distance and velocity
    VECTOR3 relPos, relVel;
    v->GetRelativePos(hTarget, relPos);
    v->GetRelativeVel(hTarget, relVel);

    double dist = length(relPos);
    char buf[64];
    FormatDistance(dist, buf, sizeof(buf));
    printf("Dist: %s\n", buf);

    // Closure rate (positive = approaching)
    double cvel = -dotp(relVel, relPos) / dist;
    printf("CRate: %.2f m/s\n", cvel);

    // Calculate alignment data if we have docking ports
    if (nDocks > 0 && tgtDocks > 0) {
        DockAlignData align;

        if (CalcDockAlignment(v, refDock, hTarget, tgtDock, &align)) {
            // Port-to-port distance
            FormatDistance(align.dist, buf, sizeof(buf));
            printf("Port Dist: %s\n", buf);

            // Closure rate in dock frame
            printf("Closing: %.2f m/s\n", -align.vZ);

            // Show attitude if it's off - helps user understand what needs fixing
            double maxAtt = fmax(fabs(align.yaw), fmax(fabs(align.pitch), fabs(align.bank)));
            if (maxAtt > 10.0) {
                printf("\nAttitude (rotation mode, Num/ to toggle):\n");
                printf("  Yaw: %+.0f deg\n", align.yaw);
                printf("  Pitch: %+.0f deg\n", align.pitch);
                printf("  Roll: %+.0f deg\n", align.bank);
            }

            // Get next action
            char action[64], key[16];
            GetNextDockAction(align, action, sizeof(action), key, sizeof(key));

            printf("\nAction: %s\n", action);
            printf("Key: %s\n", key);
        }
    } else {
        // No dock ports - just show basic velocity info
        MATRIX3 rot;
        v->GetRotationMatrix(rot);
        VECTOR3 lv = tmul(rot, relVel);
        printf("Vx: %+.2f  Vy: %+.2f  Vz: %+.2f m/s\n", lv.x, lv.y, lv.z);
    }
}

// Thruster group flags for tank classification
enum TankFlags {
    TANK_MAIN  = 0x01,
    TANK_RETRO = 0x02,
    TANK_HOVER = 0x04,
    TANK_RCS   = 0x08
};

// Check if any thruster in a standard group uses this propellant handle
static bool GroupUsesTank(VESSEL* v, THGROUP_TYPE grp, PROPELLANT_HANDLE ph) {
    DWORD n = v->GetGroupThrusterCount(grp);
    for (DWORD i = 0; i < n; i++) {
        THRUSTER_HANDLE th = v->GetGroupThruster(grp, i);
        if (th && v->GetThrusterResource(th) == ph) return true;
    }
    return false;
}

// Classify a propellant tank by scanning which thruster groups use it
static int ClassifyTank(VESSEL* v, PROPELLANT_HANDLE ph) {
    int flags = 0;
    if (GroupUsesTank(v, THGROUP_MAIN, ph))  flags |= TANK_MAIN;
    if (GroupUsesTank(v, THGROUP_RETRO, ph)) flags |= TANK_RETRO;
    if (GroupUsesTank(v, THGROUP_HOVER, ph)) flags |= TANK_HOVER;

    // Check any RCS attitude group
    static const THGROUP_TYPE rcsGroups[] = {
        THGROUP_ATT_PITCHUP, THGROUP_ATT_PITCHDOWN,
        THGROUP_ATT_YAWLEFT, THGROUP_ATT_YAWRIGHT,
        THGROUP_ATT_BANKLEFT, THGROUP_ATT_BANKRIGHT,
        THGROUP_ATT_UP, THGROUP_ATT_DOWN,
        THGROUP_ATT_LEFT, THGROUP_ATT_RIGHT,
        THGROUP_ATT_FORWARD, THGROUP_ATT_BACK
    };
    for (int i = 0; i < 12; i++) {
        if (GroupUsesTank(v, rcsGroups[i], ph)) {
            flags |= TANK_RCS;
            break;
        }
    }
    return flags;
}

// Build a human-readable tank name from classification flags
static void BuildTankName(int flags, int index, char* buf, int len) {
    if (flags == 0) {
        snprintf(buf, len, "Tank %d", index);
        return;
    }
    buf[0] = '\0';
    if (flags & TANK_MAIN)  strncat(buf, "Main", len - strlen(buf) - 1);
    if (flags & TANK_RETRO) {
        if (buf[0]) strncat(buf, "/", len - strlen(buf) - 1);
        strncat(buf, "Retro", len - strlen(buf) - 1);
    }
    if (flags & TANK_HOVER) {
        if (buf[0]) strncat(buf, "/", len - strlen(buf) - 1);
        strncat(buf, "Hover", len - strlen(buf) - 1);
    }
    if (flags & TANK_RCS) {
        if (buf[0]) strncat(buf, "/", len - strlen(buf) - 1);
        strncat(buf, "RCS", len - strlen(buf) - 1);
    }
}

void PrintFuel(const char* arg) {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    DWORD nTanks = v->GetPropellantCount();
    if (nTanks == 0) {
        printf("No fuel tanks\n");
        return;
    }

    double vesselMass = v->GetMass();

    // Check if a specific tank index was requested
    int tankIdx = -1;
    if (arg && arg[0] != '\0') {
        char* endptr;
        long val = strtol(arg, &endptr, 10);
        if (*endptr == '\0' && val >= 0 && val < (long)nTanks) {
            tankIdx = (int)val;
        } else {
            printf("Invalid tank index: %s (vessel has %u tanks)\n", arg, nTanks);
            return;
        }
    }

    // Detailed single-tank view
    if (tankIdx >= 0) {
        PROPELLANT_HANDLE ph = v->GetPropellantHandleByIndex(tankIdx);
        double mass = v->GetPropellantMass(ph);
        double maxMass = v->GetPropellantMaxMass(ph);
        double flow = v->GetPropellantFlowrate(ph);
        double pct = (maxMass > 0) ? (mass / maxMass) * 100.0 : 0.0;

        int flags = ClassifyTank(v, ph);
        char name[64];
        BuildTankName(flags, tankIdx, name, sizeof(name));

        // Find best vacuum ISP from rocket thrusters using this tank
        // Skip air-breathing engines (e.g. scramjets) which have near-zero vacuum ISP
        double bestIsp = 0;
        DWORD nTh = v->GetThrusterCount();
        for (DWORD i = 0; i < nTh; i++) {
            THRUSTER_HANDLE th = v->GetThrusterHandleByIndex(i);
            if (v->GetThrusterResource(th) == ph) {
                double isp = v->GetThrusterIsp0(th);
                if (isp > 10.0 && isp > bestIsp) bestIsp = isp;
            }
        }

        printf("=== %s (Tank %d) ===\n", name, tankIdx);
        printf("Mass: %.1f / %.1f kg\n", mass, maxMass);
        printf("Level: %.1f%%\n", pct);

        // Delta-V
        if (bestIsp > 0 && vesselMass > mass) {
            double dv = bestIsp * log(vesselMass / (vesselMass - mass));
            printf("Delta-V: %.0f m/s\n", dv);
        } else {
            printf("Delta-V: N/A\n");
        }

        // ISP
        if (bestIsp > 0)
            printf("ISP: %.0f m/s (vacuum)\n", bestIsp);
        else
            printf("ISP: N/A\n");

        // Flow rate
        if (fabs(flow) > 0.001) {
            printf("Flow: %.2f kg/s\n", fabs(flow));
            // Time to empty
            if (mass > 0 && fabs(flow) > 0.001) {
                double tte = mass / fabs(flow);
                char timeBuf[64];
                FormatTime(tte, timeBuf, sizeof(timeBuf));
                printf("Time to empty: %s\n", timeBuf);
            }
        } else {
            printf("Flow: idle\n");
        }
        return;
    }

    // Summary view: all tanks
    double totalMass = 0, totalMax = 0;

    for (DWORD i = 0; i < nTanks; i++) {
        PROPELLANT_HANDLE ph = v->GetPropellantHandleByIndex(i);
        double mass = v->GetPropellantMass(ph);
        double maxMass = v->GetPropellantMaxMass(ph);
        double flow = v->GetPropellantFlowrate(ph);
        double pct = (maxMass > 0) ? (mass / maxMass) * 100.0 : 0.0;

        int flags = ClassifyTank(v, ph);
        char name[64];
        BuildTankName(flags, i, name, sizeof(name));

        totalMass += mass;
        totalMax += maxMass;

        // Find best vacuum ISP for delta-V (skip air-breathing engines)
        double bestIsp = 0;
        DWORD nTh = v->GetThrusterCount();
        for (DWORD j = 0; j < nTh; j++) {
            THRUSTER_HANDLE th = v->GetThrusterHandleByIndex(j);
            if (v->GetThrusterResource(th) == ph) {
                double isp = v->GetThrusterIsp0(th);
                if (isp > 10.0 && isp > bestIsp) bestIsp = isp;
            }
        }

        // Build output line
        printf("%s: %.0f/%.0f kg (%.0f%%)", name, mass, maxMass, pct);

        if (bestIsp > 0 && vesselMass > mass) {
            double dv = bestIsp * log(vesselMass / (vesselMass - mass));
            printf(" dV=%.0f m/s", dv);
        }

        if (fabs(flow) > 0.001)
            printf(" [%.2f kg/s]", fabs(flow));

        printf("\n");
    }

    // Total line if multiple tanks
    if (nTanks > 1) {
        double totalPct = (totalMax > 0) ? (totalMass / totalMax) * 100.0 : 0.0;
        printf("Total: %.0f/%.0f kg (%.0f%%)\n", totalMass, totalMax, totalPct);
    }
}

void PrintMap(const char* arg) {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    OBJHANDLE hRef = v->GetSurfaceRef();
    if (!hRef) {
        printf("No surface reference\n");
        return;
    }

    char refName[256];
    oapiGetObjectName(hRef, refName, 256);

    double lng, lat, rad;
    v->GetEquPos(lng, lat, rad);

    double refRadius = oapiGetSize(hRef);

    // Check for "bases" subcommand
    if (arg && _stricmp(arg, "bases") == 0) {
        DWORD baseCount = oapiGetBaseCount(hRef);
        if (baseCount == 0) {
            printf("No bases on %s\n", refName);
            return;
        }

        printf("Bases on %s:\n", refName);

        for (DWORD i = 0; i < baseCount; i++) {
            OBJHANDLE hBase = oapiGetBaseByIndex(hRef, i);
            if (!hBase) continue;

            char baseName[256];
            oapiGetObjectName(hBase, baseName, 256);

            double baseLng, baseLat;
            oapiGetBaseEquPos(hBase, &baseLng, &baseLat);

            double dist = CalcDistance(lat, lng, baseLat, baseLng, refRadius);
            double bearing = CalcBearing(lat, lng, baseLat, baseLng);

            char distBuf[64];
            FormatDistance(dist, distBuf, sizeof(distBuf));

            printf("  %s: %s, bearing %.0f%c\n", baseName, distBuf, bearing, 0xB0);
        }
        return;
    }

    // Default: show current position
    printf("Ref: %s\n", refName);

    char posBuf[64];
    FormatLatLon(lat, lng, posBuf, sizeof(posBuf));
    printf("Position: %s\n", posBuf);

    double alt = v->GetAltitude();
    char altBuf[64];
    FormatDistance(alt, altBuf, sizeof(altBuf));
    printf("Altitude: %s\n", altBuf);

    // Ground track - heading and speed
    VECTOR3 groundVel;
    v->GetGroundspeedVector(FRAME_HORIZON, groundVel);
    double groundSpeed = sqrt(groundVel.x * groundVel.x + groundVel.z * groundVel.z);
    double trackHeading = posangle(atan2(groundVel.x, groundVel.z)) * DEG;

    if (groundSpeed >= 1000)
        printf("Ground Track: %.0f%c at %.2f km/s\n", trackHeading, 0xB0, groundSpeed / 1000);
    else
        printf("Ground Track: %.0f%c at %.1f m/s\n", trackHeading, 0xB0, groundSpeed);
}

// Helper: Print altitude data
static void PrintSurfaceAltitude(VESSEL* v, bool detailed) {
    char buf[64];

    double alt = v->GetAltitude();
    FormatDistance(alt, buf, sizeof(buf));
    printf(detailed ? "Altitude: %s\n" : "Alt: %s\n", buf);

    if (detailed) {
        double altGround = v->GetAltitude(ALTMODE_GROUND);
        FormatDistance(altGround, buf, sizeof(buf));
        printf("Radar Alt: %s\n", buf);
    }

    VECTOR3 groundVel;
    v->GetGroundspeedVector(FRAME_HORIZON, groundVel);
    double vspd = groundVel.y;
    if (detailed)
        printf("Vertical Speed: %.1f m/s\n", vspd);
    else if (fabs(vspd) >= 100)
        printf("VS: %.0f m/s\n", vspd);
    else
        printf("VS: %.1f m/s\n", vspd);
}

// Helper: Print speed data
static void PrintSurfaceSpeed(VESSEL* v, bool detailed) {
    char buf[64];

    double airspeed = v->GetAirspeed();
    FormatSpeed(airspeed, buf, sizeof(buf));
    printf(detailed ? "True Airspeed: %s\n" : "TAS: %s\n", buf);

    double groundspeed = v->GetGroundspeed();
    FormatSpeed(groundspeed, buf, sizeof(buf));
    printf(detailed ? "Ground Speed: %s\n" : "GS: %s\n", buf);
}

// Helper: Print attitude data
static void PrintSurfaceAttitude(VESSEL* v, bool detailed) {
    double heading = posangle(v->GetYaw()) * DEG;
    printf(detailed ? "Heading: %.1f deg\n" : "Hdg: %.1f deg\n", heading);
    printf("Pitch: %.1f deg\n", v->GetPitch() * DEG);
    printf("Bank: %.1f deg\n", v->GetBank() * DEG);
    printf("AOA: %.1f deg\n", v->GetAOA() * DEG);
    printf("Slip: %.1f deg\n", v->GetSlipAngle() * DEG);
}

// Helper: Print atmosphere data
static void PrintSurfaceAtmosphere(VESSEL* v, bool detailed) {
    char buf[64];

    double mach = v->GetMachNumber();
    printf("Mach: %.2f\n", mach);

    double temp = v->GetAtmTemperature();
    printf(detailed ? "Temperature: %.1f K (%.1f C)\n" : "OAT: %.1f K (%.1f C)\n",
           temp, temp - 273.15);

    double pressure = v->GetAtmPressure();
    FormatPressure(pressure, buf, sizeof(buf));
    printf(detailed ? "Static Pressure: %s\n" : "Pressure: %s\n", buf);

    if (detailed) {
        double density = v->GetAtmDensity();
        printf("Density: %.6f kg/m3\n", density);
    }

    double dynP = v->GetDynPressure();
    FormatPressure(dynP, buf, sizeof(buf));
    printf(detailed ? "Dynamic Pressure: %s\n" : "Dyn Press: %s\n", buf);
}

// Helper: Print aerodynamic forces
static void PrintSurfaceForces(VESSEL* v, bool detailed) {
    char buf[64];

    double lift = v->GetLift();
    FormatForce(lift, buf, sizeof(buf));
    printf("Lift: %s\n", buf);

    double drag = v->GetDrag();
    FormatForce(drag, buf, sizeof(buf));
    printf("Drag: %s\n", buf);

    if (fabs(drag) > 1e-6) {
        printf(detailed ? "L/D Ratio: %.2f\n" : "L/D: %.2f\n", lift / drag);
    }

    if (detailed) {
        printf("\nAngle of Attack: %.1f deg\n", v->GetAOA() * DEG);
        printf("Slip Angle: %.1f deg\n", v->GetSlipAngle() * DEG);
    }
}

void PrintSurface(const char* arg) {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    // Detect atmosphere
    bool hasAtmosphere = (v->GetAtmPressure() > 0);

    // No argument: show summary
    if (!arg || arg[0] == '\0') {
        PrintSurfaceAltitude(v, false);
        PrintSurfaceSpeed(v, false);
        PrintSurfaceAttitude(v, false);
        if (hasAtmosphere) {
            PrintSurfaceAtmosphere(v, false);
        }
        PrintSurfaceForces(v, false);
        return;
    }

    // Subcommand: alt
    if (_stricmp(arg, "alt") == 0) {
        printf("=== Altitude ===\n");
        PrintSurfaceAltitude(v, true);
        printf("\n=== Speed ===\n");
        PrintSurfaceSpeed(v, true);
        printf("\n=== Attitude ===\n");
        PrintSurfaceAttitude(v, true);
        return;
    }

    // Subcommand: atm
    if (_stricmp(arg, "atm") == 0) {
        if (!hasAtmosphere) {
            printf("No atmosphere\n");
            return;
        }
        printf("=== Atmosphere ===\n");
        PrintSurfaceAtmosphere(v, true);
        return;
    }

    // Subcommand: forces
    if (_stricmp(arg, "forces") == 0) {
        printf("=== Aerodynamic Forces ===\n");
        PrintSurfaceForces(v, true);
        return;
    }

    // Unknown subcommand
    printf("Unknown surface command: %s\n", arg);
    printf("Options: alt, atm, forces\n");
}

// Helper: Parse MFD side argument to an MFD index
// Returns -1 on error
static int ParseMfdSide(const char* arg) {
    if (!arg || arg[0] == '\0') return -1;

    if (_stricmp(arg, "l") == 0 || _stricmp(arg, "left") == 0)
        return MFD_LEFT;
    if (_stricmp(arg, "r") == 0 || _stricmp(arg, "right") == 0)
        return MFD_RIGHT;

    char* endptr;
    long val = strtol(arg, &endptr, 10);
    if (*endptr == '\0' && val >= 0 && val < 12)
        return (int)val;

    return -1;
}

void PrintButtons(const char* arg) {
    int mfdIndex = ParseMfdSide(arg);
    if (mfdIndex < 0) {
        printf("Usage: buttons <l|r|0-11>\n");
        return;
    }

    int mode = oapiGetMFDMode(mfdIndex);
    if (mode == MFD_NONE) {
        printf("MFD %d is off\n", mfdIndex);
        return;
    }

    printf("MFD %d (%s) buttons:\n", mfdIndex, GetMFDModeName(mode));

    // Left column (0-5)
    for (int bt = 0; bt < 6; bt++) {
        const char* label = oapiMFDButtonLabel(mfdIndex, bt);
        if (label)
            printf("  [%d] %s", bt, label);
    }
    printf("\n");

    // Right column (6-11)
    for (int bt = 6; bt < 12; bt++) {
        const char* label = oapiMFDButtonLabel(mfdIndex, bt);
        if (label)
            printf("  [%d] %s", bt, label);
    }
    printf("\n");
}

void PressButton(const char* arg) {
    if (!arg || arg[0] == '\0') {
        printf("Usage: press <l|r|0-11> <button#>\n");
        return;
    }

    // Parse first token as MFD side
    char sideStr[16] = "";
    const char* rest = arg;
    int i = 0;
    while (*rest && *rest != ' ' && i < 15) {
        sideStr[i++] = *rest++;
    }
    sideStr[i] = '\0';
    while (*rest == ' ') rest++;

    int mfdIndex = ParseMfdSide(sideStr);
    if (mfdIndex < 0) {
        printf("Usage: press <l|r|0-11> <button#>\n");
        return;
    }

    // Parse second token as button number
    if (*rest == '\0') {
        printf("Usage: press <l|r|0-11> <button#>\n");
        return;
    }

    char* endptr;
    long bt = strtol(rest, &endptr, 10);
    if (*endptr != '\0' || bt < 0 || bt > 11) {
        printf("Invalid button number: %s (0-11)\n", rest);
        return;
    }

    int mode = oapiGetMFDMode(mfdIndex);
    if (mode == MFD_NONE) {
        printf("MFD %d is off\n", mfdIndex);
        return;
    }

    const char* label = oapiMFDButtonLabel(mfdIndex, (int)bt);
    bool ok = oapiProcessMFDButton(mfdIndex, (int)bt, PANEL_MOUSE_LBDOWN);

    if (ok)
        printf("Pressed [%ld] %s on MFD %d\n", bt, label ? label : "(?)", mfdIndex);
    else
        printf("Button [%ld] not handled by MFD %d\n", bt, mfdIndex);

    // Show updated labels
    PrintButtons(sideStr);
}

void PrintAll() {
    printf("=== VESSEL ===\n");
    PrintVessel();
    printf("=== ORBIT ===\n");
    PrintOrbit();
    printf("=== FLIGHT ===\n");
    PrintFlight();
    printf("=== MFD ===\n");
    PrintMFD("");
    printf("=== FUEL ===\n");
    PrintFuel("");
    printf("=== DOCK ===\n");
    PrintDock("");
}
