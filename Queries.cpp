// Queries.cpp - Data query functions for vessel, orbit, flight, etc.

#include "Queries.h"
#include "Formatting.h"
#include "MfdCapture.h"
#ifdef HAS_XRVESSELCTRL
#include "XRVesselCtrl.h"
#include "Controls.h"
#endif
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
        // Scale roll authority: full when direction is close, reduced when far off
        // (at large direction errors, prioritize pointing before roll)
        double rollScale = 1.0;
        if (dirAngle > 0.35) {  // > 20 degrees
            rollScale = 0.35 / dirAngle;  // Taper smoothly
        }
        if (rollAngle > 0.01) {
            rotAxis->z += rollAngle * rollSign * rollScale;
            AlignLog("Added roll correction: %.4f rad (scale %.2f)\n",
                     rollAngle * rollSign * rollScale, rollScale);
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

        // Get angular velocity relative to target (in our ship coords)
        VECTOR3 angVel, tgtAngVel;
        v->GetAngularVel(angVel);
        VESSEL* tgtV = oapiGetVesselInterface(hTarget);
        tgtV->GetAngularVel(tgtAngVel);

        MATRIX3 ourRotMat, tgtRotMat;
        v->GetRotationMatrix(ourRotMat);
        tgtV->GetRotationMatrix(tgtRotMat);

        // Convert target angular velocity: target local -> global -> our local
        VECTOR3 tgtAngVelGlobal = mul(tgtRotMat, tgtAngVel);
        VECTOR3 tgtAngVelOurs = tmul(ourRotMat, tgtAngVelGlobal);
        VECTOR3 relAngVel = _V(angVel.x - tgtAngVelOurs.x,
                               angVel.y - tgtAngVelOurs.y,
                               angVel.z - tgtAngVelOurs.z);

        AlignLog("Angular velocity: %.4f, %.4f, %.4f rad/s\n", angVel.x, angVel.y, angVel.z);
        AlignLog("Relative angular velocity: %.4f, %.4f, %.4f rad/s\n",
                 relAngVel.x, relAngVel.y, relAngVel.z);

        // Update display
        printf("\rError: %.1f deg   ", error);
        fflush(stdout);

        // Check if aligned and relative rotation nearly zero
        double relAngVelMag = length(relAngVel);
        if (error < ALIGNED_THRESHOLD && relAngVelMag < 0.01) {
            // Stop rotation
            v->SetAttitudeRotLevel(_V(0, 0, 0));
            AlignLog("ALIGNED! Stopping.\n");
            printf("\n\nALIGNED! Error: %.1f deg\n", error);
            break;
        }

        // Deactivate navmodes every tick — they can re-engage and override RCS
        v->DeactivateNavmode(NAVMODE_KILLROT);
        v->DeactivateNavmode(NAVMODE_PROGRADE);
        v->DeactivateNavmode(NAVMODE_RETROGRADE);
        v->DeactivateNavmode(NAVMODE_HLEVEL);
        v->DeactivateNavmode(NAVMODE_NORMAL);
        v->DeactivateNavmode(NAVMODE_ANTINORMAL);

        // Calculate control input with PD control
        // NEGATE rotAxis because cross product gives rotation FROM current TO desired,
        // but SetAttitudeRotLevel applies rotation TO the ship
        // Also add damping term to reduce angular velocity
        double px = max(-1.0, min(1.0, -rotAxis.x * RATE_GAIN - relAngVel.x * DAMP_GAIN));
        double py = max(-1.0, min(1.0, -rotAxis.y * RATE_GAIN - relAngVel.y * DAMP_GAIN));
        double pz = max(-1.0, min(1.0, -rotAxis.z * RATE_GAIN - relAngVel.z * DAMP_GAIN));

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

#ifdef HAS_XRVESSELCTRL
    {
        XRVesselCtrl* xr = GetXRVessel(true);
        if (xr) {
            XRSystemStatusRead st = {};
            xr->GetXRSystemStatus(st);

            printf("Cabin O2: %.1f%%\n", st.CabinO2Level * 100.0);

            if (st.CenterOfGravity != 0.0)
                printf("COG: %.3f m%s\n", st.CenterOfGravity,
                       st.COGAutoMode ? " (auto)" : "");

            if (st.MasterWarning == XRWarningState::XRW_warningActive)
                printf("*** MASTER WARNING ACTIVE ***\n");
            if (st.InternalSystemsFailure)
                printf("*** INTERNAL SYSTEMS FAILURE ***\n");
        }
    }
#endif
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

    // Velocity: dock-to-dock relative velocity (accounts for target rotation)
    // GetRelativeVel returns our COM velocity minus target COM velocity (global)
    VECTOR3 relVel;
    v->GetRelativeVel(hTarget, relVel);

    // Add rotational velocity of each dock port.
    // In Orbiter's left-handed frame, tangential velocity is v = -crossp(ω, r)
    VECTOR3 ourAngVel, tgtAngVel;
    v->GetAngularVel(ourAngVel);
    tgt->GetAngularVel(tgtAngVel);

    VECTOR3 ourAngVelGlobal = mul(ourRotMat, ourAngVel);
    VECTOR3 tgtAngVelGlobal = mul(tgtRotMat, tgtAngVel);

    // Dock offsets in global frame (already computed: ourPos/tgtPos are local)
    VECTOR3 ourDockOffset = mul(ourRotMat, ourPos);
    VECTOR3 tgtDockOffset = mul(tgtRotMat, tgtPos);

    // Dock-to-dock relative velocity = COM relative vel + our dock rot vel - target dock rot vel
    VECTOR3 dockRelVel = relVel - crossp(ourAngVelGlobal, ourDockOffset)
                                + crossp(tgtAngVelGlobal, tgtDockOffset);

    VECTOR3 relVelLocal = tmul(ourRotMat, dockRelVel);
    VECTOR3 relVelDock = mul(dockFrame, relVelLocal);
    // Positive = we're moving in that direction relative to target dock
    data->vX = relVelDock.x;
    data->vY = relVelDock.y;
    data->vZ = relVelDock.z;

    return true;
}

// Helper: Get next docking action based on priority
// Phases: 1) Stop, 2) Align, 3) Translate, 4) Approach, 5) Hold
// attError = total angular error in degrees from CalcDockAngularError
// rotAxis = rotation error in ship coords (x=pitch, y=yaw, z=roll) in radians
static void GetNextDockAction(const DockAlignData& d, double attError,
                              const VECTOR3& rotAxis,
                              char* action, size_t actionLen,
                              char* key, size_t keyLen) {
    // Velocity convention: vX/vY/vZ are OUR velocity relative to target in dock frame
    // vX > 0 = drifting right, vY > 0 = drifting up, vZ > 0 = approaching
    // To stop: thrust opposite (vX>0 -> Num1 left, vY>0 -> Num8 down, vZ>0 -> Num9 back)
    double totalVel = sqrt(d.vX*d.vX + d.vY*d.vY + d.vZ*d.vZ);

    // ============ PHASE 1: STOP (velocity too high for distance) ============
    double velLimit = fmin(2.0, fmax(0.3, d.dist * 0.003));

    if (totalVel > velLimit) {
        // Report dominant velocity axis
        if (fabs(d.vX) >= fabs(d.vY) && fabs(d.vX) >= fabs(d.vZ)) {
            snprintf(action, actionLen, "STOP: %.1fm/s, drift %s",
                     fabs(d.vX), d.vX > 0 ? "right" : "left");
            snprintf(key, keyLen, d.vX > 0 ? "Num1" : "Num3");
        } else if (fabs(d.vY) >= fabs(d.vZ)) {
            snprintf(action, actionLen, "STOP: %.1fm/s, drift %s",
                     fabs(d.vY), d.vY > 0 ? "up" : "down");
            snprintf(key, keyLen, d.vY > 0 ? "Num8" : "Num2");
        } else {
            snprintf(action, actionLen, "STOP: %.1fm/s %s",
                     fabs(d.vZ), d.vZ > 0 ? "too fast" : "backing away");
            snprintf(key, keyLen, d.vZ > 0 ? "Num9" : "Num6");
        }
        return;
    }

    // ============ PHASE 2: ALIGN (attitude error too large) ============
    // rotAxis components are in radians: x=pitch error, y=yaw error, z=roll error
    double yawErr = fabs(rotAxis.y) * DEG;
    double pitchErr = fabs(rotAxis.x) * DEG;
    double rollErr = fabs(rotAxis.z) * DEG;

    if (attError > 10.0) {
        if (attError > 45.0) {
            snprintf(action, actionLen, "ALIGN: off by %.0f%c, recommend dock align",
                     attError, 0xB0);
            snprintf(key, keyLen, "---");
        } else if (rollErr > yawErr && rollErr > pitchErr) {
            snprintf(action, actionLen, "ROLL %s %.0f%c",
                     rotAxis.z > 0 ? "LEFT" : "RIGHT", rollErr, 0xB0);
            snprintf(key, keyLen, rotAxis.z > 0 ? "Num7" : "Num9");
        } else if (yawErr >= pitchErr) {
            snprintf(action, actionLen, "ALIGN: yaw %s %.0f%c",
                     rotAxis.y > 0 ? "left" : "right", yawErr, 0xB0);
            snprintf(key, keyLen, rotAxis.y > 0 ? "Num1" : "Num3");
        } else {
            snprintf(action, actionLen, "ALIGN: pitch %s %.0f%c",
                     rotAxis.x > 0 ? "down" : "up", pitchErr, 0xB0);
            snprintf(key, keyLen, rotAxis.x > 0 ? "Num2" : "Num8");
        }
        return;
    }

    if (attError > 5.0) {
        if (rollErr > yawErr && rollErr > pitchErr) {
            snprintf(action, actionLen, "FINE: roll %s %.0f%c",
                     rotAxis.z > 0 ? "left" : "right", rollErr, 0xB0);
            snprintf(key, keyLen, rotAxis.z > 0 ? "Num7" : "Num9");
        } else if (yawErr >= pitchErr) {
            snprintf(action, actionLen, "FINE: yaw %s %.0f%c",
                     rotAxis.y > 0 ? "left" : "right", yawErr, 0xB0);
            snprintf(key, keyLen, rotAxis.y > 0 ? "Num1" : "Num3");
        } else {
            snprintf(action, actionLen, "FINE: pitch %s %.0f%c",
                     rotAxis.x > 0 ? "down" : "up", pitchErr, 0xB0);
            snprintf(key, keyLen, rotAxis.x > 0 ? "Num2" : "Num8");
        }
        return;
    }

    // ============ PHASE 3: TRANSLATE (lateral offset > 0.5m) ============
    // Work on larger offset axis first. Null cross-axis velocity before switching.
    double hOff = fabs(d.hDisp);
    double vOff = fabs(d.vDisp);
    bool hNeedsWork = hOff > 0.5;
    bool vNeedsWork = vOff > 0.5;

    if (hNeedsWork || vNeedsWork) {
        // Pick axis with larger offset
        bool doH = hNeedsWork && (!vNeedsWork || hOff >= vOff);

        if (doH) {
            // Null cross-axis velocity before lateral work
            if (vNeedsWork && fabs(d.vY) > 0.15) {
                snprintf(action, actionLen, "STOP DRIFT %s %.1fm/s",
                         d.vY > 0 ? "up" : "down", fabs(d.vY));
                snprintf(key, keyLen, d.vY > 0 ? "Num8" : "Num2");
                return;
            }

            // Distance-scaled target speed
            double targetSpd = (hOff > 50.0) ? 1.0 : (hOff > 10.0) ? 0.5 : (hOff > 2.0) ? 0.2 : 0.1;
            // vel > 0 means moving in correct direction to reduce offset
            double vel = (d.hDisp > 0) ? d.vX : -d.vX;
            double brakeDist = vel * vel / 0.4;

            if (vel < -0.1) {
                snprintf(action, actionLen, "TRANSLATE %s %.1fm",
                         d.hDisp > 0 ? "RIGHT" : "LEFT", hOff);
                snprintf(key, keyLen, d.hDisp > 0 ? "Num3" : "Num1");
            } else if (vel > 0.05 && (brakeDist > hOff * 0.8 || vel > targetSpd * 1.3)) {
                snprintf(action, actionLen, "BRAKE %s %.1fm",
                         d.hDisp > 0 ? "right" : "left", hOff);
                snprintf(key, keyLen, d.hDisp > 0 ? "Num1" : "Num3");
            } else if (vel > targetSpd * 0.7) {
                snprintf(action, actionLen, "COAST %s %.1fm",
                         d.hDisp > 0 ? "right" : "left", hOff);
                snprintf(key, keyLen, "---");
            } else {
                snprintf(action, actionLen, "TRANSLATE %s %.1fm",
                         d.hDisp > 0 ? "RIGHT" : "LEFT", hOff);
                snprintf(key, keyLen, d.hDisp > 0 ? "Num3" : "Num1");
            }
            return;
        } else {
            // Null cross-axis velocity before lateral work
            if (hNeedsWork && fabs(d.vX) > 0.15) {
                snprintf(action, actionLen, "STOP DRIFT %s %.1fm/s",
                         d.vX > 0 ? "right" : "left", fabs(d.vX));
                snprintf(key, keyLen, d.vX > 0 ? "Num1" : "Num3");
                return;
            }

            double targetSpd = (vOff > 50.0) ? 1.0 : (vOff > 10.0) ? 0.5 : (vOff > 2.0) ? 0.2 : 0.1;
            double vel = (d.vDisp > 0) ? d.vY : -d.vY;
            double brakeDist = vel * vel / 0.4;

            if (vel < -0.1) {
                snprintf(action, actionLen, "TRANSLATE %s %.1fm",
                         d.vDisp > 0 ? "UP" : "DOWN", vOff);
                snprintf(key, keyLen, d.vDisp > 0 ? "Num2" : "Num8");
            } else if (vel > 0.05 && (brakeDist > vOff * 0.8 || vel > targetSpd * 1.3)) {
                snprintf(action, actionLen, "BRAKE %s %.1fm",
                         d.vDisp > 0 ? "up" : "down", vOff);
                snprintf(key, keyLen, d.vDisp > 0 ? "Num8" : "Num2");
            } else if (vel > targetSpd * 0.7) {
                snprintf(action, actionLen, "COAST %s %.1fm",
                         d.vDisp > 0 ? "up" : "down", vOff);
                snprintf(key, keyLen, "---");
            } else {
                snprintf(action, actionLen, "TRANSLATE %s %.1fm",
                         d.vDisp > 0 ? "UP" : "DOWN", vOff);
                snprintf(key, keyLen, d.vDisp > 0 ? "Num2" : "Num8");
            }
            return;
        }
    }

    // ============ PHASE 4: APPROACH (closing distance) ============
    // Speed tiers by distance
    double targetVz;
    if (d.dist > 100.0)     targetVz = 2.0;
    else if (d.dist > 50.0) targetVz = 1.0;
    else if (d.dist > 10.0) targetVz = 0.5;
    else if (d.dist > 5.0)  targetVz = 0.3;
    else                     targetVz = 0.1;

    double vzError = d.vZ - targetVz;

    if (d.dist < 2.0 && d.vZ > 0.05) {
        snprintf(action, actionLen, "CLOSING (hold steady)");
        snprintf(key, keyLen, "---");
        return;
    }

    if (fabs(vzError) > 0.15) {
        if (vzError > 0) {
            snprintf(action, actionLen, "BRAKE: %.1fm/s, target %.1fm/s", d.vZ, targetVz);
            snprintf(key, keyLen, "Num9");
        } else {
            snprintf(action, actionLen, "APPROACH: %.1fm/s, target %.1fm/s", d.vZ, targetVz);
            snprintf(key, keyLen, "Num6");
        }
        return;
    }

    if (d.vZ > 0.05) {
        snprintf(action, actionLen, "COAST (closing)");
        snprintf(key, keyLen, "---");
        return;
    }

    // ============ PHASE 5: HOLD (everything good) ============
    if (d.vZ > 0.02) {
        snprintf(action, actionLen, "HOLD: closing %.1fm/s", d.vZ);
    } else {
        snprintf(action, actionLen, "HOLD: approach, Num6");
        snprintf(key, keyLen, "Num6");
        return;
    }
    snprintf(key, keyLen, "---");
}

// Helper: Compute relative angular velocity in our ship coords
static VECTOR3 CalcRelAngVel(VESSEL* v, OBJHANDLE hTarget) {
    VECTOR3 angVel, tgtAngVel;
    v->GetAngularVel(angVel);
    VESSEL* tgt = oapiGetVesselInterface(hTarget);
    tgt->GetAngularVel(tgtAngVel);

    MATRIX3 ourRotMat, tgtRotMat;
    v->GetRotationMatrix(ourRotMat);
    tgt->GetRotationMatrix(tgtRotMat);

    VECTOR3 tgtAngVelGlobal = mul(tgtRotMat, tgtAngVel);
    VECTOR3 tgtAngVelOurs = tmul(ourRotMat, tgtAngVelGlobal);
    return _V(angVel.x - tgtAngVelOurs.x,
              angVel.y - tgtAngVelOurs.y,
              angVel.z - tgtAngVelOurs.z);
}

// Full auto-docking: sequential stages with precondition checks
static void DockAutoApproach(VESSEL* v, OBJHANDLE hTarget, UINT tgtDock) {
    // === PRECONDITION CHECKS ===
    VECTOR3 angVel;
    v->GetAngularVel(angVel);
    double angVelMag = length(angVel);
    if (angVelMag > 0.1) {
        printf("You're tumbling (%.1f deg/s). Run kilrot first.\n", angVelMag * DEG);
        return;
    }

    DockAlignData d;
    if (!CalcDockAlignment(v, 0, hTarget, tgtDock, &d)) {
        printf("Cannot calculate dock alignment.\n");
        return;
    }

    VECTOR3 rotAxis;
    double attError = CalcDockAngularError(v, 0, hTarget, tgtDock, &rotAxis);
    if (d.dist < 2.0 && attError > 30.0) {
        printf("Too close (%.1fm) and misaligned (%.0f deg). Back off first.\n",
               d.dist, attError);
        return;
    }

    printf("Auto-docking... Press Enter to abort.\n\n");

    // === STAGE 1: ALIGN ===
    printf("Stage 1: Aligning...\n");
    DockAutoAlign(v, hTarget, tgtDock);

    // Check if user aborted during align
    if (_kbhit()) {
        _getch();
        return;
    }

    // Verify alignment succeeded
    attError = CalcDockAngularError(v, 0, hTarget, tgtDock, &rotAxis);
    VECTOR3 relAngVel = CalcRelAngVel(v, hTarget);
    if (attError > 5.0 || length(relAngVel) > 0.01) {
        printf("Alignment incomplete (%.1f deg). Aborting.\n", attError);
        return;
    }

    // === STAGE 2: APPROACH ===
    printf("\nStage 2: Approaching...\n");

    const double ROT_GAIN = 1.0;
    const double ROT_DAMP = 3.0;
    const double TRANS_GAIN = 0.5;
    const double TRANS_DAMP = 3.0;
    const double DOCK_DIST = 0.3;

    while (!_kbhit()) {
        if (!CalcDockAlignment(v, 0, hTarget, tgtDock, &d)) {
            printf("\rCannot calculate alignment   ");
            Sleep(100);
            continue;
        }

        attError = CalcDockAngularError(v, 0, hTarget, tgtDock, &rotAxis);
        relAngVel = CalcRelAngVel(v, hTarget);

        printf("\rDist:%.1fm H:%+.1f V:%+.1f Att:%.0f   ",
               d.dist, d.hDisp, d.vDisp, attError);
        fflush(stdout);

        // Done?
        if (d.dist < DOCK_DIST) {
            v->SetAttitudeRotLevel(_V(0, 0, 0));
            v->SetAttitudeLinLevel(_V(0, 0, 0));
            printf("\n\nDOCKED!\n");
            break;
        }

        // Safety: if attitude error exceeds 30°, kill all translation
        if (attError > 30.0) {
            v->SetAttitudeLinLevel(_V(0, 0, 0));
            printf("\n\nAttitude error %.0f deg - stopping translation.\n", attError);
            printf("Abort and restart, or run dock align first.\n");
            // Keep running rotation control to try to recover
        }

        // === ROTATION: PD control, deactivate navmodes every tick ===
        v->DeactivateNavmode(NAVMODE_KILLROT);
        v->DeactivateNavmode(NAVMODE_PROGRADE);
        v->DeactivateNavmode(NAVMODE_RETROGRADE);
        v->DeactivateNavmode(NAVMODE_HLEVEL);
        v->DeactivateNavmode(NAVMODE_NORMAL);
        v->DeactivateNavmode(NAVMODE_ANTINORMAL);

        double rx = fmax(-1.0, fmin(1.0, -rotAxis.x * ROT_GAIN - relAngVel.x * ROT_DAMP));
        double ry = fmax(-1.0, fmin(1.0, -rotAxis.y * ROT_GAIN - relAngVel.y * ROT_DAMP));
        double rz = fmax(-1.0, fmin(1.0, -rotAxis.z * ROT_GAIN - relAngVel.z * ROT_DAMP));
        v->SetAttitudeRotLevel(_V(rx, ry, rz));

        // === TRANSLATION: PD control on all axes simultaneously ===
        if (attError < 30.0) {
            // Lateral: drive hDisp/vDisp to zero with speed cap scaled by offset
            double targetVx = d.hDisp * TRANS_GAIN;
            double targetVy = d.vDisp * TRANS_GAIN;
            double lateralOff = fmax(fabs(d.hDisp), fabs(d.vDisp));
            double latCap = (lateralOff > 50.0) ? 1.5 : (lateralOff > 10.0) ? 1.0 : 0.5;
            targetVx = fmax(-latCap, fmin(latCap, targetVx));
            targetVy = fmax(-latCap, fmin(latCap, targetVy));

            double tx = fmax(-1.0, fmin(1.0, (targetVx - d.vX) * TRANS_DAMP));
            double ty = fmax(-1.0, fmin(1.0, (targetVy - d.vY) * TRANS_DAMP));

            // Forward: speed tiered by distance
            double targetVz;
            if (d.dist > 200.0)      targetVz = 2.0;
            else if (d.dist > 50.0)  targetVz = 1.0;
            else if (d.dist > 10.0)  targetVz = 0.5;
            else if (d.dist > 5.0)   targetVz = 0.3;
            else                      targetVz = 0.1;

            double tz = fmax(-1.0, fmin(1.0, (targetVz - d.vZ) * TRANS_DAMP));
            v->SetAttitudeLinLevel(_V(tx, ty, tz));
        }

        Sleep(100);
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

            // Get attitude error for guidance
            VECTOR3 rotAxis;
            double attError = CalcDockAngularError(v, 0, hTarget, tgtDock, &rotAxis);

            // Get action
            char action[64], key[16], clipText[128];
            GetNextDockAction(align, attError, rotAxis,
                              action, sizeof(action), key, sizeof(key));

            // Format: distance, closing speed, action, key
            snprintf(clipText, sizeof(clipText), "%.0fm %+.1fm/s: %s, %s",
                     align.dist, align.vZ, action, key);

            // Only update clipboard if changed (reduces chatter)
            if (strcmp(clipText, lastClip) != 0) {
                CopyToClipboard(clipText);
                strcpy(lastClip, clipText);
                printf("%s\n", clipText);
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

    // Header: target and port info
    const char* idsTag = (ndata.type == TRANSMITTER_IDS) ? " [IDS]" : "";
    printf("Dock: %s port %u%s\n", tgtName, tgtDock, idsTag);

    // Calculate alignment data if we have docking ports
    if (nDocks > 0 && tgtDocks > 0) {
        DockAlignData align;

        if (CalcDockAlignment(v, refDock, hTarget, tgtDock, &align)) {
            // Distance and closing speed
            char buf[64];
            FormatDistance(align.dist, buf, sizeof(buf));
            printf("Distance: %s, closing %.1f m/s\n", buf, align.vZ);

            // Lateral offset
            printf("Offset: H%+.1fm V%+.1fm\n", align.hDisp, align.vDisp);

            // Attitude error from angular error calculation
            VECTOR3 rotAxis;
            double attError = CalcDockAngularError(v, 0, hTarget, tgtDock, &rotAxis);
            double yawErr = fabs(rotAxis.y) * DEG;
            double pitchErr = fabs(rotAxis.x) * DEG;
            double rollErr = fabs(rotAxis.z) * DEG;
            printf("Attitude: %.0f%c off (yaw %.0f%c pitch %.0f%c roll %.0f%c)\n",
                   attError, 0xB0, yawErr, 0xB0, pitchErr, 0xB0, rollErr, 0xB0);

            // Next action
            char action[64], key[16];
            GetNextDockAction(align, attError, rotAxis,
                              action, sizeof(action), key, sizeof(key));
            printf("Next: %s, %s\n", action, key);
        }
    } else {
        // No dock ports - just show basic distance/velocity
        VECTOR3 relPos, relVel;
        v->GetRelativePos(hTarget, relPos);
        v->GetRelativeVel(hTarget, relVel);

        double dist = length(relPos);
        char buf[64];
        FormatDistance(dist, buf, sizeof(buf));
        printf("Dist: %s\n", buf);

        double cvel = -dotp(relVel, relPos) / dist;
        printf("CRate: %.2f m/s\n", cvel);

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

#ifdef HAS_XRVESSELCTRL
    // Show XR-specific supply data if available
    XRVesselCtrl* xr = GetXRVessel(true);
    if (xr) {
        XRSystemStatusRead status;
        xr->GetXRSystemStatus(status);

        printf("\n=== XR Supply Status ===\n");
        printf("APU Fuel: %.1f%% (%.0f/%.0f kg)\n",
            status.APUFuelLevel * 100, status.APUFuelLevel * status.APUMaxFuelMass, status.APUMaxFuelMass);
        printf("LOX:      %.1f%% (%.0f/%.0f kg)\n",
            status.LOXLevel * 100, status.LOXLevel * status.LOXMaxMass, status.LOXMaxMass);

        printf("Fuel Hatch: %s\n", DoorStateStr(xr->GetFuelHatchState()));
        printf("LOX Hatch:  %s\n", DoorStateStr(xr->GetLoxHatchState()));
        printf("Supply Lines:\n");
        PrintSupplyLineStatus(xr, XRSupplyLineID::XRS_MainFuel);
        PrintSupplyLineStatus(xr, XRSupplyLineID::XRS_ScramFuel);
        PrintSupplyLineStatus(xr, XRSupplyLineID::XRS_ApuFuel);
        PrintSupplyLineStatus(xr, XRSupplyLineID::XRS_Lox);
    }
#endif
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
        printf("Usage: press <l|r|0-11> <button#> [<seconds>s]\n");
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
        printf("Usage: press <l|r|0-11> <button#> [<seconds>s]\n");
        return;
    }

    // Parse second token as button number
    if (*rest == '\0') {
        printf("Usage: press <l|r|0-11> <button#> [<seconds>s]\n");
        return;
    }

    char* endptr;
    long bt = strtol(rest, &endptr, 10);
    if (bt < 0 || bt > 11) {
        printf("Invalid button number (0-11)\n");
        return;
    }

    // Skip spaces after button number
    while (*endptr == ' ') endptr++;

    // Parse optional hold duration
    double holdSeconds = 0.0;
    if (*endptr != '\0') {
        char* durEnd;
        holdSeconds = strtod(endptr, &durEnd);
        if (*durEnd == 's' || *durEnd == 'S') durEnd++;
        while (*durEnd == ' ') durEnd++;
        if (*durEnd != '\0' || holdSeconds <= 0.0) {
            printf("Invalid hold duration: %s\n", endptr);
            return;
        }
        if (holdSeconds > 30.0) {
            printf("Hold duration must be 30 seconds or less\n");
            return;
        }
    }

    int mode = oapiGetMFDMode(mfdIndex);
    if (mode == MFD_NONE) {
        printf("MFD %d is off\n", mfdIndex);
        return;
    }

    const char* label = oapiMFDButtonLabel(mfdIndex, (int)bt);

    if (holdSeconds > 0.0) {
        // Hold mode: LBDOWN, then LBPRESSED stream, then LBUP
        bool ok = oapiProcessMFDButton(mfdIndex, (int)bt, PANEL_MOUSE_LBDOWN);
        if (!ok) {
            printf("Button [%ld] not handled by MFD %d\n", bt, mfdIndex);
            return;
        }

        printf("Holding [%ld] %s on MFD %d for %.1fs...\n",
               bt, label ? label : "(?)", mfdIndex, holdSeconds);

        DWORD startTick = GetTickCount();
        DWORD holdMs = (DWORD)(holdSeconds * 1000.0);
        DWORD lastReportTick = startTick;

        while ((GetTickCount() - startTick) < holdMs) {
            Sleep(50);
            oapiProcessMFDButton(mfdIndex, (int)bt, PANEL_MOUSE_LBPRESSED);

            // Print progress every second
            DWORD now = GetTickCount();
            if ((now - lastReportTick) >= 1000) {
                double elapsed = (now - startTick) / 1000.0;
                printf("  %.0fs...\n", elapsed);
                lastReportTick = now;
            }
        }

        oapiProcessMFDButton(mfdIndex, (int)bt, PANEL_MOUSE_LBUP);
        printf("Released.\n");

        // Show MFD text so user can see resulting value
        PrintMFD(sideStr);
    } else {
        // Single press mode (existing behavior)
        bool ok = oapiProcessMFDButton(mfdIndex, (int)bt, PANEL_MOUSE_LBDOWN);

        if (ok)
            printf("Pressed [%ld] %s on MFD %d\n", bt, label ? label : "(?)", mfdIndex);
        else
            printf("Button [%ld] not handled by MFD %d\n", bt, mfdIndex);

        // Show updated labels
        PrintButtons(sideStr);
    }
}

// =========================================================================
// XR Vessel Damage Report
// =========================================================================

void PrintDamage(const char* arg) {
#ifdef HAS_XRVESSELCTRL
    XRVesselCtrl* xr = GetXRVessel();
    if (!xr) return;

    XRSystemStatusRead st = {};
    xr->GetXRSystemStatus(st);

    bool fullMode = (arg && ((_stricmp(arg, "all") == 0) || (_stricmp(arg, "full") == 0)));

    // Helpers for printing
    struct DmgDouble { const char* name; double value; };
    struct DmgState { const char* name; XRDamageState value; };

    DmgDouble doubles[] = {
        {"Left Wing",           st.LeftWing},
        {"Right Wing",          st.RightWing},
        {"Left Main Engine",    st.LeftMainEngine},
        {"Right Main Engine",   st.RightMainEngine},
        {"Left SCRAM Engine",   st.LeftSCRAMEngine},
        {"Right SCRAM Engine",  st.RightSCRAMEngine},
        {"Fore Hover Engine",   st.ForeHoverEngine},
        {"Aft Hover Engine",    st.AftHoverEngine},
        {"Left Retro Engine",   st.LeftRetroEngine},
        {"Right Retro Engine",  st.RightRetroEngine},
        {"Fwd Lower RCS",      st.ForwardLowerRCS},
        {"Aft Upper RCS",       st.AftUpperRCS},
        {"Fwd Upper RCS",       st.ForwardUpperRCS},
        {"Aft Lower RCS",       st.AftLowerRCS},
        {"Fwd Stbd RCS",        st.ForwardStarboardRCS},
        {"Aft Port RCS",        st.AftPortRCS},
        {"Fwd Port RCS",        st.ForwardPortRCS},
        {"Aft Stbd RCS",        st.AftStarboardRCS},
        {"Outbd Upper Port RCS",     st.OutboardUpperPortRCS},
        {"Outbd Lower Stbd RCS",     st.OutboardLowerStarboardRCS},
        {"Outbd Upper Stbd RCS",     st.OutboardUpperStarboardRCS},
        {"Outbd Lower Port RCS",     st.OutboardLowerPortRCS},
        {"Aft RCS",             st.AftRCS},
        {"Forward RCS",         st.ForwardRCS},
    };
    int numDoubles = sizeof(doubles) / sizeof(doubles[0]);

    DmgState states[] = {
        {"Left Aileron",   st.LeftAileron},
        {"Right Aileron",  st.RightAileron},
        {"Landing Gear",   st.LandingGear},
        {"Docking Port",   st.DockingPort},
        {"Retro Doors",    st.RetroDoors},
        {"Top Hatch",      st.TopHatch},
        {"Radiator",       st.Radiator},
        {"Speedbrake",     st.Speedbrake},
        {"Bay Doors",      st.PayloadBayDoors},
        {"Crew Elevator",  st.CrewElevator},
    };
    int numStates = sizeof(states) / sizeof(states[0]);

    if (fullMode) {
        // Full listing
        printf("=== Damage Report (Full) ===\n");

        printf("Wings:\n");
        printf("  Left Wing:  %.0f%%\n", st.LeftWing * 100.0);
        printf("  Right Wing: %.0f%%\n", st.RightWing * 100.0);

        printf("Main Engines:\n");
        printf("  Left:  %.0f%%\n", st.LeftMainEngine * 100.0);
        printf("  Right: %.0f%%\n", st.RightMainEngine * 100.0);

        printf("SCRAM Engines:\n");
        printf("  Left:  %.0f%%\n", st.LeftSCRAMEngine * 100.0);
        printf("  Right: %.0f%%\n", st.RightSCRAMEngine * 100.0);

        printf("Hover Engines:\n");
        printf("  Fore: %.0f%%\n", st.ForeHoverEngine * 100.0);
        printf("  Aft:  %.0f%%\n", st.AftHoverEngine * 100.0);

        printf("Retro Engines:\n");
        printf("  Left:  %.0f%%\n", st.LeftRetroEngine * 100.0);
        printf("  Right: %.0f%%\n", st.RightRetroEngine * 100.0);

        printf("RCS Thrusters:\n");
        for (int i = 10; i < numDoubles; i++) {
            if (doubles[i].value >= 0)
                printf("  %-24s %.0f%%\n", doubles[i].name, doubles[i].value * 100.0);
        }

        printf("Control Surfaces:\n");
        for (int i = 0; i < 2; i++) {
            if (states[i].value != XRDamageState::XRDMG_NotSupported)
                printf("  %-24s %s\n", states[i].name,
                       states[i].value == XRDamageState::XRDMG_online ? "online" : "OFFLINE");
        }

        printf("Mechanical:\n");
        for (int i = 2; i < numStates; i++) {
            if (states[i].value != XRDamageState::XRDMG_NotSupported)
                printf("  %-24s %s\n", states[i].name,
                       states[i].value == XRDamageState::XRDMG_online ? "online" : "OFFLINE");
        }

        // Warnings
        printf("Warnings:\n");
        printf("  Master:      %s\n", st.MasterWarning == XRWarningState::XRW_warningActive ? "ACTIVE" : "off");
        printf("  Hull Temp:   %s\n", st.HullTemperatureWarning == XRWarningState::XRW_warningActive ? "ACTIVE" : "off");
        printf("  Main Fuel:   %s\n", st.MainFuelWarning == XRWarningState::XRW_warningActive ? "ACTIVE" : "off");
        printf("  RCS Fuel:    %s\n", st.RCSFuelWarning == XRWarningState::XRW_warningActive ? "ACTIVE" : "off");
        printf("  APU Fuel:    %s\n", st.APUFuelWarning == XRWarningState::XRW_warningActive ? "ACTIVE" : "off");
        printf("  LOX:         %s\n", st.LOXWarning == XRWarningState::XRW_warningActive ? "ACTIVE" : "off");
        printf("  Dyn Press:   %s\n", st.DynamicPressureWarning == XRWarningState::XRW_warningActive ? "ACTIVE" : "off");
        printf("  Coolant:     %s\n", st.CoolantWarning == XRWarningState::XRW_warningActive ? "ACTIVE" : "off");
    } else {
        // Summary mode - only show problems
        int issues = 0;
        int checked = 0;

        for (int i = 0; i < numDoubles; i++) {
            if (doubles[i].value >= 0) {
                checked++;
                if (doubles[i].value < 1.0) {
                    printf("  %-24s %3.0f%%\n", doubles[i].name, doubles[i].value * 100.0);
                    issues++;
                }
            }
        }
        for (int i = 0; i < numStates; i++) {
            if (states[i].value != XRDamageState::XRDMG_NotSupported) {
                checked++;
                if (states[i].value == XRDamageState::XRDMG_offline) {
                    printf("  %-24s OFFLINE\n", states[i].name);
                    issues++;
                }
            }
        }

        // Active warnings
        if (st.MasterWarning == XRWarningState::XRW_warningActive) {
            printf("  *** MASTER WARNING ACTIVE ***\n");
            issues++;
        }
        if (st.InternalSystemsFailure) {
            printf("  *** INTERNAL SYSTEMS FAILURE ***\n");
            issues++;
        }

        if (issues == 0)
            printf("All systems nominal (%d checked)\n", checked);
        else
            printf("%d issue%s found\n", issues, issues == 1 ? "" : "s");
    }
#else
    printf("XR vessel support not compiled in\n");
#endif
}

// =========================================================================
// XR Vessel Hull Temperatures
// =========================================================================

void PrintTemps(const char* arg) {
#ifdef HAS_XRVESSELCTRL
    XRVesselCtrl* xr = GetXRVessel();
    if (!xr) return;

    XRSystemStatusRead st = {};
    xr->GetXRSystemStatus(st);

    printf("Hull Temperatures:\n");

    struct HullSurf { const char* name; double temp; double maxSafe; };
    HullSurf surfaces[] = {
        {"Nosecone",   st.NoseconeTemp,  st.MaxSafeNoseconeTemp},
        {"Left Wing",  st.LeftWingTemp,   st.MaxSafeWingTemp},
        {"Right Wing", st.RightWingTemp,  st.MaxSafeWingTemp},
        {"Cockpit",    st.CockpitTemp,    st.MaxSafeCockpitTemp},
        {"Top Hull",   st.TopHullTemp,    st.MaxSafeTopHullTemp},
    };

    for (int i = 0; i < 5; i++) {
        if (surfaces[i].temp < 0 || surfaces[i].maxSafe <= 0) continue;
        double pct = (surfaces[i].temp / surfaces[i].maxSafe) * 100.0;
        const char* warn = "";
        if (pct >= 100.0)     warn = " ** OVER LIMIT **";
        else if (pct >= 90.0) warn = " ** CRITICAL **";
        else if (pct >= 80.0) warn = " ** HIGH **";
        printf("  %-10s %5.0fK / %5.0fK  (%3.0f%%)%s\n",
               surfaces[i].name, surfaces[i].temp, surfaces[i].maxSafe, pct, warn);
    }

    if (st.CoolantTemp >= -270)
        printf("  Coolant:   %.0f C\n", st.CoolantTemp);

    if (st.HullTemperatureWarning == XRWarningState::XRW_warningActive)
        printf("  *** HULL TEMPERATURE WARNING ***\n");
    if (st.CoolantWarning == XRWarningState::XRW_warningActive)
        printf("  *** COOLANT WARNING ***\n");
#else
    printf("XR vessel support not compiled in\n");
#endif
}

// =========================================================================
// XR Vessel Reentry Checklist
// =========================================================================

void PrintReentry(const char* arg) {
#ifdef HAS_XRVESSELCTRL
    XRVesselCtrl* xr = GetXRVessel();
    if (!xr) return;

    struct ReentryDoor { const char* name; XRDoorID id; };
    static const ReentryDoor doors[] = {
        {"SCRAM Doors", XRDoorID::XRD_ScramDoors},
        {"Hover Doors", XRDoorID::XRD_HoverDoors},
        {"Gear",        XRDoorID::XRD_Gear},
        {"Retro Doors", XRDoorID::XRD_RetroDoors},
        {"Radiator",    XRDoorID::XRD_Radiator},
        {"Speedbrake",  XRDoorID::XRD_Speedbrake},
    };
    static const int numDoors = sizeof(doors) / sizeof(doors[0]);

    bool doClose = (arg && (_stricmp(arg, "close") == 0));

    if (doClose) {
        // Close any open reentry doors
        for (int i = 0; i < numDoors; i++) {
            XRDoorState state = xr->GetDoorState(doors[i].id);
            if (state == XRDoorState::XRDS_Open || state == XRDoorState::XRDS_Opening) {
                if (xr->SetDoorState(doors[i].id, XRDoorState::XRDS_Closing))
                    printf("Closing %s\n", doors[i].name);
                else
                    printf("Failed to close %s\n", doors[i].name);
            }
        }
        printf("\n");
    }

    // Show checklist
    printf("Reentry Checklist:\n");
    int noGo = 0;
    for (int i = 0; i < numDoors; i++) {
        XRDoorState state = xr->GetDoorState(doors[i].id);
        if (state == XRDoorState::XRDS_DoorNotSupported) continue;
        const char* status;
        if (state == XRDoorState::XRDS_Closed) {
            status = "go";
        } else if (state == XRDoorState::XRDS_Opening || state == XRDoorState::XRDS_Closing) {
            status = "...";
            noGo++;
        } else {
            status = "NO-GO";
            noGo++;
        }
        printf("  %-12s %s\n", doors[i].name, status);
    }

    if (noGo == 0)
        printf("GO for reentry\n");
    else {
        printf("NO-GO (%d door%s not closed)\n", noGo, noGo == 1 ? "" : "s");
        if (!doClose)
            printf("Type \"re close\" to close all open doors.\n");
    }
#else
    printf("XR vessel support not compiled in\n");
#endif
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
