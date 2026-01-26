// Copyright (c) Martin Schweiger
// Licensed under the MIT License

// ==============================================================
//                 ORBITER MODULE: AccessibleMFD
//                    Part of the ORBITER SDK
//
// AccessibleMFD.cpp
//
// Console-based accessible flight data display.
// Type commands to query flight data on demand.
// Access via Ctrl+F4 "Custom Functions" menu in Orbiter.
// ==============================================================

#define STRICT
#define ORBITER_MODULE
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <process.h>
#include "orbitersdk.h"

// Console state
static HANDLE g_hConsole = NULL;
static HANDLE g_hThread = NULL;
static volatile bool g_bRunning = false;
static HINSTANCE g_hInst = NULL;
static DWORD g_dwCmd = 0;

// Transfer planner state
static OBJHANDLE g_hTarget = NULL;
static bool g_bTargetIsVessel = false;

// Forward declarations
static void PrintVessel();
static void PrintOrbit();
static void PrintFlight();
static void PrintMFD();
static void PrintDock();
static void PrintAll();
static void PrintHelp();
static void PrintNav(const char* arg);
static void PrintThrottle(const char* arg);
static void PrintFuel();
static void PrintWarp(const char* arg);
static void PrintMap(const char* arg);
static void PrintTarget(const char* arg);
static void PrintTransfer(const char* arg);
static void PrintHohmann();
static void PrintPhase();
static void PrintPlane();
static void PrintRendezvous();
static unsigned __stdcall ConsoleThread(void* param);

// Helper to get navmode name
static const char* GetNavmodeName(int mode) {
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

// Helper to get engine type name
static const char* GetEngineName(ENGINETYPE eng) {
    switch (eng) {
        case ENGINE_MAIN:  return "Main";
        case ENGINE_RETRO: return "Retro";
        case ENGINE_HOVER: return "Hover";
        default:           return "Unknown";
    }
}

// Helper to get MFD mode name
static const char* GetMFDModeName(int mode) {
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

// Format helpers
static void FormatDistance(double meters, char* buf, int len) {
    if (meters >= 1e9)
        _snprintf(buf, len, "%.2f Gm", meters / 1e9);
    else if (meters >= 1e6)
        _snprintf(buf, len, "%.2f Mm", meters / 1e6);
    else if (meters >= 1e3)
        _snprintf(buf, len, "%.2f km", meters / 1e3);
    else
        _snprintf(buf, len, "%.1f m", meters);
    buf[len-1] = '\0';
}

static void FormatTime(double seconds, char* buf, int len) {
    if (seconds < 0) {
        _snprintf(buf, len, "N/A");
    } else if (seconds >= 86400) {
        int d = (int)(seconds / 86400);
        int h = (int)((seconds - d * 86400) / 3600);
        int m = (int)((seconds - d * 86400 - h * 3600) / 60);
        _snprintf(buf, len, "%dd %dh %dm", d, h, m);
    } else if (seconds >= 3600) {
        int h = (int)(seconds / 3600);
        int m = (int)((seconds - h * 3600) / 60);
        int s = (int)(seconds - h * 3600 - m * 60);
        _snprintf(buf, len, "%dh %dm %ds", h, m, s);
    } else if (seconds >= 60) {
        int m = (int)(seconds / 60);
        int s = (int)(seconds - m * 60);
        _snprintf(buf, len, "%dm %ds", m, s);
    } else {
        _snprintf(buf, len, "%.1fs", seconds);
    }
    buf[len-1] = '\0';
}

// Map helpers - great circle calculations
static double CalcDistance(double lat1, double lon1, double lat2, double lon2, double radius) {
    // Haversine formula
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return radius * c;
}

static double CalcBearing(double lat1, double lon1, double lat2, double lon2) {
    // Initial bearing from point 1 to point 2
    double dLon = lon2 - lon1;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x);
    return posangle(bearing) * DEG;  // Convert to degrees 0-360
}

static void FormatLatLon(double lat, double lon, char* buf, int len) {
    // lat/lon are in radians, convert to degrees
    double latDeg = fabs(lat * DEG);
    double lonDeg = fabs(lon * DEG);
    char latDir = (lat >= 0) ? 'N' : 'S';
    char lonDir = (lon >= 0) ? 'E' : 'W';
    _snprintf(buf, len, "%.2f%c%c, %.2f%c%c", latDeg, 0xB0, latDir, lonDeg, 0xB0, lonDir);
    buf[len-1] = '\0';
}

// === TRANSFER CALCULATION STRUCTURES ===

struct HohmannTransfer {
    double departDV;     // m/s
    double arriveDV;     // m/s
    double totalDV;      // m/s
    double transferTime; // seconds
    bool valid;
};

struct PhaseAngleData {
    double currentPhase;  // degrees
    double requiredPhase; // degrees
    double phaseDiff;     // degrees (positive = wait, negative = missed)
    double timeToWindow;  // seconds
    bool valid;
};

struct PlaneChangeData {
    double relInc;        // degrees
    double planeChangeDV; // m/s
    double nodeAngle;     // degrees from current position to AN
    bool valid;
};

struct RendezvousData {
    double distance;      // m
    double closureRate;   // m/s (positive = approaching)
    double relVelMag;     // m/s
    VECTOR3 relVel;       // local frame relative velocity
    double timeToClose;   // seconds at current rate
    bool valid;
};

// Gravitational constant
static const double G_CONST = 6.67430e-11;  // m^3 kg^-1 s^-2

// Get GM (gravitational parameter) for a body
static double GetGM(OBJHANDLE hBody) {
    double mass = oapiGetMass(hBody);
    return G_CONST * mass;
}

// Calculate Hohmann transfer between two circular orbits
// r1 = departure orbit radius, r2 = target orbit radius, gm = gravitational parameter
static HohmannTransfer CalcHohmann(double r1, double r2, double gm) {
    HohmannTransfer ht = {0, 0, 0, 0, false};

    if (r1 <= 0 || r2 <= 0 || gm <= 0) return ht;

    double a_t = (r1 + r2) / 2.0;  // Transfer orbit semi-major axis

    // Circular velocity at r1
    double v1_circ = sqrt(gm / r1);

    // Transfer orbit velocity at r1 (periapsis if r2 > r1, apoapsis if r2 < r1)
    double v1_trans = sqrt(gm * (2.0 / r1 - 1.0 / a_t));

    // Circular velocity at r2
    double v2_circ = sqrt(gm / r2);

    // Transfer orbit velocity at r2
    double v2_trans = sqrt(gm * (2.0 / r2 - 1.0 / a_t));

    // Delta-V calculations
    if (r2 > r1) {
        // Raising orbit: burn prograde at r1, then prograde at r2
        ht.departDV = v1_trans - v1_circ;
        ht.arriveDV = v2_circ - v2_trans;
    } else {
        // Lowering orbit: burn retrograde at r1, then retrograde at r2
        ht.departDV = v1_circ - v1_trans;
        ht.arriveDV = v2_trans - v2_circ;
    }

    ht.totalDV = fabs(ht.departDV) + fabs(ht.arriveDV);

    // Transfer time = half the transfer orbit period
    ht.transferTime = PI * sqrt(a_t * a_t * a_t / gm);

    ht.valid = true;
    return ht;
}

// Calculate phase angle data between vessel and target
// Returns current phase, required phase for Hohmann, and time to window
static PhaseAngleData CalcPhaseAngle(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef) {
    PhaseAngleData pa = {0, 0, 0, 0, false};

    if (!v || !hTarget || !hRef) return pa;

    // Get vessel orbital elements
    ELEMENTS el_v;
    ORBITPARAM prm_v;
    if (!v->GetElements(hRef, el_v, &prm_v, 0, FRAME_ECL)) return pa;

    // Skip if vessel is on escape trajectory
    if (el_v.e >= 1.0) return pa;

    // Get vessel's true longitude (angle from reference direction)
    double vesselTrL = prm_v.TrL;  // True longitude in radians

    // Get target position
    VECTOR3 targetPos, refPos;
    oapiGetGlobalPos(hTarget, &targetPos);
    oapiGetGlobalPos(hRef, &refPos);
    VECTOR3 relTargetPos = targetPos - refPos;

    // Calculate target's angular position (in ecliptic plane)
    double targetAngle = atan2(relTargetPos.z, relTargetPos.x);
    if (targetAngle < 0) targetAngle += 2.0 * PI;

    // Current phase angle (target ahead of vessel is positive)
    double currentPhase = targetAngle - vesselTrL;
    while (currentPhase < 0) currentPhase += 2.0 * PI;
    while (currentPhase >= 2.0 * PI) currentPhase -= 2.0 * PI;

    pa.currentPhase = currentPhase * DEG;

    // Calculate required phase angle for Hohmann transfer
    double gm = GetGM(hRef);
    double r1 = el_v.a * (1.0 - el_v.e * el_v.e) / (1.0 + el_v.e * cos(prm_v.TrA));  // Current radius
    double r2 = length(relTargetPos);  // Target radius (assuming circular)

    double a_t = (r1 + r2) / 2.0;
    double transferTime = PI * sqrt(a_t * a_t * a_t / gm);

    // Target angular velocity (assuming circular orbit)
    double targetPeriod = 2.0 * PI * sqrt(r2 * r2 * r2 / gm);
    double targetAngVel = 2.0 * PI / targetPeriod;  // rad/s

    // Required phase = 180 degrees minus angle target travels during transfer
    double requiredPhase = PI - targetAngVel * transferTime;
    while (requiredPhase < 0) requiredPhase += 2.0 * PI;

    pa.requiredPhase = requiredPhase * DEG;

    // Phase difference and time to window
    double phaseDiff = requiredPhase - currentPhase * RAD / DEG * RAD;  // Convert back properly
    phaseDiff = (requiredPhase * RAD) - currentPhase * RAD;

    // Synodic period calculation
    double vesselAngVel = 2.0 * PI / prm_v.T;
    double synodicRate = fabs(vesselAngVel - targetAngVel);

    if (synodicRate > 1e-10) {
        // Normalize phase difference
        double diff = (pa.requiredPhase - pa.currentPhase) * RAD;
        while (diff < 0) diff += 2.0 * PI;
        while (diff >= 2.0 * PI) diff -= 2.0 * PI;

        pa.phaseDiff = diff * DEG;
        pa.timeToWindow = diff / synodicRate;
    } else {
        pa.phaseDiff = 0;
        pa.timeToWindow = -1;  // Same orbital period, window always exists
    }

    pa.valid = true;
    return pa;
}

// Calculate plane change requirements
static PlaneChangeData CalcPlaneChange(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef) {
    PlaneChangeData pc = {0, 0, 0, false};

    if (!v || !hTarget || !hRef) return pc;

    // Get vessel orbital elements
    ELEMENTS el_v;
    ORBITPARAM prm_v;
    if (!v->GetElements(hRef, el_v, &prm_v, 0, FRAME_ECL)) return pc;

    // Get vessel position and velocity to compute orbital plane normal
    VECTOR3 vesselPos, vesselVel, refPos, refVel;
    v->GetGlobalPos(vesselPos);
    v->GetGlobalVel(vesselVel);
    oapiGetGlobalPos(hRef, &refPos);
    oapiGetGlobalVel(hRef, &refVel);

    VECTOR3 relPos_v = vesselPos - refPos;
    VECTOR3 relVel_v = vesselVel - refVel;

    // Vessel orbital plane normal: h = r x v
    VECTOR3 h_v = crossp(relPos_v, relVel_v);
    double h_v_mag = length(h_v);
    if (h_v_mag < 1e-10) return pc;
    h_v = h_v / h_v_mag;  // Normalize

    // Get target orbital plane normal
    VECTOR3 targetPos, targetVel;
    oapiGetGlobalPos(hTarget, &targetPos);
    oapiGetGlobalVel(hTarget, &targetVel);

    VECTOR3 relPos_t = targetPos - refPos;
    VECTOR3 relVel_t = targetVel - refVel;

    VECTOR3 h_t = crossp(relPos_t, relVel_t);
    double h_t_mag = length(h_t);
    if (h_t_mag < 1e-10) return pc;
    h_t = h_t / h_t_mag;  // Normalize

    // Relative inclination = angle between orbital plane normals
    double cosInc = dotp(h_v, h_t);
    if (cosInc > 1.0) cosInc = 1.0;
    if (cosInc < -1.0) cosInc = -1.0;
    double relInc = acos(cosInc);

    pc.relInc = relInc * DEG;

    // Calculate plane change delta-V (at current velocity)
    // DV = 2 * v * sin(inc/2)
    double v_mag = length(relVel_v);
    pc.planeChangeDV = 2.0 * v_mag * sin(relInc / 2.0);

    // Calculate angle to ascending node
    // Line of nodes = h_v x h_t (intersection of orbital planes)
    VECTOR3 nodeDir = crossp(h_v, h_t);
    double nodeDir_mag = length(nodeDir);
    if (nodeDir_mag > 1e-10) {
        nodeDir = nodeDir / nodeDir_mag;

        // Angle from current position to ascending node
        double dotNode = dotp(relPos_v / length(relPos_v), nodeDir);
        if (dotNode > 1.0) dotNode = 1.0;
        if (dotNode < -1.0) dotNode = -1.0;
        pc.nodeAngle = acos(dotNode) * DEG;
    } else {
        pc.nodeAngle = 0;  // Planes are coplanar
    }

    pc.valid = true;
    return pc;
}

// Calculate rendezvous data for vessel targets
static RendezvousData CalcRendezvous(VESSEL* v, OBJHANDLE hTarget) {
    RendezvousData rd = {0, 0, 0, {0,0,0}, 0, false};

    if (!v || !hTarget) return rd;

    // Get relative position and velocity
    VECTOR3 relPos, relVel;
    v->GetRelativePos(hTarget, relPos);
    v->GetRelativeVel(hTarget, relVel);

    rd.distance = length(relPos);
    rd.relVelMag = length(relVel);

    // Closure rate (positive = approaching)
    if (rd.distance > 1e-10) {
        rd.closureRate = -dotp(relVel, relPos) / rd.distance;
    }

    // Time to closest approach at current rate
    if (rd.closureRate > 1e-10) {
        rd.timeToClose = rd.distance / rd.closureRate;
    } else {
        rd.timeToClose = -1;  // Not approaching
    }

    // Convert relative velocity to vessel local frame
    MATRIX3 rot;
    v->GetRotationMatrix(rot);
    rd.relVel = tmul(rot, relVel);

    rd.valid = true;
    return rd;
}

// === PRINT FUNCTIONS ===

static void PrintVessel() {
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

static void PrintOrbit() {
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

static void PrintFlight() {
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

static void PrintMFD() {
    printf("Left: %s\n", GetMFDModeName(oapiGetMFDMode(MFD_LEFT)));
    printf("Right: %s\n", GetMFDModeName(oapiGetMFDMode(MFD_RIGHT)));
}

static void PrintDock() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

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

    // Local frame velocities
    MATRIX3 rot;
    v->GetRotationMatrix(rot);
    VECTOR3 lv = tmul(rot, relVel);

    printf("Vx: %.2f m/s\n", lv.x);
    printf("Vy: %.2f m/s\n", lv.y);
    printf("Vz: %.2f m/s\n", lv.z);
}

static void PrintAll() {
    printf("=== VESSEL ===\n");
    PrintVessel();
    printf("=== ORBIT ===\n");
    PrintOrbit();
    printf("=== FLIGHT ===\n");
    PrintFlight();
    printf("=== MFD ===\n");
    PrintMFD();
    printf("=== DOCK ===\n");
    PrintDock();
}

// Navigation autopilot control
static void PrintNav(const char* arg) {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    // No argument - show status
    if (!arg || arg[0] == '\0') {
        printf("Autopilot Status:\n");
        printf("  Kill Rot:    %s\n", v->GetNavmodeState(NAVMODE_KILLROT) ? "ON" : "off");
        printf("  Hold Level:  %s\n", v->GetNavmodeState(NAVMODE_HLEVEL) ? "ON" : "off");
        printf("  Prograde:    %s\n", v->GetNavmodeState(NAVMODE_PROGRADE) ? "ON" : "off");
        printf("  Retrograde:  %s\n", v->GetNavmodeState(NAVMODE_RETROGRADE) ? "ON" : "off");
        printf("  Normal:      %s\n", v->GetNavmodeState(NAVMODE_NORMAL) ? "ON" : "off");
        printf("  Anti-Normal: %s\n", v->GetNavmodeState(NAVMODE_ANTINORMAL) ? "ON" : "off");
        printf("  Hold Alt:    %s\n", v->GetNavmodeState(NAVMODE_HOLDALT) ? "ON" : "off");
        return;
    }

    // Parse subcommand
    if (_stricmp(arg, "off") == 0) {
        v->DeactivateNavmode(NAVMODE_KILLROT);
        v->DeactivateNavmode(NAVMODE_HLEVEL);
        v->DeactivateNavmode(NAVMODE_PROGRADE);
        v->DeactivateNavmode(NAVMODE_RETROGRADE);
        v->DeactivateNavmode(NAVMODE_NORMAL);
        v->DeactivateNavmode(NAVMODE_ANTINORMAL);
        v->DeactivateNavmode(NAVMODE_HOLDALT);
        printf("All autopilot modes disabled\n");
    } else if (_stricmp(arg, "killrot") == 0 || _stricmp(arg, "kill") == 0) {
        v->ToggleNavmode(NAVMODE_KILLROT);
        printf("Kill Rotation: %s\n", v->GetNavmodeState(NAVMODE_KILLROT) ? "ON" : "off");
    } else if (_stricmp(arg, "hlevel") == 0 || _stricmp(arg, "level") == 0) {
        v->ToggleNavmode(NAVMODE_HLEVEL);
        printf("Hold Level: %s\n", v->GetNavmodeState(NAVMODE_HLEVEL) ? "ON" : "off");
    } else if (_stricmp(arg, "prograde") == 0 || _stricmp(arg, "pro") == 0) {
        v->ToggleNavmode(NAVMODE_PROGRADE);
        printf("Prograde: %s\n", v->GetNavmodeState(NAVMODE_PROGRADE) ? "ON" : "off");
    } else if (_stricmp(arg, "retrograde") == 0 || _stricmp(arg, "retro") == 0) {
        v->ToggleNavmode(NAVMODE_RETROGRADE);
        printf("Retrograde: %s\n", v->GetNavmodeState(NAVMODE_RETROGRADE) ? "ON" : "off");
    } else if (_stricmp(arg, "normal") == 0 || _stricmp(arg, "nml") == 0) {
        v->ToggleNavmode(NAVMODE_NORMAL);
        printf("Normal: %s\n", v->GetNavmodeState(NAVMODE_NORMAL) ? "ON" : "off");
    } else if (_stricmp(arg, "antinormal") == 0 || _stricmp(arg, "anml") == 0) {
        v->ToggleNavmode(NAVMODE_ANTINORMAL);
        printf("Anti-Normal: %s\n", v->GetNavmodeState(NAVMODE_ANTINORMAL) ? "ON" : "off");
    } else if (_stricmp(arg, "holdalt") == 0 || _stricmp(arg, "halt") == 0) {
        v->ToggleNavmode(NAVMODE_HOLDALT);
        printf("Hold Altitude: %s\n", v->GetNavmodeState(NAVMODE_HOLDALT) ? "ON" : "off");
    } else {
        printf("Unknown navmode: %s\n", arg);
        printf("Options: killrot, hlevel, prograde, retrograde, normal, antinormal, holdalt, off\n");
    }
}

// Throttle/engine control
static void PrintThrottle(const char* arg) {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    // No argument - show status
    if (!arg || arg[0] == '\0') {
        printf("Engine Status:\n");
        printf("  Main:  %5.1f%%\n", v->GetEngineLevel(ENGINE_MAIN) * 100.0);
        printf("  Retro: %5.1f%%\n", v->GetEngineLevel(ENGINE_RETRO) * 100.0);
        printf("  Hover: %5.1f%%\n", v->GetEngineLevel(ENGINE_HOVER) * 100.0);
        return;
    }

    // Parse: "th <level>" or "th <engine> <level>"
    char eng[32] = "";
    double level = -1;

    // Try parsing as "th <level>" first
    if (sscanf(arg, "%lf", &level) == 1 && strchr(arg, ' ') == NULL) {
        // Single number - set main engine
        if (level < 0 || level > 100) {
            printf("Level must be 0-100\n");
            return;
        }
        v->SetEngineLevel(ENGINE_MAIN, level / 100.0);
        printf("Main: %.1f%%\n", level);
        return;
    }

    // Try parsing as "th <engine> <level>"
    if (sscanf(arg, "%31s %lf", eng, &level) == 2) {
        if (level < 0 || level > 100) {
            printf("Level must be 0-100\n");
            return;
        }

        ENGINETYPE etype;
        if (_stricmp(eng, "main") == 0) {
            etype = ENGINE_MAIN;
        } else if (_stricmp(eng, "retro") == 0) {
            etype = ENGINE_RETRO;
        } else if (_stricmp(eng, "hover") == 0) {
            etype = ENGINE_HOVER;
        } else {
            printf("Unknown engine: %s\n", eng);
            printf("Options: main, retro, hover\n");
            return;
        }

        v->SetEngineLevel(etype, level / 100.0);
        printf("%s: %.1f%%\n", GetEngineName(etype), level);
        return;
    }

    printf("Usage: th [<level>] or th <engine> <level>\n");
    printf("  th 50        - Set main engine to 50%%\n");
    printf("  th main 100  - Set main engine to 100%%\n");
    printf("  th hover 30  - Set hover engine to 30%%\n");
}

// Fuel status
static void PrintFuel() {
    VESSEL* v = oapiGetFocusInterface();
    if (!v) {
        printf("No vessel\n");
        return;
    }

    double fuel = v->GetFuelMass();
    double maxFuel = v->GetMaxFuelMass();

    if (maxFuel <= 0) {
        printf("No fuel tank\n");
        return;
    }

    double pct = (fuel / maxFuel) * 100.0;

    printf("Fuel: %.1f / %.1f kg (%.1f%%)\n", fuel, maxFuel, pct);

    // Show total propellant if different from main tank
    double total = v->GetTotalPropellantMass();
    if (total > maxFuel * 1.01) {  // More than 1% difference suggests multiple tanks
        printf("Total Propellant: %.1f kg\n", total);
    }
}

// Time warp control
static void PrintWarp(const char* arg) {
    // No argument - show current warp
    if (!arg || arg[0] == '\0') {
        double warp = oapiGetTimeAcceleration();
        if (warp < 1.0)
            printf("Time Warp: %.2fx (slow motion)\n", warp);
        else
            printf("Time Warp: %.0fx\n", warp);
        return;
    }

    // Parse warp factor (0.1 to 100000)
    double warp = 0;
    if (sscanf(arg, "%lf", &warp) != 1 || warp < 0.1 || warp > 100000) {
        printf("Usage: warp [factor]\n");
        printf("  warp        - Show current time warp\n");
        printf("  warp 0.1    - Slow motion (minimum)\n");
        printf("  warp 1      - Normal time\n");
        printf("  warp 10     - 10x time acceleration\n");
        printf("  warp 100000 - 100000x (maximum)\n");
        return;
    }

    oapiSetTimeAcceleration(warp);
    if (warp < 1.0)
        printf("Time Warp: %.2fx (slow motion)\n", warp);
    else
        printf("Time Warp: %.0fx\n", warp);
}

// Map position and bases
static void PrintMap(const char* arg) {
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

static void PrintHelp() {
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

// Target selection and display
static void PrintTarget(const char* arg) {
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

// Print Hohmann transfer data
static void PrintHohmann() {
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

// Print phase angle data
static void PrintPhase() {
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

// Print plane change data
static void PrintPlane() {
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

// Print rendezvous data (for vessel targets)
static void PrintRendezvous() {
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

// Transfer summary dispatcher
static void PrintTransfer(const char* arg) {
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

// Console thread
static unsigned __stdcall ConsoleThread(void* param) {
    char line[256];

    printf("Accessible Flight Data Console\n");
    printf("Type ? for help\n\n");

    while (g_bRunning) {
        printf("> ");
        fflush(stdout);

        if (!fgets(line, sizeof(line), stdin)) {
            Sleep(100);
            continue;
        }

        // Trim newline
        char* p = line;
        while (*p && *p != '\n' && *p != '\r') p++;
        *p = '\0';

        // Parse command - extract first word and remainder
        char cmd[32] = "";
        char* arg = NULL;
        sscanf(line, "%31s", cmd);
        arg = line + strlen(cmd);
        while (*arg == ' ') arg++;  // Skip leading spaces in argument

        if (_stricmp(cmd, "v") == 0 || _stricmp(cmd, "vessel") == 0) {
            PrintVessel();
        } else if (_stricmp(cmd, "o") == 0 || _stricmp(cmd, "orbit") == 0) {
            PrintOrbit();
        } else if (_stricmp(cmd, "f") == 0 || _stricmp(cmd, "flight") == 0) {
            PrintFlight();
        } else if (_stricmp(cmd, "m") == 0 || _stricmp(cmd, "mfd") == 0) {
            PrintMFD();
        } else if (_stricmp(cmd, "d") == 0 || _stricmp(cmd, "dock") == 0) {
            PrintDock();
        } else if (_stricmp(cmd, "a") == 0 || _stricmp(cmd, "all") == 0) {
            PrintAll();
        } else if (_stricmp(cmd, "na") == 0 || _stricmp(cmd, "nav") == 0) {
            PrintNav(arg);
        } else if (_stricmp(cmd, "th") == 0 || _stricmp(cmd, "throttle") == 0) {
            PrintThrottle(arg);
        } else if (_stricmp(cmd, "fuel") == 0) {
            PrintFuel();
        } else if (_stricmp(cmd, "warp") == 0 || _stricmp(cmd, "w") == 0) {
            PrintWarp(arg);
        } else if (_stricmp(cmd, "map") == 0) {
            PrintMap(arg);
        } else if (_stricmp(cmd, "tgt") == 0 || _stricmp(cmd, "target") == 0) {
            PrintTarget(arg);
        } else if (_stricmp(cmd, "tr") == 0 || _stricmp(cmd, "transfer") == 0) {
            PrintTransfer(arg);
        } else if (_stricmp(cmd, "?") == 0 || _stricmp(cmd, "help") == 0) {
            PrintHelp();
        } else if (_stricmp(cmd, "q") == 0 || _stricmp(cmd, "quit") == 0) {
            printf("Closing console...\n");
            break;
        } else if (cmd[0] != '\0') {
            printf("Unknown command. Type ? for help.\n");
        }
    }

    return 0;
}

// Open console
static void OpenConsole() {
    if (g_hConsole) {
        // Already open, just bring to front
        HWND hWnd = GetConsoleWindow();
        if (hWnd) SetForegroundWindow(hWnd);
        return;
    }

    if (!AllocConsole()) return;

    g_hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

    // Redirect stdout/stdin
    freopen("CONOUT$", "w", stdout);
    freopen("CONIN$", "r", stdin);

    SetConsoleTitle("Orbiter - Accessible Flight Data");

    // Start console thread
    g_bRunning = true;
    g_hThread = (HANDLE)_beginthreadex(NULL, 0, ConsoleThread, NULL, 0, NULL);
}

static void CloseConsole() {
    if (!g_hConsole) return;

    g_bRunning = false;

    if (g_hThread) {
        // Give thread time to exit
        WaitForSingleObject(g_hThread, 1000);
        CloseHandle(g_hThread);
        g_hThread = NULL;
    }

    FreeConsole();
    g_hConsole = NULL;
}

// Custom command callback
static void OpenAccessibleConsole(void* context) {
    OpenConsole();
}

// === ORBITER MODULE INTERFACE ===

DLLCLBK void InitModule(HINSTANCE hDLL) {
    g_hInst = hDLL;

    // Register custom command in Ctrl+F4 menu
    g_dwCmd = oapiRegisterCustomCmd(
        "Accessible Flight Data",
        "Opens console window for accessible flight data queries",
        OpenAccessibleConsole,
        NULL
    );
}

DLLCLBK void ExitModule(HINSTANCE hDLL) {
    oapiUnregisterCustomCmd(g_dwCmd);
    CloseConsole();
}
