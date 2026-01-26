// Queries.cpp - Data query functions for vessel, orbit, flight, etc.

#include "Queries.h"
#include "Formatting.h"
#include <stdio.h>
#include <string.h>

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

void PrintMFD() {
    printf("Left: %s\n", GetMFDModeName(oapiGetMFDMode(MFD_LEFT)));
    printf("Right: %s\n", GetMFDModeName(oapiGetMFDMode(MFD_RIGHT)));
}

void PrintDock() {
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

void PrintFuel() {
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

void PrintAll() {
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
