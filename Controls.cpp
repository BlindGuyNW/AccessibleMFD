// Controls.cpp - Control commands (autopilot, throttle, time warp, resupply)

#include "Controls.h"
#include "Queries.h"
#include "orbitersdk.h"
#include <stdio.h>
#include <string.h>

#ifdef HAS_XRVESSELCTRL
#include "XRVesselCtrl.h"
#endif

void PrintNav(const char* arg) {
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

void PrintThrottle(const char* arg) {
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

void PrintWarp(const char* arg) {
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

// =========================================================================
// XR Vessel Resupply
// =========================================================================

#ifdef HAS_XRVESSELCTRL

const char* DoorStateStr(XRDoorState s) {
    switch (s) {
    case XRDoorState::XRDS_Open:    return "OPEN";
    case XRDoorState::XRDS_Closed:  return "closed";
    case XRDoorState::XRDS_Opening: return "opening";
    case XRDoorState::XRDS_Closing: return "closing";
    case XRDoorState::XRDS_Failed:  return "FAILED";
    default:                        return "N/A";
    }
}

const char* SupplyLineName(XRSupplyLineID id) {
    switch (id) {
    case XRSupplyLineID::XRS_MainFuel:  return "Main Fuel";
    case XRSupplyLineID::XRS_ScramFuel: return "SCRAM Fuel";
    case XRSupplyLineID::XRS_ApuFuel:   return "APU Fuel";
    case XRSupplyLineID::XRS_Lox:       return "LOX";
    default:                            return "Unknown";
    }
}

XRVesselCtrl* GetXRVessel(bool quiet) {
    // Must use oapiGetVesselInterface (returns DLL-side object with correct vtable)
    // NOT oapiGetFocusInterface (returns Orbiter internal proxy)
    OBJHANDLE hFocus = oapiGetFocusObject();
    if (!hFocus) {
        if (!quiet) printf("No vessel\n");
        return nullptr;
    }

    VESSEL* v = oapiGetVesselInterface(hFocus);
    if (!v) {
        if (!quiet) printf("No vessel\n");
        return nullptr;
    }

    if (!XRVesselCtrl::IsXRVesselCtrl(v)) {
        if (!quiet) printf("Not an XR vessel\n");
        return nullptr;
    }

    XRVesselCtrl* xr = static_cast<XRVesselCtrl*>(v);
    float ver = xr->GetCtrlAPIVersion();
    if (ver < 5.0f) {
        if (!quiet) printf("XRVesselCtrl API %.1f too old (need 5.0+)\n", ver);
        return nullptr;
    }

    return xr;
}

void PrintSupplyLineStatus(XRVesselCtrl* xr, XRSupplyLineID id) {
    XRSupplyLineStatus st = {};
    if (!xr->GetExternalSupplyLineStatus(id, st))
        return;
    printf("  %-10s Flow: %-3s  Pressure: %6.1f / %.1f PSI %s\n",
        SupplyLineName(id),
        st.FlowSwitch ? "ON" : "off",
        st.PressurePSI,
        st.NominalPressurePSI,
        st.PressureNominal ? "(nominal)" : "");
}

static void ShowResupplyStatus(XRVesselCtrl* xr) {
    printf("Fuel Hatch: %s\n", DoorStateStr(xr->GetFuelHatchState()));
    printf("LOX Hatch:  %s\n", DoorStateStr(xr->GetLoxHatchState()));
    printf("Supply Lines:\n");
    PrintSupplyLineStatus(xr, XRSupplyLineID::XRS_MainFuel);
    PrintSupplyLineStatus(xr, XRSupplyLineID::XRS_ScramFuel);
    PrintSupplyLineStatus(xr, XRSupplyLineID::XRS_ApuFuel);
    PrintSupplyLineStatus(xr, XRSupplyLineID::XRS_Lox);
}

void PrintResupply(const char* arg) {
    XRVesselCtrl* xr = GetXRVessel();
    if (!xr) return;

    // No argument - show status
    if (!arg || arg[0] == '\0') {
        ShowResupplyStatus(xr);
        return;
    }

    // "fuel open/close" - fuel hatch
    if (_stricmp(arg, "fuel open") == 0) {
        if (xr->SetFuelHatchState(true))
            printf("Fuel hatch opened\n");
        else
            printf("Failed to open fuel hatch\n");
    } else if (_stricmp(arg, "fuel close") == 0) {
        if (xr->SetFuelHatchState(false))
            printf("Fuel hatch closed\n");
        else
            printf("Failed to close fuel hatch\n");
    }
    // "lox open/close" - LOX hatch
    else if (_stricmp(arg, "lox open") == 0) {
        if (xr->SetLoxHatchState(true))
            printf("LOX hatch opened\n");
        else
            printf("Failed to open LOX hatch\n");
    } else if (_stricmp(arg, "lox close") == 0) {
        if (xr->SetLoxHatchState(false))
            printf("LOX hatch closed\n");
        else
            printf("Failed to close LOX hatch\n");
    }
    // "main on/off", "scram on/off", "apu on/off", "loxline on/off" - supply lines
    else {
        char line[32] = "";
        char state[32] = "";
        if (sscanf(arg, "%31s %31s", line, state) != 2) {
            printf("Usage: rs [fuel|lox] [open|close]\n");
            printf("       rs [main|scram|apu|loxline] [on|off]\n");
            return;
        }

        XRSupplyLineID id;
        if (_stricmp(line, "main") == 0)
            id = XRSupplyLineID::XRS_MainFuel;
        else if (_stricmp(line, "scram") == 0)
            id = XRSupplyLineID::XRS_ScramFuel;
        else if (_stricmp(line, "apu") == 0)
            id = XRSupplyLineID::XRS_ApuFuel;
        else if (_stricmp(line, "loxline") == 0)
            id = XRSupplyLineID::XRS_Lox;
        else {
            printf("Unknown: %s\n", line);
            printf("Hatches: fuel, lox (open/close)\n");
            printf("Lines:   main, scram, apu, loxline (on/off)\n");
            return;
        }

        bool bOpen;
        if (_stricmp(state, "on") == 0)
            bOpen = true;
        else if (_stricmp(state, "off") == 0)
            bOpen = false;
        else {
            printf("Use 'on' or 'off'\n");
            return;
        }

        if (xr->SetExternalSupplyLineState(id, bOpen))
            printf("%s flow: %s\n", SupplyLineName(id), bOpen ? "ON" : "off");
        else
            printf("Failed to set %s flow\n", SupplyLineName(id));
    }
}

// =========================================================================
// XR Vessel Fuel Dump
// =========================================================================

static const char* FuelDumpTankName(XRFuelDumpID id) {
    switch (id) {
    case XRFuelDumpID::XRFD_MainFuel:  return "Main Fuel";
    case XRFuelDumpID::XRFD_RcsFuel:   return "RCS Fuel";
    case XRFuelDumpID::XRFD_ScramFuel: return "SCRAM Fuel";
    case XRFuelDumpID::XRFD_ApuFuel:   return "APU Fuel";
    case XRFuelDumpID::XRFD_Lox:       return "LOX";
    default:                           return "Unknown";
    }
}

static void PrintDumpTankStatus(XRVesselCtrl* xr, XRFuelDumpID id) {
    bool bDumping = false;
    if (xr->GetFuelDumpState(id, bDumping))
        printf("  %-10s %s\n", FuelDumpTankName(id), bDumping ? "DUMPING" : "off");
}

void PrintFuelDump(const char* arg) {
    XRVesselCtrl* xr = GetXRVessel();
    if (!xr) return;

    if (xr->GetCtrlAPIVersion() < 6.0f) {
        printf("XRVesselCtrl API %.1f too old (need 6.0+ for fuel dump)\n", xr->GetCtrlAPIVersion());
        return;
    }

    // No argument - show status
    if (!arg || arg[0] == '\0') {
        printf("Fuel Dump Status:\n");
        PrintDumpTankStatus(xr, XRFuelDumpID::XRFD_MainFuel);
        PrintDumpTankStatus(xr, XRFuelDumpID::XRFD_RcsFuel);
        PrintDumpTankStatus(xr, XRFuelDumpID::XRFD_ScramFuel);
        PrintDumpTankStatus(xr, XRFuelDumpID::XRFD_ApuFuel);
        PrintDumpTankStatus(xr, XRFuelDumpID::XRFD_Lox);
        return;
    }

    // "off" - stop all dumping
    if (_stricmp(arg, "off") == 0) {
        xr->SetFuelDumpState(XRFuelDumpID::XRFD_MainFuel, false);
        xr->SetFuelDumpState(XRFuelDumpID::XRFD_RcsFuel, false);
        xr->SetFuelDumpState(XRFuelDumpID::XRFD_ScramFuel, false);
        xr->SetFuelDumpState(XRFuelDumpID::XRFD_ApuFuel, false);
        xr->SetFuelDumpState(XRFuelDumpID::XRFD_Lox, false);
        printf("All fuel dumping stopped\n");
        return;
    }

    // Parse "<tank> on/off"
    char tank[32] = "";
    char state[32] = "";
    if (sscanf(arg, "%31s %31s", tank, state) != 2) {
        printf("Usage: dump [<tank> on/off | off]\n");
        printf("Tanks: main, rcs, scram, apu, lox\n");
        return;
    }

    XRFuelDumpID id;
    if (_stricmp(tank, "main") == 0)
        id = XRFuelDumpID::XRFD_MainFuel;
    else if (_stricmp(tank, "rcs") == 0)
        id = XRFuelDumpID::XRFD_RcsFuel;
    else if (_stricmp(tank, "scram") == 0)
        id = XRFuelDumpID::XRFD_ScramFuel;
    else if (_stricmp(tank, "apu") == 0)
        id = XRFuelDumpID::XRFD_ApuFuel;
    else if (_stricmp(tank, "lox") == 0)
        id = XRFuelDumpID::XRFD_Lox;
    else {
        printf("Unknown tank: %s\n", tank);
        printf("Tanks: main, rcs, scram, apu, lox\n");
        return;
    }

    bool bDump;
    if (_stricmp(state, "on") == 0)
        bDump = true;
    else if (_stricmp(state, "off") == 0)
        bDump = false;
    else {
        printf("Use 'on' or 'off'\n");
        return;
    }

    if (xr->SetFuelDumpState(id, bDump))
        printf("%s: %s\n", FuelDumpTankName(id), bDump ? "DUMPING" : "stopped");
    else
        printf("Failed to set %s dump state\n", FuelDumpTankName(id));
}

// =========================================================================
// XR Vessel Cross-Feed
// =========================================================================

static const char* CrossFeedModeStr(XRXFEED_STATE state) {
    switch (state) {
    case XRXFEED_STATE::XRXF_MAIN: return "MAIN";
    case XRXFEED_STATE::XRXF_OFF:  return "off";
    case XRXFEED_STATE::XRXF_RCS:  return "RCS";
    default:                        return "N/A";
    }
}

void PrintCrossFeed(const char* arg) {
    XRVesselCtrl* xr = GetXRVessel();
    if (!xr) return;

    // No argument - show status
    if (!arg || arg[0] == '\0') {
        if (xr->GetCtrlAPIVersion() >= 6.0f) {
            printf("Cross-Feed: %s\n", CrossFeedModeStr(xr->GetCrossFeedMode()));
        } else {
            printf("Cross-Feed status requires API 6.0+ (have %.1f)\n", xr->GetCtrlAPIVersion());
            printf("Use 'xf main/off/rcs' to set mode\n");
        }
        return;
    }

    XRXFEED_STATE state;
    if (_stricmp(arg, "main") == 0)
        state = XRXFEED_STATE::XRXF_MAIN;
    else if (_stricmp(arg, "off") == 0)
        state = XRXFEED_STATE::XRXF_OFF;
    else if (_stricmp(arg, "rcs") == 0)
        state = XRXFEED_STATE::XRXF_RCS;
    else {
        printf("Unknown mode: %s\n", arg);
        printf("Options: main, off, rcs\n");
        return;
    }

    if (xr->SetCrossFeedMode(state))
        printf("Cross-Feed: %s\n", CrossFeedModeStr(state));
    else
        printf("Failed to set cross-feed mode\n");
}

#else // !HAS_XRVESSELCTRL

void PrintResupply(const char*) {
    printf("XR vessel support not compiled in\n");
}

void PrintFuelDump(const char*) {
    printf("XR vessel support not compiled in\n");
}

void PrintCrossFeed(const char*) {
    printf("XR vessel support not compiled in\n");
}

#endif
