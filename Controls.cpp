// Controls.cpp - Control commands (autopilot, throttle, time warp)

#include "Controls.h"
#include "Queries.h"
#include "orbitersdk.h"
#include <stdio.h>
#include <string.h>

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
