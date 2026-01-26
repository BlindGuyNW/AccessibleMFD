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
#include "orbitersdk.h"
#include "Console.h"
#include "Commands.h"
#include "AutopilotModule.h"
#include "AutopilotController.h"

static HINSTANCE g_hInst = NULL;
static DWORD g_dwCmd = 0;
static AutopilotModule* g_pAutopilotModule = NULL;

// Custom command callback
static void OpenAccessibleConsole(void* context) {
    OpenConsole();
}

// === ORBITER MODULE INTERFACE ===

DLLCLBK void InitModule(HINSTANCE hDLL) {
    g_hInst = hDLL;
    InitCommands();

    // Register custom command in Ctrl+F4 menu
    g_dwCmd = oapiRegisterCustomCmd(
        "Accessible Flight Data",
        "Opens console window for accessible flight data queries",
        OpenAccessibleConsole,
        NULL
    );

    // Register autopilot module for per-frame callbacks
    g_pAutopilotModule = new AutopilotModule(hDLL);
    oapiRegisterModule(g_pAutopilotModule);
}

DLLCLBK void ExitModule(HINSTANCE hDLL) {
    oapiUnregisterCustomCmd(g_dwCmd);

    // Abort any active autopilot before shutdown
    if (g_autopilotController.IsActive()) {
        g_autopilotController.Abort();
    }

    CloseConsole();

    // Note: Orbiter handles module cleanup, but we should clean up our pointer
    // The module will be deleted by Orbiter's module manager
    g_pAutopilotModule = NULL;
}
