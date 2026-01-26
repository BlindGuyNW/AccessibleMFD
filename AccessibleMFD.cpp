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

static HINSTANCE g_hInst = NULL;
static DWORD g_dwCmd = 0;

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
}

DLLCLBK void ExitModule(HINSTANCE hDLL) {
    oapiUnregisterCustomCmd(g_dwCmd);
    CloseConsole();
}
