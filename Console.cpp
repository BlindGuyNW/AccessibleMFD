// Console.cpp - Console window management

#include "Console.h"
#include "Commands.h"
#include <windows.h>
#include <stdio.h>
#include <process.h>

// Console state
static HANDLE g_hConsole = NULL;
static HANDLE g_hThread = NULL;
static volatile bool g_bRunning = false;

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

        // Execute command (returns false on quit)
        if (!ExecuteCommand(line)) {
            break;
        }
    }

    return 0;
}

void OpenConsole() {
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

void CloseConsole() {
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
