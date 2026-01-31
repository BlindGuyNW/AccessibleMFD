// Autopilot.cpp - Launch command handler implementation

#include "Autopilot.h"
#include "AutopilotController.h"
#include <stdio.h>
#include <string.h>

void PrintLaunch(const char* arg) {
    // No argument - show status
    if (!arg || arg[0] == '\0') {
        char statusBuf[256];
        g_autopilotController.GetStatusText(statusBuf, sizeof(statusBuf));
        printf("%s\n", statusBuf);

        if (!g_autopilotController.IsActive()) {
            printf("\nUsage:\n");
            printf("  launch <altitude> [azimuth]  - Launch to orbit\n");
            printf("  launch abort                 - Abort autopilot\n");
            printf("\nExamples:\n");
            printf("  launch 200      - 200 km orbit, heading 90 (east)\n");
            printf("  launch 200 42   - 200 km orbit, heading 42 deg\n");
            printf("  launch 300 270  - 300 km orbit, heading 270 (west)\n");
            printf("\nAzimuth: 0=north, 90=east, 180=south, 270=west\n");
        }
        return;
    }

    // Check for abort command
    if (_stricmp(arg, "abort") == 0 || _stricmp(arg, "off") == 0) {
        g_autopilotController.Abort();
        return;
    }

    // Check for status command
    if (_stricmp(arg, "status") == 0 || _stricmp(arg, "s") == 0) {
        char statusBuf[256];
        g_autopilotController.GetStatusText(statusBuf, sizeof(statusBuf));
        printf("%s\n", statusBuf);
        return;
    }

    // Parse altitude (supports formats: "200", "200k", "200km") and optional azimuth
    double altitude = 0;
    char suffix[16] = "";

    int parsed = sscanf(arg, "%lf%15s", &altitude, suffix);
    if (parsed < 1 || altitude <= 0) {
        printf("Invalid altitude. Usage: launch <altitude_km> [azimuth]\n");
        printf("Examples: launch 200, launch 200 42\n");
        return;
    }

    // Handle suffix (k or km means kilometers, which is default anyway)
    // If user types "200000" (meters), warn them
    if (altitude > 1000 && suffix[0] == '\0') {
        printf("Note: Altitude should be in km. Did you mean %.0f km?\n", altitude / 1000);
        printf("If you want %.0f km, type: launch %.0f\n", altitude, altitude);
        return;
    }

    // Parse optional azimuth: scan past the first token to find a second number
    double azimuth = 90.0;  // Default: east
    const char* p = arg;
    // Skip leading whitespace
    while (*p == ' ' || *p == '\t') p++;
    // Skip first token (altitude + optional suffix)
    while (*p && *p != ' ' && *p != '\t') p++;
    // Skip whitespace between tokens
    while (*p == ' ' || *p == '\t') p++;
    // If there's more text, try to parse azimuth
    if (*p) {
        if (sscanf(p, "%lf", &azimuth) != 1) {
            printf("Invalid azimuth. Usage: launch <altitude_km> [azimuth]\n");
            return;
        }
    }

    // Start launch
    g_autopilotController.StartLaunch(altitude, azimuth);
}
