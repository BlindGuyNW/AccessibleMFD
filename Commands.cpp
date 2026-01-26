// Commands.cpp - Command registry and dispatch

#include "Commands.h"
#include "Queries.h"
#include "Controls.h"
#include "Transfer.h"
#include <stdio.h>
#include <string.h>

// Command handler function types
typedef void (*SimpleHandler)();
typedef void (*ArgHandler)(const char* arg);

// Command entry structure
struct Command {
    const char* name;       // Primary command name
    const char* alias;      // Short alias (may be NULL)
    bool hasArg;            // Whether this command takes an argument
    SimpleHandler simple;   // Handler for commands without args
    ArgHandler withArg;     // Handler for commands with args
};

// Wrapper functions for simple handlers called through arg interface
static void WrapVessel(const char*) { PrintVessel(); }
static void WrapOrbit(const char*) { PrintOrbit(); }
static void WrapFlight(const char*) { PrintFlight(); }
static void WrapMFD(const char*) { PrintMFD(); }
static void WrapDock(const char*) { PrintDock(); }
static void WrapFuel(const char*) { PrintFuel(); }
static void WrapAll(const char*) { PrintAll(); }
static void WrapHelp(const char*) { PrintHelp(); }

// Command table
static Command s_commands[] = {
    // Data queries (no args)
    {"vessel",   "v",    false, PrintVessel, NULL},
    {"orbit",    "o",    false, PrintOrbit, NULL},
    {"flight",   "f",    false, PrintFlight, NULL},
    {"mfd",      "m",    false, PrintMFD, NULL},
    {"dock",     "d",    false, PrintDock, NULL},
    {"fuel",     NULL,   false, PrintFuel, NULL},
    {"all",      "a",    false, PrintAll, NULL},

    // Data queries (with args)
    {"map",      NULL,   true,  NULL, PrintMap},
    {"surface",  "sf",   true,  NULL, PrintSurface},

    // Control commands (with args)
    {"nav",      "na",   true,  NULL, PrintNav},
    {"throttle", "th",   true,  NULL, PrintThrottle},
    {"warp",     "w",    true,  NULL, PrintWarp},

    // Transfer planner (with args)
    {"target",   "tgt",  true,  NULL, PrintTarget},
    {"transfer", "tr",   true,  NULL, PrintTransfer},

    // Help
    {"help",     "?",    false, PrintHelp, NULL},
};

static const int s_numCommands = sizeof(s_commands) / sizeof(s_commands[0]);

void InitCommands() {
    // Currently no initialization needed, but reserved for future use
}

bool ExecuteCommand(const char* line) {
    // Parse command - extract first word and remainder
    char cmd[32] = "";
    const char* arg = NULL;

    sscanf(line, "%31s", cmd);
    arg = line + strlen(cmd);
    while (*arg == ' ') arg++;  // Skip leading spaces in argument

    // Empty command - do nothing
    if (cmd[0] == '\0') {
        return true;
    }

    // Check for quit command
    if (_stricmp(cmd, "q") == 0 || _stricmp(cmd, "quit") == 0) {
        printf("Closing console...\n");
        return false;
    }

    // Search command table
    for (int i = 0; i < s_numCommands; i++) {
        const Command& c = s_commands[i];
        bool match = (_stricmp(cmd, c.name) == 0) ||
                     (c.alias && _stricmp(cmd, c.alias) == 0);

        if (match) {
            if (c.hasArg) {
                c.withArg(arg);
            } else {
                c.simple();
            }
            return true;
        }
    }

    printf("Unknown command. Type ? for help.\n");
    return true;
}
