// Commands.h - Command registry and dispatch

#ifndef COMMANDS_H
#define COMMANDS_H

// Initialize the command registry
void InitCommands();

// Execute a command line (returns true if command was found, false if quit)
bool ExecuteCommand(const char* line);

#endif // COMMANDS_H
