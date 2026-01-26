// Transfer.h - Transfer planner and help

#ifndef TRANSFER_H
#define TRANSFER_H

// Target selection and display
void PrintTarget(const char* arg);

// Transfer calculations
void PrintHohmann();
void PrintPhase();
void PrintPlane();
void PrintRendezvous();
void PrintTransfer(const char* arg);

// Help display
void PrintHelp();

#endif // TRANSFER_H
