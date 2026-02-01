// Queries.h - Data query functions for vessel, orbit, flight, etc.

#ifndef QUERIES_H
#define QUERIES_H

#include "orbitersdk.h"

// Name helper functions
const char* GetNavmodeName(int mode);
const char* GetEngineName(ENGINETYPE eng);
const char* GetMFDModeName(int mode);

// Data query functions
void PrintVessel();
void PrintOrbit();
void PrintFlight();
void PrintMFD(const char* arg);
void PrintDock(const char* arg);
void PrintFuel(const char* arg);
void PrintMap(const char* arg);
void PrintSurface(const char* arg);
void PrintAll();
void PrintCockpit();
void PrintButtons(const char* arg);
void PressButton(const char* arg);

// XR vessel queries
void PrintDamage(const char* arg);
void PrintTemps(const char* arg);
void PrintReentry(const char* arg);

#endif // QUERIES_H
