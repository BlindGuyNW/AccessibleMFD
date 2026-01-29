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
void PrintMFD();
void PrintDock(const char* arg);
void PrintFuel(const char* arg);
void PrintMap(const char* arg);
void PrintSurface(const char* arg);
void PrintAll();

#endif // QUERIES_H
