// Formatting.h - Distance, time, and coordinate formatting utilities

#ifndef FORMATTING_H
#define FORMATTING_H

// Format a distance value with appropriate units (m, km, Mm, Gm)
void FormatDistance(double meters, char* buf, int len);

// Format a time value with appropriate units (s, m, h, d)
void FormatTime(double seconds, char* buf, int len);

// Format latitude/longitude in degrees with direction (N/S, E/W)
void FormatLatLon(double lat, double lon, char* buf, int len);

// Calculate great circle distance using Haversine formula
double CalcDistance(double lat1, double lon1, double lat2, double lon2, double radius);

// Calculate initial bearing from point 1 to point 2
double CalcBearing(double lat1, double lon1, double lat2, double lon2);

#endif // FORMATTING_H
