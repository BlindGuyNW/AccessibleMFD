// Formatting.cpp - Distance, time, and coordinate formatting utilities

#include "Formatting.h"
#include <stdio.h>
#include <math.h>
#include "orbitersdk.h"

void FormatDistance(double meters, char* buf, int len) {
    if (meters >= 1e9)
        _snprintf(buf, len, "%.2f Gm", meters / 1e9);
    else if (meters >= 1e6)
        _snprintf(buf, len, "%.2f Mm", meters / 1e6);
    else if (meters >= 1e3)
        _snprintf(buf, len, "%.2f km", meters / 1e3);
    else
        _snprintf(buf, len, "%.1f m", meters);
    buf[len-1] = '\0';
}

void FormatTime(double seconds, char* buf, int len) {
    if (seconds < 0) {
        _snprintf(buf, len, "N/A");
    } else if (seconds >= 86400) {
        int d = (int)(seconds / 86400);
        int h = (int)((seconds - d * 86400) / 3600);
        int m = (int)((seconds - d * 86400 - h * 3600) / 60);
        _snprintf(buf, len, "%dd %dh %dm", d, h, m);
    } else if (seconds >= 3600) {
        int h = (int)(seconds / 3600);
        int m = (int)((seconds - h * 3600) / 60);
        int s = (int)(seconds - h * 3600 - m * 60);
        _snprintf(buf, len, "%dh %dm %ds", h, m, s);
    } else if (seconds >= 60) {
        int m = (int)(seconds / 60);
        int s = (int)(seconds - m * 60);
        _snprintf(buf, len, "%dm %ds", m, s);
    } else {
        _snprintf(buf, len, "%.1fs", seconds);
    }
    buf[len-1] = '\0';
}

double CalcDistance(double lat1, double lon1, double lat2, double lon2, double radius) {
    // Haversine formula
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return radius * c;
}

double CalcBearing(double lat1, double lon1, double lat2, double lon2) {
    // Initial bearing from point 1 to point 2
    double dLon = lon2 - lon1;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x);
    return posangle(bearing) * DEG;  // Convert to degrees 0-360
}

void FormatLatLon(double lat, double lon, char* buf, int len) {
    // lat/lon are in radians, convert to degrees
    double latDeg = fabs(lat * DEG);
    double lonDeg = fabs(lon * DEG);
    char latDir = (lat >= 0) ? 'N' : 'S';
    char lonDir = (lon >= 0) ? 'E' : 'W';
    _snprintf(buf, len, "%.2f%c%c, %.2f%c%c", latDeg, 0xB0, latDir, lonDeg, 0xB0, lonDir);
    buf[len-1] = '\0';
}

void FormatSpeed(double mps, char* buf, int len) {
    double absMps = fabs(mps);
    if (absMps >= 1000.0)
        _snprintf(buf, len, "%.2f km/s", mps / 1000.0);
    else
        _snprintf(buf, len, "%.1f m/s", mps);
    buf[len-1] = '\0';
}

void FormatForce(double newtons, char* buf, int len) {
    double absN = fabs(newtons);
    if (absN >= 1e6)
        _snprintf(buf, len, "%.2f MN", newtons / 1e6);
    else if (absN >= 1e3)
        _snprintf(buf, len, "%.2f kN", newtons / 1e3);
    else
        _snprintf(buf, len, "%.1f N", newtons);
    buf[len-1] = '\0';
}

void FormatPressure(double pascals, char* buf, int len) {
    double absPa = fabs(pascals);
    double atm = pascals / 101325.0;
    if (fabs(atm) >= 0.01)
        _snprintf(buf, len, "%.3f atm", atm);
    else if (absPa >= 1000.0)
        _snprintf(buf, len, "%.2f kPa", pascals / 1000.0);
    else
        _snprintf(buf, len, "%.1f Pa", pascals);
    buf[len-1] = '\0';
}
