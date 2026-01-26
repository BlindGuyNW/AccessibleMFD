// OrbitalCalc.cpp - Orbital mechanics calculations

#include "OrbitalCalc.h"
#include <math.h>

const double G_CONST = 6.67430e-11;  // m^3 kg^-1 s^-2

double GetGM(OBJHANDLE hBody) {
    double mass = oapiGetMass(hBody);
    return G_CONST * mass;
}

HohmannTransfer CalcHohmann(double r1, double r2, double gm) {
    HohmannTransfer ht = {0, 0, 0, 0, false};

    if (r1 <= 0 || r2 <= 0 || gm <= 0) return ht;

    double a_t = (r1 + r2) / 2.0;  // Transfer orbit semi-major axis

    // Circular velocity at r1
    double v1_circ = sqrt(gm / r1);

    // Transfer orbit velocity at r1 (periapsis if r2 > r1, apoapsis if r2 < r1)
    double v1_trans = sqrt(gm * (2.0 / r1 - 1.0 / a_t));

    // Circular velocity at r2
    double v2_circ = sqrt(gm / r2);

    // Transfer orbit velocity at r2
    double v2_trans = sqrt(gm * (2.0 / r2 - 1.0 / a_t));

    // Delta-V calculations
    if (r2 > r1) {
        // Raising orbit: burn prograde at r1, then prograde at r2
        ht.departDV = v1_trans - v1_circ;
        ht.arriveDV = v2_circ - v2_trans;
    } else {
        // Lowering orbit: burn retrograde at r1, then retrograde at r2
        ht.departDV = v1_circ - v1_trans;
        ht.arriveDV = v2_trans - v2_circ;
    }

    ht.totalDV = fabs(ht.departDV) + fabs(ht.arriveDV);

    // Transfer time = half the transfer orbit period
    ht.transferTime = PI * sqrt(a_t * a_t * a_t / gm);

    ht.valid = true;
    return ht;
}

PhaseAngleData CalcPhaseAngle(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef) {
    PhaseAngleData pa = {0, 0, 0, 0, false};

    if (!v || !hTarget || !hRef) return pa;

    // Get vessel orbital elements
    ELEMENTS el_v;
    ORBITPARAM prm_v;
    if (!v->GetElements(hRef, el_v, &prm_v, 0, FRAME_ECL)) return pa;

    // Skip if vessel is on escape trajectory
    if (el_v.e >= 1.0) return pa;

    // Get vessel's true longitude (angle from reference direction)
    double vesselTrL = prm_v.TrL;  // True longitude in radians

    // Get target position
    VECTOR3 targetPos, refPos;
    oapiGetGlobalPos(hTarget, &targetPos);
    oapiGetGlobalPos(hRef, &refPos);
    VECTOR3 relTargetPos = targetPos - refPos;

    // Calculate target's angular position (in ecliptic plane)
    double targetAngle = atan2(relTargetPos.z, relTargetPos.x);
    if (targetAngle < 0) targetAngle += 2.0 * PI;

    // Current phase angle (target ahead of vessel is positive)
    double currentPhase = targetAngle - vesselTrL;
    while (currentPhase < 0) currentPhase += 2.0 * PI;
    while (currentPhase >= 2.0 * PI) currentPhase -= 2.0 * PI;

    pa.currentPhase = currentPhase * DEG;

    // Calculate required phase angle for Hohmann transfer
    double gm = GetGM(hRef);
    double r1 = el_v.a * (1.0 - el_v.e * el_v.e) / (1.0 + el_v.e * cos(prm_v.TrA));  // Current radius
    double r2 = length(relTargetPos);  // Target radius (assuming circular)

    // Guard against zero/near-zero target radius (e.g., targeting reference body)
    if (r2 < 1000.0) return pa;  // Less than 1km is invalid

    double a_t = (r1 + r2) / 2.0;
    double transferTime = PI * sqrt(a_t * a_t * a_t / gm);

    // Target angular velocity (assuming circular orbit)
    double targetPeriod = 2.0 * PI * sqrt(r2 * r2 * r2 / gm);
    if (targetPeriod < 1.0) return pa;  // Guard against division by zero
    double targetAngVel = 2.0 * PI / targetPeriod;  // rad/s

    // Required phase = 180 degrees minus angle target travels during transfer
    double requiredPhase = PI - targetAngVel * transferTime;
    while (requiredPhase < 0) requiredPhase += 2.0 * PI;

    pa.requiredPhase = requiredPhase * DEG;

    // Phase difference and time to window
    double phaseDiff = requiredPhase - currentPhase * RAD / DEG * RAD;  // Convert back properly
    phaseDiff = (requiredPhase * RAD) - currentPhase * RAD;

    // Synodic period calculation
    double vesselAngVel = 2.0 * PI / prm_v.T;
    double synodicRate = fabs(vesselAngVel - targetAngVel);

    if (synodicRate > 1e-10) {
        // Normalize phase difference
        double diff = (pa.requiredPhase - pa.currentPhase) * RAD;
        while (diff < 0) diff += 2.0 * PI;
        while (diff >= 2.0 * PI) diff -= 2.0 * PI;

        pa.phaseDiff = diff * DEG;
        pa.timeToWindow = diff / synodicRate;
    } else {
        pa.phaseDiff = 0;
        pa.timeToWindow = -1;  // Same orbital period, window always exists
    }

    pa.valid = true;
    return pa;
}

PlaneChangeData CalcPlaneChange(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef) {
    PlaneChangeData pc = {0, 0, 0, false};

    if (!v || !hTarget || !hRef) return pc;

    // Get vessel orbital elements
    ELEMENTS el_v;
    ORBITPARAM prm_v;
    if (!v->GetElements(hRef, el_v, &prm_v, 0, FRAME_ECL)) return pc;

    // Get vessel position and velocity to compute orbital plane normal
    VECTOR3 vesselPos, vesselVel, refPos, refVel;
    v->GetGlobalPos(vesselPos);
    v->GetGlobalVel(vesselVel);
    oapiGetGlobalPos(hRef, &refPos);
    oapiGetGlobalVel(hRef, &refVel);

    VECTOR3 relPos_v = vesselPos - refPos;
    VECTOR3 relVel_v = vesselVel - refVel;

    // Vessel orbital plane normal: h = r x v
    VECTOR3 h_v = crossp(relPos_v, relVel_v);
    double h_v_mag = length(h_v);
    if (h_v_mag < 1e-10) return pc;
    h_v = h_v / h_v_mag;  // Normalize

    // Get target orbital plane normal
    VECTOR3 targetPos, targetVel;
    oapiGetGlobalPos(hTarget, &targetPos);
    oapiGetGlobalVel(hTarget, &targetVel);

    VECTOR3 relPos_t = targetPos - refPos;
    VECTOR3 relVel_t = targetVel - refVel;

    VECTOR3 h_t = crossp(relPos_t, relVel_t);
    double h_t_mag = length(h_t);
    if (h_t_mag < 1e-10) return pc;
    h_t = h_t / h_t_mag;  // Normalize

    // Relative inclination = angle between orbital plane normals
    double cosInc = dotp(h_v, h_t);
    if (cosInc > 1.0) cosInc = 1.0;
    if (cosInc < -1.0) cosInc = -1.0;
    double relInc = acos(cosInc);

    pc.relInc = relInc * DEG;

    // Calculate plane change delta-V (at current velocity)
    // DV = 2 * v * sin(inc/2)
    double v_mag = length(relVel_v);
    pc.planeChangeDV = 2.0 * v_mag * sin(relInc / 2.0);

    // Calculate angle to ascending node
    // Line of nodes = h_v x h_t (intersection of orbital planes)
    VECTOR3 nodeDir = crossp(h_v, h_t);
    double nodeDir_mag = length(nodeDir);
    if (nodeDir_mag > 1e-10) {
        nodeDir = nodeDir / nodeDir_mag;

        // Angle from current position to ascending node
        double dotNode = dotp(relPos_v / length(relPos_v), nodeDir);
        if (dotNode > 1.0) dotNode = 1.0;
        if (dotNode < -1.0) dotNode = -1.0;
        pc.nodeAngle = acos(dotNode) * DEG;
    } else {
        pc.nodeAngle = 0;  // Planes are coplanar
    }

    pc.valid = true;
    return pc;
}

RendezvousData CalcRendezvous(VESSEL* v, OBJHANDLE hTarget) {
    RendezvousData rd = {0, 0, 0, {0,0,0}, 0, false};

    if (!v || !hTarget) return rd;

    // Get relative position and velocity
    VECTOR3 relPos, relVel;
    v->GetRelativePos(hTarget, relPos);
    v->GetRelativeVel(hTarget, relVel);

    rd.distance = length(relPos);
    rd.relVelMag = length(relVel);

    // Closure rate (positive = approaching)
    if (rd.distance > 1e-10) {
        rd.closureRate = -dotp(relVel, relPos) / rd.distance;
    }

    // Time to closest approach at current rate
    if (rd.closureRate > 1e-10) {
        rd.timeToClose = rd.distance / rd.closureRate;
    } else {
        rd.timeToClose = -1;  // Not approaching
    }

    // Convert relative velocity to vessel local frame
    MATRIX3 rot;
    v->GetRotationMatrix(rot);
    rd.relVel = tmul(rot, relVel);

    rd.valid = true;
    return rd;
}
