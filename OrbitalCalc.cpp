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

// Helper: Calculate true anomaly at a given position relative to orbital elements
static double CalcTrueAnomaly(const VECTOR3& pos, const VECTOR3& h, double e, const VECTOR3& periDir) {
    // Project position onto orbital plane
    double r = length(pos);
    if (r < 1e-10) return 0;

    VECTOR3 posUnit = pos / r;

    // True anomaly is angle from periapsis direction
    double cosTA = dotp(posUnit, periDir);
    if (cosTA > 1.0) cosTA = 1.0;
    if (cosTA < -1.0) cosTA = -1.0;

    double ta = acos(cosTA);

    // Determine sign using cross product with orbital angular momentum
    VECTOR3 cross = crossp(periDir, posUnit);
    if (dotp(cross, h) < 0) {
        ta = 2.0 * PI - ta;
    }

    return ta;
}

// Helper: Calculate time from current true anomaly to target true anomaly
static double CalcTimeToTrueAnomaly(double currentTA, double targetTA, double a, double e, double gm) {
    if (a <= 0 || gm <= 0) return -1;

    // Mean motion
    double n = sqrt(gm / (a * a * a));

    // Convert true anomaly to eccentric anomaly
    auto trueToEccentric = [e](double ta) -> double {
        double cosTA = cos(ta);
        double cosE = (e + cosTA) / (1.0 + e * cosTA);
        if (cosE > 1.0) cosE = 1.0;
        if (cosE < -1.0) cosE = -1.0;
        double E = acos(cosE);
        if (ta > PI) E = 2.0 * PI - E;
        return E;
    };

    // Convert eccentric anomaly to mean anomaly
    auto eccentricToMean = [e](double E) -> double {
        return E - e * sin(E);
    };

    double E_current = trueToEccentric(currentTA);
    double E_target = trueToEccentric(targetTA);

    double M_current = eccentricToMean(E_current);
    double M_target = eccentricToMean(E_target);

    // Time difference
    double dM = M_target - M_current;
    if (dM < 0) dM += 2.0 * PI;  // Wrap to positive

    return dM / n;
}

PlaneAlignData CalcPlaneAlign(VESSEL* v, OBJHANDLE hTarget, OBJHANDLE hRef, AlignMode mode) {
    PlaneAlignData pa = {0};
    pa.valid = false;

    if (!v || !hTarget || !hRef) return pa;

    // Get vessel orbital elements
    ELEMENTS el_v;
    ORBITPARAM prm_v;
    if (!v->GetElements(hRef, el_v, &prm_v, 0, FRAME_ECL)) return pa;

    // Skip if vessel is on escape trajectory
    if (el_v.e >= 1.0) return pa;

    // Extract vessel inclination and LAN
    pa.vesselInc = el_v.i * DEG;
    pa.vesselLAN = el_v.theta * DEG;

    // Get vessel position and velocity relative to reference body
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
    if (h_v_mag < 1e-10) return pa;
    VECTOR3 h_v_unit = h_v / h_v_mag;

    // Get target orbital plane
    VECTOR3 targetPos, targetVel;
    oapiGetGlobalPos(hTarget, &targetPos);
    oapiGetGlobalVel(hTarget, &targetVel);

    VECTOR3 relPos_t = targetPos - refPos;
    VECTOR3 relVel_t = targetVel - refVel;

    VECTOR3 h_t = crossp(relPos_t, relVel_t);
    double h_t_mag = length(h_t);
    if (h_t_mag < 1e-10) return pa;
    VECTOR3 h_t_unit = h_t / h_t_mag;

    // Calculate target inclination and LAN from angular momentum vector
    // In ecliptic frame, y-axis is the ecliptic normal
    pa.targetInc = acos(h_t_unit.y) * DEG;

    // LAN is the angle of the ascending node in the ecliptic plane
    // Node vector = ecliptic_normal x orbital_normal = [0,1,0] x h_t
    VECTOR3 eclipticNormal = {0, 1, 0};
    VECTOR3 nodeVec = crossp(eclipticNormal, h_t_unit);
    double nodeVecMag = length(nodeVec);
    if (nodeVecMag > 1e-10) {
        nodeVec = nodeVec / nodeVecMag;
        pa.targetLAN = atan2(nodeVec.z, nodeVec.x) * DEG;
        if (pa.targetLAN < 0) pa.targetLAN += 360.0;
    } else {
        pa.targetLAN = 0;  // Equatorial orbit
    }

    // Relative inclination = angle between orbital plane normals
    double cosRelInc = dotp(h_v_unit, h_t_unit);
    if (cosRelInc > 1.0) cosRelInc = 1.0;
    if (cosRelInc < -1.0) cosRelInc = -1.0;
    pa.relInc = acos(cosRelInc) * DEG;

    // Line of nodes = h_v x h_t (intersection of orbital planes)
    VECTOR3 nodeDir = crossp(h_v_unit, h_t_unit);
    double nodeDirMag = length(nodeDir);

    if (nodeDirMag < 1e-10) {
        // Planes are coplanar
        pa.angleToAN = 0;
        pa.angleToDN = 180.0;
        pa.timeToAN = 0;
        pa.timeToDN = prm_v.T / 2.0;
        pa.burnDV = 0;
        pa.burnTime = 0;
        pa.timeToBurn = 0;
        pa.burnAtAN = true;
        pa.valid = true;
        return pa;
    }

    // Normalize node direction (this points to descending node from vessel's perspective)
    // Ascending node is where vessel crosses from below to above target plane
    VECTOR3 anDir = nodeDir / nodeDirMag;  // Points to ascending node
    VECTOR3 dnDir = anDir * -1.0;          // Points to descending node

    // Calculate angle from current position to each node
    VECTOR3 posUnit = relPos_v / length(relPos_v);

    // Angle to ascending node
    double dotAN = dotp(posUnit, anDir);
    if (dotAN > 1.0) dotAN = 1.0;
    if (dotAN < -1.0) dotAN = -1.0;
    double angleAN = acos(dotAN);

    // Determine direction using cross product with orbital plane normal
    VECTOR3 crossAN = crossp(posUnit, anDir);
    if (dotp(crossAN, h_v_unit) < 0) {
        angleAN = 2.0 * PI - angleAN;
    }
    pa.angleToAN = angleAN * DEG;

    // Angle to descending node (180 degrees away)
    double angleDN = angleAN + PI;
    if (angleDN >= 2.0 * PI) angleDN -= 2.0 * PI;
    pa.angleToDN = angleDN * DEG;

    // Calculate time to each node
    double gm = GetGM(hRef);
    double meanMotion = sqrt(gm / (el_v.a * el_v.a * el_v.a));

    // Approximate time using mean motion and angle (good for near-circular orbits)
    // For more accurate calculation, we'd need to convert through eccentric anomaly
    if (el_v.e < 0.1) {
        // Near-circular approximation
        pa.timeToAN = angleAN / meanMotion;
        pa.timeToDN = angleDN / meanMotion;
    } else {
        // For eccentric orbits, use proper Kepler calculation
        // First find periapsis direction
        VECTOR3 eVec = crossp(relVel_v, h_v) / gm - relPos_v / length(relPos_v);
        double eMag = length(eVec);
        VECTOR3 periDir = (eMag > 1e-10) ? eVec / eMag : relPos_v / length(relPos_v);

        // Current true anomaly
        double currentTA = prm_v.TrA;

        // True anomaly at ascending node
        double dotPeriAN = dotp(periDir, anDir);
        if (dotPeriAN > 1.0) dotPeriAN = 1.0;
        if (dotPeriAN < -1.0) dotPeriAN = -1.0;
        double taAN = acos(dotPeriAN);
        VECTOR3 crossPeriAN = crossp(periDir, anDir);
        if (dotp(crossPeriAN, h_v_unit) < 0) taAN = 2.0 * PI - taAN;

        // True anomaly at descending node
        double taDN = taAN + PI;
        if (taDN >= 2.0 * PI) taDN -= 2.0 * PI;

        pa.timeToAN = CalcTimeToTrueAnomaly(currentTA, taAN, el_v.a, el_v.e, gm);
        pa.timeToDN = CalcTimeToTrueAnomaly(currentTA, taDN, el_v.a, el_v.e, gm);
    }

    // Calculate burn delta-V at each node
    // Velocity at node = sqrt(gm * (2/r - 1/a))
    // For simplicity, use current velocity magnitude
    double v_mag = length(relVel_v);
    double relIncRad = pa.relInc * RAD;
    pa.burnDV = 2.0 * v_mag * sin(relIncRad / 2.0);

    // Select optimal node (whichever comes first, unless one is much more efficient)
    // For now, simply pick the closer node
    if (pa.timeToAN <= pa.timeToDN) {
        pa.burnAtAN = true;
        pa.timeToBurn = pa.timeToAN;
    } else {
        pa.burnAtAN = false;
        pa.timeToBurn = pa.timeToDN;
    }

    // Estimate burn time based on thrust
    double mass = v->GetMass();
    double thrust = v->GetMaxThrust(ENGINE_MAIN);
    if (thrust > 0) {
        // Simple estimate: t = m * dv / F
        pa.burnTime = mass * pa.burnDV / thrust;
        // Adjust time to burn to center the burn on the node
        pa.timeToBurn -= pa.burnTime / 2.0;
        if (pa.timeToBurn < 0) pa.timeToBurn = 0;
    } else {
        pa.burnTime = 0;
    }

    pa.valid = true;
    return pa;
}
