// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChHydraulicCircuit.h"

namespace chrono {

// ---------------------------------------------------------------------------------------------------------------------

ChHydraulicCylinder::ChHydraulicCylinder() : length_exceeded(false) {
    A.first = CH_C_PI * pistonD * pistonD / 4;
    A.second = A.first - CH_C_PI * rodD * rodD / 4;
}

double2 ChHydraulicCylinder::ComputeChamberLengths(double Delta_s) const {
    return double2(L0.first + Delta_s, L0.second - Delta_s);
}

double2 ChHydraulicCylinder::ComputeChamberVolumes(const double2& L) const {
    return double2(A.first * L.first, A.second * L.second);
}

double ChHydraulicCylinder::EvalForce(const double2& p, double Delta_s, double sd) {
    double c = 1e5;
    double k_end_damper = 1e7;
    double c_end_damper = 5e3;
    double length_end_damper = 0.008;
    auto L = ComputeChamberLengths(Delta_s);

    length_exceeded = false;
    double force_end_damper = 0;

    if (L.first <= length_end_damper) {
        // When l1 is approaching zero, this end damper needs to increase the cylinder force, and,
        // thus it needs to have a positive sign.
        force_end_damper = k_end_damper * (length_end_damper - L.first) - c_end_damper * sd;
        length_exceeded = true;
    } else if (L.second <= length_end_damper) {
        // Here, in turn, a negative force is required.
        force_end_damper = -k_end_damper * (length_end_damper - L.second) - c_end_damper * sd;
        length_exceeded = true;
    }

    return (p.first * A.first - p.second * A.second) - sd * c + force_end_damper;
}

// ---------------------------------------------------------------------------------------------------------------------

ChHydraulicDirectionalValve4x3::ChHydraulicDirectionalValve4x3() {
    Cv = (12.0 / 6e4) / (10 * std::sqrt(35e5));
    time_constant = 1 / (CH_C_2PI * fm45);
}

double ChHydraulicDirectionalValve4x3::EvaluateSpoolPositionRate(double t, double U, double Uref) {
    return (Uref - U) / time_constant;
}

double2 ChHydraulicDirectionalValve4x3::ComputeVolumeFlows(double p1, double p2, double p3, double p4, double U) {
    double QV3 = 0;
    double Q2V = 0;

    if (U >= dead_zone) {  // -------------------------------- p1 & p4, and p2 & p3 connected

        if (std::abs(p1 - p4) < linear_limit) {
            // Linearized model for laminar flow

            // Step 1: Compute flow at the limit between laminar and tubulent models.
            // Assume positive sign.
            double QV3at2bar = Cv * std::abs(U) * std::sqrt(linear_limit);

            // Step 2: Compute the actual flow as a factor of the flow at the limit.
            // Sign of (p1 - p4) defines sign of the volume flow.
            QV3 = QV3at2bar * (p1 - p4) / linear_limit;
        } else {
            // Turbulent volume flow
            QV3 = Cv * U * std::copysign(1.0, p1 - p4) * std::sqrt(std::abs(p1 - p4));
        }

        if (std::abs(p3 - p2) < linear_limit) {
            double Q2Vat2bar = Cv * std::abs(U) * std::sqrt(linear_limit);
            Q2V = ((p3 - p2) / linear_limit) * Q2Vat2bar;
        } else {
            Q2V = Cv * U * std::copysign(1.0, p3 - p2) * std::sqrt(std::abs(p3 - p2));
        }

    } else if (-dead_zone < U && U < dead_zone) {  // -------- valve is shut, no volume flows are allowed

        QV3 = 0;
        Q2V = 0;

    } else {  // (U <= -dead_zone) -------------------------- p1 & p3, and p2 & p4 connected

        if (std::abs(p4 - p2) < linear_limit) {
            double QV3at2bar = -Cv * std::abs(U) * std::sqrt(linear_limit);
            QV3 = QV3at2bar * (p4 - p2) / linear_limit;
        } else {
            QV3 = Cv * U * std::copysign(1.0, p4 - p2) * std::sqrt(std::abs(p4 - p2));
        }

        if (std::abs(p1 - p3) < linear_limit) {
            double Q2Vat2bar = -Cv * std::abs(U) * std::sqrt(linear_limit);
            Q2V = Q2Vat2bar * (p1 - p3) / linear_limit;
        } else {
            Q2V = Cv * U * std::copysign(1.0, p1 - p3) * std::sqrt(std::abs(p1 - p3));
        }
    }

    return double2(QV3, Q2V);
}

// ---------------------------------------------------------------------------------------------------------------------

ChHydraulicThrottleValve::ChHydraulicThrottleValve() {
    double A = CH_C_PI * valveD * valveD / 4;
    Cv = Cd * A * std::sqrt(2 / Do);
}

double ChHydraulicThrottleValve::ComputeVolumeFlow(double p1, double p2) {
    double Q = 0;

    if (std::abs(p1 - p2) > lin_limit) {
        Q = Cv * std::copysign(1.0, p1 - p2) * std::sqrt(std::abs(p1 - p2));
    } else {
        // Linearized model for laminar flow.
        // Compute flow at the limit between laminar and tubulent models. Assume positive sign.
        double Qat2bar = Cv * std::sqrt(lin_limit);
        Q = Qat2bar * (p1 - p2) / lin_limit;
    }

    return Q;
}

// ---------------------------------------------------------------------------------------------------------------------

}  // end namespace chrono
