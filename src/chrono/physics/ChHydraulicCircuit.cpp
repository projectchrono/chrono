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

ChHydraulicCylinder::ChHydraulicCylinder()
    : pistonD(0.08), rodD(0.035), p0({3.3e6, 4.4e6}), L0({0.15, 0.15}), length_exceeded(false) {
    pistonL = L0(0) + L0(1);
    A(0) = CH_C_PI * pistonD * pistonD / 4;
    A(1) = A(0) - CH_C_PI * rodD * rodD / 4;
}

void ChHydraulicCylinder::SetDimensions(double piston_diameter, double rod_diameter) {
    pistonD = piston_diameter;
    rodD = rod_diameter;

    A(0) = CH_C_PI * pistonD * pistonD / 4;
    A(1) = A(0) - CH_C_PI * rodD * rodD / 4;
}

void ChHydraulicCylinder::SetInitialChamberLengths(double piston_side, double rod_side) {
    L0(0) = piston_side;
    L0(1) = rod_side;
    pistonL = piston_side + rod_side;
}

void ChHydraulicCylinder::SetInitialChamberPressures(double pison_side, double rod_side) {
    p0(0) = pison_side;
    p0(1) = rod_side;
}

Vec2 ChHydraulicCylinder::ComputeChamberLengths(double Delta_s) const {
    return Vec2(L0(0) + Delta_s, L0(1) - Delta_s);
}

Vec2 ChHydraulicCylinder::ComputeChamberVolumes(const Vec2& L) const {
    return Vec2(A(0) * L(0), A(1) * L(1));
}

double ChHydraulicCylinder::EvalForce(const Vec2& p, double Delta_s, double sd) {
    double c = 1e5;
    double k_end_damper = 1e7;
    double c_end_damper = 5e3;
    double length_end_damper = 0.008;
    auto L = ComputeChamberLengths(Delta_s);

    length_exceeded = false;
    double force_end_damper = 0;

    if (L(0) <= length_end_damper) {
        // When l(0) is approaching zero, this end damper needs to increase the cylinder force, and,
        // thus it needs to have a positive sign.
        force_end_damper = k_end_damper * (length_end_damper - L(0)) - c_end_damper * sd;
        length_exceeded = true;
    } else if (L(1) <= length_end_damper) {
        // Here, in turn, a negative force is required.
        force_end_damper = -k_end_damper * (length_end_damper - L(1)) - c_end_damper * sd;
        length_exceeded = true;
    }

    return (p(0) * A(0) - p(1) * A(1)) - sd * c + force_end_damper;
}

// ---------------------------------------------------------------------------------------------------------------------

ChHydraulicDirectionalValve4x3::ChHydraulicDirectionalValve4x3() : linear_limit(2e5), dead_zone(0.1e-5), U0(0) {
    // Flow rate constant corresponding to a nominal flow of 24 l/min with a pressure difference of 35 bar
    Cv = (24e-3 / 60) / (1.0 * std::sqrt(35e5));
    // Valve time constant corresponding to a -45 degree phase shift frequency of 35 Hz
    time_constant = 1 / (CH_C_2PI * 35);
}

void ChHydraulicDirectionalValve4x3::SetCharacteristicParameters(double linear_limit, double Q, double dp) {
    this->linear_limit = linear_limit;
    Cv = Q / std::sqrt(dp);
}

void ChHydraulicDirectionalValve4x3::SetTimeConstantFrequency(double fm45) {
    time_constant = 1 / (CH_C_2PI * fm45);
}

void ChHydraulicDirectionalValve4x3::SetValveDeadZone(double dead_zone) {
    this->dead_zone = dead_zone;
}

void ChHydraulicDirectionalValve4x3::SetInitialSpoolPosition(double U) {
    U0 = U;
}

double ChHydraulicDirectionalValve4x3::EvaluateSpoolPositionRate(double t, double U, double Uref) {
    return (Uref - U) / time_constant;
}

Vec2 ChHydraulicDirectionalValve4x3::ComputeVolumeFlows(double U, const Vec2& p, double pP, double pT) {
    double Q0 = 0;
    double Q1 = 0;

    // In the linear regime:
    //    1. Compute flow at the limit between laminar and tubulent models.
    //       Assume positive sign.
    //    2. Compute the actual flow as a factor of the flow at the limit.
    //       Sign of Delta_p defines sign of the volume flow.
    // In the turbulent regime:
    //    1. Use quadratic characteristic. 
    //       Sign of Delta_p defines sign of the volume flow.

    if (U >= dead_zone) {
        // pP <-> p(0) and pT <-> p(1) connected
        if (std::abs(pP - p(0)) < linear_limit) {
            double Q0limit = Cv * std::abs(U) * std::sqrt(linear_limit);
            Q0 = Q0limit * (pP - p(0)) / linear_limit;
        } else {
            Q0 = Cv * U * std::copysign(1.0, pP - p(0)) * std::sqrt(std::abs(pP - p(0)));
        }

        if (std::abs(p(1) - pT) < linear_limit) {
            double Q1limit = Cv * std::abs(U) * std::sqrt(linear_limit);
            Q1 = ((p(1) - pT) / linear_limit) * Q1limit;
        } else {
            Q1 = Cv * U * std::copysign(1.0, p(1) - pT) * std::sqrt(std::abs(p(1) - pT));
        }
    } else if (U <= -dead_zone) {
        // pP <-> p(1) and pT <-> p(0) connected
        if (std::abs(p(0) - pT) < linear_limit) {
            double Q0limit = -Cv * std::abs(U) * std::sqrt(linear_limit);
            Q0 = Q0limit * (p(0) - pT) / linear_limit;
        } else {
            Q0 = Cv * U * std::copysign(1.0, p(0) - pT) * std::sqrt(std::abs(p(0) - pT));
        }

        if (std::abs(pP - p(1)) < linear_limit) {
            double Q1limit = -Cv * std::abs(U) * std::sqrt(linear_limit);
            Q1 = Q1limit * (pP - p(1)) / linear_limit;
        } else {
            Q1 = Cv * U * std::copysign(1.0, pP - p(1)) * std::sqrt(std::abs(pP - p(1)));
        }
    } else {
        // valve is shut, no volume flows are allowed
        Q0 = 0;
        Q1 = 0;
    }

    return Vec2(Q0, Q1);
}

// ---------------------------------------------------------------------------------------------------------------------

ChHydraulicThrottleValve::ChHydraulicThrottleValve() : valveD(0.006), Do(850), linear_limit(2e5), Cd(0.8) {
    double A = CH_C_PI * valveD * valveD / 4;
    Cv = Cd * A * std::sqrt(2 / Do);
}

void ChHydraulicThrottleValve::SetParameters(double valve_diameter,
                                             double oil_density,
                                             double linear_limit,
                                             double Cd) {
    this->valveD = valve_diameter;
    this->Do = oil_density;
    this->linear_limit = linear_limit;
    this->Cd = Cd;

    double A = CH_C_PI * valveD * valveD / 4;
    Cv = Cd * A * std::sqrt(2 / Do);
}

double ChHydraulicThrottleValve::ComputeVolumeFlow(double p1, double p2) {
    double Q = 0;

    if (std::abs(p1 - p2) > linear_limit) {
        Q = Cv * std::copysign(1.0, p1 - p2) * std::sqrt(std::abs(p1 - p2));
    } else {
        // Linearized model for laminar flow.
        // Compute flow at the limit between laminar and tubulent models. Assume positive sign.
        double Qat2bar = Cv * std::sqrt(linear_limit);
        Q = Qat2bar * (p1 - p2) / linear_limit;
    }

    return Q;
}

// ---------------------------------------------------------------------------------------------------------------------

}  // end namespace chrono
