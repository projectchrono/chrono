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

#include "chrono/physics/ChHydraulicActuator.h"

namespace chrono {

ChHydraulicActuatorBase::ChHydraulicActuatorBase() : m_is_attached(false) {}

void ChHydraulicActuatorBase::Initialize() {
    m_is_attached = false;

    // Initialize the external dynamics
    ChExternalDynamics::Initialize();
}

void ChHydraulicActuatorBase::Initialize(std::shared_ptr<ChBody> body1,  // first connected body
                                         std::shared_ptr<ChBody> body2,  // second connected body
                                         bool local,                     // true if locations given in body local frames
                                         ChVector<> loc1,                // location of connection point on body 1
                                         ChVector<> loc2                 // location of connection point on body 2
) {
    m_is_attached = true;

    // Initialize the external dynamics
    ChExternalDynamics::Initialize();

    // Cache connected bodies and body local connection points
    m_body1 = body1.get();
    m_body2 = body2.get();

    if (local) {
        m_body1_loc = loc1;
        m_body2_loc = loc2;
        auto aloc1 = body1->TransformPointLocalToParent(loc1);
        auto aloc2 = body2->TransformPointLocalToParent(loc2);
        s_0 = (aloc1 - aloc2).Length();
    } else {
        m_body1_loc = body1->TransformPointParentToLocal(loc1);
        m_body2_loc = body2->TransformPointParentToLocal(loc2);
        s_0 = (loc1 - loc2).Length();
    }

    // Resize temporary vector of generalized body forces
    m_Qforce.resize(12);
}

void ChHydraulicActuatorBase::SetActuatorInitialLength(double len_0) {
    s_0 = len_0;
}

void ChHydraulicActuatorBase::SetActuatorLength(double t, double len, double vel) {
    // Do nothing if not attached to bodies
    if (m_is_attached)
        return;

    s = len;
    sd = vel;
}

double ChHydraulicActuatorBase::GetActuatorForce(double t) {
    double2 p = GetCylinderPressure();
    return cyl.EvalForce(p, s - s_0, sd);
}

double ChHydraulicActuatorBase::GetInput(double t) const {
    return ref_fun->Get_y(t);
}

void ChHydraulicActuatorBase::Update(double time, bool update_assets) {
    // Update the external dynamics
    ChExternalDynamics::Update(time, update_assets);

    // If the actuator is attached to bodies, update its length and rate from the body states
    // and calculate the generated force to the two bodies.
    if (m_is_attached) {
        auto aloc1 = m_body1->TransformPointLocalToParent(m_body1_loc);
        auto aloc2 = m_body2->TransformPointLocalToParent(m_body2_loc);

        auto avel1 = m_body1->PointSpeedLocalToParent(m_body1_loc);
        auto avel2 = m_body2->PointSpeedLocalToParent(m_body2_loc);

        ChVector<> dir = (aloc1 - aloc2).GetNormalized();

        s = (aloc1 - aloc2).Length();
        sd = Vdot(dir, avel1 - avel2);

        // Actuator force
        auto f = GetActuatorForce(time);
        ChVector<> force = f * dir;

        // Force and moment acting on body 1
        auto atorque1 = Vcross(aloc1 - m_body1->coord.pos, force);           // applied torque (absolute frame)
        auto ltorque1 = m_body1->TransformDirectionParentToLocal(atorque1);  // applied torque (local frame)
        m_Qforce.segment(0, 3) = force.eigen();
        m_Qforce.segment(3, 3) = ltorque1.eigen();

        // Force and moment acting on body 2
        auto atorque2 = Vcross(aloc2 - m_body2->coord.pos, -force);          // applied torque (absolute frame)
        auto ltorque2 = m_body2->TransformDirectionParentToLocal(atorque2);  // applied torque (local frame)
        m_Qforce.segment(6, 3) = -force.eigen();
        m_Qforce.segment(9, 3) = ltorque2.eigen();
    }
}

void ChHydraulicActuatorBase::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    if (!IsActive())
        return;

    // Load external dynamics
    ChExternalDynamics::IntLoadResidual_F(off, R, c);

    if (m_is_attached) {
        // Add forces to connected bodies (calculated in Update)
        if (m_body1->Variables().IsActive()) {
            R.segment(m_body1->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(0, 3);
            R.segment(m_body1->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(3, 3);
        }
        if (m_body2->Variables().IsActive()) {
            R.segment(m_body2->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(6, 3);
            R.segment(m_body2->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(9, 3);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void ChHydraulicActuator2::SetPressures(double pump_pressure, double tank_pressure) {
    pP = pump_pressure;
    pT = tank_pressure;
}

void ChHydraulicActuator2::SetBulkModuli(double oil_bulk_modulus, double hose_bulk_modulus, double cyl_bulk_modulus) {
    Bo = oil_bulk_modulus;
    Bh = hose_bulk_modulus;
    Bc = cyl_bulk_modulus;
}

void ChHydraulicActuator2::SetHoseVolumes(double hose_dvalve_piston, double hose_dvalve_rod) {
    hose1V = hose_dvalve_piston;
    hose2V = hose_dvalve_rod;
}

void ChHydraulicActuator2::SetInitialConditions(ChVectorDynamic<>& y0) {
    y0(0) = 0.0;  // valve initially shut
    y0(1) = 0.0;
    y0(2) = 0.0;
}

void ChHydraulicActuator2::CalculateRHS(double time,                 // current time
                                        const ChVectorDynamic<>& y,  // current ODE states
                                        ChVectorDynamic<>& rhs       // output ODE right-hand side vector
) {
    // Extract state
    double U = y(0);
    Vec2 p;
    p(0) = y(1);
    p(1) = y(2);

    // Get input signal
    double Uref = GetInput(time);

    // Evaluate state derivatives
    auto Ud = dvalve.EvaluateSpoolPositionRate(time, U, Uref);
    auto pd = EvaluatePressureRates(time, p, U);

    // Load right-hand side
    rhs(0) = Ud;
    rhs(1) = pd(0);
    rhs(2) = pd(1);
}

bool ChHydraulicActuator2::CalculateJac(double time,                   // current time
                                        const ChVectorDynamic<>& y,    // current ODE states
                                        const ChVectorDynamic<>& rhs,  // current ODE right-hand side vector
                                        ChMatrixDynamic<>& J           // output Jacobian matrix
) {
    // Do not provide Jacobian information if problem not stiff
    if (!IsStiff())
        return false;

    //// TODO: analytical Jacobian
    return false;
}

double2 ChHydraulicActuator2::GetCylinderPressure() const {
    const auto& y = GetStates();
    return double2(y(1), y(2));
}

ChHydraulicActuator2::Vec2 ChHydraulicActuator2::EvaluatePressureRates(double t, const Vec2& p, double U) {
    const auto& Ac = cyl.GetAreas();
    auto Lc = cyl.ComputeChamberLengths(s - s_0);
    auto Vc = cyl.ComputeChamberVolumes(Lc);

    // Compute volumes
    Vec2 V(hose1V + Vc.first, hose2V + Vc.second);

    // Compute bulk modulus
    double ooBo = 1.0 / Bo;
    double ooBc = 1.0 / Bc;
    double ooBh = 1.0 / Bh;
    double ooBe1 = ooBo + (Vc.first / V(0)) * ooBc + (hose1V / V(0)) * ooBh;
    double ooBe2 = ooBo + (Vc.second / V(1)) * ooBc + (hose2V / V(1)) * ooBh;
    Vec2 Be(1.0 / ooBe1, 1.0 / ooBe2);

    // Compute volume flows
    auto Q = dvalve.ComputeVolumeFlows(pP, pT, p(1), p(0), U);

    // Compute pressure rates
    Vec2 pd;
    pd(0) = (Be(0) / V(0)) * (Q.first - Ac.first * sd);
    pd(1) = (Be(1) / V(1)) * (Ac.second * sd - Q.second);
    return pd;
}

// ---------------------------------------------------------------------------------------------------------------------

void ChHydraulicActuator3::SetPressures(double pump_pressure, double tank_pressure) {
    pP = pump_pressure;
    pT = tank_pressure;
}

void ChHydraulicActuator3::SetBulkModuli(double oil_bulk_modulus, double hose_bulk_modulus, double cyl_bulk_modulus) {
    Bo = oil_bulk_modulus;
    Bh = hose_bulk_modulus;
    Bc = cyl_bulk_modulus;
}

void ChHydraulicActuator3::SetHoseVolumes(double hose_tvalve_piston,
                                          double hose_dvalve_rod,
                                          double hose_dvalve_tvalve) {
    hose1V = hose_tvalve_piston;
    hose2V = hose_dvalve_rod;
    hose3V = hose_dvalve_tvalve;
}

void ChHydraulicActuator3::SetInitialConditions(ChVectorDynamic<>& y0) {
    y0(0) = 0.0;  // valve initially shut
    y0(1) = 0.0;
    y0(2) = 0.0;
    y0(3) = 0.0;
}

void ChHydraulicActuator3::CalculateRHS(double time,                 // current time
                                        const ChVectorDynamic<>& y,  // current ODE states
                                        ChVectorDynamic<>& rhs       // output ODE right-hand side vector
) {
    // Extract state
    double U = y(0);
    Vec3 p;
    p(0) = y(1);
    p(1) = y(2);
    p(2) = y(3);

    // Get input signal
    double Uref = GetInput(time);

    // Evaluate state derivatives
    auto Ud = dvalve.EvaluateSpoolPositionRate(time, U, Uref);
    auto pd = EvaluatePressureRates(time, p, U);

    // Load right-hand side
    rhs(0) = Ud;
    rhs(1) = pd(0);
    rhs(2) = pd(1);
    rhs(3) = pd(2);
}

bool ChHydraulicActuator3::CalculateJac(double time,                   // current time
                                        const ChVectorDynamic<>& y,    // current ODE states
                                        const ChVectorDynamic<>& rhs,  // current ODE right-hand side vector
                                        ChMatrixDynamic<>& J           // output Jacobian matrix
) {
    // Do not provide Jacobian information if problem not stiff
    if (!IsStiff())
        return false;

    //// TODO: analytical Jacobian
    return false;
}

double2 ChHydraulicActuator3::GetCylinderPressure() const {
    const auto& y = GetStates();
    return double2(y(1), y(2));
}

ChHydraulicActuator3::Vec3 ChHydraulicActuator3::EvaluatePressureRates(double t, const Vec3& p, double U) {
    const auto& Ac = cyl.GetAreas();
    auto Lc = cyl.ComputeChamberLengths(s - s_0);
    auto Vc = cyl.ComputeChamberVolumes(Lc);

    // Compute volumes
    Vec3 V(hose1V + Vc.first, hose2V + Vc.second, hose3V);

    // Compute bulk modulus
    double ooBo = 1.0 / Bo;
    double ooBc = 1.0 / Bc;
    double ooBh = 1.0 / Bh;
    double ooBe1 = ooBo + (Vc.first / V(0)) * ooBc + (hose1V / V(0)) * ooBh;
    double ooBe2 = ooBo + (Vc.second / V(1)) * ooBc + (hose2V / V(1)) * ooBh;
    double ooBe3 = ooBo + ooBh;
    Vec3 Be(1.0 / ooBe1, 1.0 / ooBe2, 1.0 / ooBe3);

    // Compute volume flows
    auto Q = dvalve.ComputeVolumeFlows(pP, pT, p(1), p(0), U);
    double Q31 = tvalve.ComputeVolumeFlow(p(2), p(0));

    // Compute pressure rates
    Vec3 pd;
    pd(0) = (Be(0) / V(0)) * (Q.first - Ac.first * sd);
    pd(1) = (Be(1) / V(1)) * (Ac.second * sd - Q.second);
    pd(2) = (Be(2) / V(2)) * (Q.first - Q31);
    return pd;
}

// ---------------------------------------------------------------------------------------------------------------------

}  // end namespace chrono
