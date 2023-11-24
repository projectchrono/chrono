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

ChHydraulicActuatorBase::ChHydraulicActuatorBase()
    : pP(7.6e6), pT(0.1e6), is_attached(false), calculate_consistent_IC(false) {}

void ChHydraulicActuatorBase::SetPressures(double pump_pressure, double tank_pressure) {
    pP = pump_pressure;
    pT = tank_pressure;
}

void ChHydraulicActuatorBase::SetActuatorInitialLength(double len) {
    s_0 = len;
    s = len;
    sd = 0;
}

void ChHydraulicActuatorBase::SetActuatorLength(double len, double vel) {
    // Do nothing if not attached to bodies
    if (is_attached)
        return;

    s = len;
    sd = vel;
}

void ChHydraulicActuatorBase::SetInitialLoad(double F0) {
    this->calculate_consistent_IC = true;
    this->F0 = F0;
}

void ChHydraulicActuatorBase::Initialize() {
    OnInitialize(cyl.p0, cyl.L0, dvalve.U0);

    // Initialize the external dynamics
    ChExternalDynamics::Initialize();
}

void ChHydraulicActuatorBase::Initialize(std::shared_ptr<ChBody> body1,  // first connected body
                                         std::shared_ptr<ChBody> body2,  // second connected body
                                         bool local,                     // true if locations given in body local frames
                                         ChVector<> loc1,                // location of connection point on body 1
                                         ChVector<> loc2                 // location of connection point on body 2
) {
    is_attached = true;

    // Call base initialization
    Initialize();

    // Cache connected bodies and body local connection points
    m_body1 = body1.get();
    m_body2 = body2.get();

    if (local) {
        m_loc1 = loc1;
        m_loc2 = loc2;
        m_aloc1 = body1->TransformPointLocalToParent(loc1);
        m_aloc2 = body2->TransformPointLocalToParent(loc2);
    } else {
        m_loc1 = body1->TransformPointParentToLocal(loc1);
        m_loc2 = body2->TransformPointParentToLocal(loc2);
        m_aloc1 = loc1;
        m_aloc2 = loc2;
    }

    s_0 = (m_aloc1 - m_aloc2).Length();

    // Resize temporary vector of generalized body forces
    m_Qforce.resize(12);
}

double ChHydraulicActuatorBase::GetValvePosition() {
    return ExtractValveSpoolPosition();
}

std::array<double, 2> ChHydraulicActuatorBase::GetCylinderPressures() {
    Vec2 p = ExtractCylinderPressures();
    return {p(0), p(1)};
}

double ChHydraulicActuatorBase::GetActuatorForce() {
    Vec2 p = ExtractCylinderPressures();
    return cyl.EvalForce(p, s - s_0, sd);
}

double ChHydraulicActuatorBase::GetInput(double t) const {
    return ChClamp(ref_fun->Get_y(t), -1.0, +1.0);
}

void ChHydraulicActuatorBase::Update(double time, bool update_assets) {
    // Update the external dynamics
    ChExternalDynamics::Update(time, update_assets);

    // If the actuator is attached to bodies, update its length and rate from the body states
    // and calculate the generated force to the two bodies.
    if (is_attached) {
        m_aloc1 = m_body1->TransformPointLocalToParent(m_loc1);
        m_aloc2 = m_body2->TransformPointLocalToParent(m_loc2);

        auto avel1 = m_body1->PointSpeedLocalToParent(m_loc1);
        auto avel2 = m_body2->PointSpeedLocalToParent(m_loc2);

        ChVector<> dir = (m_aloc1 - m_aloc2).GetNormalized();

        s = (m_aloc1 - m_aloc2).Length();
        sd = Vdot(dir, avel1 - avel2);

        ////std::cout << "time = " << time << "    s=" << s << "   sd=" << sd << std::endl;

        // Actuator force
        auto f = GetActuatorForce();
        ChVector<> force = f * dir;

        // Force and moment acting on body 1
        auto atorque1 = Vcross(m_aloc1 - m_body1->coord.pos, force);         // applied torque (absolute frame)
        auto ltorque1 = m_body1->TransformDirectionParentToLocal(atorque1);  // applied torque (local frame)
        m_Qforce.segment(0, 3) = force.eigen();
        m_Qforce.segment(3, 3) = ltorque1.eigen();

        // Force and moment acting on body 2
        auto atorque2 = Vcross(m_aloc2 - m_body2->coord.pos, -force);        // applied torque (absolute frame)
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

    if (is_attached) {
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

ChHydraulicActuator2::ChHydraulicActuator2() : hose1V(3.14e-5), hose2V(7.85e-5), Bo(1500e6), Bh(150e6), Bc(31500e6) {}

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
    y0(0) = U0;
    y0(1) = pc0(0);
    y0(2) = pc0(1);
}

void ChHydraulicActuator2::CalculateRHS(double time,                 // current time
                                        const ChVectorDynamic<>& y,  // current ODE states
                                        ChVectorDynamic<>& rhs       // output ODE right-hand side vector
) {
    // Extract state
    double U = y(0);
    Vec2 p = y.segment(1, 2);

    // Get input signal
    double Uref = GetInput(time);

    ////std::cout << "      t=" << time << "  U=" << U << "  Uref=" << Uref << std::endl;

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

void ChHydraulicActuator2::OnInitialize(const Vec2& cyl_p0, const Vec2& cyl_L0, double dvalve_U0) {
    pc0 = cyl_p0;
    U0 = dvalve_U0;

    if (!calculate_consistent_IC)
        return;

    // Change pc0 and U0 so that it is consistent with cylinder L0.
    // Solve for U0 and pc0 from the following non-linear system:
    //   A * (p1 - p2) - F0 = 0
    //   dp1/dt = 0
    //   dp2/dt = 0

    //// TODO
    /*
    const auto& Ac = cyl.GetAreas();

    ChMatrixNM<double, 3, 3> dF;
    Vec3 F;

    auto pc0_d = EvaluatePressureRates(0, pc0, U0);
    F(0) = (pc0(0) * Ac(0) - pc0(1) * Ac(1)) - F0;
    F(1) = pc0_d(0);
    F(2) = pc0_d(1);

    dF(0, 0) = Ac(0);
    dF(0, 1) = -Ac(1);
    dF(0, 2) = 0;
    */
}

double ChHydraulicActuator2::ExtractValveSpoolPosition() const {
    const auto& y = GetStates();
    return y(0);
}

Vec2 ChHydraulicActuator2::ExtractCylinderPressures() const {
    const auto& y = GetStates();
    return Vec2(y(1), y(2));
}

Vec2 ChHydraulicActuator2::EvaluatePressureRates(double t, const Vec2& p, double U) {
    const auto& Ac = cyl.GetAreas();
    auto Lc = cyl.ComputeChamberLengths(s - s_0);
    auto Vc = cyl.ComputeChamberVolumes(Lc);

    // Compute volumes
    Vec2 V(hose1V + Vc(0), hose2V + Vc(1));

    // Compute bulk modulus
    double ooBo = 1.0 / Bo;
    double ooBc = 1.0 / Bc;
    double ooBh = 1.0 / Bh;
    double ooBe1 = ooBo + (Vc(0) / V(0)) * ooBc + (hose1V / V(0)) * ooBh;
    double ooBe2 = ooBo + (Vc(1) / V(1)) * ooBc + (hose2V / V(1)) * ooBh;
    Vec2 Be(1.0 / ooBe1, 1.0 / ooBe2);

    // Compute volume flows
    auto Q = dvalve.ComputeVolumeFlows(U, p.segment(0, 2), pP, pT);

    // Compute pressure rates
    Vec2 pd;
    pd(0) = (Be(0) / V(0)) * (Q(0) - Ac(0) * sd);
    pd(1) = (Be(1) / V(1)) * (Ac(1) * sd - Q(1));
    return pd;
}

// ---------------------------------------------------------------------------------------------------------------------

ChHydraulicActuator3::ChHydraulicActuator3()
    : hose1V(3.14e-5), hose2V(7.85e-5), hose3V(4.71e-5), Bo(1500e6), Bh(150e6), Bc(31500e6) {}

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
    y0(0) = U0;
    y0(1) = pc0(0);
    y0(2) = pc0(1);
    y0(3) = 0.0;  //// TODO
}

void ChHydraulicActuator3::CalculateRHS(double time,                 // current time
                                        const ChVectorDynamic<>& y,  // current ODE states
                                        ChVectorDynamic<>& rhs       // output ODE right-hand side vector
) {
    // Extract state
    double U = y(0);
    Vec3 p = y.segment(1, 3);

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

void ChHydraulicActuator3::OnInitialize(const Vec2& cyl_p0, const Vec2& cyl_L0, double dvalve_U0) {
    pc0 = cyl_p0;
    U0 = dvalve_U0;

    if (!calculate_consistent_IC)
        return;

    //// TODO: Change pc0 and U0 so that it is consistent with cylinder L_0
}

double ChHydraulicActuator3::ExtractValveSpoolPosition() const {
    const auto& y = GetStates();
    return y(0);
}

Vec2 ChHydraulicActuator3::ExtractCylinderPressures() const {
    const auto& y = GetStates();
    return Vec2(y(1), y(2));
}

Vec3 ChHydraulicActuator3::EvaluatePressureRates(double t, const Vec3& p, double U) {
    const auto& Ac = cyl.GetAreas();
    auto Lc = cyl.ComputeChamberLengths(s - s_0);
    auto Vc = cyl.ComputeChamberVolumes(Lc);

    // Compute volumes
    Vec3 V(hose1V + Vc(0), hose2V + Vc(1), hose3V);

    // Compute bulk modulus
    double ooBo = 1.0 / Bo;
    double ooBc = 1.0 / Bc;
    double ooBh = 1.0 / Bh;
    double ooBe1 = ooBo + (Vc(0) / V(0)) * ooBc + (hose1V / V(0)) * ooBh;
    double ooBe2 = ooBo + (Vc(1) / V(1)) * ooBc + (hose2V / V(1)) * ooBh;
    double ooBe3 = ooBo + ooBh;
    Vec3 Be(1.0 / ooBe1, 1.0 / ooBe2, 1.0 / ooBe3);

    // Compute volume flows
    auto Q = dvalve.ComputeVolumeFlows(U, p.segment(0, 2), pP, pT);
    double Q31 = tvalve.ComputeVolumeFlow(p(2), p(0));

    // Compute pressure rates
    Vec3 pd;
    pd(0) = (Be(0) / V(0)) * (Q(0) - Ac(0) * sd);
    pd(1) = (Be(1) / V(1)) * (Ac(1) * sd - Q(1));
    pd(2) = (Be(2) / V(2)) * (Q(0) - Q31);
    return pd;
}

// ---------------------------------------------------------------------------------------------------------------------

}  // end namespace chrono
