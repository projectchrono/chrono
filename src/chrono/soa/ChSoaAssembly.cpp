// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/soa/ChSoaAssembly.h"

namespace chrono {
namespace soa {

ChSoaAssembly::ChSoaAssembly() : m_num_q(0), m_num_u(0), m_num_c(0), m_initialized(false) {
    m_ground_body = chrono_types::make_shared<ChGroundBody>();
}

ChSoaAssembly::~ChSoaAssembly() {}

// -----------------------------------------------------------------------------

void ChSoaAssembly::AddBody(std::shared_ptr<ChSoaMobilizedBody> body) {
    ChAssertAlways(!m_initialized);

    body->m_assembly = this;
    m_bodies.push_back(body);
}

void ChSoaAssembly::RemoveBody(std::shared_ptr<ChSoaMobilizedBody> body) {
    ChAssertAlways(!m_initialized);

    auto itr = std::find(std::begin(m_bodies), std::end(m_bodies), body);
    assert(itr != m_bodies.end());
    m_bodies.erase(itr);
}

std::shared_ptr<ChSoaMobilizedBody> ChSoaAssembly::findBody(const std::string& name) const {
    auto body = std::find_if(std::begin(m_bodies), std::end(m_bodies),
                             [name](std::shared_ptr<ChSoaMobilizedBody> body) { return body->getName() == name; });
    return (body != std::end(m_bodies)) ? *body : nullptr;
}

// -----------------------------------------------------------------------------

void ChSoaAssembly::AddForce(std::shared_ptr<ChSoaForce> force) {
    m_forces.push_back(force);
}

// -----------------------------------------------------------------------------

double ChSoaAssembly::getY(int which) const {
    ////return GetStates()[which];
    return m_y[which];
}

double ChSoaAssembly::getYd(int which) const {
    ////return GetStateDerivatives()[which];
    return m_yd[which];
}

double ChSoaAssembly::getYdd(int which) const {
    return m_ydd[which];
}

void ChSoaAssembly::setY(int which, double val) {
    m_y[which] = val;
}

void ChSoaAssembly::setYd(int which, double val) {
    m_yd[which] = val;
}

void ChSoaAssembly::setYdd(int which, double val) {
    m_ydd(which) = val;
}

// -----------------------------------------------------------------------------

void ChSoaAssembly::Initialize() {
    if (m_initialized)
        return;

    // Traverse all bodies in the system to:
    // - assign their start indices in a state vector in a state derivative vector
    // - accumulate the total numbers of generalized coordinates and velocities.
    for (auto& body : m_bodies) {
        body->m_qIdx = m_num_q;
        body->m_uIdx = m_num_u;

        m_num_q += body->getNumQ();
        m_num_u += body->getNumU();
    }

    // Set size of assembly-wide initial condition vectors
    m_y0.resize(m_num_q);
    m_yd0.resize(m_num_u);

    // Traverse bodies a second time and load initial conditions
    for (auto& body : m_bodies) {
        for (int iq = 0; iq < body->getNumQ(); iq++)
            m_y0(body->m_qIdx + iq) = body->getQ0(iq);
        for (int iu = 0; iu < body->getNumU(); iu++)
            m_yd0(body->m_uIdx + iu) = body->getU0(iu);
    }

    //// TODO: constraints

    // Initialize the assembly as a ChExternalDynamicsDAE
    ChExternalDynamicsDAE::Initialize();

    // Process the initial states
    // - perform forward kinematics
    // - initialize collision
    // - apply forces at initial state
    // - calculate derivatives at initial step
    //// TODO

    calcPosAndVel(m_y0, m_yd0);

    applyForces(m_y0, m_yd0);

    // Mark SOA assembly as initialized
    m_initialized = true;
}

// -----------------------------------------------------------------------------

void ChSoaAssembly::applyForces(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (auto& body : m_bodies) {
        if (body->isGround())
            continue;

        // Gravitational forces
        if (GetSystem())
            body->ApplyGravitationalForce(GetSystem()->GetGravitationalAcceleration());

        // Mobility forces
        body->ApplyAllMobilityForces();
    }

    // External forces
    for (auto& force : m_forces) {
        force->apply();
    }

    // Controller forces
    //// TODO?
}

// -----------------------------------------------------------------------------

void ChSoaAssembly::DoForwardKinematics() {
    calcPosAndVel(m_y, m_yd);
}

void ChSoaAssembly::calcPosAndVel(const ChVectorDynamic<double>& y, const ChVectorDynamic<>& yd) {
    // Calculate the body positions and velocities, as well as any position- and velocity-dependent quantities,
    // in a base-to-tip traversal
    m_ground_body->orProcPosAndVelFD(y, yd);

    // If there are constraints, perform the tip-to-base recursive tree traversal for calculating articulated inertias,
    // then form the constraint matrix (G * Mi * Gt)
    if (m_num_c > 0) {
        m_ground_body->irProcInertiasFD(y, yd);

        calcCSMatrix();
    }
}

void ChSoaAssembly::calcAcc(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd, ChVectorDynamic<>& ydd) {
    // Calculate the body and generalized accelerations for the open-loop system; whether the system is constrained or
    // not, we only process external forces at this point (i.e. the CS forces are all zero here).
    // Note that, if the system is constrained, the tip-to-base recursive tree traversal for calculating articulated
    // inertias was already performed when positions and velocities were calculated at this configuration.
    if (m_num_c > 0)
        m_ground_body->irProcForcesFD(y, yd);
    else
        m_ground_body->irProcInertiasAndForcesFD(y, yd);

    m_ground_body->orProcAccFD(y, yd, ydd);

    // If the system is constrained, calculate the Lagrange multipliers and apply the resulting constraint forces, then
    // calculate the body and generalized accelerations for the closed-loop system (i.e. considering both external and
    // constraint forces).
    if (m_num_c > 0) {
        calcCSForces();

        m_ground_body->irProcForcesFD(y, yd);
        m_ground_body->orProcAccFD(y, yd, ydd);
    }
}

void ChSoaAssembly::calcCSMatrix() {
  //// TODO
}

void ChSoaAssembly::calcCSForces() {
    //// TODO
}

void ChSoaAssembly::calcCSPosJacobian() {
    //// TODO
}

void ChSoaAssembly::calcCSVelJacobian() {
    //// TODO
}



// -----------------------------------------------------------------------------

void ChSoaAssembly::SetInitialConditions(ChVectorDynamic<>& y0, ChVectorDynamic<>& yd0) {
    y0 = m_y0;
    yd0 = m_yd0;
}

// -----------------------------------------------------------------------------

}  // namespace soa
}  // namespace chrono
