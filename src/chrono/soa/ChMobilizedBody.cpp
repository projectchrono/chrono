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

#include "chrono/soa/ChSoaAssembly.h"
#include "chrono/soa/ChMobilizedBody.h"

namespace chrono {
namespace soa {

// Initialize Static Variables
const double ChMobilizedBody::m_angleClampLimit = 10 * CH_2PI;

// =============================================================================

ChMobilizedBody::ChMobilizedBody(std::shared_ptr<ChMobilizedBody> parent,
                                 const ChMassProps& mpropsB,
                                 const ChFramed& X_PF,
                                 const ChFramed& X_BM,
                                 const std::string& name)
    : m_mpropsB(mpropsB),
      m_parent(parent),
      m_X_PF(X_PF),
      m_X_BM(X_BM),
      m_name(name),
      m_locked(false),
      m_assembly(nullptr) {
    if (m_parent)
        m_parent->m_children.push_back(this);
}

ChMobilizedBody::ChMobilizedBody(const ChMobilizedBody& other) : ChObj(other) {
    //// TODO
}

ChMobilizedBody::~ChMobilizedBody() {
    if (m_parent) {
        auto itr = std::find(std::begin(m_parent->m_children), std::end(m_parent->m_children), this);
        assert(itr != m_parent->m_children.end());
        m_parent->m_children.erase(itr);
    }
}

void ChMobilizedBody::lock(bool val) {
    //// TODO -- do I need this?
    ////if (val) {
    ////    for (UINT i = 0; i < getNumU(); i++)
    ////        getSystem()->y0()[m_uIdx + i] = 0;
    ////}

    m_locked = val;
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::AddMobilityForce(int dof, std::shared_ptr<ChMobilityForce> force) {
    assert(0 <= dof && dof < getNumU());
    m_mobility_forces.push_back({dof, force});
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::ApplyBodyForce(const ChSpatialVec& force) {
    m_bodyForce += force;
}

void ChMobilizedBody::ApplyGravitationalForce(const ChVector3d& g) {
    ChVector3d frc_G = m_mpropsB.mass() * g;
    ApplyBodyForce(ChSpatialVec(m_com_G % frc_G, frc_G));
}

void ChMobilizedBody::ApplyAllMobilityForces() {
    for (const auto& mf : m_mobility_forces) {
        if (mf.force->isEnabled()) {
            double f = mf.force->evaluate(getQ(mf.dof), getU(mf.dof));
            ApplyMobilityForce(mf.dof, f);
        }
    }
}

// -----------------------------------------------------------------------------

ChVector3d ChMobilizedBody::getAbsLoc(const ChVector3d& p_B) const {
    return m_absPos * p_B;
}

ChVector3d ChMobilizedBody::getAbsVel(const ChVector3d& p_B) const {
    ChVector3d r = m_absPos.GetRotMat() * p_B;
    return m_absVel.lin() + m_absVel.ang() % r;
}

ChVector3d ChMobilizedBody::getAbsAcc(const ChVector3d& p_B) const {
    ChVector3d r = m_absPos.GetRotMat() * p_B;
    return m_absAcc.lin() + m_absAcc.ang() % r + m_absVel.ang() % (m_absVel.ang() % r);
}

ChVector3d ChMobilizedBody::getAbsCOMLoc() const {
    return getAbsLoc(m_mpropsB.com());
}

ChVector3d ChMobilizedBody::getAbsCOMVel() const {
    return getAbsVel(m_mpropsB.com());
}

ChVector3d ChMobilizedBody::getAbsCOMAcc() const {
    return getAbsAcc(m_mpropsB.com());
}

// -----------------------------------------------------------------------------

double ChMobilizedBody::getQ(int dof) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumQ());
    return m_assembly->getY(m_qIdx + dof);
}

double ChMobilizedBody::getU(int dof) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumU());
    return m_assembly->getYd(m_uIdx + dof);
}

double ChMobilizedBody::getUdot(int dof) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumU());
    return m_assembly->getYdd(m_uIdx + dof);
}

void ChMobilizedBody::setQ(int dof, double val) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumQ());
    m_assembly->setY(m_qIdx + dof, val);
}

void ChMobilizedBody::setU(int dof, double val) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumU());
    m_assembly->setYd(m_uIdx + dof, val);
}

void ChMobilizedBody::setUdot(int dof, double val) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumU());
    m_assembly->setYdd(m_uIdx + dof, val);
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::setRelPos(const ChFramed& relPos) {
    setRelRot(relPos.GetRotMat());
    setRelLoc(relPos.GetPos());
}

void ChMobilizedBody::setRelVel(const ChSpatialVec& relVel) {
    setRelAngVel(relVel.ang());
    setRelLinVel(relVel.lin());
}

void ChMobilizedBody::setRelAcc(const ChSpatialVec& relAcc) {
    setRelAngAcc(relAcc.ang());
    setRelLinAcc(relAcc.lin());
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::orProcPosFD(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosFD(y);
}

void ChMobilizedBody::orProcVelFD(const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcVelFD(yd);
}

void ChMobilizedBody::orProcPosAndVelFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosAndVelFD(y, yd);
}

void ChMobilizedBody::orProcAccFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd, ChVectorDynamic<>& ydd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcAccFD(y, yd, ydd);
}

void ChMobilizedBody::orProcMiF_passTwo(double* ud) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcMiF_passTwo(ud);
}

void ChMobilizedBody::orProcPosVelAccID(const ChVectorDynamic<>& y,
                                        const ChVectorDynamic<>& yd,
                                        const ChVectorDynamic<>& ydd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosVelAccID(y, yd, ydd);
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::irProcInertiasAndForcesFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcInertiasAndForcesFD(y, yd);
}

void ChMobilizedBody::irProcInertiasFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcInertiasFD(y, yd);
}

void ChMobilizedBody::irProcForcesFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcForcesFD(y, yd);
}

void ChMobilizedBody::irProcConstraintJac(double* vec) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcConstraintJac(vec);
}

void ChMobilizedBody::irProcMiF_passOne() {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcMiF_passOne();
}

void ChMobilizedBody::irProcForcesID(const ChVectorDynamic<>& y,
                                     const ChVectorDynamic<>& yd,
                                     const ChVectorDynamic<>& ydd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcForcesID(y, yd, ydd);
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::setQDot(const ChVectorDynamic<>& y, ChVectorDynamic<>& yd) const {
    assert(getNumU() == getNumQ());

    yd.segment(m_qIdx, getNumU()) = y.segment(m_uIdx, getNumU());
}

void ChMobilizedBody::setQDotDot(const ChVectorDynamic<>& y,
                                 const ChVectorDynamic<>& yd,
                                 ChVectorDynamic<>& ydd) const {
    ydd.segment(m_qIdx, getNumU()) = yd.segment(m_uIdx, getNumU());
}

// =============================================================================

ChGroundBody::ChGroundBody() : ChMobilizedBody(nullptr, ChMassProps(), ChFramed(), ChFramed(), "ground") {
    m_V_FM.setZero();
    m_absVel.setZero();
    m_absAcc.setZero();
}

ChGroundBody::ChGroundBody(const ChGroundBody& other) : ChMobilizedBody(other) {
    //// TODO
}

}  // namespace soa
}  // namespace chrono
