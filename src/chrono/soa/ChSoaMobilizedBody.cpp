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
#include "chrono/soa/ChSoaMobilizedBody.h"

namespace chrono {
namespace soa {

// Initialize Static Variables
const double ChSoaMobilizedBody::m_angleClampLimit = 10 * CH_2PI;

// =============================================================================

ChSoaMobilizedBody::ChSoaMobilizedBody(std::shared_ptr<ChSoaMobilizedBody> parent,
                                 const ChSoaMassProperties& mpropsB,
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

ChSoaMobilizedBody::ChSoaMobilizedBody(const ChSoaMobilizedBody& other) : ChObj(other) {
    //// TODO
}

ChSoaMobilizedBody::~ChSoaMobilizedBody() {
    if (m_parent) {
        auto itr = std::find(std::begin(m_parent->m_children), std::end(m_parent->m_children), this);
        assert(itr != m_parent->m_children.end());
        m_parent->m_children.erase(itr);
    }
}

void ChSoaMobilizedBody::lock(bool val) {
    //// TODO -- do I need this?
    ////if (val) {
    ////    for (UINT i = 0; i < getNumU(); i++)
    ////        getSystem()->y0()[m_uIdx + i] = 0;
    ////}

    m_locked = val;
}

// -----------------------------------------------------------------------------

void ChSoaMobilizedBody::AddMobilityForce(int dof, std::shared_ptr<ChSoaMobilityForce> force) {
    assert(0 <= dof && dof < getNumU());
    m_mobility_forces.push_back({dof, force});
}

// -----------------------------------------------------------------------------

void ChSoaMobilizedBody::ApplyBodyForce(const ChSpatialVec& force) {
    m_bodyForce += force;
}

void ChSoaMobilizedBody::ApplyGravitationalForce(const ChVector3d& g) {
    ChVector3d frc_G = m_mpropsB.mass() * g;
    ApplyBodyForce(ChSpatialVec(m_com_G % frc_G, frc_G));
}

void ChSoaMobilizedBody::ApplyAllMobilityForces() {
    for (const auto& mf : m_mobility_forces) {
        if (mf.force->isEnabled()) {
            double f = mf.force->evaluate(getQ(mf.dof), getU(mf.dof));
            ApplyMobilityForce(mf.dof, f);
        }
    }
}

// -----------------------------------------------------------------------------

ChVector3d ChSoaMobilizedBody::getAbsLoc(const ChVector3d& p_B) const {
    return m_absPos * p_B;
}

ChVector3d ChSoaMobilizedBody::getAbsVel(const ChVector3d& p_B) const {
    ChVector3d r = m_absPos.GetRotMat() * p_B;
    return m_absVel.lin() + m_absVel.ang() % r;
}

ChVector3d ChSoaMobilizedBody::getAbsAcc(const ChVector3d& p_B) const {
    ChVector3d r = m_absPos.GetRotMat() * p_B;
    return m_absAcc.lin() + m_absAcc.ang() % r + m_absVel.ang() % (m_absVel.ang() % r);
}

ChVector3d ChSoaMobilizedBody::getAbsCOMLoc() const {
    return getAbsLoc(m_mpropsB.com());
}

ChVector3d ChSoaMobilizedBody::getAbsCOMVel() const {
    return getAbsVel(m_mpropsB.com());
}

ChVector3d ChSoaMobilizedBody::getAbsCOMAcc() const {
    return getAbsAcc(m_mpropsB.com());
}

ChFramed ChSoaMobilizedBody::getAbsInboardFrame() const {
    return getParent()->getAbsPos() * m_X_PF;
}

ChFramed ChSoaMobilizedBody::getAbsOutboardFrame() const {
    return getAbsPos() * m_X_BM;
}

// -----------------------------------------------------------------------------

double ChSoaMobilizedBody::getQ(int dof) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumQ());
    return m_assembly->getY(m_qIdx + dof);
}

double ChSoaMobilizedBody::getU(int dof) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumU());
    return m_assembly->getYd(m_uIdx + dof);
}

double ChSoaMobilizedBody::getUdot(int dof) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumU());
    return m_assembly->getYdd(m_uIdx + dof);
}

void ChSoaMobilizedBody::setQ(int dof, double val) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumQ());
    m_assembly->setY(m_qIdx + dof, val);
}

void ChSoaMobilizedBody::setU(int dof, double val) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumU());
    m_assembly->setYd(m_uIdx + dof, val);
}

void ChSoaMobilizedBody::setUdot(int dof, double val) const {
    assert(m_assembly->IsInitialized());
    assert(dof >= 0 && dof < getNumU());
    m_assembly->setYdd(m_uIdx + dof, val);
}

// -----------------------------------------------------------------------------

void ChSoaMobilizedBody::setRelPos(const ChFramed& relPos) {
    setRelRot(relPos.GetRotMat());
    setRelLoc(relPos.GetPos());
}

void ChSoaMobilizedBody::setRelVel(const ChSpatialVec& relVel) {
    setRelAngVel(relVel.ang());
    setRelLinVel(relVel.lin());
}

void ChSoaMobilizedBody::setRelAcc(const ChSpatialVec& relAcc) {
    setRelAngAcc(relAcc.ang());
    setRelLinAcc(relAcc.lin());
}

// -----------------------------------------------------------------------------

void ChSoaMobilizedBody::orProcPosFD(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosFD(y);
}

void ChSoaMobilizedBody::orProcVelFD(const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcVelFD(yd);
}

void ChSoaMobilizedBody::orProcPosAndVelFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosAndVelFD(y, yd);
}

void ChSoaMobilizedBody::orProcAccFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd, ChVectorDynamic<>& ydd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcAccFD(y, yd, ydd);
}

void ChSoaMobilizedBody::orProcMiF_passTwo(double* ud) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcMiF_passTwo(ud);
}

void ChSoaMobilizedBody::orProcPosVelAccID(const ChVectorDynamic<>& y,
                                        const ChVectorDynamic<>& yd,
                                        const ChVectorDynamic<>& ydd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosVelAccID(y, yd, ydd);
}

// -----------------------------------------------------------------------------

void ChSoaMobilizedBody::irProcInertiasAndForcesFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcInertiasAndForcesFD(y, yd);
}

void ChSoaMobilizedBody::irProcInertiasFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcInertiasFD(y, yd);
}

void ChSoaMobilizedBody::irProcForcesFD(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcForcesFD(y, yd);
}

void ChSoaMobilizedBody::irProcConstraintJac(double* vec) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcConstraintJac(vec);
}

void ChSoaMobilizedBody::irProcMiF_passOne() {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcMiF_passOne();
}

void ChSoaMobilizedBody::irProcForcesID(const ChVectorDynamic<>& y,
                                     const ChVectorDynamic<>& yd,
                                     const ChVectorDynamic<>& ydd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcForcesID(y, yd, ydd);
}

// -----------------------------------------------------------------------------

void ChSoaMobilizedBody::setQDot(const ChVectorDynamic<>& y, ChVectorDynamic<>& yd) const {
    assert(getNumU() == getNumQ());

    yd.segment(m_qIdx, getNumU()) = y.segment(m_uIdx, getNumU());
}

void ChSoaMobilizedBody::setQDotDot(const ChVectorDynamic<>& y,
                                 const ChVectorDynamic<>& yd,
                                 ChVectorDynamic<>& ydd) const {
    ydd.segment(m_qIdx, getNumU()) = yd.segment(m_uIdx, getNumU());
}

// =============================================================================

ChGroundBody::ChGroundBody() : ChSoaMobilizedBody(nullptr, ChSoaMassProperties(), ChFramed(), ChFramed(), "ground") {
    m_V_FM.setZero();
    m_absVel.setZero();
    m_absAcc.setZero();
}

ChGroundBody::ChGroundBody(const ChGroundBody& other) : ChSoaMobilizedBody(other) {
    //// TODO
}

}  // namespace soa
}  // namespace chrono
