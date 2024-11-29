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

void ChMobilizedBody::AddMobilityForce(int dof, std::shared_ptr<ChMobilityForce> force) {
    assert(0 <= dof && dof < getNumU());
    m_mobility_forces.push_back({dof, force});
}

void ChMobilizedBody::ApplyMobilityForces() {
    for (const auto& mf : m_mobility_forces) {
        if (mf.force->isEnabled()) {
            double f = mf.force->evaluate(getQ(mf.dof), getU(mf.dof));
            applyMobilityForce(mf.dof, f);
        }
    }
}

// -----------------------------------------------------------------------------

double ChMobilizedBody::getQ(int dof) const {
    assert(dof >= 0 && dof < getNumQ());

    return m_assembly->getCurState(m_qIdx + dof);
}

double ChMobilizedBody::getU(int dof) const {
    assert(dof >= 0 && dof < getNumU());

    return m_assembly->getCurState(m_uIdx + dof);
}

double ChMobilizedBody::getQDot(int dof) const {
    assert(dof >= 0 && dof < getNumQ());

    return m_assembly->getCurStateDeriv(m_qIdx + dof);
}

double ChMobilizedBody::getUDot(int dof) const {
    assert(dof >= 0 && dof < getNumU());

    return m_assembly->getCurStateDeriv(m_uIdx + dof);
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::setRelPos(const ChFramed& relPos) const {
    setRelRot(relPos.GetRotMat());
    setRelLoc(relPos.GetPos());
}

void ChMobilizedBody::setRelVel(const ChSpatialVec& relVel) const {
    setRelAngVel(relVel.ang());
    setRelLinVel(relVel.lin());
}

void ChMobilizedBody::setRelAcc(const ChSpatialVec& relAcc) const {
    setRelAngAcc(relAcc.ang());
    setRelLinAcc(relAcc.lin());
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::orProcPosFD(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosFD(y);
}

void ChMobilizedBody::orProcVelFD(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcVelFD(y);
}

void ChMobilizedBody::orProcPosAndVelFD(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosAndVelFD(y);
}

void ChMobilizedBody::orProcAccFD(const ChVectorDynamic<>& y, ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcAccFD(y, yd);
}

void ChMobilizedBody::orProcMiF_passTwo(double* ud) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcMiF_passTwo(ud);
}

void ChMobilizedBody::orProcPosVelAccID(const ChVectorDynamic<>& y, const ChVectorDynamic<>& yd) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->orProcPosVelAccID(y, yd);
}

// -----------------------------------------------------------------------------

void ChMobilizedBody::irProcInertiasAndForcesFD(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcInertiasAndForcesFD(y);
}

void ChMobilizedBody::irProcInertiasFD(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcInertiasFD(y);
}

void ChMobilizedBody::irProcForcesFD(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcForcesFD(y);
}

void ChMobilizedBody::irProcConstraintJac(double* vec) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcConstraintJac(vec);
}

void ChMobilizedBody::irProcMiF_passOne() {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcMiF_passOne();
}

void ChMobilizedBody::irProcForcesID(const ChVectorDynamic<>& y) {
    for (int i = 0; i < getNumChildren(); i++)
        getChild(i)->irProcForcesID(y);
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

// -----------------------------------------------------------------------------

ChGroundBody::ChGroundBody() : ChMobilizedBody(nullptr, ChMassProps(), ChFramed(), ChFramed(), "ground") {
    m_V_FM.setZero();
    m_absVel.setZero();
    m_absAcc.setZero();
}

}  // namespace soa
}  // namespace chrono
