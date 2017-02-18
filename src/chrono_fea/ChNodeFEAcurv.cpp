// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Generic finite element node with 9 degrees of freedom representing curvature
// (2nd derivatives of the poosition vector)
// =============================================================================

#include "chrono_fea/ChNodeFEAcurv.h"

namespace chrono {
namespace fea {

ChNodeFEAcurv::ChNodeFEAcurv(const ChVector<>& rxx, const ChVector<>& ryy, const ChVector<>& rzz)
    : m_rxx(rxx),
      m_ryy(ryy),
      m_rzz(rzz),
      m_rxx_dt(VNULL),
      m_ryy_dt(VNULL),
      m_rzz_dt(VNULL),
      m_rxx_dtdt(VNULL),
      m_ryy_dtdt(VNULL),
      m_rzz_dtdt(VNULL) {
    m_variables = new ChVariablesGenericDiagonalMass(9);
    m_variables->GetMassDiagonal().FillElem(0);
}

ChNodeFEAcurv::ChNodeFEAcurv(const ChNodeFEAcurv& other) : ChNodeFEAbase(other) {
    m_rxx = other.m_rxx;
    m_ryy = other.m_ryy;
    m_rzz = other.m_rzz;
    m_rxx_dt = other.m_rxx_dt;
    m_ryy_dt = other.m_ryy_dt;
    m_rzz_dt = other.m_rzz_dt;
    m_rxx_dtdt = other.m_rxx_dtdt;
    m_ryy_dtdt = other.m_ryy_dtdt;
    m_rzz_dtdt = other.m_rzz_dtdt;

    m_variables = new ChVariablesGenericDiagonalMass(9);
    *m_variables = *other.m_variables;
}

ChNodeFEAcurv::~ChNodeFEAcurv() {
    delete m_variables;
}

ChNodeFEAcurv& ChNodeFEAcurv::operator=(const ChNodeFEAcurv& other) {
    if (&other == this)
        return *this;

    ChNodeFEAbase::operator=(other);

    *m_variables = *other.m_variables;
    m_rxx = other.m_rxx;
    m_ryy = other.m_ryy;
    m_rzz = other.m_rzz;
    m_rxx_dt = other.m_rxx_dt;
    m_ryy_dt = other.m_ryy_dt;
    m_rzz_dt = other.m_rzz_dt;
    m_rxx_dtdt = other.m_rxx_dtdt;
    m_ryy_dtdt = other.m_ryy_dtdt;
    m_rzz_dtdt = other.m_rzz_dtdt;

    return *this;
}

// -----------------------------------------------------------------------------

void ChNodeFEAcurv::Relax() {
    m_rxx = VNULL;
    m_ryy = VNULL;
    m_rzz = VNULL;
    m_rxx_dt = VNULL;
    m_ryy_dt = VNULL;
    m_rzz_dt = VNULL;
    m_rxx_dtdt = VNULL;
    m_ryy_dtdt = VNULL;
    m_rzz_dtdt = VNULL;
}

void ChNodeFEAcurv::SetNoSpeedNoAcceleration() {
    m_rxx_dt = VNULL;
    m_ryy_dt = VNULL;
    m_rzz_dt = VNULL;
    m_rxx_dtdt = VNULL;
    m_ryy_dtdt = VNULL;
    m_rzz_dtdt = VNULL;
}

// -----------------------------------------------------------------------------

void ChNodeFEAcurv::NodeIntStateGather(const unsigned int off_x,
                                       ChState& x,
                                       const unsigned int off_v,
                                       ChStateDelta& v,
                                       double& T) {
    x.PasteVector(m_rxx, off_x + 0, 0);
    x.PasteVector(m_ryy, off_x + 3, 0);
    x.PasteVector(m_rzz, off_x + 6, 0);
    v.PasteVector(m_rxx_dt, off_v + 0, 0);
    v.PasteVector(m_ryy_dt, off_v + 3, 0);
    v.PasteVector(m_rzz_dt, off_v + 6, 0);
}

void ChNodeFEAcurv::NodeIntStateScatter(const unsigned int off_x,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& v,
                                        const double T) {
    m_rxx = x.ClipVector(off_x + 0, 0);
    m_ryy = x.ClipVector(off_x + 3, 0);
    m_rzz = x.ClipVector(off_x + 6, 0);
    m_rxx_dt = v.ClipVector(off_v + 0, 0);
    m_ryy_dt = v.ClipVector(off_v + 3, 0);
    m_rzz_dt = v.ClipVector(off_v + 6, 0);
}

void ChNodeFEAcurv::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.PasteVector(m_rxx_dtdt, off_a + 0, 0);
    a.PasteVector(m_ryy_dtdt, off_a + 3, 0);
    a.PasteVector(m_rzz_dtdt, off_a + 6, 0);
}

void ChNodeFEAcurv::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    m_rxx_dtdt = a.ClipVector(off_a + 0, 0);
    m_ryy_dtdt = a.ClipVector(off_a + 3, 0);
    m_rzz_dtdt = a.ClipVector(off_a + 6, 0);
}

void ChNodeFEAcurv::NodeIntStateIncrement(const unsigned int off_x,
                                          ChState& x_new,
                                          const ChState& x,
                                          const unsigned int off_v,
                                          const ChStateDelta& Dv) {
    for (int i = 0; i < 9; i++) {
        x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
    }
}

void ChNodeFEAcurv::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    //// TODO do we even need anything here? What would the forces be?
}

void ChNodeFEAcurv::NodeIntLoadResidual_Mv(const unsigned int off,
                                           ChVectorDynamic<>& R,
                                           const ChVectorDynamic<>& w,
                                           const double c) {
    for (int i = 0; i < 9; i++) {
        R(off + i) += c * GetMassDiagonal()(i) * w(off + i);
    }
}

void ChNodeFEAcurv::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    m_variables->Get_qb().PasteClippedMatrix(v, off_v, 0, 9, 1, 0, 0);
    m_variables->Get_fb().PasteClippedMatrix(R, off_v, 0, 9, 1, 0, 0);
}

void ChNodeFEAcurv::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    v.PasteMatrix(m_variables->Get_qb(), off_v, 0);
}

// -----------------------------------------------------------------------------

void ChNodeFEAcurv::InjectVariables(ChSystemDescriptor& mdescriptor) {
    mdescriptor.InsertVariables(m_variables);
}

void ChNodeFEAcurv::VariablesFbReset() {
    m_variables->Get_fb().FillElem(0);
}

void ChNodeFEAcurv::VariablesFbLoadForces(double factor) {
    //// TODO do we even need anything here? What would the forces be?
}

void ChNodeFEAcurv::VariablesQbLoadSpeed() {
    m_variables->Get_qb().PasteVector(m_rxx_dt, 0, 0);
    m_variables->Get_qb().PasteVector(m_ryy_dt, 3, 0);
    m_variables->Get_qb().PasteVector(m_rzz_dt, 6, 0);
}

void ChNodeFEAcurv::VariablesQbSetSpeed(double step) {
    ChVector<> old_rxx_dt = m_rxx_dt;
    ChVector<> old_ryy_dt = m_ryy_dt;
    ChVector<> old_rzz_dt = m_rzz_dt;

    m_rxx_dt = m_variables->Get_qb().ClipVector(0, 0);
    m_ryy_dt = m_variables->Get_qb().ClipVector(3, 0);
    m_rzz_dt = m_variables->Get_qb().ClipVector(6, 0);

    if (step) {
        m_rxx_dtdt = (m_rxx_dt - old_rxx_dt) / step;
        m_ryy_dtdt = (m_ryy_dt - old_ryy_dt) / step;
        m_rzz_dtdt = (m_rzz_dt - old_rzz_dt) / step;
    }
}

void ChNodeFEAcurv::VariablesFbIncrementMq() {
    m_variables->Compute_inc_Mb_v(m_variables->Get_fb(), m_variables->Get_qb());
}

void ChNodeFEAcurv::VariablesQbIncrementPosition(double step) {
    ChVector<> new_rxx_dt = m_variables->Get_qb().ClipVector(0, 0);
    ChVector<> new_ryy_dt = m_variables->Get_qb().ClipVector(3, 0);
    ChVector<> new_rzz_dt = m_variables->Get_qb().ClipVector(6, 0);
    m_rxx = m_rxx + new_rxx_dt * step;
    m_ryy = m_ryy + new_ryy_dt * step;
    m_rzz = m_rzz + new_rzz_dt * step;
}

// -----------------------------------------------------------------------------

void ChNodeFEAcurv::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeFEAcurv>();
    // serialize parent class
    ChNodeFEAbase::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(m_rxx);
    marchive << CHNVP(m_ryy);
    marchive << CHNVP(m_rzz);
    marchive << CHNVP(m_rxx_dt);
    marchive << CHNVP(m_ryy_dt);
    marchive << CHNVP(m_rzz_dt);
    marchive << CHNVP(m_rxx_dtdt);
    marchive << CHNVP(m_ryy_dtdt);
    marchive << CHNVP(m_rzz_dtdt);
}

void ChNodeFEAcurv::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChNodeFEAcurv>();
    // deserialize parent class
    ChNodeFEAbase::ArchiveIN(marchive);

    // stream in all member data:
    marchive >> CHNVP(m_rxx);
    marchive >> CHNVP(m_ryy);
    marchive >> CHNVP(m_rzz);
    marchive >> CHNVP(m_rxx_dt);
    marchive >> CHNVP(m_ryy_dt);
    marchive >> CHNVP(m_rzz_dt);
    marchive >> CHNVP(m_rxx_dtdt);
    marchive >> CHNVP(m_ryy_dtdt);
    marchive >> CHNVP(m_rzz_dtdt);
}

}  // end namespace fea
}  // end namespace chrono
