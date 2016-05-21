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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkNumdiff.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
ChClassRegister<ChLinkNumdiff> a_registration_ChLinkNumdiff;

void ChLinkNumdiff::ComputeC() {
    // ** Child classes must implement this constraint evaluation, in order
    //    to allow this base class to perform backward differentiation to get
    //    all state matrices.
    //    This function can be called many times by UpdateState() for the numerical
    //    differentiation.
    //
    //    Example: C = my_constraint_functionXXX(distance_between_markers,time), etc.
    //
    //    Default: do nothing (residual always zero)

    C->Reset();  // default: always no violation. C = 0;
}

void ChLinkNumdiff::ComputeCt() {
    ChMatrixDynamic<> m_q(GetNumCoords(), 1);  // coordinates
    FetchCoords(&m_q);                         // current state coordinates = position of bodies 1&2

    double m_tdt = ChTime + BDF_STEP_VERYLOW;

    ChMatrixDynamic<> C_qt(ndoc, 1);  //  C(q,t)      // assuming C is already updated (computed)!!
    C_qt.CopyFromMatrix(*this->C);

    ChMatrixDynamic<> C_qdt(ndoc, 1);  //  C(q,t+dt)
    ImposeCoords(&m_q, m_tdt);
    ComputeC();
    C_qdt.CopyFromMatrix(*this->C);

    Ct->MatrSub(C_qdt, C_qt);  // Ct
    Ct->MatrScale(1.0 / BDF_STEP_VERYLOW);
}

void ChLinkNumdiff::ComputeCq() {
    double orig;

    ChMatrixDynamic<> m_q(GetNumCoords(), 1);  // current coordinates
    FetchCoords(&m_q);                         // current state coordinates = position of bodies 1&2

    ChMatrixDynamic<> C_dqt(ndoc, 1);
    ChMatrixDynamic<> Cq_column(ndoc, 1);
    ChMatrixDynamic<> C_qt(ndoc, 1);  //  C(q,t)      // assuming C is already updated (computed)!!
    C_qt.CopyFromMatrix(*this->C);

    for (int i = 0; i < GetNumCoords(); i++) {
        orig = m_q(i, 0);
        m_q(i, 0) = orig + BDF_STEP_VERYLOW;
        ImposeCoords(&m_q, ChTime);
        ComputeC();
        C_dqt.CopyFromMatrix(*this->C);  // C(dqi,t)

        Cq_column.MatrSub(C_dqt, C_qt);
        Cq_column.MatrScale(1.0 / BDF_STEP_VERYLOW);

        if (i < BODY_QDOF)
            this->Cq1->PasteMatrix(&Cq_column, 0, i);  // jacobians [Cq], 1&2
        else
            this->Cq2->PasteMatrix(&Cq_column, 0, (i - BODY_QDOF));
        m_q(i, 0) = orig;
    }
}

void ChLinkNumdiff::UpdateState() {
    double m_t = ChTime;
    double m_tdt = ChTime + BDF_STEP_VERYLOW;

    ChMatrixDynamic<> m_q(GetNumCoords(), 1);  // The coordinates q
    FetchCoords(&m_q);                         // current state coordinates = position of bodies 1&2

    ChMatrixDynamic<> m_qdt(GetNumCoords(), 1);  // The coordinates q
    FetchCoords_dt(&m_qdt);                      // current state coordinates = position of bodies 1&2

    ChMatrixDynamic<> C_qt(ndoc, 1);
    ComputeC();  //  C(q,t)  (assuming system is now at (q,t) )
    C_qt.CopyFromMatrix(*this->C);

    ComputeCt();  //  Ct

    ComputeCq();  //  Cq1 and Cq2

    ChMatrixDynamic<> m_qdt1(BODY_QDOF, 1);
    ChMatrixDynamic<> m_qdt2(BODY_QDOF, 1);
    m_qdt1.PasteClippedMatrix(&m_qdt, 0, 0, BODY_QDOF, 1, 0, 0);
    m_qdt2.PasteClippedMatrix(&m_qdt, BODY_QDOF, 0, BODY_QDOF, 1, 0, 0);

    ChMatrixDynamic<> m_temp(ndoc, 1);

    m_temp.MatrMultiply(*Cq1, m_qdt1);
    C_dt->CopyFromMatrix(m_temp);
    m_temp.MatrMultiply(*Cq2, m_qdt2);
    C_dt->MatrInc(m_temp);
    C_dt->MatrInc(*Ct);  // C_dt  = Ct + [Cq](dq/dt);

    // now compute vector gamma for dynamics:  ???
    //***TO DO***....
}

void ChLinkNumdiff::ImposeCoords(ChMatrix<>* mc, double t) {
    ChCoordsys<> mcsys;

    if (!(dynamic_cast<ChBody*>(Body1) && dynamic_cast<ChBody*>(Body2)))
        throw(ChException("ChLinkNumdiff had pointer to non ChBody item"));

    mcsys = mc->ClipCoordsys(0, 0);
    dynamic_cast<ChBody*>(Body1)->Update(mcsys, Body1->GetCoord_dt(), t);

    mcsys = mc->ClipCoordsys(7, 0);
    dynamic_cast<ChBody*>(Body2)->Update(mcsys, Body2->GetCoord_dt(), t);

    // - update the time dependant stuff
    UpdateTime(t);
    // - update the relative marker position
    UpdateRelMarkerCoords();
}

void ChLinkNumdiff::FetchCoords(ChMatrix<>* mc) {
    ChCoordsys<> mcsys;

    mcsys = Body1->GetCoord();
    mc->PasteCoordsys(mcsys, 0, 0);

    mcsys = Body2->GetCoord();
    mc->PasteCoordsys(mcsys, 7, 0);
}

void ChLinkNumdiff::FetchCoords_dt(ChMatrix<>* mc) {
    ChCoordsys<> mcsys;

    mcsys = Body1->GetCoord_dt();
    mc->PasteCoordsys(mcsys, 0, 0);

    mcsys = Body2->GetCoord_dt();
    mc->PasteCoordsys(mcsys, 7, 0);
}

void ChLinkNumdiff::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLinkMasked::ArchiveOUT(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkNumdiff::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLinkMasked::ArchiveIN(marchive);

    // deserialize all member data:
}

}  // end namespace chrono
