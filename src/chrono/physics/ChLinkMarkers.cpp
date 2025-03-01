// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkMarkers.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChLinkMarkers)    // NO! Abstract class!

ChLinkMarkers::ChLinkMarkers()
    : marker1(NULL),
      marker2(NULL),
      relM(CSYSNORM),
      relM_dt(CSYSNULL),
      relM_dtdt(CSYSNULL),
      relAngle(0),
      relRotaxis(VNULL),
      relWvel(VNULL),
      relWacc(VNULL),
      C_force(VNULL),
      C_torque(VNULL) {}

ChLinkMarkers::ChLinkMarkers(const ChLinkMarkers& other) : ChLink(other) {
    marker1 = NULL;
    marker2 = NULL;

    relM = other.relM;  // copy vars
    relM_dt = other.relM_dt;
    relM_dtdt = other.relM_dtdt;
    relAngle = other.relAngle;
    relRotaxis = other.relRotaxis;
    relAxis = other.relAxis;
    relWvel = other.relWvel;
    relWacc = other.relWacc;
    dist = other.dist;
    dist_dt = other.dist_dt;

    C_force = other.C_force;
    C_torque = other.C_torque;
}

void ChLinkMarkers::SetupMarkers(ChMarker* mark1, ChMarker* mark2) {
    // take care of the first link marker
    marker1 = mark1;
    if (mark1)
        m_body1 = (ChBodyFrame*)mark1->GetBody();
    else
        m_body1 = NULL;

    // take care of the second link marker
    marker2 = mark2;
    if (mark2)
        m_body2 = (ChBodyFrame*)mark2->GetBody();
    else
        m_body2 = NULL;
}

void ChLinkMarkers::Initialize(std::shared_ptr<ChMarker> mark1, std::shared_ptr<ChMarker> mark2) {
    assert(mark1 && mark2);
    assert(mark1.get() != mark2.get());
    assert(mark1->GetBody() && mark2->GetBody());
    assert(mark1->GetBody()->GetSystem() == mark2->GetBody()->GetSystem());

    SetupMarkers(mark1.get(), mark2.get());
}

void ChLinkMarkers::Initialize(std::shared_ptr<ChBody> mbody1, std::shared_ptr<ChBody> mbody2, const ChFrame<>& frame) {
    return Initialize(mbody1, mbody2, false, frame, frame);
}

void ChLinkMarkers::Initialize(std::shared_ptr<ChBody> mbody1,
                               std::shared_ptr<ChBody> mbody2,
                               bool rel_frames,
                               const ChFrame<>& frame1,
                               const ChFrame<>& frame2) {
    assert(mbody1.get() != mbody2.get());
    assert(mbody1->GetSystem() == mbody2->GetSystem());

    // create markers to add to the two bodies
    std::shared_ptr<ChMarker> mmark1(new ChMarker);
    std::shared_ptr<ChMarker> mmark2(new ChMarker);
    mbody1->AddMarker(mmark1);
    mbody2->AddMarker(mmark2);

    ChMarker* mm1 = mmark1.get();
    ChMarker* mm2 = mmark2.get();
    SetupMarkers(mm1, mm2);

    if (rel_frames) {
        mmark1->ImposeRelativeTransform(frame1);
        mmark2->ImposeRelativeTransform(frame2);
    } else {
        mmark1->ImposeAbsoluteTransform(frame1);
        mmark2->ImposeAbsoluteTransform(frame2);
    }
}

// For all relative degrees of freedom of the two markers, compute relM, relM_dt,
// and relM_dtdt, as well as auxiliary data. Cache some intermediate quantities
// for possible reuse in UpdateState by some derived classes.
void ChLinkMarkers::UpdateRelMarkerCoords() {
    PQw = marker1->GetAbsCoordsys().pos - marker2->GetAbsCoordsys().pos;
    PQw_dt = marker1->GetAbsCoordsysDt().pos - marker2->GetAbsCoordsysDt().pos;
    PQw_dtdt = marker1->GetAbsCoordsysDt2().pos - marker2->GetAbsCoordsysDt2().pos;

    dist = Vlength(PQw);                 // distance between origins, modulus
    dist_dt = Vdot(Vnorm(PQw), PQw_dt);  // speed between origins, modulus.

    ChQuaterniond qtemp1;

    ChQuaterniond temp1 = marker1->GetCoordsysDt().rot;
    ChQuaterniond temp2 = marker2->GetCoordsysDt().rot;

    if (Qnotnull(temp1) || Qnotnull(temp2)) {
        q_AD =  //  q'qqq + qqqq'
            Qadd(Qcross(Qconjugate(marker2->GetCoordsysDt().rot),
                        Qcross(Qconjugate(marker2->GetBody()->GetCoordsys().rot),
                               Qcross((marker1->GetBody()->GetCoordsys().rot), (marker1->GetCoordsys().rot)))),
                 Qcross(Qconjugate(marker2->GetCoordsys().rot),
                        Qcross(Qconjugate(marker2->GetBody()->GetCoordsys().rot),
                               Qcross((marker1->GetBody()->GetCoordsys().rot), (marker1->GetCoordsysDt().rot)))));
    } else
        q_AD = QNULL;

    q_BC =  // qq'qq + qqq'q
        Qadd(Qcross(Qconjugate(marker2->GetCoordsys().rot),
                    Qcross(Qconjugate(marker2->GetBody()->GetCoordsysDt().rot),
                           Qcross((marker1->GetBody()->GetCoordsys().rot), (marker1->GetCoordsys().rot)))),
             Qcross(Qconjugate(marker2->GetCoordsys().rot),
                    Qcross(Qconjugate(marker2->GetBody()->GetCoordsys().rot),
                           Qcross((marker1->GetBody()->GetCoordsysDt().rot), (marker1->GetCoordsys().rot)))));

    // q_8 = q''qqq + 2q'q'qq + 2q'qq'q + 2q'qqq'
    //     + 2qq'q'q + 2qq'qq' + 2qqq'q' + qqqq''
    temp2 = marker2->GetCoordsysDt2().rot;
    if (Qnotnull(temp2))
        q_8 = Qcross(Qconjugate(marker2->GetCoordsysDt2().rot),
                     Qcross(Qconjugate(m_body2->GetCoordsys().rot),
                            Qcross(m_body1->GetCoordsys().rot,
                                   marker1->GetCoordsys().rot)));  // q_dtdt'm2 * q'o2 * q,o1 * q,m1
    else
        q_8 = QNULL;
    temp1 = marker1->GetCoordsysDt2().rot;
    if (Qnotnull(temp1)) {
        qtemp1 = Qcross(Qconjugate(marker2->GetCoordsys().rot),
                        Qcross(Qconjugate(m_body2->GetCoordsys().rot),
                               Qcross(m_body1->GetCoordsys().rot,
                                      marker1->GetCoordsysDt2().rot)));  // q'm2 * q'o2 * q,o1 * q_dtdt,m1
        q_8 = Qadd(q_8, qtemp1);
    }
    temp2 = marker2->GetCoordsysDt().rot;
    if (Qnotnull(temp2)) {
        qtemp1 = Qcross(Qconjugate(marker2->GetCoordsysDt().rot),
                        Qcross(Qconjugate(m_body2->GetCoordsysDt().rot),
                               Qcross(m_body1->GetCoordsys().rot, marker1->GetCoordsys().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q_dt'm2 * q_dt'o2 * q,o1 * q,m1)
        q_8 = Qadd(q_8, qtemp1);
    }
    temp2 = marker2->GetCoordsysDt().rot;
    if (Qnotnull(temp2)) {
        qtemp1 = Qcross(Qconjugate(marker2->GetCoordsysDt().rot),
                        Qcross(Qconjugate(m_body2->GetCoordsys().rot),
                               Qcross(m_body1->GetCoordsysDt().rot, marker1->GetCoordsys().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q_dt'm2 * q'o2 * q_dt,o1 * q,m1)
        q_8 = Qadd(q_8, qtemp1);
    }
    temp1 = marker1->GetCoordsysDt().rot;
    temp2 = marker2->GetCoordsysDt().rot;
    if (Qnotnull(temp2) && Qnotnull(temp1)) {
        qtemp1 = Qcross(Qconjugate(marker2->GetCoordsysDt().rot),
                        Qcross(Qconjugate(m_body2->GetCoordsys().rot),
                               Qcross(m_body1->GetCoordsys().rot, marker1->GetCoordsysDt().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q_dt'm2 * q'o2 * q,o1 * q_dt,m1)
        q_8 = Qadd(q_8, qtemp1);
    }

    qtemp1 = Qcross(Qconjugate(marker2->GetCoordsys().rot),
                    Qcross(Qconjugate(m_body2->GetCoordsysDt().rot),
                           Qcross(m_body1->GetCoordsysDt().rot, marker1->GetCoordsys().rot)));
    qtemp1 = Qscale(qtemp1, 2);  // 2( q'm2 * q_dt'o2 * q_dt,o1 * q,m1)
    q_8 = Qadd(q_8, qtemp1);
    temp1 = marker1->GetCoordsysDt().rot;
    if (Qnotnull(temp1)) {
        qtemp1 = Qcross(Qconjugate(marker2->GetCoordsys().rot),
                        Qcross(Qconjugate(m_body2->GetCoordsysDt().rot),
                               Qcross(m_body1->GetCoordsys().rot, marker1->GetCoordsysDt().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q'm2 * q_dt'o2 * q,o1 * q_dt,m1)
        q_8 = Qadd(q_8, qtemp1);
    }
    temp1 = marker1->GetCoordsysDt().rot;
    if (Qnotnull(temp1)) {
        qtemp1 = Qcross(Qconjugate(marker2->GetCoordsys().rot),
                        Qcross(Qconjugate(m_body2->GetCoordsys().rot),
                               Qcross(m_body1->GetCoordsysDt().rot, marker1->GetCoordsysDt().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q'm2 * q'o2 * q_dt,o1 * q_dt,m1)
        q_8 = Qadd(q_8, qtemp1);
    }

    // q_4 = [Adtdt]'[A]'q + 2[Adt]'[Adt]'q
    //       + 2[Adt]'[A]'qdt + 2[A]'[Adt]'qdt
    ChMatrix33<> m2_Rel_A_dt;
    marker2->ComputeRotMatDt(m2_Rel_A_dt);
    ChMatrix33<> m2_Rel_A_dtdt;
    marker2->ComputeRotMatDt2(m2_Rel_A_dtdt);

    ChVector3d vtemp1;
    ChVector3d vtemp2;

    vtemp1 = m_body2->GetRotMatDt().transpose() * PQw;
    vtemp2 = m2_Rel_A_dt.transpose() * vtemp1;
    q_4 = Vmul(vtemp2, 2);  // 2[Aq_dt]'[Ao2_dt]'*Qpq,w

    vtemp1 = m_body2->GetRotMat().transpose() * PQw_dt;
    vtemp2 = m2_Rel_A_dt.transpose() * vtemp1;
    vtemp2 = Vmul(vtemp2, 2);  // 2[Aq_dt]'[Ao2]'*Qpq,w_dt
    q_4 = Vadd(q_4, vtemp2);

    vtemp1 = m_body2->GetRotMatDt().transpose() * PQw_dt;
    vtemp2 = marker2->GetRotMat().transpose() * vtemp1;
    vtemp2 = Vmul(vtemp2, 2);  // 2[Aq]'[Ao2_dt]'*Qpq,w_dt
    q_4 = Vadd(q_4, vtemp2);

    vtemp1 = m_body2->GetRotMat().transpose() * PQw;
    vtemp2 = m2_Rel_A_dtdt.transpose() * vtemp1;
    q_4 = Vadd(q_4, vtemp2);  //  [Aq_dtdt]'[Ao2]'*Qpq,w

    // ----------- RELATIVE MARKER COORDINATES

    // relM.pos
    relM.pos = marker2->GetRotMat().transpose() * (m_body2->GetRotMat().transpose() * PQw);

    // relM.rot
    relM.rot = Qcross(Qconjugate(marker2->GetCoordsys().rot),
                      Qcross(Qconjugate(marker2->GetBody()->GetCoordsys().rot),
                             Qcross((marker1->GetBody()->GetCoordsys().rot), (marker1->GetCoordsys().rot))));

    // relM_dt.pos
    relM_dt.pos = m2_Rel_A_dt.transpose() * (m_body2->GetRotMat().transpose() * PQw) +
                  marker2->GetRotMat().transpose() * (m_body2->GetRotMatDt().transpose() * PQw) +
                  marker2->GetRotMat().transpose() * (m_body2->GetRotMat().transpose() * PQw_dt);

    // relM_dt.rot
    relM_dt.rot = Qadd(q_AD, q_BC);

    // relM_dtdt.pos
    relM_dtdt.pos = marker2->GetRotMat().transpose() * (m_body2->GetRotMatDt2().transpose() * PQw) +
                    marker2->GetRotMat().transpose() * (m_body2->GetRotMat().transpose() * PQw_dtdt) + q_4;

    // relM_dtdt.rot
    qtemp1 = Qcross(Qconjugate(marker2->GetCoordsys().rot),
                    Qcross(Qconjugate(m_body2->GetCoordsysDt2().rot),
                           Qcross(m_body1->GetCoordsys().rot,
                                  marker1->GetCoordsys().rot)));  // ( q'm2 * q_dtdt'o2 * q,o1 * q,m1)
    relM_dtdt.rot = Qadd(q_8, qtemp1);
    qtemp1 = Qcross(Qconjugate(marker2->GetCoordsys().rot),
                    Qcross(Qconjugate(m_body2->GetCoordsys().rot),
                           Qcross(m_body1->GetCoordsysDt2().rot,
                                  marker1->GetCoordsys().rot)));  // ( q'm2 * q'o2 * q_dtdt,o1 * q,m1)
    relM_dtdt.rot = Qadd(relM_dtdt.rot, qtemp1);                  // = q_8 + qq''qq + qqq''q

    // ... and also "user-friendly" relative coordinates:

    // relAngle and relAxis
    relM.rot.GetAngleAxis(relAngle, relAxis);
    // flip rel rotation axis if jerky sign
    if (relAxis.z() < 0) {
        relAxis = Vmul(relAxis, -1);
        relAngle = -relAngle;
    }
    // rotation axis
    relRotaxis = Vmul(relAxis, relAngle);
    // relWvel
    ChGwMatrix34<> relGw(relM.rot);
    relWvel = relGw * relM_dt.rot;
    // relWacc
    relWacc = relGw * relM_dtdt.rot;
}

void ChLinkMarkers::UpdateForces(double time) {
    // reset internal force accumulators
    C_force = VNULL;  // initialize int.forces accumulators
    C_torque = VNULL;
}

void ChLinkMarkers::Update(double time, bool update_assets) {
    // Update time and assets
    ChPhysicsItem::Update(time, update_assets);

    UpdateRelMarkerCoords();
    UpdateForces(time);
}

//// STATE BOOKKEEPING FUNCTIONS

// Load residual R += c * F, starting at specified offset.
void ChLinkMarkers::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    if (!m_body1 || !m_body2)
        return;

    if (Vnotnull(C_force)) {
        // application point is always marker1
        ChVector3d m_abs_force = m_body2->GetRotMat() * (marker2->GetRotMat() * C_force);

        if (m_body2->Variables().IsActive()) {
            auto w2_abs = m_body2->AppliedForceParentToWrenchParent(m_abs_force, marker1->GetAbsCoordsys().pos);
            R.segment(m_body2->Variables().GetOffset() + 0, 3) -= c * w2_abs.force.eigen();
            R.segment(m_body2->Variables().GetOffset() + 3, 3) -=
                c * m_body2->TransformDirectionParentToLocal(w2_abs.torque).eigen();
        }

        if (m_body1->Variables().IsActive()) {
            auto w1_abs = m_body1->AppliedForceParentToWrenchParent(m_abs_force, marker1->GetAbsCoordsys().pos);
            R.segment(m_body1->Variables().GetOffset() + 0, 3) += c * w1_abs.force.eigen();
            R.segment(m_body1->Variables().GetOffset() + 3, 3) +=
                c * m_body1->TransformDirectionParentToLocal(w1_abs.torque).eigen();
        }
    }

    if (Vnotnull(C_torque)) {
        ChVector3d m_abs_torque = m_body2->GetRotMat() * (marker2->GetRotMat() * C_torque);
        // load torques in 'fb' vector accumulator of body variables (torques in local coords)
        if (m_body1->Variables().IsActive()) {
            R.segment(m_body1->Variables().GetOffset() + 3, 3) +=
                c * m_body1->TransformDirectionParentToLocal(m_abs_torque).eigen();
        }
        if (m_body2->Variables().IsActive()) {
            R.segment(m_body2->Variables().GetOffset() + 3, 3) -=
                c * m_body2->TransformDirectionParentToLocal(m_abs_torque).eigen();
        }
    }
}

// SOLVER INTERFACE

void ChLinkMarkers::ConstraintsFbLoadForces(double factor) {
    if (!m_body1 || !m_body2)
        return;

    if (Vnotnull(C_force)) {
        // application point is always marker1
        ChVector3d m_abs_force = m_body2->GetRotMat() * (marker2->GetRotMat() * C_force);

        auto w2_abs = m_body2->AppliedForceParentToWrenchParent(m_abs_force, marker1->GetAbsCoordsys().pos);
        m_body2->Variables().Force().segment(0, 3) -= factor * w2_abs.force.eigen();
        m_body2->Variables().Force().segment(3, 3) -=
            factor * m_body2->TransformDirectionParentToLocal(w2_abs.torque).eigen();

        auto w1_abs = m_body1->AppliedForceParentToWrenchParent(m_abs_force, marker1->GetAbsCoordsys().pos);
        m_body1->Variables().Force().segment(0, 3) += factor * w1_abs.force.eigen();
        m_body1->Variables().Force().segment(3, 3) +=
            factor * m_body1->TransformDirectionParentToLocal(w1_abs.torque).eigen();
    }

    if (Vnotnull(C_torque)) {
        ChVector3d m_abs_torque = m_body2->GetRotMat() * (marker2->GetRotMat() * C_torque);
        // load torques in 'fb' vector accumulator of body variables (torques in local coords)
        m_body1->Variables().Force().segment(3, 3) +=
            factor * m_body1->TransformDirectionParentToLocal(m_abs_torque).eigen();
        m_body2->Variables().Force().segment(3, 3) -=
            factor * m_body2->TransformDirectionParentToLocal(m_abs_torque).eigen();
    }
}

void ChLinkMarkers::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMarkers>();

    // serialize parent class
    ChLink::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(marker1);
    archive_out << CHNVP(marker2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMarkers::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMarkers>();

    // deserialize parent class
    ChLink::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(marker1);
    archive_in >> CHNVP(marker2);
}

}  // end namespace chrono
