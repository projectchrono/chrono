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
      markID1(0),
      markID2(0),
      relM(CSYSNORM),
      relM_dt(CSYSNULL),
      relM_dtdt(CSYSNULL),
      relAngle(0),
      relRotaxis(VNULL),
      relWvel(VNULL),
      relWacc(VNULL),
      C_force(VNULL),
      C_torque(VNULL),
      Scr_force(VNULL),
      Scr_torque(VNULL) {}

ChLinkMarkers::ChLinkMarkers(const ChLinkMarkers& other) : ChLink(other) {
    marker1 = NULL;
    marker2 = NULL;

    markID1 = other.markID1;
    markID2 = other.markID2;

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

    Scr_force = other.Scr_force;
    Scr_torque = other.Scr_torque;
}

void ChLinkMarkers::SetUpMarkers(ChMarker* mark1, ChMarker* mark2) {
    // take care of the first link marker
    marker1 = mark1;
    if (mark1)
        Body1 = (ChBodyFrame*)mark1->GetBody();
    else
        Body1 = NULL;

    // take care of the second link marker
    marker2 = mark2;
    if (mark2)
        Body2 = (ChBodyFrame*)mark2->GetBody();
    else
        Body2 = NULL;
}

bool ChLinkMarkers::ReferenceMarkers(ChMarker* mark1, ChMarker* mark2) {
    this->SetUpMarkers(mark1, mark2);

    if (mark1)
        SetMarkID1(mark1->GetIdentifier());
    else
        SetMarkID1(0);
    if (mark2)
        SetMarkID2(mark2->GetIdentifier());
    else
        SetMarkID2(0);

    return mark1 && mark2;
}

void ChLinkMarkers::Initialize(std::shared_ptr<ChMarker> mmark1, std::shared_ptr<ChMarker> mmark2) {
    ChMarker* mm1 = mmark1.get();
    ChMarker* mm2 = mmark2.get();
    assert(mm1 && mm2);
    assert(mm1 != mm2);
    assert(mm1->GetBody() && mm2->GetBody());
    assert(mm1->GetBody()->GetSystem() == mm2->GetBody()->GetSystem());

    ReferenceMarkers(mm1, mm2);
}

void ChLinkMarkers::Initialize(std::shared_ptr<ChBody> mbody1,
                               std::shared_ptr<ChBody> mbody2,
                               const ChCoordsys<>& mpos) {
    return Initialize(mbody1, mbody2, false, mpos, mpos);
}

void ChLinkMarkers::Initialize(std::shared_ptr<ChBody> mbody1,
                               std::shared_ptr<ChBody> mbody2,
                               bool pos_are_relative,
                               const ChCoordsys<>& mpos1,
                               const ChCoordsys<>& mpos2) {
    assert(mbody1.get() != mbody2.get());
    assert(mbody1->GetSystem() == mbody2->GetSystem());

    // create markers to add to the two bodies
    std::shared_ptr<ChMarker> mmark1(new ChMarker);
    std::shared_ptr<ChMarker> mmark2(new ChMarker);
    mbody1->AddMarker(mmark1);
    mbody2->AddMarker(mmark2);

    ChMarker* mm1 = mmark1.get();
    ChMarker* mm2 = mmark2.get();
    ReferenceMarkers(mm1, mm2);

    if (pos_are_relative) {
        mmark1->Impose_Rel_Coord(mpos1);
        mmark2->Impose_Rel_Coord(mpos2);
    } else {
        mmark1->Impose_Abs_Coord(mpos1);
        mmark2->Impose_Abs_Coord(mpos2);
    }
}

// For all relative degrees of freedom of the two markers, compute relM, relM_dt,
// and relM_dtdt, as well as auxiliary data. Cache some intermediate quantities
// for possible reuse in UpdateState by some derived classes.
void ChLinkMarkers::UpdateRelMarkerCoords() {
    PQw = Vsub(marker1->GetAbsCoord().pos, marker2->GetAbsCoord().pos);
    PQw_dt = Vsub(marker1->GetAbsCoord_dt().pos, marker2->GetAbsCoord_dt().pos);
    PQw_dtdt = Vsub(marker1->GetAbsCoord_dtdt().pos, marker2->GetAbsCoord_dtdt().pos);

    dist = Vlength(PQw);                 // distance between origins, modulus
    dist_dt = Vdot(Vnorm(PQw), PQw_dt);  // speed between origins, modulus.

    Vector vtemp1;
    Vector vtemp2;
    Quaternion qtemp1;
    ChMatrixNM<double, 3, 4> relGw;

    Quaternion temp1 = marker1->GetCoord_dt().rot;
    Quaternion temp2 = marker2->GetCoord_dt().rot;

    if (Qnotnull(temp1) || Qnotnull(temp2)) {
        q_AD =  //  q'qqq + qqqq'
            Qadd(Qcross(Qconjugate(marker2->GetCoord_dt().rot),
                        Qcross(Qconjugate(marker2->GetBody()->GetCoord().rot),
                               Qcross((marker1->GetBody()->GetCoord().rot), (marker1->GetCoord().rot)))),
                 Qcross(Qconjugate(marker2->GetCoord().rot),
                        Qcross(Qconjugate(marker2->GetBody()->GetCoord().rot),
                               Qcross((marker1->GetBody()->GetCoord().rot), (marker1->GetCoord_dt().rot)))));
    } else
        q_AD = QNULL;

    q_BC =  // qq'qq + qqq'q
        Qadd(Qcross(Qconjugate(marker2->GetCoord().rot),
                    Qcross(Qconjugate(marker2->GetBody()->GetCoord_dt().rot),
                           Qcross((marker1->GetBody()->GetCoord().rot), (marker1->GetCoord().rot)))),
             Qcross(Qconjugate(marker2->GetCoord().rot),
                    Qcross(Qconjugate(marker2->GetBody()->GetCoord().rot),
                           Qcross((marker1->GetBody()->GetCoord_dt().rot), (marker1->GetCoord().rot)))));

    // q_8 = q''qqq + 2q'q'qq + 2q'qq'q + 2q'qqq'
    //     + 2qq'q'q + 2qq'qq' + 2qqq'q' + qqqq''
    temp2 = marker2->GetCoord_dtdt().rot;
    if (Qnotnull(temp2))
        q_8 = Qcross(Qconjugate(marker2->GetCoord_dtdt().rot),
                     Qcross(Qconjugate(Body2->GetCoord().rot),
                            Qcross(Body1->GetCoord().rot,
                                   marker1->GetCoord().rot)));  // q_dtdt'm2 * q'o2 * q,o1 * q,m1
    else
        q_8 = QNULL;
    temp1 = marker1->GetCoord_dtdt().rot;
    if (Qnotnull(temp1)) {
        qtemp1 = Qcross(Qconjugate(marker2->GetCoord().rot),
                        Qcross(Qconjugate(Body2->GetCoord().rot),
                               Qcross(Body1->GetCoord().rot,
                                      marker1->GetCoord_dtdt().rot)));  // q'm2 * q'o2 * q,o1 * q_dtdt,m1
        q_8 = Qadd(q_8, qtemp1);
    }
    temp2 = marker2->GetCoord_dt().rot;
    if (Qnotnull(temp2)) {
        qtemp1 = Qcross(
            Qconjugate(marker2->GetCoord_dt().rot),
            Qcross(Qconjugate(Body2->GetCoord_dt().rot), Qcross(Body1->GetCoord().rot, marker1->GetCoord().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q_dt'm2 * q_dt'o2 * q,o1 * q,m1)
        q_8 = Qadd(q_8, qtemp1);
    }
    temp2 = marker2->GetCoord_dt().rot;
    if (Qnotnull(temp2)) {
        qtemp1 = Qcross(
            Qconjugate(marker2->GetCoord_dt().rot),
            Qcross(Qconjugate(Body2->GetCoord().rot), Qcross(Body1->GetCoord_dt().rot, marker1->GetCoord().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q_dt'm2 * q'o2 * q_dt,o1 * q,m1)
        q_8 = Qadd(q_8, qtemp1);
    }
    temp1 = marker1->GetCoord_dt().rot;
    temp2 = marker2->GetCoord_dt().rot;
    if (Qnotnull(temp2) && Qnotnull(temp1)) {
        qtemp1 = Qcross(
            Qconjugate(marker2->GetCoord_dt().rot),
            Qcross(Qconjugate(Body2->GetCoord().rot), Qcross(Body1->GetCoord().rot, marker1->GetCoord_dt().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q_dt'm2 * q'o2 * q,o1 * q_dt,m1)
        q_8 = Qadd(q_8, qtemp1);
    }

    qtemp1 =
        Qcross(Qconjugate(marker2->GetCoord().rot),
               Qcross(Qconjugate(Body2->GetCoord_dt().rot), Qcross(Body1->GetCoord_dt().rot, marker1->GetCoord().rot)));
    qtemp1 = Qscale(qtemp1, 2);  // 2( q'm2 * q_dt'o2 * q_dt,o1 * q,m1)
    q_8 = Qadd(q_8, qtemp1);
    temp1 = marker1->GetCoord_dt().rot;
    if (Qnotnull(temp1)) {
        qtemp1 = Qcross(
            Qconjugate(marker2->GetCoord().rot),
            Qcross(Qconjugate(Body2->GetCoord_dt().rot), Qcross(Body1->GetCoord().rot, marker1->GetCoord_dt().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q'm2 * q_dt'o2 * q,o1 * q_dt,m1)
        q_8 = Qadd(q_8, qtemp1);
    }
    temp1 = marker1->GetCoord_dt().rot;
    if (Qnotnull(temp1)) {
        qtemp1 = Qcross(
            Qconjugate(marker2->GetCoord().rot),
            Qcross(Qconjugate(Body2->GetCoord().rot), Qcross(Body1->GetCoord_dt().rot, marker1->GetCoord_dt().rot)));
        qtemp1 = Qscale(qtemp1, 2);  // 2( q'm2 * q'o2 * q_dt,o1 * q_dt,m1)
        q_8 = Qadd(q_8, qtemp1);
    }

    // q_4 = [Adtdt]'[A]'q + 2[Adt]'[Adt]'q
    //       + 2[Adt]'[A]'qdt + 2[A]'[Adt]'qdt
    ChMatrix33<> m2_Rel_A_dt;
    marker2->Compute_Adt(m2_Rel_A_dt);
    ChMatrix33<> m2_Rel_A_dtdt;
    marker2->Compute_Adtdt(m2_Rel_A_dtdt);

    vtemp1 = Body2->GetA_dt().MatrT_x_Vect(PQw);
    vtemp2 = m2_Rel_A_dt.MatrT_x_Vect(vtemp1);
    q_4 = Vmul(vtemp2, 2);  // 2[Aq_dt]'[Ao2_dt]'*Qpq,w

    vtemp1 = Body2->GetA().MatrT_x_Vect(PQw_dt);
    vtemp2 = m2_Rel_A_dt.MatrT_x_Vect(vtemp1);
    vtemp2 = Vmul(vtemp2, 2);  // 2[Aq_dt]'[Ao2]'*Qpq,w_dt
    q_4 = Vadd(q_4, vtemp2);

    vtemp1 = Body2->GetA_dt().MatrT_x_Vect(PQw_dt);
    vtemp2 = marker2->GetA().MatrT_x_Vect(vtemp1);
    vtemp2 = Vmul(vtemp2, 2);  // 2[Aq]'[Ao2_dt]'*Qpq,w_dt
    q_4 = Vadd(q_4, vtemp2);

    vtemp1 = Body2->GetA().MatrT_x_Vect(PQw);
    vtemp2 = m2_Rel_A_dtdt.MatrT_x_Vect(vtemp1);
    q_4 = Vadd(q_4, vtemp2);  //  [Aq_dtdt]'[Ao2]'*Qpq,w

    // ----------- RELATIVE MARKER COORDINATES

    // relM.pos
    relM.pos = marker2->GetA().MatrT_x_Vect(Body2->GetA().MatrT_x_Vect(PQw));

    // relM.rot
    relM.rot = Qcross(Qconjugate(marker2->GetCoord().rot),
                      Qcross(Qconjugate(marker2->GetBody()->GetCoord().rot),
                             Qcross((marker1->GetBody()->GetCoord().rot), (marker1->GetCoord().rot))));

    // relM_dt.pos
    relM_dt.pos = Vadd(Vadd(m2_Rel_A_dt.MatrT_x_Vect(Body2->GetA().MatrT_x_Vect(PQw)),
                            marker2->GetA().MatrT_x_Vect(Body2->GetA_dt().MatrT_x_Vect(PQw))),
                       marker2->GetA().MatrT_x_Vect(Body2->GetA().MatrT_x_Vect(PQw_dt)));

    // relM_dt.rot
    relM_dt.rot = Qadd(q_AD, q_BC);

    // relM_dtdt.pos
    relM_dtdt.pos = Vadd(Vadd(marker2->GetA().MatrT_x_Vect(Body2->GetA_dtdt().MatrT_x_Vect(PQw)),
                              marker2->GetA().MatrT_x_Vect(Body2->GetA().MatrT_x_Vect(PQw_dtdt))),
                         q_4);

    // relM_dtdt.rot
    qtemp1 = Qcross(Qconjugate(marker2->GetCoord().rot),
                    Qcross(Qconjugate(Body2->GetCoord_dtdt().rot),
                           Qcross(Body1->GetCoord().rot,
                                  marker1->GetCoord().rot)));  // ( q'm2 * q_dtdt'o2 * q,o1 * q,m1)
    relM_dtdt.rot = Qadd(q_8, qtemp1);
    qtemp1 = Qcross(Qconjugate(marker2->GetCoord().rot),
                    Qcross(Qconjugate(Body2->GetCoord().rot),
                           Qcross(Body1->GetCoord_dtdt().rot,
                                  marker1->GetCoord().rot)));  // ( q'm2 * q'o2 * q_dtdt,o1 * q,m1)
    relM_dtdt.rot = Qadd(relM_dtdt.rot, qtemp1);               // = q_8 + qq''qq + qqq''q

    // ... and also "user-friendly" relative coordinates:

    // relAngle and relAxis
    Q_to_AngAxis(relM.rot, relAngle, relAxis);
    // flip rel rotation axis if jerky sign
    if (relAxis.z() < 0) {
        relAxis = Vmul(relAxis, -1);
        relAngle = -relAngle;
    }
    // rotation axis
    relRotaxis = Vmul(relAxis, relAngle);
    // relWvel
    ChFrame<>::SetMatrix_Gw(relGw, relM.rot);  // relGw.Set_Gw_matrix(relM.rot);
    relWvel = relGw.Matr34_x_Quat(relM_dt.rot);
    // relWacc
    relWacc = relGw.Matr34_x_Quat(relM_dtdt.rot);
}

void ChLinkMarkers::UpdateForces(double mytime) {
    C_force = VNULL;  // initialize int.forces accumulators
    C_torque = VNULL;

    // First and only operation: add the 'externally set' script forces (torques)
    C_force = Vadd(C_force, Scr_force);
    C_torque = Vadd(C_torque, Scr_torque);
}

void ChLinkMarkers::Update(double time, bool update_assets) {
    UpdateTime(time);
    UpdateRelMarkerCoords();
    UpdateForces(time);

    // Update assets
    ChPhysicsItem::Update(ChTime, update_assets);
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMarkers::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                      ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                      const double c           // a scaling factor
) {
    if (!Body1 || !Body2)
        return;

    Vector mbody_force;
    Vector mbody_torque;
    if (Vnotnull(C_force)) {
        Vector m_abs_force = Body2->GetA().Matr_x_Vect(marker2->GetA().Matr_x_Vect(C_force));

        if (Body2->Variables().IsActive()) {
            Body2->To_abs_forcetorque(m_abs_force,
                                      marker1->GetAbsCoord().pos,  // absolute application point is always marker1
                                      false,                       // from abs. space
                                      mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
            R.PasteSumVector(mbody_force * -c, Body2->Variables().GetOffset(), 0);
            R.PasteSumVector(Body2->TransformDirectionParentToLocal(mbody_torque) * -c,
                             Body2->Variables().GetOffset() + 3, 0);
        }

        if (Body1->Variables().IsActive()) {
            Body1->To_abs_forcetorque(m_abs_force,
                                      marker1->GetAbsCoord().pos,  // absolute application point is always marker1
                                      false,                       // from abs. space
                                      mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
            R.PasteSumVector(mbody_force * c, Body1->Variables().GetOffset(), 0);
            R.PasteSumVector(Body1->TransformDirectionParentToLocal(mbody_torque) * c,
                             Body1->Variables().GetOffset() + 3, 0);
        }
    }
    if (Vnotnull(C_torque)) {
        Vector m_abs_torque = Body2->GetA().Matr_x_Vect(marker2->GetA().Matr_x_Vect(C_torque));
        // load torques in 'fb' vector accumulator of body variables (torques in local coords)
        if (Body1->Variables().IsActive()) {
            R.PasteSumVector(Body1->TransformDirectionParentToLocal(m_abs_torque) * c,
                             Body1->Variables().GetOffset() + 3, 0);
        }
        if (Body2->Variables().IsActive()) {
            R.PasteSumVector(Body2->TransformDirectionParentToLocal(m_abs_torque) * -c,
                             Body2->Variables().GetOffset() + 3, 0);
        }
    }
}

// SOLVER INTERFACE

void ChLinkMarkers::ConstraintsFbLoadForces(double factor) {
    if (!Body1 || !Body2)
        return;

    Vector mbody_force;
    Vector mbody_torque;
    if (Vnotnull(C_force)) {
        Vector m_abs_force = Body2->GetA().Matr_x_Vect(marker2->GetA().Matr_x_Vect(C_force));
        Body2->To_abs_forcetorque(m_abs_force,
                                  marker1->GetAbsCoord().pos,  // absolute application point is always marker1
                                  false,                       // from abs. space
                                  mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
        Body2->Variables().Get_fb().PasteSumVector(mbody_force * -factor, 0, 0);
        Body2->Variables().Get_fb().PasteSumVector(Body2->TransformDirectionParentToLocal(mbody_torque) * -factor, 3,
                                                   0);

        Body1->To_abs_forcetorque(m_abs_force,
                                  marker1->GetAbsCoord().pos,  // absolute application point is always marker1
                                  false,                       // from abs. space
                                  mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
        Body1->Variables().Get_fb().PasteSumVector(mbody_force * factor, 0, 0);
        Body1->Variables().Get_fb().PasteSumVector(Body1->TransformDirectionParentToLocal(mbody_torque) * factor, 3, 0);
    }
    if (Vnotnull(C_torque)) {
        Vector m_abs_torque = Body2->GetA().Matr_x_Vect(marker2->GetA().Matr_x_Vect(C_torque));
        // load torques in 'fb' vector accumulator of body variables (torques in local coords)
        Body1->Variables().Get_fb().PasteSumVector(Body1->TransformDirectionParentToLocal(m_abs_torque) * factor, 3, 0);
        Body2->Variables().Get_fb().PasteSumVector(Body2->TransformDirectionParentToLocal(m_abs_torque) * -factor, 3,
                                                   0);
    }
}

void ChLinkMarkers::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMarkers>();

    // serialize parent class
    ChLink::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(markID1);
    marchive << CHNVP(markID2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMarkers::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMarkers>();

    // deserialize parent class
    ChLink::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(markID1);
    marchive >> CHNVP(markID2);
}

}  // end namespace chrono
