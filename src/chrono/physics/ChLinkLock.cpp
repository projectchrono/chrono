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
// Authors: Alessandro Tasora, Radu Serban, Arman Pazouki
// =============================================================================

//// TODO
////    Serialization/deserialization of unique_ptr members is currently commented
////    out until support for unique_ptr is implemented in ChArchive.

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLock)

ChLinkLock::ChLinkLock() : type(LinkType::FREE), ndoc_d(0), ndoc_c(0), ndoc(0), d_restlength(0) {
    // Need to zero out the bottom-right 4x3 block
    Cq1_temp.setZero();
    Cq2_temp.setZero();

    // Note: the joint is not completely built at this time.
    // Requires definition of the mask (based on concrete joint type)
}

ChLinkLock::ChLinkLock(const ChLinkLock& other) : ChLinkMarkers(other) {
    mask = other.mask;

    force_D.reset(other.force_D->Clone());
    force_R.reset(other.force_R->Clone());
    force_X.reset(other.force_X->Clone());
    force_Y.reset(other.force_Y->Clone());
    force_Z.reset(other.force_Z->Clone());
    force_Rx.reset(other.force_Rx->Clone());
    force_Ry.reset(other.force_Ry->Clone());
    force_Rz.reset(other.force_Rz->Clone());

    d_restlength = other.d_restlength;

    type = other.type;

    limit_X.reset(other.limit_X->Clone());
    limit_Y.reset(other.limit_Y->Clone());
    limit_Z.reset(other.limit_Z->Clone());
    limit_Rx.reset(other.limit_Rx->Clone());
    limit_Ry.reset(other.limit_Ry->Clone());
    limit_Rz.reset(other.limit_Rz->Clone());
    limit_Rp.reset(other.limit_Rp->Clone());
    limit_D.reset(other.limit_D->Clone());

    Ct_temp = other.Ct_temp;

    BuildLinkType(other.type);
}

ChLinkLock::~ChLinkLock() {}

//// Note: ability to explicitly provide joint forces was removed.
//// If ever needed, these functions can be re-enabled. In that case,
//// a typical user call would be:
////         auto my_force = std::make_unique<ChLinkForce>();
////         my_joint->SetForce_X(std::move(my_force));

/*
void ChLinkLock::SetForce_D(std::unique_ptr<ChLinkForce>&& force) {
    force_D = std::move(force);
}
void ChLinkLock::SetForce_R(std::unique_ptr<ChLinkForce>&& force) {
    force_R = std::move(force);
}
void ChLinkLock::SetForce_X(std::unique_ptr<ChLinkForce>&& force) {
    force_X = std::move(force);
}
void ChLinkLock::SetForce_Y(std::unique_ptr<ChLinkForce>&& force) {
    force_Y = std::move(force);
}
void ChLinkLock::SetForce_Z(std::unique_ptr<ChLinkForce>&& force) {
    force_Z = std::move(force);
}
void ChLinkLock::SetForce_Rx(std::unique_ptr<ChLinkForce>&& force) {
    force_Rx = std::move(force);
}
void ChLinkLock::SetForce_Ry(std::unique_ptr<ChLinkForce>&& force) {
    force_Ry = std::move(force);
}
void ChLinkLock::SetForce_Rz(std::unique_ptr<ChLinkForce>&& force) {
    force_Rz = std::move(force);
}
*/

ChLinkForce& ChLinkLock::GetForce_D() {
    if (!force_D)
        force_D = std::make_unique<ChLinkForce>();
    return *force_D;
}
ChLinkForce& ChLinkLock::GetForce_R() {
    if (!force_R)
        force_R = std::make_unique<ChLinkForce>();
    return *force_R;
}
ChLinkForce& ChLinkLock::GetForce_X() {
    if (!force_X)
        force_X = std::make_unique<ChLinkForce>();
    return *force_X;
}
ChLinkForce& ChLinkLock::GetForce_Y() {
    if (!force_Y)
        force_Y = std::make_unique<ChLinkForce>();
    return *force_Y;
}
ChLinkForce& ChLinkLock::GetForce_Z() {
    if (!force_Z)
        force_Z = std::make_unique<ChLinkForce>();
    return *force_Z;
}
ChLinkForce& ChLinkLock::GetForce_Rx() {
    if (!force_Rx)
        force_Rx = std::make_unique<ChLinkForce>();
    return *force_Rx;
}
ChLinkForce& ChLinkLock::GetForce_Ry() {
    if (!force_Ry)
        force_Ry = std::make_unique<ChLinkForce>();
    return *force_Ry;
}
ChLinkForce& ChLinkLock::GetForce_Rz() {
    if (!force_Rz)
        force_Rz = std::make_unique<ChLinkForce>();
    return *force_Rz;
}

//// Note: ability to explicitly provide limits was removed.
//// If ever needed, these functions can be re-enabled. In that case,
//// a typical user call would be:
////         auto my_limit = std::make_unique<ChLinkLimit>();
////         my_joint->SetLimit_X(std::move(my_force));

/*
void ChLinkLock::SetLimit_X(std::unique_ptr<ChLinkLimit>&& limit) {
    limit_X = std::move(limit);
}
void ChLinkLock::SetLimit_Y(std::unique_ptr<ChLinkLimit>&& limit) {
    limit_Y = std::move(limit);
}
void ChLinkLock::SetLimit_Z(std::unique_ptr<ChLinkLimit>&& limit) {
    limit_Z = std::move(limit);
}
void ChLinkLock::SetLimit_Rx(std::unique_ptr<ChLinkLimit>&& limit) {
    limit_Rx = std::move(limit);
}
void ChLinkLock::SetLimit_Ry(std::unique_ptr<ChLinkLimit>&& limit) {
    limit_Ry = std::move(limit);
}
void ChLinkLock::SetLimit_Rz(std::unique_ptr<ChLinkLimit>&& limit) {
    limit_Rz = std::move(limit);
}
void ChLinkLock::SetLimit_Rp(std::unique_ptr<ChLinkLimit>&& limit) {
    limit_Rp = std::move(limit);
}
void ChLinkLock::SetLimit_D(std::unique_ptr<ChLinkLimit>&& limit) {
    limit_D = std::move(limit);
}
*/

ChLinkLimit& ChLinkLock::GetLimit_X() {
    if (!limit_X)
        limit_X = std::make_unique<ChLinkLimit>();
    return *limit_X;
}
ChLinkLimit& ChLinkLock::GetLimit_Y() {
    if (!limit_Y)
        limit_Y = std::make_unique<ChLinkLimit>();
    return *limit_Y;
}
ChLinkLimit& ChLinkLock::GetLimit_Z() {
    if (!limit_Z)
        limit_Z = std::make_unique<ChLinkLimit>();
    return *limit_Z;
}
ChLinkLimit& ChLinkLock::GetLimit_Rx() {
    if (!limit_Rx)
        limit_Rx = std::make_unique<ChLinkLimit>();
    return *limit_Rx;
}
ChLinkLimit& ChLinkLock::GetLimit_Ry() {
    if (!limit_Ry)
        limit_Ry = std::make_unique<ChLinkLimit>();
    return *limit_Ry;
}
ChLinkLimit& ChLinkLock::GetLimit_Rz() {
    if (!limit_Rz)
        limit_Rz = std::make_unique<ChLinkLimit>();
    return *limit_Rz;
}
ChLinkLimit& ChLinkLock::GetLimit_Rp() {
    if (!limit_Rp)
        limit_Rp = std::make_unique<ChLinkLimit>();
    return *limit_Rp;
}
ChLinkLimit& ChLinkLock::GetLimit_D() {
    if (!limit_D)
        limit_D = std::make_unique<ChLinkLimit>();
    return *limit_D;
}

void ChLinkLock::SetDisabled(bool mdis) {
    ChLinkMarkers::SetDisabled(mdis);

    if (mask.SetAllDisabled(mdis) > 0)
        BuildLink();
}

void ChLinkLock::SetBroken(bool mbro) {
    ChLinkMarkers::SetBroken(mbro);

    if (mask.SetAllBroken(mbro) > 0)
        BuildLink();
}

int ChLinkLock::RestoreRedundant() {
    int mchanges = mask.RestoreRedundant();
    if (mchanges)
        BuildLink();

    return mchanges;
}

void ChLinkLock::SetUpMarkers(ChMarker* mark1, ChMarker* mark2) {
    ChLinkMarkers::SetUpMarkers(mark1, mark2);
    assert(this->Body1 && this->Body2);

    mask.SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());

    // We must call BuildLink here again, because only now are the constraints properly activated
    // (and hence the correct NDOC available).
    BuildLink();
}

void ChLinkLock::BuildLink() {
    // set ndoc by counting non-dofs
    ndoc = mask.GetMaskDoc();
    ndoc_c = mask.GetMaskDoc_c();
    ndoc_d = mask.GetMaskDoc_d();

    // create matrices
    C.resize(ndoc);
    C_dt.resize(ndoc);
    C_dtdt.resize(ndoc);
    react.resize(ndoc);
    Qc.resize(ndoc);
    Ct.resize(ndoc);
    Cq1.resize(ndoc, BODY_QDOF);
    Cq2.resize(ndoc, BODY_QDOF);
    Cqw1.resize(ndoc, BODY_DOF);
    Cqw2.resize(ndoc, BODY_DOF);

    // Zero out vectors of constraint violations
    C.setZero();
    C_dt.setZero();
    C_dtdt.setZero();
    react.setZero();

    // Need to zero out the first 3 entries in rows corrsponding to rotation constraints
    Cq1.setZero();
    Cq2.setZero();
}

void ChLinkLock::BuildLink(bool x, bool y, bool z, bool e0, bool e1, bool e2, bool e3) {
    mask.SetLockMask(x, y, z, e0, e1, e2, e3);
    BuildLink();
}

void ChLinkLock::BuildLinkType(LinkType link_type) {
    type = link_type;

    // SetLockMask() sets the constraints for the link coordinates: (X,Y,Z, E0,E1,E2,E3)
    switch (type) {
        case LinkType::FREE:
            BuildLink(false, false, false, false, false, false, false);
            break;
        case LinkType::LOCK:
            // this should never happen
            BuildLink(true, true, true, false, true, true, true);
            break;
        case LinkType::SPHERICAL:
            BuildLink(true, true, true, false, false, false, false);
            break;
        case LinkType::POINTPLANE:
            BuildLink(false, false, true, false, false, false, false);
            break;
        case LinkType::POINTLINE:
            BuildLink(false, true, true, false, false, false, false);
            break;
        case LinkType::REVOLUTE:
            BuildLink(true, true, true, false, true, true, false);
            break;
        case LinkType::CYLINDRICAL:
            BuildLink(true, true, false, false, true, true, false);
            break;
        case LinkType::PRISMATIC:
            BuildLink(true, true, false, false, true, true, true);
            break;
        case LinkType::PLANEPLANE:
            BuildLink(false, false, true, false, true, true, false);
            break;
        case LinkType::OLDHAM:
            BuildLink(false, false, true, false, true, true, true);
            break;
        case LinkType::ALIGN:
            BuildLink(false, false, false, false, true, true, true);
        case LinkType::PARALLEL:
            BuildLink(false, false, false, false, true, true, false);
            break;
        case LinkType::PERPEND:
            BuildLink(false, false, false, false, true, false, true);
            break;
        case LinkType::REVOLUTEPRISMATIC:
            BuildLink(false, true, true, false, true, true, false);
            break;
        default:
            BuildLink(false, false, false, false, false, false, false);
            break;
    }
}

void ChLinkLock::ChangeLinkType(LinkType new_link_type) {
    BuildLinkType(new_link_type);

    limit_X.reset(nullptr);
    limit_Y.reset(nullptr);
    limit_Z.reset(nullptr);
    limit_Rx.reset(nullptr);
    limit_Ry.reset(nullptr);
    limit_Rz.reset(nullptr);
    limit_D.reset(nullptr);
    limit_Rp.reset(nullptr);
}

// setup the functions when user changes them.

// -----------------------------------------------------------------------------
// UPDATING PROCEDURES

// Complete update.
void ChLinkLock::Update(double time, bool update_assets) {
    UpdateTime(time);
    UpdateRelMarkerCoords();
    UpdateState();
    UpdateCqw();
    UpdateForces(time);

    // Update assets
    ChPhysicsItem::Update(ChTime, update_assets);
}

// Updates Cq1_temp, Cq2_temp, Qc_temp, etc., i.e. all LOCK-FORMULATION temp.matrices
void ChLinkLock::UpdateState() {
    // ----------- SOME PRECALCULATED VARIABLES, to optimize speed

    ChStarMatrix33<> P1star(marker1->GetCoord().pos);  // [P] star matrix of rel pos of mark1
    ChStarMatrix33<> Q2star(marker2->GetCoord().pos);  // [Q] star matrix of rel pos of mark2

    ChGlMatrix34<> body1Gl(Body1->GetCoord().rot);
    ChGlMatrix34<> body2Gl(Body2->GetCoord().rot);

    // COMPUTE THE  Cq Ct Qc    matrices (temporary, for complete lock constraint)

    ChMatrix33<> m2_Rel_A_dt;
    marker2->Compute_Adt(m2_Rel_A_dt);
    ChMatrix33<> m2_Rel_A_dtdt;
    marker2->Compute_Adtdt(m2_Rel_A_dtdt);

    // ----------- PARTIAL DERIVATIVE Ct OF CONSTRAINT
    Ct_temp.pos =
        m2_Rel_A_dt.transpose() * (Body2->GetA().transpose() * PQw) +
        marker2->GetA().transpose() *
            (Body2->GetA().transpose() * (Body1->GetA() * marker1->GetCoord_dt().pos) - marker2->GetCoord_dt().pos);

    Ct_temp.rot = q_AD;

    //------------ COMPLETE JACOBIANS Cq1_temp AND Cq2_temp AND Qc_temp VECTOR.
    // [Cq_temp]= [[CqxT] [CqxR]]     {Qc_temp} ={[Qcx]}
    //            [[ 0  ] [CqrR]]                {[Qcr]}

    //  JACOBIANS Cq1_temp, Cq2_temp:

    ChMatrix33<> CqxT = marker2->GetA().transpose() * Body2->GetA().transpose();  // [CqxT]=[Aq]'[Ao2]'
    ChStarMatrix33<> tmpStar(Body2->GetA().transpose() * PQw);

    Cq1_temp.topLeftCorner<3, 3>() = CqxT;                                              // *- -- Cq1_temp(1-3) =  [Aqo2]
    Cq2_temp.topLeftCorner<3, 3>() = -CqxT;                                             // -- *- Cq2_temp(1-3) = -[Aqo2]
    Cq1_temp.topRightCorner<3, 4>() = -CqxT * Body1->GetA() * P1star * body1Gl;         // -* -- Cq1_temp(4-7)
    Cq2_temp.topRightCorner<3, 4>() = CqxT * Body2->GetA() * Q2star * body2Gl +         //
                                      marker2->GetA().transpose() * tmpStar * body2Gl;  // -- -* Cq2_temp(4-7)

    {
        ChStarMatrix44<> stempQ1(Qcross(Qconjugate(marker2->GetCoord().rot), Qconjugate(Body2->GetCoord().rot)));
        ChStarMatrix44<> stempQ2(marker1->GetCoord().rot);
        stempQ2.semiTranspose();
        Cq1_temp.bottomRightCorner<4, 4>() = stempQ1 * stempQ2;  // =* == Cq1_temp(col 4-7, row 4-7) ... CqrR
    }

    {
        ChStarMatrix44<> stempQ1(Qconjugate(marker2->GetCoord().rot));
        ChStarMatrix44<> stempQ2(Qcross(Body1->GetCoord().rot, marker1->GetCoord().rot));
        stempQ2.semiTranspose();
        stempQ2.semiNegate();
        Cq2_temp.bottomRightCorner<4, 4>() = stempQ1 * stempQ2;  // == =* Cq2_temp(col 4-7, row 4-7) ... CqrR
    }

    //--------- COMPLETE Qc VECTOR
    ChVector<> Qcx;
    ChVector<> vtemp1;
    ChVector<> vtemp2;

    vtemp1 = Vcross(Body1->GetWvel_loc(), Vcross(Body1->GetWvel_loc(), marker1->GetCoord().pos));
    vtemp1 = Vadd(vtemp1, marker1->GetCoord_dtdt().pos);
    vtemp1 = Vadd(vtemp1, Vmul(Vcross(Body1->GetWvel_loc(), marker1->GetCoord_dt().pos), 2));

    vtemp2 = Vcross(Body2->GetWvel_loc(), Vcross(Body2->GetWvel_loc(), marker2->GetCoord().pos));
    vtemp2 = Vadd(vtemp2, marker2->GetCoord_dtdt().pos);
    vtemp2 = Vadd(vtemp2, Vmul(Vcross(Body2->GetWvel_loc(), marker2->GetCoord_dt().pos), 2));

    Qcx = CqxT * (Body1->GetA() * vtemp1 - Body2->GetA() * vtemp2);

    ChStarMatrix33<> mtemp1(Body2->GetWvel_loc());
    ChMatrix33<> mtemp3 = Body2->GetA() * mtemp1 * mtemp1;
    vtemp2 = marker2->GetA().transpose() * (mtemp3.transpose() * PQw);  // [Aq]'[[A2][w2][w2]]'*Qpq,w
    Qcx = Vadd(Qcx, vtemp2);
    Qcx = Vadd(Qcx, q_4);  // [Adtdt]'[A]'q + 2[Adt]'[Adt]'q + 2[Adt]'[A]'qdt + 2[A]'[Adt]'qdt

    Qc_temp.segment(0, 3) = Qcx.eigen();  // * Qc_temp, for all translational coords
    Qc_temp.segment(3, 4) = q_8.eigen();  // * Qc_temp, for all rotational coords (Qcr = q_8)

    // *** NOTE! The final Qc must change sign, to be used in
    // lagrangian equation:    [Cq]*q_dtdt = Qc
    // because until now we have computed it as [Cq]*q_dtdt + "Qc" = 0
    Qc_temp *= -1;

    // ---------------------
    // Updates Cq1, Cq2, Qc,
    // C, C_dt, C_dtdt, Ct.
    // ---------------------
    int index = 0;

    if (mask.Constr_X().IsActive()) {
        Cq1.block<1, 7>(index, 0) = Cq1_temp.block<1, 7>(0, 0);
        Cq2.block<1, 7>(index, 0) = Cq2_temp.block<1, 7>(0, 0);

        Qc(index) = Qc_temp(0);

        C(index) = relM.pos.x();
        C_dt(index) = relM_dt.pos.x();
        C_dtdt(index) = relM_dtdt.pos.x();

        Ct(index) = Ct_temp.pos.x();

        index++;
    }

    if (mask.Constr_Y().IsActive()) {
        Cq1.block<1, 7>(index, 0) = Cq1_temp.block<1, 7>(1, 0);
        Cq2.block<1, 7>(index, 0) = Cq2_temp.block<1, 7>(1, 0);

        Qc(index) = Qc_temp(1);

        C(index) = relM.pos.y();
        C_dt(index) = relM_dt.pos.y();
        C_dtdt(index) = relM_dtdt.pos.y();

        Ct(index) = Ct_temp.pos.y();

        index++;
    }

    if (mask.Constr_Z().IsActive()) {
        Cq1.block<1, 7>(index, 0) = Cq1_temp.block<1, 7>(2, 0);
        Cq2.block<1, 7>(index, 0) = Cq2_temp.block<1, 7>(2, 0);

        Qc(index) = Qc_temp(2);

        C(index) = relM.pos.z();
        C_dt(index) = relM_dt.pos.z();
        C_dtdt(index) = relM_dtdt.pos.z();

        Ct(index) = Ct_temp.pos.z();

        index++;
    }

    if (mask.Constr_E0().IsActive()) {
        Cq1.block<1, 4>(index, 3) = Cq1_temp.block<1, 4>(3, 3);
        Cq2.block<1, 4>(index, 3) = Cq2_temp.block<1, 4>(3, 3);

        Qc(index) = Qc_temp(3);

        C(index) = relM.rot.e0();
        C_dt(index) = relM_dt.rot.e0();
        C_dtdt(index) = relM_dtdt.rot.e0();

        Ct(index) = Ct_temp.rot.e0();

        index++;
    }

    if (mask.Constr_E1().IsActive()) {
        Cq1.block<1, 4>(index, 3) = Cq1_temp.block<1, 4>(4, 3);
        Cq2.block<1, 4>(index, 3) = Cq2_temp.block<1, 4>(4, 3);

        Qc(index) = Qc_temp(4);

        C(index) = relM.rot.e1();
        C_dt(index) = relM_dt.rot.e1();
        C_dtdt(index) = relM_dtdt.rot.e1();

        Ct(index) = Ct_temp.rot.e1();

        index++;
    }

    if (mask.Constr_E2().IsActive()) {
        Cq1.block<1, 4>(index, 3) = Cq1_temp.block<1, 4>(5, 3);
        Cq2.block<1, 4>(index, 3) = Cq2_temp.block<1, 4>(5, 3);

        Qc(index) = Qc_temp(5);

        C(index) = relM.rot.e2();
        C_dt(index) = relM_dt.rot.e2();
        C_dtdt(index) = relM_dtdt.rot.e2();

        Ct(index) = Ct_temp.rot.e2();

        index++;
    }

    if (mask.Constr_E3().IsActive()) {
        Cq1.block<1, 4>(index, 3) = Cq1_temp.block<1, 4>(6, 3);
        Cq2.block<1, 4>(index, 3) = Cq2_temp.block<1, 4>(6, 3);

        Qc(index) = Qc_temp(6);

        C(index) = relM.rot.e3();
        C_dt(index) = relM_dt.rot.e3();
        C_dtdt(index) = relM_dtdt.rot.e3();

        Ct(index) = Ct_temp.rot.e3();

        index++;
    }
}

static void Transform_Cq_to_Cqw(const ChLinkLock::ChConstraintMatrixX7& mCq,
                                ChLinkLock::ChConstraintMatrixX6& mCqw,
                                ChBodyFrame* mbody) {
    // translational part - not changed
    mCqw.block(0, 0, mCq.rows(), 3) = mCq.block(0, 0, mCq.rows(), 3);

    // rotational part [Cq_w] = [Cq_q]*[Gl]'*1/4
    ChGlMatrix34<> mGl(mbody->GetCoord().rot);
    for (int colres = 0; colres < 3; colres++) {
        for (int row = 0; row < mCq.rows(); row++) {
            double sum = 0;
            for (int col = 0; col < 4; col++) {
                sum += mCq(row, col + 3) * mGl(colres, col);
            }
            mCqw(row, colres + 3) = sum * 0.25;
        }
    }
    //// RADU: explicit loop slightly more performant than Eigen expressions
    ////mCqw.block(0, 3, mCq.rows(), 3) = 0.25 * mCq.block(0, 3, mCq.rows(), 4) * mGl.transpose();
}

void ChLinkLock::UpdateCqw() {
    Transform_Cq_to_Cqw(Cq1, Cqw1, Body1);
    Transform_Cq_to_Cqw(Cq2, Cqw2, Body2);
}

// Override UpdateForces to include possible contributions from joint limits.
void ChLinkLock::UpdateForces(double mytime) {
    ChLinkMarkers::UpdateForces(mytime);

    ChVector<> m_force = VNULL;
    ChVector<> m_torque = VNULL;

    // COMPUTE THE FORCES IN THE LINK, FOR EXAMPLE
    // CAUSED BY SPRINGS
    // NOTE!!!!!   C_force and C_torque   are considered in the reference coordsystem
    // of marker2  (the MAIN marker), and their application point is considered the
    // origin of marker1 (the SLAVE marker)

    // 1)========== the generic spring-damper

    if (force_D && force_D->IsActive()) {
        double dfor;
        dfor = force_D->GetForce((dist - d_restlength), dist_dt, ChTime);
        m_force = Vmul(Vnorm(relM.pos), dfor);

        C_force = Vadd(C_force, m_force);
    }

    // 2)========== the generic torsional spring / torsional damper

    if (force_R && force_R->IsActive()) {
        double tor;
        // 1) the tors. spring
        tor = force_R->GetForce(relAngle, 0, ChTime);
        m_torque = Vmul(relAxis, tor);
        C_torque = Vadd(C_torque, m_torque);
        // 2) the tors. damper
        double angle_dt = Vlength(relWvel);
        tor = force_R->GetForce(0, angle_dt, ChTime);
        m_torque = Vmul(Vnorm(relWvel), tor);
        C_torque = Vadd(C_torque, m_torque);
    }

    // 3)========== the XYZ forces

    m_force = VNULL;

    if (force_X && force_X->IsActive()) {
        m_force.x() = force_X->GetForce(relM.pos.x(), relM_dt.pos.x(), ChTime);
    }

    if (force_Y && force_Y->IsActive()) {
        m_force.y() = force_Y->GetForce(relM.pos.y(), relM_dt.pos.y(), ChTime);
    }

    if (force_Z && force_Z->IsActive()) {
        m_force.z() = force_Z->GetForce(relM.pos.z(), relM_dt.pos.z(), ChTime);
    }

    C_force = Vadd(C_force, m_force);

    // 4)========== the RxRyRz forces (torques)

    m_torque = VNULL;

    if (force_Rx && force_Rx->IsActive()) {
        m_torque.x() = force_Rx->GetForce(relRotaxis.x(), relWvel.x(), ChTime);
    }

    if (force_Ry && force_Ry->IsActive()) {
        m_torque.y() = force_Ry->GetForce(relRotaxis.y(), relWvel.y(), ChTime);
    }

    if (force_Rz && force_Rz->IsActive()) {
        m_torque.z() = force_Rz->GetForce(relRotaxis.z(), relWvel.z(), ChTime);
    }

    C_torque = Vadd(C_torque, m_torque);

    // ========== the link-limits "cushion forces"

    m_force = VNULL;
    m_torque = VNULL;

    if (limit_X && limit_X->IsActive()) {
        m_force.x() = limit_X->GetForce(relM.pos.x(), relM_dt.pos.x());
    }
    if (limit_Y && limit_Y->IsActive()) {
        m_force.y() = limit_Y->GetForce(relM.pos.y(), relM_dt.pos.y());
    }
    if (limit_Z && limit_Z->IsActive()) {
        m_force.z() = limit_Z->GetForce(relM.pos.z(), relM_dt.pos.z());
    }

    if (limit_D && limit_D->IsActive()) {
        m_force = Vadd(m_force, Vmul(Vnorm(relM.pos), limit_D->GetForce(dist, dist_dt)));
    }

    if (limit_Rx && limit_Rx->IsActive()) {
        m_torque.x() = limit_Rx->GetForce(relRotaxis.x(), relWvel.x());
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        m_torque.y() = limit_Ry->GetForce(relRotaxis.y(), relWvel.y());
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        m_torque.z() = limit_Rz->GetForce(relRotaxis.z(), relWvel.z());
    }
    if (limit_Rp && limit_Rp->IsActive()) {
        ChVector<> arm_xaxis = VaxisXfromQuat(relM.rot);  // the X axis of the marker1, respect to m2.
        double zenith = VangleYZplaneNorm(arm_xaxis);     // the angle of m1 Xaxis about normal to YZ plane
        double polar = VangleRX(arm_xaxis);               // the polar angle of m1 Xaxis spinning about m2 Xaxis

        ChVector<> projected_arm(0, arm_xaxis.y(), arm_xaxis.z());
        ChVector<> torq_axis;
        torq_axis = Vcross(VECT_X, projected_arm);
        torq_axis = Vnorm(torq_axis);  // the axis of torque, laying on YZ plane.

        double zenithspeed = Vdot(torq_axis, relWvel);  // the speed of zenith rotation toward cone.

        m_torque = Vadd(m_torque, Vmul(torq_axis, limit_Rp->GetPolarForce(zenith, zenithspeed, polar)));
    }

    C_force = Vadd(C_force, m_force);     // +++
    C_torque = Vadd(C_torque, m_torque);  // +++

    // ========== other forces??
}

// Count also unilateral constraints from joint limits (if any)
int ChLinkLock::GetDOC_d() {
    int mdocd = ndoc_d;

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive())
            ++mdocd;
        if (limit_X->constr_upper.IsActive())
            ++mdocd;
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive())
            ++mdocd;
        if (limit_Y->constr_upper.IsActive())
            ++mdocd;
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive())
            ++mdocd;
        if (limit_Z->constr_upper.IsActive())
            ++mdocd;
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive())
            ++mdocd;
        if (limit_Rx->constr_upper.IsActive())
            ++mdocd;
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive())
            ++mdocd;
        if (limit_Ry->constr_upper.IsActive())
            ++mdocd;
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive())
            ++mdocd;
        if (limit_Rz->constr_upper.IsActive())
            ++mdocd;
    }

    return mdocd;
}

void ChLinkLock::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react_force = VNULL;
    react_torque = VNULL;

    react = L.segment(off_L, react.size());

    // From react vector to the 'intuitive' react_force and react_torque
    const ChQuaternion<>& q2 = Body2->GetRot();
    const ChQuaternion<>& q1p = marker1->GetAbsCoord().rot;
    const ChQuaternion<>& qs = marker2->GetCoord().rot;
    const ChMatrix33<>& Cs = marker2->GetA();

    ChMatrix44<> Chi__q1p_barT;  //[Chi] * [transpose(bar(q1p))]
    Chi__q1p_barT(0, 0) = q1p.e0();
    Chi__q1p_barT(0, 1) = q1p.e1();
    Chi__q1p_barT(0, 2) = q1p.e2();
    Chi__q1p_barT(0, 3) = q1p.e3();
    Chi__q1p_barT(1, 0) = q1p.e1();
    Chi__q1p_barT(1, 1) = -q1p.e0();
    Chi__q1p_barT(1, 2) = q1p.e3();
    Chi__q1p_barT(1, 3) = -q1p.e2();
    Chi__q1p_barT(2, 0) = q1p.e2();
    Chi__q1p_barT(2, 1) = -q1p.e3();
    Chi__q1p_barT(2, 2) = -q1p.e0();
    Chi__q1p_barT(2, 3) = q1p.e1();
    Chi__q1p_barT(3, 0) = q1p.e3();
    Chi__q1p_barT(3, 1) = q1p.e2();
    Chi__q1p_barT(3, 2) = -q1p.e1();
    Chi__q1p_barT(3, 3) = -q1p.e0();

    ChMatrix44<> qs_tilde;
    qs_tilde(0, 0) = qs.e0();
    qs_tilde(0, 1) = -qs.e1();
    qs_tilde(0, 2) = -qs.e2();
    qs_tilde(0, 3) = -qs.e3();
    qs_tilde(1, 0) = qs.e1();
    qs_tilde(1, 1) = qs.e0();
    qs_tilde(1, 2) = -qs.e3();
    qs_tilde(1, 3) = qs.e2();
    qs_tilde(2, 0) = qs.e2();
    qs_tilde(2, 1) = qs.e3();
    qs_tilde(2, 2) = qs.e0();
    qs_tilde(2, 3) = -qs.e1();
    qs_tilde(3, 0) = qs.e3();
    qs_tilde(3, 1) = -qs.e2();
    qs_tilde(3, 2) = qs.e1();
    qs_tilde(3, 3) = qs.e0();

    // Ts = 0.5*CsT*G(q2)*Chi*(q1 qp)_barT*qs~*KT*lambda
    ChGlMatrix34<> Gl_q2(q2);
    ChMatrix34<> Ts = 0.25 * Cs.transpose() * Gl_q2 * Chi__q1p_barT * qs_tilde;

    // Translational constraint reaction force = -lambda_translational
    // Translational constraint reaction torque = -d~''(t)*lambda_translational
    // No reaction force from the rotational constraints

    int local_off = 0;

    if (mask.Constr_X().IsActive()) {
        react_force.x() = -react(local_off);
        react_torque.y() = -relM.pos.z() * react(local_off);
        react_torque.z() = relM.pos.y() * react(local_off);
        local_off++;
    }
    if (mask.Constr_Y().IsActive()) {
        react_force.y() = -react(local_off);
        react_torque.x() = relM.pos.z() * react(local_off);
        react_torque.z() += -relM.pos.x() * react(local_off);
        local_off++;
    }
    if (mask.Constr_Z().IsActive()) {
        react_force.z() = -react(local_off);
        react_torque.x() += -relM.pos.y() * react(local_off);
        react_torque.y() += relM.pos.x() * react(local_off);
        local_off++;
    }

    if (mask.Constr_E1().IsActive()) {
        react_torque.x() += Ts(0, 1) * (react(local_off));
        react_torque.y() += Ts(1, 1) * (react(local_off));
        react_torque.z() += Ts(2, 1) * (react(local_off));
        local_off++;
    }
    if (mask.Constr_E2().IsActive()) {
        react_torque.x() += Ts(0, 2) * (react(local_off));
        react_torque.y() += Ts(1, 2) * (react(local_off));
        react_torque.z() += Ts(2, 2) * (react(local_off));
        local_off++;
    }
    if (mask.Constr_E3().IsActive()) {
        react_torque.x() += Ts(0, 3) * (react(local_off));
        react_torque.y() += Ts(1, 3) * (react(local_off));
        react_torque.z() += Ts(2, 3) * (react(local_off));
        local_off++;
    }

    // ***TO DO***?: TRASFORMATION FROM delta COORDS TO LINK COORDS, if
    // non-default delta
    // if delta rotation?

    // add also the contribution from link limits to the react_force and
    // react_torque.
    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            react_force.x() -= L(off_L + local_off);
            local_off++;
        }
        if (limit_X->constr_upper.IsActive()) {
            react_force.x() += L(off_L + local_off);
            local_off++;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            react_force.y() -= L(off_L + local_off);
            local_off++;
        }
        if (limit_Y->constr_upper.IsActive()) {
            react_force.y() += L(off_L + local_off);
            local_off++;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            react_force.z() -= L(off_L + local_off);
            local_off++;
        }
        if (limit_Z->constr_upper.IsActive()) {
            react_force.z() += L(off_L + local_off);
            local_off++;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            react_torque.x() -= 0.5 * L(off_L + local_off);
            local_off++;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            react_torque.x() += 0.5 * L(off_L + local_off);
            local_off++;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            react_torque.y() -= 0.5 * L(off_L + local_off);
            local_off++;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            react_torque.y() += 0.5 * L(off_L + local_off);
            local_off++;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            react_torque.z() -= 0.5 * L(off_L + local_off);
            local_off++;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            react_torque.z() += 0.5 * L(off_L + local_off);
            local_off++;
        }
    }

    // the internal forces add their contribute to the reactions
    // NOT NEEDED?, since C_force and react_force must stay separated???
    // react_force  = Vadd(react_force, C_force);
    // react_torque = Vadd(react_torque, C_torque);
}

void ChLinkLock::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L.segment(off_L, react.size()) = react;

    int local_off = this->GetDOC_c();

    // gather also the contribution from link limits
    // TODO not yet implemented
}

void ChLinkLock::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                     ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  // the L vector
                                     const double c)              // a scaling factor
{
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            mask.Constr_N(i).MultiplyTandAdd(R, L(off_L + cnt) * c);
            cnt++;
        }
    }

    int local_offset = this->GetDOC_c();

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.MultiplyTandAdd(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
}

void ChLinkLock::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                     ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                     const double c,            // a scaling factor
                                     bool do_clamp,             // apply clamping to c*C?
                                     double recovery_clamp)     // value for min/max clamping of c*C
{
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            if (do_clamp) {
                if (mask.Constr_N(i).IsUnilateral())
                    Qc(off_L + cnt) += ChMax(c * C(cnt), -recovery_clamp);
                else
                    Qc(off_L + cnt) += ChMin(ChMax(c * C(cnt), -recovery_clamp), recovery_clamp);
            } else
                Qc(off_L + cnt) += c * C(cnt);
            cnt++;
        }
    }

    if (!do_clamp)
        recovery_clamp = 1e24;

    int local_offset = this->GetDOC_c();

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (-limit_X->GetMin() + relM.pos.x()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_X->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (limit_X->GetMax() - relM.pos.x()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (-limit_Y->GetMin() + relM.pos.y()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Y->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (limit_Y->GetMax() - relM.pos.y()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (-limit_Z->GetMin() + relM.pos.z()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Z->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (limit_Z->GetMax() - relM.pos.z()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (-sin(0.5 * limit_Rx->GetMin()) + relM.rot.e1()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (sin(0.5 * limit_Rx->GetMax()) - relM.rot.e1()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (-sin(0.5 * limit_Ry->GetMin()) + relM.rot.e2()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (sin(0.5 * limit_Ry->GetMax()) - relM.rot.e2()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (-sin(0.5 * limit_Rz->GetMin()) + relM.rot.e3()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += ChMax(c * (sin(0.5 * limit_Rz->GetMax()) - relM.rot.e3()), -recovery_clamp);
            ++local_offset;
        }
    }
}

void ChLinkLock::IntLoadConstraint_Ct(const unsigned int off_L,  // offset in Qc residual
                                      ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*Ct
                                      const double c)            // a scaling factor
{
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            Qc(off_L + cnt) += c * Ct(cnt);
            cnt++;
        }
    }

    // nothing to do for ChLinkLimit
}

void ChLinkLock::IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            mask.Constr_N(i).Set_l_i(L(off_L + cnt));
            mask.Constr_N(i).Set_b_i(Qc(off_L + cnt));
            cnt++;
        }
    }

    int local_offset = this->GetDOC_c();

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.Set_l_i(L(off_L + local_offset));
            limit_X->constr_lower.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.Set_l_i(L(off_L + local_offset));
            limit_X->constr_upper.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.Set_l_i(L(off_L + local_offset));
            limit_Y->constr_lower.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.Set_l_i(L(off_L + local_offset));
            limit_Y->constr_upper.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.Set_l_i(L(off_L + local_offset));
            limit_Z->constr_lower.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.Set_l_i(L(off_L + local_offset));
            limit_Z->constr_upper.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.Set_l_i(L(off_L + local_offset));
            limit_Rx->constr_lower.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.Set_l_i(L(off_L + local_offset));
            limit_Rx->constr_upper.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.Set_l_i(L(off_L + local_offset));
            limit_Ry->constr_lower.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.Set_l_i(L(off_L + local_offset));
            limit_Ry->constr_upper.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.Set_l_i(L(off_L + local_offset));
            limit_Rz->constr_lower.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.Set_l_i(L(off_L + local_offset));
            limit_Rz->constr_upper.Set_b_i(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
}

void ChLinkLock::IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) {
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            L(off_L + cnt) = mask.Constr_N(i).Get_l_i();
            cnt++;
        }
    }

    int local_offset = this->GetDOC_c();

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_X->constr_lower.Get_l_i();
            ++local_offset;
        }
        if (limit_X->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_X->constr_upper.Get_l_i();
            ++local_offset;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Y->constr_lower.Get_l_i();
            ++local_offset;
        }
        if (limit_Y->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Y->constr_upper.Get_l_i();
            ++local_offset;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Z->constr_lower.Get_l_i();
            ++local_offset;
        }
        if (limit_Z->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Z->constr_upper.Get_l_i();
            ++local_offset;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Rx->constr_lower.Get_l_i();
            ++local_offset;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Rx->constr_upper.Get_l_i();
            ++local_offset;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Ry->constr_lower.Get_l_i();
            ++local_offset;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Ry->constr_upper.Get_l_i();
            ++local_offset;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Rz->constr_lower.Get_l_i();
            ++local_offset;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Rz->constr_upper.Get_l_i();
            ++local_offset;
        }
    }
}

void ChLinkLock::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    if (!this->IsActive())
        return;

    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive())
            mdescriptor.InsertConstraint(&mask.Constr_N(i));
    }

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_X->constr_lower);
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_X->constr_upper);
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Y->constr_lower);
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Y->constr_upper);
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Z->constr_lower);
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Z->constr_upper);
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Rx->constr_lower);
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Rx->constr_upper);
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Ry->constr_lower);
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Ry->constr_upper);
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Rz->constr_lower);
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            mdescriptor.InsertConstraint(&limit_Rz->constr_upper);
        }
    }
}

void ChLinkLock::ConstraintsBiReset() {
    for (int i = 0; i < mask.nconstr; i++) {
        mask.Constr_N(i).Set_b_i(0.);
    }

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.Set_b_i(0.);
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.Set_b_i(0.);
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.Set_b_i(0.);
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.Set_b_i(0.);
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.Set_b_i(0.);
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.Set_b_i(0.);
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.Set_b_i(0.);
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.Set_b_i(0.);
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.Set_b_i(0.);
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.Set_b_i(0.);
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.Set_b_i(0.);
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.Set_b_i(0.);
        }
    }
}

void ChLinkLock::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            if (do_clamp) {
                if (mask.Constr_N(i).IsUnilateral())
                    mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() + ChMax(factor * C(cnt), -recovery_clamp));
                else
                    mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() +
                                             ChMin(ChMax(factor * C(cnt), -recovery_clamp), recovery_clamp));
            } else
                mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() + factor * C(cnt));

            cnt++;
        }
    }

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_X->constr_lower.Set_b_i(limit_X->constr_lower.Get_b_i() +
                                              factor * (-limit_X->GetMin() + relM.pos.x()));
            } else {
                limit_X->constr_lower.Set_b_i(limit_X->constr_lower.Get_b_i() +
                                              ChMax(factor * (-limit_X->GetMin() + relM.pos.x()), -recovery_clamp));
            }
        }
        if (limit_X->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_X->constr_upper.Set_b_i(limit_X->constr_upper.Get_b_i() +
                                              factor * (limit_X->GetMax() - relM.pos.x()));
            } else {
                limit_X->constr_upper.Set_b_i(limit_X->constr_upper.Get_b_i() +
                                              ChMax(factor * (limit_X->GetMax() - relM.pos.x()), -recovery_clamp));
            }
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Y->constr_lower.Set_b_i(limit_Y->constr_lower.Get_b_i() +
                                              factor * (-limit_Y->GetMin() + relM.pos.y()));
            } else {
                limit_Y->constr_lower.Set_b_i(limit_Y->constr_lower.Get_b_i() +
                                              ChMax(factor * (-limit_Y->GetMin() + relM.pos.y()), -recovery_clamp));
            }
        }
        if (limit_Y->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Y->constr_upper.Set_b_i(limit_Y->constr_upper.Get_b_i() +
                                              factor * (limit_Y->GetMax() - relM.pos.y()));
            } else {
                limit_Y->constr_upper.Set_b_i(limit_Y->constr_upper.Get_b_i() +
                                              ChMax(factor * (limit_Y->GetMax() - relM.pos.y()), -recovery_clamp));
            }
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Z->constr_lower.Set_b_i(limit_Z->constr_lower.Get_b_i() +
                                              factor * (-limit_Z->GetMin() + relM.pos.z()));
            } else {
                limit_Z->constr_lower.Set_b_i(limit_Z->constr_lower.Get_b_i() +
                                              ChMax(factor * (-limit_Z->GetMin() + relM.pos.z()), -recovery_clamp));
            }
        }
        if (limit_Z->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Z->constr_upper.Set_b_i(limit_Z->constr_upper.Get_b_i() +
                                              factor * (limit_Z->GetMax() - relM.pos.z()));
            } else {
                limit_Z->constr_upper.Set_b_i(limit_Z->constr_upper.Get_b_i() +
                                              ChMax(factor * (limit_Z->GetMax() - relM.pos.z()), -recovery_clamp));
            }
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Rx->constr_lower.Set_b_i(limit_Rx->constr_lower.Get_b_i() +
                                               factor * (-sin(0.5 * limit_Rx->GetMin()) + relM.rot.e1()));
            } else {
                limit_Rx->constr_lower.Set_b_i(
                    limit_Rx->constr_lower.Get_b_i() +
                    ChMax(factor * (-sin(0.5 * limit_Rx->GetMin()) + relM.rot.e1()), -recovery_clamp));
            }
        }
        if (limit_Rx->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Rx->constr_upper.Set_b_i(limit_Rx->constr_upper.Get_b_i() +
                                               factor * (sin(0.5 * limit_Rx->GetMax()) - relM.rot.e1()));
            } else {
                limit_Rx->constr_upper.Set_b_i(
                    limit_Rx->constr_upper.Get_b_i() +
                    ChMax(factor * (sin(0.5 * limit_Rx->GetMax()) - relM.rot.e1()), -recovery_clamp));
            }
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Ry->constr_lower.Set_b_i(limit_Ry->constr_lower.Get_b_i() +
                                               factor * (-sin(0.5 * limit_Ry->GetMin()) + relM.rot.e2()));
            } else {
                limit_Ry->constr_lower.Set_b_i(
                    limit_Ry->constr_lower.Get_b_i() +
                    ChMax(factor * (-sin(0.5 * limit_Ry->GetMin()) + relM.rot.e2()), -recovery_clamp));
            }
        }
        if (limit_Ry->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Ry->constr_upper.Set_b_i(limit_Ry->constr_upper.Get_b_i() +
                                               factor * (sin(0.5 * limit_Ry->GetMax()) - relM.rot.e2()));
            } else {
                limit_Ry->constr_upper.Set_b_i(
                    limit_Ry->constr_upper.Get_b_i() +
                    ChMax(factor * (sin(0.5 * limit_Ry->GetMax()) - relM.rot.e2()), -recovery_clamp));
            }
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Rz->constr_lower.Set_b_i(limit_Rz->constr_lower.Get_b_i() +
                                               factor * (-sin(0.5 * limit_Rz->GetMin()) + relM.rot.e3()));
            } else {
                limit_Rz->constr_lower.Set_b_i(
                    limit_Rz->constr_lower.Get_b_i() +
                    ChMax(factor * (-sin(0.5 * limit_Rz->GetMin()) + relM.rot.e3()), -recovery_clamp));
            }
        }
        if (limit_Rz->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Rz->constr_upper.Set_b_i(limit_Rz->constr_upper.Get_b_i() +
                                               factor * (sin(0.5 * limit_Rz->GetMax()) - relM.rot.e3()));
            } else {
                limit_Rz->constr_upper.Set_b_i(
                    limit_Rz->constr_upper.Get_b_i() +
                    ChMax(factor * (sin(0.5 * limit_Rz->GetMax()) - relM.rot.e3()), -recovery_clamp));
            }
        }
    }
}

void ChLinkLock::ConstraintsBiLoad_Ct(double factor) {
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() + factor * Ct(cnt));
            cnt++;
        }
    }
}

void ChLinkLock::ConstraintsBiLoad_Qc(double factor) {
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            mask.Constr_N(i).Set_b_i(mask.Constr_N(i).Get_b_i() + factor * Qc(cnt));
            cnt++;
        }
    }
}

void Transform_Cq_to_Cqw_row(const ChMatrixNM<double, 7, BODY_QDOF>& mCq, int qrow, ChMatrixRef mCqw, int qwrow, const ChGlMatrix34<>& Gl) {
    // translational part - not changed
    mCqw.block<1, 3>(qwrow, 0) = mCq.block<1, 3>(qrow, 0);

    // rotational part [Cq_w] = [Cq_q]*[Gl]'*1/4
    for (int colres = 0; colres < 3; colres++) {
        double sum = 0;
        for (int col = 0; col < 4; col++) {
            sum += mCq(qrow, col + 3) * Gl(colres, col);
        }
        mCqw(qwrow, colres + 3) = sum * 0.25;
    }
    //// RADU: explicit loop slightly more performant than Eigen expressions
    ////mCqw.block<1, 3>(qwrow, 3) = 0.25 * mCq.block<1, 4>(qrow, 3) * Gl.transpose();
}

void ChLinkLock::ConstraintsLoadJacobians() {
    if (ndoc == 0)
        return;

    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            mask.Constr_N(i).Get_Cq_a().block(0, 0, 1, Cqw1.cols()) = Cqw1.block(cnt, 0, 1, Cqw1.cols());
            mask.Constr_N(i).Get_Cq_b().block(0, 0, 1, Cqw2.cols()) = Cqw2.block(cnt, 0, 1, Cqw2.cols());
            cnt++;

            // sets also the CFM term
            // mask->Constr_N(i).Set_cfm_i(this->attractor);
        }
    }

    ChGlMatrix34<> Gl1(Body1->GetCoord().rot);
    ChGlMatrix34<> Gl2(Body2->GetCoord().rot);

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 0, limit_X->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 0, limit_X->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 0, limit_X->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 0, limit_X->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_X->constr_upper.Get_Cq_a() *= -1;
            limit_X->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 1, limit_Y->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 1, limit_Y->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 1, limit_Y->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 1, limit_Y->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Y->constr_upper.Get_Cq_a() *= -1;
            limit_Y->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 2, limit_Z->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 2, limit_Z->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 2, limit_Z->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 2, limit_Z->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Z->constr_upper.Get_Cq_a() *= -1;
            limit_Z->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 4, limit_Rx->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 4, limit_Rx->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 4, limit_Rx->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 4, limit_Rx->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Rx->constr_upper.Get_Cq_a() *= -1;
            limit_Rx->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 5, limit_Ry->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 5, limit_Ry->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 5, limit_Ry->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 5, limit_Ry->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Ry->constr_upper.Get_Cq_a() *= -1;
            limit_Ry->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 6, limit_Rz->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 6, limit_Rz->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.SetVariables(&Body1->Variables(), &Body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 6, limit_Rz->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 6, limit_Rz->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Rz->constr_upper.Get_Cq_a() *= -1;
            limit_Rz->constr_upper.Get_Cq_b() *= -1;
        }
    }
}

void ChLinkLock::ConstraintsFetch_react(double factor) {
    react_force = VNULL;
    react_torque = VNULL;

    // From constraints to react vector:
    int cnt = 0;
    for (int i = 0; i < mask.nconstr; i++) {
        if (mask.Constr_N(i).IsActive()) {
            react(cnt) = mask.Constr_N(i).Get_l_i() * factor;
            cnt++;
        }
    }

    // From react vector to the 'intuitive' react_force and react_torque
    const ChQuaternion<>& q2 = Body2->GetRot();
    const ChQuaternion<>& q1p = marker1->GetAbsCoord().rot;
    const ChQuaternion<>& qs = marker2->GetCoord().rot;
    const ChMatrix33<>& Cs = marker2->GetA();

    ChMatrix44<> Chi__q1p_barT;  //[Chi] * [transpose(bar(q1p))]
    Chi__q1p_barT(0, 0) = q1p.e0();
    Chi__q1p_barT(0, 1) = q1p.e1();
    Chi__q1p_barT(0, 2) = q1p.e2();
    Chi__q1p_barT(0, 3) = q1p.e3();
    Chi__q1p_barT(1, 0) = q1p.e1();
    Chi__q1p_barT(1, 1) = -q1p.e0();
    Chi__q1p_barT(1, 2) = q1p.e3();
    Chi__q1p_barT(1, 3) = -q1p.e2();
    Chi__q1p_barT(2, 0) = q1p.e2();
    Chi__q1p_barT(2, 1) = -q1p.e3();
    Chi__q1p_barT(2, 2) = -q1p.e0();
    Chi__q1p_barT(2, 3) = q1p.e1();
    Chi__q1p_barT(3, 0) = q1p.e3();
    Chi__q1p_barT(3, 1) = q1p.e2();
    Chi__q1p_barT(3, 2) = -q1p.e1();
    Chi__q1p_barT(3, 3) = -q1p.e0();

    ChMatrix44<> qs_tilde;
    qs_tilde(0, 0) = qs.e0();
    qs_tilde(0, 1) = -qs.e1();
    qs_tilde(0, 2) = -qs.e2();
    qs_tilde(0, 3) = -qs.e3();
    qs_tilde(1, 0) = qs.e1();
    qs_tilde(1, 1) = qs.e0();
    qs_tilde(1, 2) = -qs.e3();
    qs_tilde(1, 3) = qs.e2();
    qs_tilde(2, 0) = qs.e2();
    qs_tilde(2, 1) = qs.e3();
    qs_tilde(2, 2) = qs.e0();
    qs_tilde(2, 3) = -qs.e1();
    qs_tilde(3, 0) = qs.e3();
    qs_tilde(3, 1) = -qs.e2();
    qs_tilde(3, 2) = qs.e1();
    qs_tilde(3, 3) = qs.e0();

    // Ts = 0.5*CsT*G(q2)*Chi*(q1 qp)_barT*qs~*KT*lambda
    ChGlMatrix34<> Gl_q2(q2);
    ChMatrix34<> Ts = 0.25 * Cs.transpose() * Gl_q2 * Chi__q1p_barT * qs_tilde;

    // Translational constraint reaction force = -lambda_translational
    // Translational constraint reaction torque = -d~''(t)*lambda_translational
    // No reaction force from the rotational constraints

    int n_constraint = 0;

    if (mask.Constr_X().IsActive()) {
        react_force.x() = -react(n_constraint);
        react_torque.y() = -relM.pos.z() * react(n_constraint);
        react_torque.z() = relM.pos.y() * react(n_constraint);
        n_constraint++;
    }
    if (mask.Constr_Y().IsActive()) {
        react_force.y() = -react(n_constraint);
        react_torque.x() = relM.pos.z() * react(n_constraint);
        react_torque.z() += -relM.pos.x() * react(n_constraint);
        n_constraint++;
    }
    if (mask.Constr_Z().IsActive()) {
        react_force.z() = -react(n_constraint);
        react_torque.x() += -relM.pos.y() * react(n_constraint);
        react_torque.y() += relM.pos.x() * react(n_constraint);
        n_constraint++;
    }

    if (mask.Constr_E1().IsActive()) {
        react_torque.x() += Ts(0, 1) * (react(n_constraint));
        react_torque.y() += Ts(1, 1) * (react(n_constraint));
        react_torque.z() += Ts(2, 1) * (react(n_constraint));
        n_constraint++;
    }
    if (mask.Constr_E2().IsActive()) {
        react_torque.x() += Ts(0, 2) * (react(n_constraint));
        react_torque.y() += Ts(1, 2) * (react(n_constraint));
        react_torque.z() += Ts(2, 2) * (react(n_constraint));
        n_constraint++;
    }
    if (mask.Constr_E3().IsActive()) {
        react_torque.x() += Ts(0, 3) * (react(n_constraint));
        react_torque.y() += Ts(1, 3) * (react(n_constraint));
        react_torque.z() += Ts(2, 3) * (react(n_constraint));
        n_constraint++;
    }

    // ***TO DO***?: TRANSFORMATION FROM delta COORDS TO LINK COORDS, if
    // non-default delta
    // if delta rotation?

    // add also the contribution from link limits to the react_force and
    // react_torque.
    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            react_force.x() -= factor * limit_X->constr_lower.Get_l_i();
        }
        if (limit_X->constr_upper.IsActive()) {
            react_force.x() += factor * limit_X->constr_upper.Get_l_i();
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            react_force.y() -= factor * limit_Y->constr_lower.Get_l_i();
        }
        if (limit_Y->constr_upper.IsActive()) {
            react_force.y() += factor * limit_Y->constr_upper.Get_l_i();
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            react_force.z() -= factor * limit_Z->constr_lower.Get_l_i();
        }
        if (limit_Z->constr_upper.IsActive()) {
            react_force.z() += factor * limit_Z->constr_upper.Get_l_i();
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            react_torque.x() -= 0.5 * factor * limit_Rx->constr_lower.Get_l_i();
        }
        if (limit_Rx->constr_upper.IsActive()) {
            react_torque.x() += 0.5 * factor * limit_Rx->constr_upper.Get_l_i();
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            react_torque.y() -= 0.5 * factor * limit_Ry->constr_lower.Get_l_i();
        }
        if (limit_Ry->constr_upper.IsActive()) {
            react_torque.y() += 0.5 * factor * limit_Ry->constr_upper.Get_l_i();
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            react_torque.z() -= 0.5 * factor * limit_Rz->constr_lower.Get_l_i();
        }
        if (limit_Rz->constr_upper.IsActive()) {
            react_torque.z() += 0.5 * factor * limit_Rz->constr_upper.Get_l_i();
        }
    }

    // the internal forces add their contribute to the reactions
    // NOT NEEDED?, since C_force and react_force must stay separated???
    // react_force  = Vadd(react_force, C_force);
    // react_torque = Vadd(react_torque, C_torque);
}

// SERIALIZATION

// To avoid putting the following mapper macro inside the class definition,
// enclose macros in local 'my_enum_mappers_types'.
class my_enum_mappers_types : public ChLinkLock {
  public:
    CH_ENUM_MAPPER_BEGIN(LinkType);
    CH_ENUM_VAL(LinkType::LOCK);
    CH_ENUM_VAL(LinkType::SPHERICAL);
    CH_ENUM_VAL(LinkType::POINTPLANE);
    CH_ENUM_VAL(LinkType::POINTLINE);
    CH_ENUM_VAL(LinkType::CYLINDRICAL);
    CH_ENUM_VAL(LinkType::PRISMATIC);
    CH_ENUM_VAL(LinkType::PLANEPLANE);
    CH_ENUM_VAL(LinkType::OLDHAM);
    CH_ENUM_VAL(LinkType::REVOLUTE);
    CH_ENUM_VAL(LinkType::FREE);
    CH_ENUM_VAL(LinkType::ALIGN);
    CH_ENUM_VAL(LinkType::PARALLEL);
    CH_ENUM_VAL(LinkType::PERPEND);
    CH_ENUM_VAL(LinkType::TRAJECTORY);
    CH_ENUM_VAL(LinkType::CLEARANCE);
    CH_ENUM_VAL(LinkType::REVOLUTEPRISMATIC);
    CH_ENUM_MAPPER_END(LinkType);
};

void ChLinkLock::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkLock>();

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);

    // serialize all member data
    my_enum_mappers_types::LinkType_mapper typemapper;
    marchive << CHNVP(typemapper(type), "link_type");

    ////marchive << CHNVP(mask); //// TODO: needed?

    marchive << CHNVP(d_restlength);

    marchive << CHNVP(force_D.get());

    ////marchive << CHNVP(force_D);
    ////marchive << CHNVP(force_R);
    ////marchive << CHNVP(force_X);
    ////marchive << CHNVP(force_Y);
    ////marchive << CHNVP(force_Z);
    ////marchive << CHNVP(force_Rx);
    ////marchive << CHNVP(force_Ry);
    ////marchive << CHNVP(force_Rz);

    ////marchive << CHNVP(limit_X);
    ////marchive << CHNVP(limit_Y);
    ////marchive << CHNVP(limit_Z);
    ////marchive << CHNVP(limit_Rx);
    ////marchive << CHNVP(limit_Ry);
    ////marchive << CHNVP(limit_Rz);
    ////marchive << CHNVP(limit_Rp);
    ////marchive << CHNVP(limit_D);
}

void ChLinkLock::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkLock>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);

    // deserialize all member data
    my_enum_mappers_types::LinkType_mapper typemapper;
    LinkType link_type;
    marchive >> CHNVP(typemapper(link_type), "link_type");
    ChangeLinkType(link_type);

    ////if (mask) delete (mask); marchive >> CHNVP(mask); //// TODO: needed?

    marchive >> CHNVP(d_restlength);

    {
        ChLinkForce* force_D_ptr;
        marchive >> CHNVP(force_D_ptr);
        force_D.reset(force_D_ptr);
    }
    ////marchive >> CHNVP(force_D);
    ////marchive >> CHNVP(force_R);
    ////marchive >> CHNVP(force_X);
    ////marchive >> CHNVP(force_Y);
    ////marchive >> CHNVP(force_Z);
    ////marchive >> CHNVP(force_Rx);
    ////marchive >> CHNVP(force_Ry);
    ////marchive >> CHNVP(force_Rz);

    ////marchive >> CHNVP(limit_X);
    ////marchive >> CHNVP(limit_Y);
    ////marchive >> CHNVP(limit_Z);
    ////marchive >> CHNVP(limit_Rx);
    ////marchive >> CHNVP(limit_Ry);
    ////marchive >> CHNVP(limit_Rz);
    ////marchive >> CHNVP(limit_Rp);
    ////marchive >> CHNVP(limit_D);
}

// =======================================================================================
// ChLinkLockLock implementation
// =======================================================================================

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockLock)

ChLinkLockLock::ChLinkLockLock()
    : motion_axis(VECT_Z),
      angleset(AngleSet::ANGLE_AXIS),
      relC(CSYSNORM),
      relC_dt(CSYSNULL),
      relC_dtdt(CSYSNULL),
      deltaC(CSYSNORM),
      deltaC_dt(CSYSNULL),
      deltaC_dtdt(CSYSNULL) {
    type = LinkType::LOCK;
    BuildLink(true, true, true, false, true, true, true);

    motion_X = chrono_types::make_shared<ChFunction_Const>(0);  // default: no motion
    motion_Y = chrono_types::make_shared<ChFunction_Const>(0);
    motion_Z = chrono_types::make_shared<ChFunction_Const>(0);
    motion_ang = chrono_types::make_shared<ChFunction_Const>(0);
    motion_ang2 = chrono_types::make_shared<ChFunction_Const>(0);
    motion_ang3 = chrono_types::make_shared<ChFunction_Const>(0);
}

ChLinkLockLock::ChLinkLockLock(const ChLinkLockLock& other) : ChLinkLock(other) {
    type = LinkType::LOCK;
    BuildLink(true, true, true, false, true, true, true);

    motion_X = std::shared_ptr<ChFunction>(other.motion_X->Clone());
    motion_Y = std::shared_ptr<ChFunction>(other.motion_Y->Clone());
    motion_Z = std::shared_ptr<ChFunction>(other.motion_Z->Clone());
    motion_ang = std::shared_ptr<ChFunction>(other.motion_ang->Clone());
    motion_ang2 = std::shared_ptr<ChFunction>(other.motion_ang2->Clone());
    motion_ang3 = std::shared_ptr<ChFunction>(other.motion_ang3->Clone());

    motion_axis = other.motion_axis;
    angleset = other.angleset;

    deltaC = other.deltaC;
    deltaC_dt = other.deltaC_dt;
    deltaC_dtdt = other.deltaC_dtdt;
    relC = other.relC;
    relC_dt = other.relC_dt;
    relC_dtdt = other.relC_dtdt;
}

void ChLinkLockLock::SetMotion_X(std::shared_ptr<ChFunction> m_funct) {
    motion_X = m_funct;
}

void ChLinkLockLock::SetMotion_Y(std::shared_ptr<ChFunction> m_funct) {
    motion_Y = m_funct;
}

void ChLinkLockLock::SetMotion_Z(std::shared_ptr<ChFunction> m_funct) {
    motion_Z = m_funct;
}

void ChLinkLockLock::SetMotion_ang(std::shared_ptr<ChFunction> m_funct) {
    motion_ang = m_funct;
}

void ChLinkLockLock::SetMotion_ang2(std::shared_ptr<ChFunction> m_funct) {
    motion_ang2 = m_funct;
}

void ChLinkLockLock::SetMotion_ang3(std::shared_ptr<ChFunction> m_funct) {
    motion_ang3 = m_funct;
}

void ChLinkLockLock::SetMotion_axis(Vector m_axis) {
    motion_axis = m_axis;
}

// Sequence of calls for full update:
//     UpdateTime(time);
//     UpdateRelMarkerCoords();
//     UpdateState();
//     UpdateCqw();
//     UpdateForces(time);
// Override UpdateTime to include possible contributions from imposed motion.
void ChLinkLockLock::UpdateTime(double time) {
    ChLinkLock::UpdateTime(time);

    double ang, ang_dt, ang_dtdt;

    // If some limit is provided, the delta values may have been changed by limits themselves,
    // so no further modifications by motion laws.
    if ((limit_X && limit_X->IsActive()) || (limit_Y && limit_Y->IsActive()) ||
        (limit_Z && limit_Z->IsActive()) || (limit_Rx && limit_Rx->IsActive()) ||
        (limit_Ry && limit_Ry->IsActive()) || (limit_Rz && limit_Rz->IsActive()))
        return;

    // Update motion position/speed/acceleration by motion laws
    // as expressed by specific link CH functions
    deltaC.pos.x() = motion_X->Get_y(time);
    deltaC_dt.pos.x() = motion_X->Get_y_dx(time);
    deltaC_dtdt.pos.x() = motion_X->Get_y_dxdx(time);

    deltaC.pos.y() = motion_Y->Get_y(time);
    deltaC_dt.pos.y() = motion_Y->Get_y_dx(time);
    deltaC_dtdt.pos.y() = motion_Y->Get_y_dxdx(time);

    deltaC.pos.z() = motion_Z->Get_y(time);
    deltaC_dt.pos.z() = motion_Z->Get_y_dx(time);
    deltaC_dtdt.pos.z() = motion_Z->Get_y_dxdx(time);

    switch (angleset) {
        case AngleSet::ANGLE_AXIS:
            ang = motion_ang->Get_y(time);
            ang_dt = motion_ang->Get_y_dx(time);
            ang_dtdt = motion_ang->Get_y_dxdx(time);

            if ((ang != 0) || (ang_dt != 0) || (ang_dtdt != 0)) {
                deltaC.rot = Q_from_AngAxis(ang, motion_axis);
                deltaC_dt.rot = Qdt_from_AngAxis(deltaC.rot, ang_dt, motion_axis);
                deltaC_dtdt.rot = Qdtdt_from_AngAxis(ang_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
            } else {
                deltaC.rot = QUNIT;
                deltaC_dt.rot = QNULL;
                deltaC_dtdt.rot = QNULL;
            }
            break;
        case AngleSet::EULERO:
        case AngleSet::CARDANO:
        case AngleSet::HPB:
        case AngleSet::RXYZ: {
            Vector vangles, vangles_dt, vangles_dtdt;
            vangles.x() = motion_ang->Get_y(time);
            vangles.y() = motion_ang2->Get_y(time);
            vangles.z() = motion_ang3->Get_y(time);
            vangles_dt.x() = motion_ang->Get_y_dx(time);
            vangles_dt.y() = motion_ang2->Get_y_dx(time);
            vangles_dt.z() = motion_ang3->Get_y_dx(time);
            vangles_dtdt.x() = motion_ang->Get_y_dxdx(time);
            vangles_dtdt.y() = motion_ang2->Get_y_dxdx(time);
            vangles_dtdt.z() = motion_ang3->Get_y_dxdx(time);
            deltaC.rot = Angle_to_Quat(angleset, vangles);
            deltaC_dt.rot = AngleDT_to_QuatDT(angleset, vangles_dt, deltaC.rot);
            deltaC_dtdt.rot = AngleDTDT_to_QuatDTDT(angleset, vangles_dtdt, deltaC.rot);
            break;
        }
        default:
            break;
    }
}

// Updates Cq1_temp, Cq2_temp, Qc_temp, etc., i.e. all LOCK-FORMULATION temp.matrices
void ChLinkLockLock::UpdateState() {
    // ----------- SOME PRECALCULATED VARIABLES, to optimize speed

    ChStarMatrix33<> P1star(marker1->GetCoord().pos);  // [P] star matrix of rel pos of mark1
    ChStarMatrix33<> Q2star(marker2->GetCoord().pos);  // [Q] star matrix of rel pos of mark2

    ChGlMatrix34<> body1Gl(Body1->GetCoord().rot);
    ChGlMatrix34<> body2Gl(Body2->GetCoord().rot);

    // ----------- RELATIVE LINK-LOCK COORDINATES (violations)

    // relC.pos
    relC.pos = Vsub(relM.pos, deltaC.pos);

    // relC.rot
    relC.rot = Qcross(Qconjugate(deltaC.rot), relM.rot);

    // relC_dt.pos
    relC_dt.pos = Vsub(relM_dt.pos, deltaC_dt.pos);

    // relC_dt.rot
    relC_dt.rot = Qadd(Qcross(Qconjugate(deltaC_dt.rot), relM.rot), Qcross(Qconjugate(deltaC.rot), relM_dt.rot));

    // relC_dtdt.pos
    relC_dtdt.pos = Vsub(relM_dtdt.pos, deltaC_dtdt.pos);

    // relC_dtdt.rot
    relC_dtdt.rot =
        Qadd(Qadd(Qcross(Qconjugate(deltaC_dtdt.rot), relM.rot), Qcross(Qconjugate(deltaC.rot), relM_dtdt.rot)),
             Qscale(Qcross(Qconjugate(deltaC_dt.rot), relM_dt.rot), 2));

    // Compute the Cq Ct Qc matrices (temporary, for complete lock constraint)

    ChMatrix33<> m2_Rel_A_dt;
    marker2->Compute_Adt(m2_Rel_A_dt);
    ChMatrix33<> m2_Rel_A_dtdt;
    marker2->Compute_Adtdt(m2_Rel_A_dtdt);

    // ----------- PARTIAL DERIVATIVE Ct OF CONSTRAINT
    Ct_temp.pos =
        m2_Rel_A_dt.transpose() * (Body2->GetA().transpose() * PQw) +
        marker2->GetA().transpose() *
            (Body2->GetA().transpose() * (Body1->GetA() * marker1->GetCoord_dt().pos) - marker2->GetCoord_dt().pos);
    Ct_temp.pos -= deltaC_dt.pos;  // the deltaC contribute

    // deltaC^*(q_AD) + deltaC_dt^*q_pq
    Ct_temp.rot = Qcross(Qconjugate(deltaC.rot), q_AD) + Qcross(Qconjugate(deltaC_dt.rot), relM.rot);

    //------------ COMPLETE JACOBIANS Cq1_temp AND Cq2_temp AND Qc_temp VECTOR.
    // [Cq_temp]= [[CqxT] [CqxR]]     {Qc_temp} ={[Qcx]}
    //            [[ 0  ] [CqrR]]                {[Qcr]}

    //  JACOBIANS Cq1_temp, Cq2_temp:

    ChMatrix33<> CqxT = marker2->GetA().transpose() * Body2->GetA().transpose();  // [CqxT]=[Aq]'[Ao2]'
    ChStarMatrix33<> tmpStar(Body2->GetA().transpose() * PQw);

    Cq1_temp.topLeftCorner<3, 3>() = CqxT;                                              // *- -- Cq1_temp(1-3)  =[Aqo2]
    Cq2_temp.topLeftCorner<3, 3>() = -CqxT;                                             // -- *- Cq2_temp(1-3)  =-[Aqo2]
    Cq1_temp.topRightCorner<3, 4>() = -CqxT * Body1->GetA() * P1star * body1Gl;         // -* -- Cq1_temp(4-7)
    Cq2_temp.topRightCorner<3, 4>() = CqxT * Body2->GetA() * Q2star * body2Gl +         //
                                      marker2->GetA().transpose() * tmpStar * body2Gl;  // -- -* Cq2_temp(4-7)

    {
        ChStarMatrix44<> stempQ1(Qcross(Qconjugate(marker2->GetCoord().rot), Qconjugate(Body2->GetCoord().rot)));
        ChStarMatrix44<> stempQ2(marker1->GetCoord().rot);
        ChStarMatrix44<> stempDC(Qconjugate(deltaC.rot));
        stempQ2.semiTranspose();
        Cq1_temp.bottomRightCorner<4, 4>() = stempDC * stempQ1 * stempQ2;  // =* == Cq1_temp(col 4-7, row 4-7) ... CqrR
    }

    {
        ChStarMatrix44<> stempQ1(Qconjugate(marker2->GetCoord().rot));
        ChStarMatrix44<> stempQ2(Qcross(Body1->GetCoord().rot, marker1->GetCoord().rot));
        ChStarMatrix44<> stempDC(Qconjugate(deltaC.rot));
        stempQ2.semiTranspose();
        stempQ2.semiNegate();
        Cq2_temp.bottomRightCorner<4, 4>() = stempDC * stempQ1 * stempQ2;  // == =* Cq2_temp(col 4-7, row 4-7) ... CqrR
    }

    //--------- COMPLETE Qc VECTOR
    ChVector<> Qcx;
    ChQuaternion<> Qcr;
    ChVector<> vtemp1;
    ChVector<> vtemp2;

    vtemp1 = Vcross(Body1->GetWvel_loc(), Vcross(Body1->GetWvel_loc(), marker1->GetCoord().pos));
    vtemp1 = Vadd(vtemp1, marker1->GetCoord_dtdt().pos);
    vtemp1 = Vadd(vtemp1, Vmul(Vcross(Body1->GetWvel_loc(), marker1->GetCoord_dt().pos), 2));

    vtemp2 = Vcross(Body2->GetWvel_loc(), Vcross(Body2->GetWvel_loc(), marker2->GetCoord().pos));
    vtemp2 = Vadd(vtemp2, marker2->GetCoord_dtdt().pos);
    vtemp2 = Vadd(vtemp2, Vmul(Vcross(Body2->GetWvel_loc(), marker2->GetCoord_dt().pos), 2));

    Qcx = CqxT * (Body1->GetA() * vtemp1 - Body2->GetA() * vtemp2);

    ChStarMatrix33<> mtemp1(Body2->GetWvel_loc());
    ChMatrix33<> mtemp3 = Body2->GetA() * mtemp1 * mtemp1;
    vtemp2 = marker2->GetA().transpose() * (mtemp3.transpose() * PQw);  // [Aq]'[[A2][w2][w2]]'*Qpq,w
    Qcx = Vadd(Qcx, vtemp2);
    Qcx = Vadd(Qcx, q_4);              // [Adtdt]'[A]'q + 2[Adt]'[Adt]'q + 2[Adt]'[A]'qdt + 2[A]'[Adt]'qdt
    Qcx = Vsub(Qcx, deltaC_dtdt.pos);  // ... - deltaC_dtdt

    Qcr = Qcross(Qconjugate(deltaC.rot), q_8);
    Qcr = Qadd(Qcr, Qscale(Qcross(Qconjugate(deltaC_dt.rot), relM_dt.rot), 2));
    Qcr = Qadd(Qcr, Qcross(Qconjugate(deltaC_dtdt.rot), relM.rot));  // = deltaC'*q_8 + 2*deltaC_dt'*q_dt,po +
                                                                     // deltaC_dtdt'*q,po

    Qc_temp.segment(0, 3) = Qcx.eigen();  // * Qc_temp, for all translational coords
    Qc_temp.segment(3, 4) = Qcr.eigen();  // * Qc_temp, for all rotational coords

    // *** NOTE! The definitive  Qc must change sign, to be used in
    // lagrangian equation:    [Cq]*q_dtdt = Qc
    // because until now we have computed it as [Cq]*q_dtdt + "Qc" = 0,
    // but the most used form is the previous, so let's change sign!!
    Qc_temp *= -1;

    // ---------------------
    // Updates Cq1, Cq2, Qc,
    // C, C_dt, C_dtdt, Ct.
    // ---------------------
    int index = 0;

    if (mask.Constr_X().IsActive()) {
        Cq1.block<1, 7>(index, 0) = Cq1_temp.block<1, 7>(0, 0);
        Cq2.block<1, 7>(index, 0) = Cq2_temp.block<1, 7>(0, 0);

        Qc(index) = Qc_temp(0);

        C(index) = relC.pos.x();
        C_dt(index) = relC_dt.pos.x();
        C_dtdt(index) = relC_dtdt.pos.x();

        Ct(index) = Ct_temp.pos.x();

        index++;
    }

    if (mask.Constr_Y().IsActive()) {
        Cq1.block<1, 7>(index, 0) = Cq1_temp.block<1, 7>(1, 0);
        Cq2.block<1, 7>(index, 0) = Cq2_temp.block<1, 7>(1, 0);

        Qc(index) = Qc_temp(1);

        C(index) = relC.pos.y();
        C_dt(index) = relC_dt.pos.y();
        C_dtdt(index) = relC_dtdt.pos.y();

        Ct(index) = Ct_temp.pos.y();

        index++;
    }

    if (mask.Constr_Z().IsActive()) {
        Cq1.block<1, 7>(index, 0) = Cq1_temp.block<1, 7>(2, 0);
        Cq2.block<1, 7>(index, 0) = Cq2_temp.block<1, 7>(2, 0);

        Qc(index) = Qc_temp(2);

        C(index) = relC.pos.z();
        C_dt(index) = relC_dt.pos.z();
        C_dtdt(index) = relC_dtdt.pos.z();

        Ct(index) = Ct_temp.pos.z();

        index++;
    }

    if (mask.Constr_E0().IsActive()) {
        Cq1.block<1, 4>(index, 3) = Cq1_temp.block<1, 4>(3, 3);
        Cq2.block<1, 4>(index, 3) = Cq2_temp.block<1, 4>(3, 3);

        Qc(index) = Qc_temp(3);

        C(index) = relC.rot.e0();
        C_dt(index) = relC_dt.rot.e0();
        C_dtdt(index) = relC_dtdt.rot.e0();

        Ct(index) = Ct_temp.rot.e0();

        index++;
    }

    if (mask.Constr_E1().IsActive()) {
        Cq1.block<1, 4>(index, 3) = Cq1_temp.block<1, 4>(4, 3);
        Cq2.block<1, 4>(index, 3) = Cq2_temp.block<1, 4>(4, 3);

        Qc(index) = Qc_temp(4);

        C(index) = relC.rot.e1();
        C_dt(index) = relC_dt.rot.e1();
        C_dtdt(index) = relC_dtdt.rot.e1();

        Ct(index) = Ct_temp.rot.e1();

        index++;
    }

    if (mask.Constr_E2().IsActive()) {
        Cq1.block<1, 4>(index, 3) = Cq1_temp.block<1, 4>(5, 3);
        Cq2.block<1, 4>(index, 3) = Cq2_temp.block<1, 4>(5, 3);

        Qc(index) = Qc_temp(5);

        C(index) = relC.rot.e2();
        C_dt(index) = relC_dt.rot.e2();
        C_dtdt(index) = relC_dtdt.rot.e2();

        Ct(index) = Ct_temp.rot.e2();

        index++;
    }

    if (mask.Constr_E3().IsActive()) {
        Cq1.block<1, 4>(index, 3) = Cq1_temp.block<1, 4>(6, 3);
        Cq2.block<1, 4>(index, 3) = Cq2_temp.block<1, 4>(6, 3);

        Qc(index) = Qc_temp(6);

        C(index) = relC.rot.e3();
        C_dt(index) = relC_dt.rot.e3();
        C_dtdt(index) = relC_dtdt.rot.e3();

        Ct(index) = Ct_temp.rot.e3();

        index++;
    }
}

// To avoid putting the following mapper macro inside the class definition,
// enclose macros in local 'my_enum_mappers_angles'.
class my_enum_mappers_angles : public ChLinkLockLock {
  public:
    CH_ENUM_MAPPER_BEGIN(AngleSet);
    CH_ENUM_VAL(AngleSet::ANGLE_AXIS);
    CH_ENUM_VAL(AngleSet::EULERO);
    CH_ENUM_VAL(AngleSet::CARDANO);
    CH_ENUM_VAL(AngleSet::HPB);
    CH_ENUM_VAL(AngleSet::RXYZ);
    CH_ENUM_VAL(AngleSet::RODRIGUEZ);
    CH_ENUM_VAL(AngleSet::QUATERNION);
    CH_ENUM_MAPPER_END(AngleSet);
};

void ChLinkLockLock::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkLockLock>();

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);

    // serialize all member data
    ////marchive << CHNVP(mask); //// TODO: needed?

    marchive << CHNVP(d_restlength);

    ////marchive << CHNVP(force_D);
    ////marchive << CHNVP(force_R);
    ////marchive << CHNVP(force_X);
    ////marchive << CHNVP(force_Y);
    ////marchive << CHNVP(force_Z);
    ////marchive << CHNVP(force_Rx);
    ////marchive << CHNVP(force_Ry);
    ////marchive << CHNVP(force_Rz);

    ////marchive << CHNVP(limit_X);
    ////marchive << CHNVP(limit_Y);
    ////marchive << CHNVP(limit_Z);
    ////marchive << CHNVP(limit_Rx);
    ////marchive << CHNVP(limit_Ry);
    ////marchive << CHNVP(limit_Rz);
    ////marchive << CHNVP(limit_Rp);
    ////marchive << CHNVP(limit_D);

    marchive << CHNVP(motion_X);
    marchive << CHNVP(motion_Y);
    marchive << CHNVP(motion_Z);
    marchive << CHNVP(motion_ang);
    marchive << CHNVP(motion_ang2);
    marchive << CHNVP(motion_ang3);
    marchive << CHNVP(motion_axis);

    my_enum_mappers_angles::AngleSet_mapper setmapper;
    marchive << CHNVP(setmapper(angleset), "angle_set");
}

void ChLinkLockLock::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkLockLock>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);

    // deserialize all member data
    ////if (mask) delete (mask); marchive >> CHNVP(mask); //// TODO: needed?

    marchive >> CHNVP(d_restlength);

    ////marchive >> CHNVP(force_D);
    ////marchive >> CHNVP(force_R);
    ////marchive >> CHNVP(force_X);
    ////marchive >> CHNVP(force_Y);
    ////marchive >> CHNVP(force_Z);
    ////marchive >> CHNVP(force_Rx);
    ////marchive >> CHNVP(force_Ry);
    ////marchive >> CHNVP(force_Rz);

    ////marchive >> CHNVP(limit_X);
    ////marchive >> CHNVP(limit_Y);
    ////marchive >> CHNVP(limit_Z);
    ////marchive >> CHNVP(limit_Rx);
    ////marchive >> CHNVP(limit_Ry);
    ////marchive >> CHNVP(limit_Rz);
    ////marchive >> CHNVP(limit_Rp);
    ////marchive >> CHNVP(limit_D);

    marchive >> CHNVP(motion_X);
    marchive >> CHNVP(motion_Y);
    marchive >> CHNVP(motion_Z);
    marchive >> CHNVP(motion_ang);
    marchive >> CHNVP(motion_ang2);
    marchive >> CHNVP(motion_ang3);
    marchive >> CHNVP(motion_axis);

    my_enum_mappers_angles::AngleSet_mapper setmapper;
    marchive >> CHNVP(setmapper(angleset), "angle_set");
}

// =======================================================================================

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockRevolute)
CH_FACTORY_REGISTER(ChLinkLockSpherical)
CH_FACTORY_REGISTER(ChLinkLockCylindrical)
CH_FACTORY_REGISTER(ChLinkLockPrismatic)
CH_FACTORY_REGISTER(ChLinkLockPointPlane)
CH_FACTORY_REGISTER(ChLinkLockPointLine)
CH_FACTORY_REGISTER(ChLinkLockPlanePlane)
CH_FACTORY_REGISTER(ChLinkLockOldham)
CH_FACTORY_REGISTER(ChLinkLockFree)
CH_FACTORY_REGISTER(ChLinkLockAlign)
CH_FACTORY_REGISTER(ChLinkLockParallel)
CH_FACTORY_REGISTER(ChLinkLockPerpend)
CH_FACTORY_REGISTER(ChLinkLockRevolutePrismatic)

}  // end namespace chrono
