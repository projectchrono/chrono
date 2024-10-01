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

#include <cmath>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkLock.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLock)

ChLinkLock::ChLinkLock()
    : type(Type::FREE), m_num_constr(0), m_num_constr_bil(0), m_num_constr_uni(0), d_restlength(0) {
    // Need to zero out the bottom-right 4x3 block
    Cq1_temp.setZero();
    Cq2_temp.setZero();

    // Note: the joint is not completely built at this time.
    // Requires definition of the mask (based on concrete joint type)
}

ChLinkLock::ChLinkLock(const ChLinkLock& other) : ChLinkMarkers(other) {
    mask = other.mask;

    if (other.force_D)
        force_D.reset(other.force_D->Clone());
    if (other.force_R)
        force_R.reset(other.force_R->Clone());
    if (other.force_X)
        force_X.reset(other.force_X->Clone());
    if (other.force_Y)
        force_Y.reset(other.force_Y->Clone());
    if (other.force_Z)
        force_Z.reset(other.force_Z->Clone());
    if (other.force_Rx)
        force_Rx.reset(other.force_Rx->Clone());
    if (other.force_Ry)
        force_Ry.reset(other.force_Ry->Clone());
    if (other.force_Rz)
        force_Rz.reset(other.force_Rz->Clone());

    d_restlength = other.d_restlength;

    type = other.type;

    if (other.limit_X)
        limit_X.reset(other.limit_X->Clone());
    if (other.limit_Y)
        limit_Y.reset(other.limit_Y->Clone());
    if (other.limit_Z)
        limit_Z.reset(other.limit_Z->Clone());
    if (other.limit_Rx)
        limit_Rx.reset(other.limit_Rx->Clone());
    if (other.limit_Ry)
        limit_Ry.reset(other.limit_Ry->Clone());
    if (other.limit_Rz)
        limit_Rz.reset(other.limit_Rz->Clone());
    if (other.limit_Rp)
        limit_Rp.reset(other.limit_Rp->Clone());
    if (other.limit_D)
        limit_D.reset(other.limit_D->Clone());

    Ct_temp = other.Ct_temp;

    BuildLinkType(other.type);
}

ChLinkLock::~ChLinkLock() {}

//// Note: ability to explicitly provide joint forces was removed.
//// If ever needed, these functions can be re-enabled. In that case,
//// a typical user call would be:
////         auto my_force = chrono_types::make_unique<ChLinkForce>();
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

ChLinkForce& ChLinkLock::ForceD() {
    if (!force_D)
        force_D = chrono_types::make_unique<ChLinkForce>();
    return *force_D;
}
ChLinkForce& ChLinkLock::ForceRp() {
    if (!force_R)
        force_R = chrono_types::make_unique<ChLinkForce>();
    return *force_R;
}
ChLinkForce& ChLinkLock::ForceX() {
    if (!force_X)
        force_X = chrono_types::make_unique<ChLinkForce>();
    return *force_X;
}
ChLinkForce& ChLinkLock::ForceY() {
    if (!force_Y)
        force_Y = chrono_types::make_unique<ChLinkForce>();
    return *force_Y;
}
ChLinkForce& ChLinkLock::ForceZ() {
    if (!force_Z)
        force_Z = chrono_types::make_unique<ChLinkForce>();
    return *force_Z;
}
ChLinkForce& ChLinkLock::ForceRx() {
    if (!force_Rx)
        force_Rx = chrono_types::make_unique<ChLinkForce>();
    return *force_Rx;
}
ChLinkForce& ChLinkLock::ForceRy() {
    if (!force_Ry)
        force_Ry = chrono_types::make_unique<ChLinkForce>();
    return *force_Ry;
}
ChLinkForce& ChLinkLock::ForceRz() {
    if (!force_Rz)
        force_Rz = chrono_types::make_unique<ChLinkForce>();
    return *force_Rz;
}

//// Note: ability to explicitly provide limits was removed.
//// If ever needed, these functions can be re-enabled. In that case,
//// a typical user call would be:
////         auto my_limit = chrono_types::make_unique<ChLinkLimit>();
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

ChLinkLimit& ChLinkLock::LimitX() {
    if (!limit_X)
        limit_X = chrono_types::make_unique<ChLinkLimit>();
    return *limit_X;
}
ChLinkLimit& ChLinkLock::LimitY() {
    if (!limit_Y)
        limit_Y = chrono_types::make_unique<ChLinkLimit>();
    return *limit_Y;
}
ChLinkLimit& ChLinkLock::LimitZ() {
    if (!limit_Z)
        limit_Z = chrono_types::make_unique<ChLinkLimit>();
    return *limit_Z;
}
ChLinkLimit& ChLinkLock::LimitRx() {
    if (!limit_Rx)
        limit_Rx = chrono_types::make_unique<ChLinkLimit>();
    return *limit_Rx;
}
ChLinkLimit& ChLinkLock::LimitRy() {
    if (!limit_Ry)
        limit_Ry = chrono_types::make_unique<ChLinkLimit>();
    return *limit_Ry;
}
ChLinkLimit& ChLinkLock::LimitRz() {
    if (!limit_Rz)
        limit_Rz = chrono_types::make_unique<ChLinkLimit>();
    return *limit_Rz;
}
ChLinkLimit& ChLinkLock::LimitRp() {
    if (!limit_Rp)
        limit_Rp = chrono_types::make_unique<ChLinkLimit>();
    return *limit_Rp;
}
ChLinkLimit& ChLinkLock::LimitD() {
    if (!limit_D)
        limit_D = chrono_types::make_unique<ChLinkLimit>();
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

void ChLinkLock::SetupMarkers(ChMarker* mark1, ChMarker* mark2) {
    ChLinkMarkers::SetupMarkers(mark1, mark2);
    assert(this->m_body1 && this->m_body2);

    mask.SetTwoBodiesVariables(&m_body1->Variables(), &m_body2->Variables());

    // We must call BuildLink here again, because only now are the constraints properly activated
    // (and hence the correct number of constraints is available).
    BuildLink();
}

void ChLinkLock::BuildLink() {
    // set m_num_constr by counting non-dofs
    m_num_constr = mask.GetNumConstraintsActive();
    m_num_constr_bil = mask.GetNumConstraintsBilateralActive();
    m_num_constr_uni = mask.GetNumConstraintsUnilateralActive();

    // create matrices
    C.resize(m_num_constr);
    C_dt.resize(m_num_constr);
    C_dtdt.resize(m_num_constr);
    react.resize(m_num_constr);
    Qc.resize(m_num_constr);
    Ct.resize(m_num_constr);
    Cq1.resize(m_num_constr, BODY_QDOF);
    Cq2.resize(m_num_constr, BODY_QDOF);
    Cqw1.resize(m_num_constr, BODY_DOF);
    Cqw2.resize(m_num_constr, BODY_DOF);

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

void ChLinkLock::BuildLinkType(Type link_type) {
    type = link_type;

    // SetLockMask() sets the constraints for the link coordinates: (X,Y,Z, E0,E1,E2,E3)
    switch (type) {
        case Type::FREE:
            BuildLink(false, false, false, false, false, false, false);
            break;
        case Type::LOCK:
            // this should never happen
            BuildLink(true, true, true, false, true, true, true);
            break;
        case Type::SPHERICAL:
            BuildLink(true, true, true, false, false, false, false);
            break;
        case Type::POINTPLANE:
            BuildLink(false, false, true, false, false, false, false);
            break;
        case Type::POINTLINE:
            BuildLink(false, true, true, false, false, false, false);
            break;
        case Type::REVOLUTE:
            BuildLink(true, true, true, false, true, true, false);
            break;
        case Type::CYLINDRICAL:
            BuildLink(true, true, false, false, true, true, false);
            break;
        case Type::PRISMATIC:
            BuildLink(true, true, false, false, true, true, true);
            break;
        case Type::PLANEPLANE:
            BuildLink(false, false, true, false, true, true, false);
            break;
        case Type::OLDHAM:
            BuildLink(false, false, true, false, true, true, true);
            break;
        case Type::ALIGN:
            BuildLink(false, false, false, false, true, true, true);
            break;
        case Type::PARALLEL:
            BuildLink(false, false, false, false, true, true, false);
            break;
        case Type::PERPEND:
            BuildLink(false, false, false, false, true, false, true);
            break;
        case Type::REVOLUTEPRISMATIC:
            BuildLink(false, true, true, false, true, true, false);
            break;
        default:
            BuildLink(false, false, false, false, false, false, false);
            break;
    }
}

void ChLinkLock::ChangeType(Type new_link_type) {
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

    ChStarMatrix33<> P1star(marker1->GetCoordsys().pos);  // [P] star matrix of rel pos of mark1
    ChStarMatrix33<> Q2star(marker2->GetCoordsys().pos);  // [Q] star matrix of rel pos of mark2

    ChGlMatrix34<> body1Gl(m_body1->GetCoordsys().rot);
    ChGlMatrix34<> body2Gl(m_body2->GetCoordsys().rot);

    // COMPUTE THE  Cq Ct Qc    matrices (temporary, for complete lock constraint)

    ChMatrix33<> m2_Rel_A_dt;
    marker2->ComputeRotMatDt(m2_Rel_A_dt);
    ChMatrix33<> m2_Rel_A_dtdt;
    marker2->ComputeRotMatDt2(m2_Rel_A_dtdt);

    // ----------- PARTIAL DERIVATIVE Ct OF CONSTRAINT
    Ct_temp.pos = m2_Rel_A_dt.transpose() * (m_body2->GetRotMat().transpose() * PQw) +
                  marker2->GetRotMat().transpose() *
                      (m_body2->GetRotMat().transpose() * (m_body1->GetRotMat() * marker1->GetCoordsysDt().pos) -
                       marker2->GetCoordsysDt().pos);

    Ct_temp.rot = q_AD;

    //------------ COMPLETE JACOBIANS Cq1_temp AND Cq2_temp AND Qc_temp VECTOR.
    // [Cq_temp]= [[CqxT] [CqxR]]     {Qc_temp} ={[Qcx]}
    //            [[ 0  ] [CqrR]]                {[Qcr]}

    //  JACOBIANS Cq1_temp, Cq2_temp:

    ChMatrix33<> CqxT = marker2->GetRotMat().transpose() * m_body2->GetRotMat().transpose();  // [CqxT]=[Aq]'[Ao2]'
    ChStarMatrix33<> tmpStar(m_body2->GetRotMat().transpose() * PQw);

    Cq1_temp.topLeftCorner<3, 3>() = CqxT;                                              // *- -- Cq1_temp(1-3) =  [Aqo2]
    Cq2_temp.topLeftCorner<3, 3>() = -CqxT;                                             // -- *- Cq2_temp(1-3) = -[Aqo2]
    Cq1_temp.topRightCorner<3, 4>() = -CqxT * m_body1->GetRotMat() * P1star * body1Gl;  // -* -- Cq1_temp(4-7)
    Cq2_temp.topRightCorner<3, 4>() = CqxT * m_body2->GetRotMat() * Q2star * body2Gl +  //
                                      marker2->GetRotMat().transpose() * tmpStar * body2Gl;  // -- -* Cq2_temp(4-7)

    {
        ChStarMatrix44<> stempQ1(
            Qcross(Qconjugate(marker2->GetCoordsys().rot), Qconjugate(m_body2->GetCoordsys().rot)));
        ChStarMatrix44<> stempQ2(marker1->GetCoordsys().rot);
        stempQ2.semiTranspose();
        Cq1_temp.bottomRightCorner<4, 4>() = stempQ1 * stempQ2;  // =* == Cq1_temp(col 4-7, row 4-7) ... CqrR
    }

    {
        ChStarMatrix44<> stempQ1(Qconjugate(marker2->GetCoordsys().rot));
        ChStarMatrix44<> stempQ2(Qcross(m_body1->GetCoordsys().rot, marker1->GetCoordsys().rot));
        stempQ2.semiTranspose();
        stempQ2.semiNegate();
        Cq2_temp.bottomRightCorner<4, 4>() = stempQ1 * stempQ2;  // == =* Cq2_temp(col 4-7, row 4-7) ... CqrR
    }

    //--------- COMPLETE Qc VECTOR
    ChVector3d Qcx;
    ChVector3d vtemp1;
    ChVector3d vtemp2;

    vtemp1 = Vcross(m_body1->GetAngVelLocal(), Vcross(m_body1->GetAngVelLocal(), marker1->GetCoordsys().pos));
    vtemp1 = Vadd(vtemp1, marker1->GetCoordsysDt2().pos);
    vtemp1 = Vadd(vtemp1, Vmul(Vcross(m_body1->GetAngVelLocal(), marker1->GetCoordsysDt().pos), 2));

    vtemp2 = Vcross(m_body2->GetAngVelLocal(), Vcross(m_body2->GetAngVelLocal(), marker2->GetCoordsys().pos));
    vtemp2 = Vadd(vtemp2, marker2->GetCoordsysDt2().pos);
    vtemp2 = Vadd(vtemp2, Vmul(Vcross(m_body2->GetAngVelLocal(), marker2->GetCoordsysDt().pos), 2));

    Qcx = CqxT * (m_body1->GetRotMat() * vtemp1 - m_body2->GetRotMat() * vtemp2);

    ChStarMatrix33<> mtemp1(m_body2->GetAngVelLocal());
    ChMatrix33<> mtemp3 = m_body2->GetRotMat() * mtemp1 * mtemp1;
    vtemp2 = marker2->GetRotMat().transpose() * (mtemp3.transpose() * PQw);  // [Aq]'[[A2][w2][w2]]'*Qpq,w
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
    ChGlMatrix34<> mGl(mbody->GetCoordsys().rot);
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
    Transform_Cq_to_Cqw(Cq1, Cqw1, m_body1);
    Transform_Cq_to_Cqw(Cq2, Cqw2, m_body2);
}

// Override UpdateForces to include possible contributions from joint limits.
void ChLinkLock::UpdateForces(double mytime) {
    ChLinkMarkers::UpdateForces(mytime);

    ChVector3d m_force = VNULL;
    ChVector3d m_torque = VNULL;

    // COMPUTE THE FORCES IN THE LINK, FOR EXAMPLE
    // CAUSED BY SPRINGS
    // NOTE!!!!!   C_force and C_torque   are considered in the reference coordsystem
    // of marker2  (the MAIN marker), and their application point is considered the
    // origin of marker1 (the SLAVE marker)

    // 1)========== the generic spring-damper

    if (force_D && force_D->IsActive()) {
        double dfor;
        dfor = force_D->GetForceTorque((dist - d_restlength), dist_dt, ChTime);
        m_force = Vmul(Vnorm(relM.pos), dfor);

        C_force = Vadd(C_force, m_force);
    }

    // 2)========== the generic torsional spring / torsional damper

    if (force_R && force_R->IsActive()) {
        double tor;
        // 1) the tors. spring
        tor = force_R->GetForceTorque(relAngle, 0, ChTime);
        m_torque = Vmul(relAxis, tor);
        C_torque = Vadd(C_torque, m_torque);
        // 2) the tors. damper
        double angle_dt = Vlength(relWvel);
        tor = force_R->GetForceTorque(0, angle_dt, ChTime);
        m_torque = Vmul(Vnorm(relWvel), tor);
        C_torque = Vadd(C_torque, m_torque);
    }

    // 3)========== the XYZ forces

    m_force = VNULL;

    if (force_X && force_X->IsActive()) {
        m_force.x() = force_X->GetForceTorque(relM.pos.x(), relM_dt.pos.x(), ChTime);
    }

    if (force_Y && force_Y->IsActive()) {
        m_force.y() = force_Y->GetForceTorque(relM.pos.y(), relM_dt.pos.y(), ChTime);
    }

    if (force_Z && force_Z->IsActive()) {
        m_force.z() = force_Z->GetForceTorque(relM.pos.z(), relM_dt.pos.z(), ChTime);
    }

    C_force = Vadd(C_force, m_force);

    // 4)========== the RxRyRz forces (torques)

    m_torque = VNULL;

    if (force_Rx && force_Rx->IsActive()) {
        m_torque.x() = force_Rx->GetForceTorque(relRotaxis.x(), relWvel.x(), ChTime);
    }

    if (force_Ry && force_Ry->IsActive()) {
        m_torque.y() = force_Ry->GetForceTorque(relRotaxis.y(), relWvel.y(), ChTime);
    }

    if (force_Rz && force_Rz->IsActive()) {
        m_torque.z() = force_Rz->GetForceTorque(relRotaxis.z(), relWvel.z(), ChTime);
    }

    C_torque = Vadd(C_torque, m_torque);

    // ========== the link-limits "cushion forces"

    m_force = VNULL;
    m_torque = VNULL;

    if (limit_X && limit_X->IsActive()) {
        m_force.x() = limit_X->GetForceTorque(relM.pos.x(), relM_dt.pos.x());
    }
    if (limit_Y && limit_Y->IsActive()) {
        m_force.y() = limit_Y->GetForceTorque(relM.pos.y(), relM_dt.pos.y());
    }
    if (limit_Z && limit_Z->IsActive()) {
        m_force.z() = limit_Z->GetForceTorque(relM.pos.z(), relM_dt.pos.z());
    }

    if (limit_D && limit_D->IsActive()) {
        m_force = Vadd(m_force, Vmul(Vnorm(relM.pos), limit_D->GetForceTorque(dist, dist_dt)));
    }

    if (limit_Rx && limit_Rx->IsActive()) {
        m_torque.x() = limit_Rx->GetForceTorque(relRotaxis.x(), relWvel.x());
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        m_torque.y() = limit_Ry->GetForceTorque(relRotaxis.y(), relWvel.y());
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        m_torque.z() = limit_Rz->GetForceTorque(relRotaxis.z(), relWvel.z());
    }
    if (limit_Rp && limit_Rp->IsActive()) {
        ChVector3d arm_xaxis = VaxisXfromQuat(relM.rot);  // the X axis of the marker1, respect to m2.
        double zenith = VangleYZplaneNorm(arm_xaxis);     // the angle of m1 Xaxis about normal to YZ plane
        double polar = VangleRX(arm_xaxis);               // the polar angle of m1 Xaxis spinning about m2 Xaxis

        ChVector3d projected_arm(0, arm_xaxis.y(), arm_xaxis.z());
        ChVector3d torq_axis;
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
unsigned int ChLinkLock::GetNumConstraintsUnilateral() {
    unsigned int mdocd = m_num_constr_uni;

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
    // const ChQuaternion<>& q2 = m_body2->GetRot();
    // const ChQuaternion<>& q1p = marker1->GetAbsCoordsys().rot;
    // const ChQuaternion<>& qs = marker2->GetCoordsys().rot;
    // const ChMatrix33<>& Cs = marker2->GetRotMat();

    // ChMatrix44<> Chi__q1p_barT;  //[Chi] * [transpose(bar(q1p))]
    // Chi__q1p_barT(0, 0) = q1p.e0();
    // Chi__q1p_barT(0, 1) = q1p.e1();
    // Chi__q1p_barT(0, 2) = q1p.e2();
    // Chi__q1p_barT(0, 3) = q1p.e3();
    // Chi__q1p_barT(1, 0) = q1p.e1();
    // Chi__q1p_barT(1, 1) = -q1p.e0();
    // Chi__q1p_barT(1, 2) = q1p.e3();
    // Chi__q1p_barT(1, 3) = -q1p.e2();
    // Chi__q1p_barT(2, 0) = q1p.e2();
    // Chi__q1p_barT(2, 1) = -q1p.e3();
    // Chi__q1p_barT(2, 2) = -q1p.e0();
    // Chi__q1p_barT(2, 3) = q1p.e1();
    // Chi__q1p_barT(3, 0) = q1p.e3();
    // Chi__q1p_barT(3, 1) = q1p.e2();
    // Chi__q1p_barT(3, 2) = -q1p.e1();
    // Chi__q1p_barT(3, 3) = -q1p.e0();

    // ChMatrix44<> qs_tilde;
    // qs_tilde(0, 0) = qs.e0();
    // qs_tilde(0, 1) = -qs.e1();
    // qs_tilde(0, 2) = -qs.e2();
    // qs_tilde(0, 3) = -qs.e3();
    // qs_tilde(1, 0) = qs.e1();
    // qs_tilde(1, 1) = qs.e0();
    // qs_tilde(1, 2) = -qs.e3();
    // qs_tilde(1, 3) = qs.e2();
    // qs_tilde(2, 0) = qs.e2();
    // qs_tilde(2, 1) = qs.e3();
    // qs_tilde(2, 2) = qs.e0();
    // qs_tilde(2, 3) = -qs.e1();
    // qs_tilde(3, 0) = qs.e3();
    // qs_tilde(3, 1) = -qs.e2();
    // qs_tilde(3, 2) = qs.e1();
    // qs_tilde(3, 3) = qs.e0();

    // ChGlMatrix34<> Gl_q2(q2);

    //// Ts = 0.5*G(q2)*Chi*(q1 qp)_barT*qs~*KT*lambda  (What is KT?)
    // ChMatrix34<> Ts = 0.25 * Gl_q2 * Chi__q1p_barT * qs_tilde;
    //// Ts.rightCols(3) is equal to the rotational block of Cqw2.T for completely fixed joint.
    //// The rotational block of Cqw2.T is a part of Ts.
    //// Cqw2.T*lambda is the reaction torque acting on m_body2, expressed in the local frame of m_body2
    //
    // Ts_F2 = CsT*Ts;
    //// The reaction torque is then rotated to the local frame of Marker2 (frame2, F2, master frame of link)

    // Cqw2.T is used directly to avoid computing the above complex Ts to improve performance.
    ChMatrixDynamic<> Cqw2T = Cqw2.transpose();

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

    ChVector3d m_torque_L;  // = Cqw2.T * lambda, reaction torque in local frame of m_body2
    if (mask.Constr_E1().IsActive()) {
        m_torque_L.x() += Cqw2T(3, local_off) * (react(local_off));
        m_torque_L.y() += Cqw2T(4, local_off) * (react(local_off));
        m_torque_L.z() += Cqw2T(5, local_off) * (react(local_off));
        local_off++;
    }
    if (mask.Constr_E2().IsActive()) {
        m_torque_L.x() += Cqw2T(3, local_off) * (react(local_off));
        m_torque_L.y() += Cqw2T(4, local_off) * (react(local_off));
        m_torque_L.z() += Cqw2T(5, local_off) * (react(local_off));
        local_off++;
    }
    if (mask.Constr_E3().IsActive()) {
        m_torque_L.x() += Cqw2T(3, local_off) * (react(local_off));
        m_torque_L.y() += Cqw2T(4, local_off) * (react(local_off));
        m_torque_L.z() += Cqw2T(5, local_off) * (react(local_off));
        local_off++;
    }
    react_torque += marker2->GetRotMat().transpose() * m_torque_L;

    // ***TO DO***?: TRANSFORMATION FROM delta COORDS TO LINK COORDS, if
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

    ////int local_off = this->GetNumConstraintsBilateral();

    // gather also the contribution from link limits
    // TODO not yet implemented
}

void ChLinkLock::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                     ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                     const ChVectorDynamic<>& L,  // the L vector
                                     const double c)              // a scaling factor
{
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            mask.GetConstraint(i).AddJacobianTransposedTimesScalarInto(R, L(off_L + cnt) * c);
            cnt++;
        }
    }

    unsigned int local_offset = this->GetNumConstraintsBilateral();

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
            ++local_offset;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.AddJacobianTransposedTimesScalarInto(R, L(off_L + local_offset) * c);
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
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            if (do_clamp) {
                if (mask.GetConstraint(i).IsUnilateral())
                    Qc(off_L + cnt) += std::max(c * C(cnt), -recovery_clamp);
                else
                    Qc(off_L + cnt) += std::min(std::max(c * C(cnt), -recovery_clamp), recovery_clamp);
            } else
                Qc(off_L + cnt) += c * C(cnt);
            cnt++;
        }
    }

    if (!do_clamp)
        recovery_clamp = 1e24;

    unsigned int local_offset = this->GetNumConstraintsBilateral();

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += std::max(c * (-limit_X->GetMin() + relM.pos.x()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_X->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += std::max(c * (limit_X->GetMax() - relM.pos.x()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += std::max(c * (-limit_Y->GetMin() + relM.pos.y()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Y->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += std::max(c * (limit_Y->GetMax() - relM.pos.y()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            Qc(off_L + local_offset) += std::max(c * (-limit_Z->GetMin() + relM.pos.z()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Z->constr_upper.IsActive()) {
            Qc(off_L + local_offset) += std::max(c * (limit_Z->GetMax() - relM.pos.z()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            Qc(off_L + local_offset) +=
                std::max(c * (-std::sin(0.5 * limit_Rx->GetMin()) + relM.rot.e1()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            Qc(off_L + local_offset) +=
                std::max(c * (std::sin(0.5 * limit_Rx->GetMax()) - relM.rot.e1()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            Qc(off_L + local_offset) +=
                std::max(c * (-std::sin(0.5 * limit_Ry->GetMin()) + relM.rot.e2()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            Qc(off_L + local_offset) +=
                std::max(c * (std::sin(0.5 * limit_Ry->GetMax()) - relM.rot.e2()), -recovery_clamp);
            ++local_offset;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            Qc(off_L + local_offset) +=
                std::max(c * (-std::sin(0.5 * limit_Rz->GetMin()) + relM.rot.e3()), -recovery_clamp);
            ++local_offset;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            Qc(off_L + local_offset) +=
                std::max(c * (std::sin(0.5 * limit_Rz->GetMax()) - relM.rot.e3()), -recovery_clamp);
            ++local_offset;
        }
    }
}

void ChLinkLock::IntLoadConstraint_Ct(const unsigned int off_L,  // offset in Qc residual
                                      ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*Ct
                                      const double c)            // a scaling factor
{
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
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
    unsigned int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            mask.GetConstraint(i).SetLagrangeMultiplier(L(off_L + cnt));
            mask.GetConstraint(i).SetRightHandSide(Qc(off_L + cnt));
            cnt++;
        }
    }

    unsigned int local_offset = this->GetNumConstraintsBilateral();

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_X->constr_lower.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_X->constr_upper.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Y->constr_lower.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Y->constr_upper.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Z->constr_lower.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Z->constr_upper.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Rx->constr_lower.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Rx->constr_upper.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Ry->constr_lower.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Ry->constr_upper.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Rz->constr_lower.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.SetLagrangeMultiplier(L(off_L + local_offset));
            limit_Rz->constr_upper.SetRightHandSide(Qc(off_L + local_offset));
            ++local_offset;
        }
    }
}

void ChLinkLock::IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) {
    unsigned int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            L(off_L + cnt) = mask.GetConstraint(i).GetLagrangeMultiplier();
            cnt++;
        }
    }

    unsigned int local_offset = this->GetNumConstraintsBilateral();

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_X->constr_lower.GetLagrangeMultiplier();
            ++local_offset;
        }
        if (limit_X->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_X->constr_upper.GetLagrangeMultiplier();
            ++local_offset;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Y->constr_lower.GetLagrangeMultiplier();
            ++local_offset;
        }
        if (limit_Y->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Y->constr_upper.GetLagrangeMultiplier();
            ++local_offset;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Z->constr_lower.GetLagrangeMultiplier();
            ++local_offset;
        }
        if (limit_Z->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Z->constr_upper.GetLagrangeMultiplier();
            ++local_offset;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Rx->constr_lower.GetLagrangeMultiplier();
            ++local_offset;
        }
        if (limit_Rx->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Rx->constr_upper.GetLagrangeMultiplier();
            ++local_offset;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Ry->constr_lower.GetLagrangeMultiplier();
            ++local_offset;
        }
        if (limit_Ry->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Ry->constr_upper.GetLagrangeMultiplier();
            ++local_offset;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            L(off_L + local_offset) = limit_Rz->constr_lower.GetLagrangeMultiplier();
            ++local_offset;
        }
        if (limit_Rz->constr_upper.IsActive()) {
            L(off_L + local_offset) = limit_Rz->constr_upper.GetLagrangeMultiplier();
            ++local_offset;
        }
    }
}

void ChLinkLock::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!this->IsActive())
        return;

    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive())
            descriptor.InsertConstraint(&mask.GetConstraint(i));
    }

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_X->constr_lower);
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_X->constr_upper);
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Y->constr_lower);
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Y->constr_upper);
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Z->constr_lower);
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Z->constr_upper);
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Rx->constr_lower);
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Rx->constr_upper);
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Ry->constr_lower);
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Ry->constr_upper);
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Rz->constr_lower);
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            descriptor.InsertConstraint(&limit_Rz->constr_upper);
        }
    }
}

void ChLinkLock::ConstraintsBiReset() {
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        mask.GetConstraint(i).SetRightHandSide(0.);
    }

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.SetRightHandSide(0.);
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.SetRightHandSide(0.);
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.SetRightHandSide(0.);
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.SetRightHandSide(0.);
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.SetRightHandSide(0.);
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.SetRightHandSide(0.);
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.SetRightHandSide(0.);
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.SetRightHandSide(0.);
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.SetRightHandSide(0.);
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.SetRightHandSide(0.);
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.SetRightHandSide(0.);
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.SetRightHandSide(0.);
        }
    }
}

void ChLinkLock::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            if (do_clamp) {
                if (mask.GetConstraint(i).IsUnilateral())
                    mask.GetConstraint(i).SetRightHandSide(mask.GetConstraint(i).GetRightHandSide() +
                                                  std::max(factor * C(cnt), -recovery_clamp));
                else
                    mask.GetConstraint(i).SetRightHandSide(mask.GetConstraint(i).GetRightHandSide() +
                                                  std::min(std::max(factor * C(cnt), -recovery_clamp), recovery_clamp));
            } else
                mask.GetConstraint(i).SetRightHandSide(mask.GetConstraint(i).GetRightHandSide() + factor * C(cnt));

            cnt++;
        }
    }

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_X->constr_lower.SetRightHandSide(limit_X->constr_lower.GetRightHandSide() +
                                              factor * (-limit_X->GetMin() + relM.pos.x()));
            } else {
                limit_X->constr_lower.SetRightHandSide(limit_X->constr_lower.GetRightHandSide() +
                                              std::max(factor * (-limit_X->GetMin() + relM.pos.x()), -recovery_clamp));
            }
        }
        if (limit_X->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_X->constr_upper.SetRightHandSide(limit_X->constr_upper.GetRightHandSide() +
                                              factor * (limit_X->GetMax() - relM.pos.x()));
            } else {
                limit_X->constr_upper.SetRightHandSide(limit_X->constr_upper.GetRightHandSide() +
                                              std::max(factor * (limit_X->GetMax() - relM.pos.x()), -recovery_clamp));
            }
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Y->constr_lower.SetRightHandSide(limit_Y->constr_lower.GetRightHandSide() +
                                              factor * (-limit_Y->GetMin() + relM.pos.y()));
            } else {
                limit_Y->constr_lower.SetRightHandSide(limit_Y->constr_lower.GetRightHandSide() +
                                              std::max(factor * (-limit_Y->GetMin() + relM.pos.y()), -recovery_clamp));
            }
        }
        if (limit_Y->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Y->constr_upper.SetRightHandSide(limit_Y->constr_upper.GetRightHandSide() +
                                              factor * (limit_Y->GetMax() - relM.pos.y()));
            } else {
                limit_Y->constr_upper.SetRightHandSide(limit_Y->constr_upper.GetRightHandSide() +
                                              std::max(factor * (limit_Y->GetMax() - relM.pos.y()), -recovery_clamp));
            }
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Z->constr_lower.SetRightHandSide(limit_Z->constr_lower.GetRightHandSide() +
                                              factor * (-limit_Z->GetMin() + relM.pos.z()));
            } else {
                limit_Z->constr_lower.SetRightHandSide(limit_Z->constr_lower.GetRightHandSide() +
                                              std::max(factor * (-limit_Z->GetMin() + relM.pos.z()), -recovery_clamp));
            }
        }
        if (limit_Z->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Z->constr_upper.SetRightHandSide(limit_Z->constr_upper.GetRightHandSide() +
                                              factor * (limit_Z->GetMax() - relM.pos.z()));
            } else {
                limit_Z->constr_upper.SetRightHandSide(limit_Z->constr_upper.GetRightHandSide() +
                                              std::max(factor * (limit_Z->GetMax() - relM.pos.z()), -recovery_clamp));
            }
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Rx->constr_lower.SetRightHandSide(limit_Rx->constr_lower.GetRightHandSide() +
                                                        factor * (-std::sin(0.5 * limit_Rx->GetMin()) + relM.rot.e1()));
            } else {
                limit_Rx->constr_lower.SetRightHandSide(
                    limit_Rx->constr_lower.GetRightHandSide() +
                    std::max(factor * (-std::sin(0.5 * limit_Rx->GetMin()) + relM.rot.e1()), -recovery_clamp));
            }
        }
        if (limit_Rx->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Rx->constr_upper.SetRightHandSide(limit_Rx->constr_upper.GetRightHandSide() +
                                                        factor * (std::sin(0.5 * limit_Rx->GetMax()) - relM.rot.e1()));
            } else {
                limit_Rx->constr_upper.SetRightHandSide(
                    limit_Rx->constr_upper.GetRightHandSide() +
                    std::max(factor * (std::sin(0.5 * limit_Rx->GetMax()) - relM.rot.e1()), -recovery_clamp));
            }
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Ry->constr_lower.SetRightHandSide(limit_Ry->constr_lower.GetRightHandSide() +
                                                        factor * (-std::sin(0.5 * limit_Ry->GetMin()) + relM.rot.e2()));
            } else {
                limit_Ry->constr_lower.SetRightHandSide(
                    limit_Ry->constr_lower.GetRightHandSide() +
                    std::max(factor * (-std::sin(0.5 * limit_Ry->GetMin()) + relM.rot.e2()), -recovery_clamp));
            }
        }
        if (limit_Ry->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Ry->constr_upper.SetRightHandSide(limit_Ry->constr_upper.GetRightHandSide() +
                                                        factor * (std::sin(0.5 * limit_Ry->GetMax()) - relM.rot.e2()));
            } else {
                limit_Ry->constr_upper.SetRightHandSide(
                    limit_Ry->constr_upper.GetRightHandSide() +
                    std::max(factor * (std::sin(0.5 * limit_Ry->GetMax()) - relM.rot.e2()), -recovery_clamp));
            }
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            if (!do_clamp) {
                limit_Rz->constr_lower.SetRightHandSide(limit_Rz->constr_lower.GetRightHandSide() +
                                                        factor * (-std::sin(0.5 * limit_Rz->GetMin()) + relM.rot.e3()));
            } else {
                limit_Rz->constr_lower.SetRightHandSide(
                    limit_Rz->constr_lower.GetRightHandSide() +
                    std::max(factor * (-std::sin(0.5 * limit_Rz->GetMin()) + relM.rot.e3()), -recovery_clamp));
            }
        }
        if (limit_Rz->constr_upper.IsActive()) {
            if (!do_clamp) {
                limit_Rz->constr_upper.SetRightHandSide(limit_Rz->constr_upper.GetRightHandSide() +
                                                        factor * (std::sin(0.5 * limit_Rz->GetMax()) - relM.rot.e3()));
            } else {
                limit_Rz->constr_upper.SetRightHandSide(
                    limit_Rz->constr_upper.GetRightHandSide() +
                    std::max(factor * (std::sin(0.5 * limit_Rz->GetMax()) - relM.rot.e3()), -recovery_clamp));
            }
        }
    }
}

void ChLinkLock::ConstraintsBiLoad_Ct(double factor) {
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            mask.GetConstraint(i).SetRightHandSide(mask.GetConstraint(i).GetRightHandSide() + factor * Ct(cnt));
            cnt++;
        }
    }
}

void ChLinkLock::ConstraintsBiLoad_Qc(double factor) {
    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            mask.GetConstraint(i).SetRightHandSide(mask.GetConstraint(i).GetRightHandSide() + factor * Qc(cnt));
            cnt++;
        }
    }
}

void Transform_Cq_to_Cqw_row(const ChMatrixNM<double, 7, BODY_QDOF>& mCq,
                             int qrow,
                             ChMatrixRef mCqw,
                             int qwrow,
                             const ChGlMatrix34<>& Gl) {
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

void ChLinkLock::LoadConstraintJacobians() {
    if (m_num_constr == 0)
        return;

    int cnt = 0;
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            mask.GetConstraint(i).Get_Cq_a().block(0, 0, 1, Cqw1.cols()) = Cqw1.block(cnt, 0, 1, Cqw1.cols());
            mask.GetConstraint(i).Get_Cq_b().block(0, 0, 1, Cqw2.cols()) = Cqw2.block(cnt, 0, 1, Cqw2.cols());
            cnt++;

            // sets also the CFM term
            // mask->GetConstraint(i).SetComplianceTerm(this->attractor);
        }
    }

    ChGlMatrix34<> Gl1(m_body1->GetCoordsys().rot);
    ChGlMatrix34<> Gl2(m_body2->GetCoordsys().rot);

    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            limit_X->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 0, limit_X->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 0, limit_X->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_X->constr_upper.IsActive()) {
            limit_X->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 0, limit_X->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 0, limit_X->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_X->constr_upper.Get_Cq_a() *= -1;
            limit_X->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            limit_Y->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 1, limit_Y->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 1, limit_Y->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Y->constr_upper.IsActive()) {
            limit_Y->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 1, limit_Y->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 1, limit_Y->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Y->constr_upper.Get_Cq_a() *= -1;
            limit_Y->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            limit_Z->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 2, limit_Z->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 2, limit_Z->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Z->constr_upper.IsActive()) {
            limit_Z->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 2, limit_Z->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 2, limit_Z->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Z->constr_upper.Get_Cq_a() *= -1;
            limit_Z->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            limit_Rx->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 4, limit_Rx->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 4, limit_Rx->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Rx->constr_upper.IsActive()) {
            limit_Rx->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 4, limit_Rx->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 4, limit_Rx->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Rx->constr_upper.Get_Cq_a() *= -1;
            limit_Rx->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            limit_Ry->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 5, limit_Ry->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 5, limit_Ry->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Ry->constr_upper.IsActive()) {
            limit_Ry->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 5, limit_Ry->constr_upper.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 5, limit_Ry->constr_upper.Get_Cq_b(), 0, Gl2);
            limit_Ry->constr_upper.Get_Cq_a() *= -1;
            limit_Ry->constr_upper.Get_Cq_b() *= -1;
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            limit_Rz->constr_lower.SetVariables(&m_body1->Variables(), &m_body2->Variables());
            Transform_Cq_to_Cqw_row(Cq1_temp, 6, limit_Rz->constr_lower.Get_Cq_a(), 0, Gl1);
            Transform_Cq_to_Cqw_row(Cq2_temp, 6, limit_Rz->constr_lower.Get_Cq_b(), 0, Gl2);
        }
        if (limit_Rz->constr_upper.IsActive()) {
            limit_Rz->constr_upper.SetVariables(&m_body1->Variables(), &m_body2->Variables());
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
    for (unsigned int i = 0; i < mask.GetNumConstraints(); i++) {
        if (mask.GetConstraint(i).IsActive()) {
            react(cnt) = mask.GetConstraint(i).GetLagrangeMultiplier() * factor;
            cnt++;
        }
    }

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

    // Cqw2.T*lambda is the reaction torque acting on m_body2, expressed in the local frame of m_body2
    ChMatrixDynamic<> Cqw2T = Cqw2.transpose();
    ChVector3d m_torque_L;  // = Cqw2.T * lambda
    if (mask.Constr_E1().IsActive()) {
        m_torque_L.x() += Cqw2T(3, n_constraint) * (react(n_constraint));
        m_torque_L.y() += Cqw2T(4, n_constraint) * (react(n_constraint));
        m_torque_L.z() += Cqw2T(5, n_constraint) * (react(n_constraint));
        n_constraint++;
    }
    if (mask.Constr_E2().IsActive()) {
        m_torque_L.x() += Cqw2T(3, n_constraint) * (react(n_constraint));
        m_torque_L.y() += Cqw2T(4, n_constraint) * (react(n_constraint));
        m_torque_L.z() += Cqw2T(5, n_constraint) * (react(n_constraint));
        n_constraint++;
    }
    if (mask.Constr_E3().IsActive()) {
        m_torque_L.x() += Cqw2T(3, n_constraint) * (react(n_constraint));
        m_torque_L.y() += Cqw2T(4, n_constraint) * (react(n_constraint));
        m_torque_L.z() += Cqw2T(5, n_constraint) * (react(n_constraint));
        n_constraint++;
    }
    // The reaction torque is rotated to the local frame of Marker2 (frame2, F2, master frame of link)
    react_torque += marker2->GetRotMat().transpose() * m_torque_L;

    // ***TO DO***?: TRANSFORMATION FROM delta COORDS TO LINK COORDS, if
    // non-default delta
    // if delta rotation?

    // add also the contribution from link limits to the react_force and
    // react_torque.
    if (limit_X && limit_X->IsActive()) {
        if (limit_X->constr_lower.IsActive()) {
            react_force.x() -= factor * limit_X->constr_lower.GetLagrangeMultiplier();
        }
        if (limit_X->constr_upper.IsActive()) {
            react_force.x() += factor * limit_X->constr_upper.GetLagrangeMultiplier();
        }
    }
    if (limit_Y && limit_Y->IsActive()) {
        if (limit_Y->constr_lower.IsActive()) {
            react_force.y() -= factor * limit_Y->constr_lower.GetLagrangeMultiplier();
        }
        if (limit_Y->constr_upper.IsActive()) {
            react_force.y() += factor * limit_Y->constr_upper.GetLagrangeMultiplier();
        }
    }
    if (limit_Z && limit_Z->IsActive()) {
        if (limit_Z->constr_lower.IsActive()) {
            react_force.z() -= factor * limit_Z->constr_lower.GetLagrangeMultiplier();
        }
        if (limit_Z->constr_upper.IsActive()) {
            react_force.z() += factor * limit_Z->constr_upper.GetLagrangeMultiplier();
        }
    }
    if (limit_Rx && limit_Rx->IsActive()) {
        if (limit_Rx->constr_lower.IsActive()) {
            react_torque.x() -= 0.5 * factor * limit_Rx->constr_lower.GetLagrangeMultiplier();
        }
        if (limit_Rx->constr_upper.IsActive()) {
            react_torque.x() += 0.5 * factor * limit_Rx->constr_upper.GetLagrangeMultiplier();
        }
    }
    if (limit_Ry && limit_Ry->IsActive()) {
        if (limit_Ry->constr_lower.IsActive()) {
            react_torque.y() -= 0.5 * factor * limit_Ry->constr_lower.GetLagrangeMultiplier();
        }
        if (limit_Ry->constr_upper.IsActive()) {
            react_torque.y() += 0.5 * factor * limit_Ry->constr_upper.GetLagrangeMultiplier();
        }
    }
    if (limit_Rz && limit_Rz->IsActive()) {
        if (limit_Rz->constr_lower.IsActive()) {
            react_torque.z() -= 0.5 * factor * limit_Rz->constr_lower.GetLagrangeMultiplier();
        }
        if (limit_Rz->constr_upper.IsActive()) {
            react_torque.z() += 0.5 * factor * limit_Rz->constr_upper.GetLagrangeMultiplier();
        }
    }

    // the internal forces add their contribute to the reactions
    // NOT NEEDED?, since C_force and react_force must stay separated???
    // react_force  = Vadd(react_force, C_force);
    // react_torque = Vadd(react_torque, C_torque);
}

// SERIALIZATION

// To avoid putting the following mapper macro inside the class definition,
// enclose macros in local 'ChLinkLock_Type_enum_mapper'.
class ChLinkLock_Type_enum_mapper : public ChLinkLock {
  public:
    CH_ENUM_MAPPER_BEGIN(Type);
    CH_ENUM_VAL(Type::LOCK);
    CH_ENUM_VAL(Type::SPHERICAL);
    CH_ENUM_VAL(Type::POINTPLANE);
    CH_ENUM_VAL(Type::POINTLINE);
    CH_ENUM_VAL(Type::CYLINDRICAL);
    CH_ENUM_VAL(Type::PRISMATIC);
    CH_ENUM_VAL(Type::PLANEPLANE);
    CH_ENUM_VAL(Type::OLDHAM);
    CH_ENUM_VAL(Type::REVOLUTE);
    CH_ENUM_VAL(Type::FREE);
    CH_ENUM_VAL(Type::ALIGN);
    CH_ENUM_VAL(Type::PARALLEL);
    CH_ENUM_VAL(Type::PERPEND);
    CH_ENUM_VAL(Type::TRAJECTORY);
    CH_ENUM_VAL(Type::CLEARANCE);
    CH_ENUM_VAL(Type::REVOLUTEPRISMATIC);
    CH_ENUM_MAPPER_END(Type);
};

void ChLinkLock::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLock>();

    // serialize parent class
    ChLinkMarkers::ArchiveOut(archive_out);

    // serialize all member data
    ChLinkLock_Type_enum_mapper::Type_mapper typemapper;
    archive_out << CHNVP(typemapper(type), "link_type");

    archive_out << CHNVP(mask);  //// TODO: needed?

    archive_out << CHNVP(d_restlength);

    archive_out << CHNVP(force_D.get(), "force_D_ptr");

    ////archive_out << CHNVP(force_D);
    ////archive_out << CHNVP(force_R);
    ////archive_out << CHNVP(force_X);
    ////archive_out << CHNVP(force_Y);
    ////archive_out << CHNVP(force_Z);
    ////archive_out << CHNVP(force_Rx);
    ////archive_out << CHNVP(force_Ry);
    ////archive_out << CHNVP(force_Rz);

    ////archive_out << CHNVP(limit_X);
    ////archive_out << CHNVP(limit_Y);
    ////archive_out << CHNVP(limit_Z);
    ////archive_out << CHNVP(limit_Rx);
    ////archive_out << CHNVP(limit_Ry);
    ////archive_out << CHNVP(limit_Rz);
    ////archive_out << CHNVP(limit_Rp);
    ////archive_out << CHNVP(limit_D);
}

void ChLinkLock::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLock>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIn(archive_in);

    // deserialize all member data
    ChLinkLock_Type_enum_mapper::Type_mapper typemapper;
    Type link_type;
    archive_in >> CHNVP(typemapper(link_type), "link_type");
    ChangeType(link_type);

    ////if (mask) delete (mask);
    archive_in >> CHNVP(mask);  //// TODO: needed?

    archive_in >> CHNVP(d_restlength);

    {
        ChLinkForce* force_D_ptr;
        archive_in >> CHNVP(force_D_ptr);
        force_D.reset(force_D_ptr);
    }
    ////archive_in >> CHNVP(force_D);
    ////archive_in >> CHNVP(force_R);
    ////archive_in >> CHNVP(force_X);
    ////archive_in >> CHNVP(force_Y);
    ////archive_in >> CHNVP(force_Z);
    ////archive_in >> CHNVP(force_Rx);
    ////archive_in >> CHNVP(force_Ry);
    ////archive_in >> CHNVP(force_Rz);

    ////archive_in >> CHNVP(limit_X);
    ////archive_in >> CHNVP(limit_Y);
    ////archive_in >> CHNVP(limit_Z);
    ////archive_in >> CHNVP(limit_Rx);
    ////archive_in >> CHNVP(limit_Ry);
    ////archive_in >> CHNVP(limit_Rz);
    ////archive_in >> CHNVP(limit_Rp);
    ////archive_in >> CHNVP(limit_D);

    mask.SetTwoBodiesVariables(&m_body1->Variables(), &m_body2->Variables());

    BuildLink();
}

// =======================================================================================

void ChLinkLockRevolute::Lock(bool lock) {
    BuildLink(true, true, true, false, true, true, lock);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockSpherical::Lock(bool lock) {
    BuildLink(true, true, true, false, lock, lock, lock);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockCylindrical::Lock(bool lock) {
    BuildLink(true, true, lock, false, true, true, lock);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockPrismatic::Lock(bool lock) {
    BuildLink(true, true, lock, false, true, true, true);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockPointPlane::Lock(bool lock) {
    BuildLink(lock, lock, true, false, lock, lock, lock);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockPointLine::Lock(bool lock) {
    BuildLink(lock, true, true, false, lock, lock, lock);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockPlanar::Lock(bool lock) {
    BuildLink(lock, lock, true, false, true, true, lock);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockOldham::Lock(bool lock) {
    BuildLink(lock, lock, true, false, true, true, true);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockFree::Lock(bool lock) {
    BuildLink(lock, lock, lock, false, lock, lock, lock);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockAlign::Lock(bool lock) {
    BuildLink(lock, lock, lock, false, true, true, true);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockParallel::Lock(bool lock) {
    BuildLink(lock, lock, lock, false, true, true, lock);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockPerpend::Lock(bool lock) {
    BuildLink(lock, lock, lock, false, true, lock, true);
    if (system) {
        system->ForceUpdate();
    }
}

void ChLinkLockRevolutePrismatic::Lock(bool lock) {
    BuildLink(lock, true, true, false, true, true, lock);
    if (system) {
        system->ForceUpdate();
    }
}

// =======================================================================================
// ChLinkLockLock implementation
// =======================================================================================

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockLock)

ChLinkLockLock::ChLinkLockLock()
    : motion_axis(VECT_Z),
      angleset(RotRepresentation::ANGLE_AXIS),
      relC(CSYSNORM),
      relC_dt(CSYSNULL),
      relC_dtdt(CSYSNULL),
      deltaC(CSYSNORM),
      deltaC_dt(CSYSNULL),
      deltaC_dtdt(CSYSNULL) {
    type = Type::LOCK;
    BuildLink(true, true, true, false, true, true, true);

    motion_X = chrono_types::make_shared<ChFunctionConst>(0);  // default: no motion
    motion_Y = chrono_types::make_shared<ChFunctionConst>(0);
    motion_Z = chrono_types::make_shared<ChFunctionConst>(0);
    motion_ang = chrono_types::make_shared<ChFunctionConst>(0);
    motion_ang2 = chrono_types::make_shared<ChFunctionConst>(0);
    motion_ang3 = chrono_types::make_shared<ChFunctionConst>(0);
}

ChLinkLockLock::ChLinkLockLock(const ChLinkLockLock& other) : ChLinkLock(other) {
    type = Type::LOCK;
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

void ChLinkLockLock::SetMotionX(std::shared_ptr<ChFunction> m_funct) {
    motion_X = m_funct;
}

void ChLinkLockLock::SetMotionY(std::shared_ptr<ChFunction> m_funct) {
    motion_Y = m_funct;
}

void ChLinkLockLock::SetMotionZ(std::shared_ptr<ChFunction> m_funct) {
    motion_Z = m_funct;
}

void ChLinkLockLock::SetMotionAng1(std::shared_ptr<ChFunction> m_funct) {
    motion_ang = m_funct;
}

void ChLinkLockLock::SetMotionAng2(std::shared_ptr<ChFunction> m_funct) {
    motion_ang2 = m_funct;
}

void ChLinkLockLock::SetMotionAng3(std::shared_ptr<ChFunction> m_funct) {
    motion_ang3 = m_funct;
}

void ChLinkLockLock::SetMotionAxis(ChVector3d m_axis) {
    motion_axis = m_axis;
}

void ChLinkLockLock::SetRotationRepresentation(RotRepresentation rot_rep) {
    if (rot_rep != RotRepresentation::ANGLE_AXIS || rot_rep != RotRepresentation::EULER_ANGLES_ZXZ ||
        rot_rep != RotRepresentation::CARDAN_ANGLES_XYZ || rot_rep != RotRepresentation::CARDAN_ANGLES_ZXY ||
        rot_rep != RotRepresentation::CARDAN_ANGLES_ZYX) {
        std::cerr << "Unknown input rotation representation" << std::endl;
        throw std::runtime_error("Unknown input rotation representation");
        return;
    }

    angleset = rot_rep;
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
    if ((limit_X && limit_X->IsActive()) || (limit_Y && limit_Y->IsActive()) || (limit_Z && limit_Z->IsActive()) ||
        (limit_Rx && limit_Rx->IsActive()) || (limit_Ry && limit_Ry->IsActive()) || (limit_Rz && limit_Rz->IsActive()))
        return;

    // Update motion position/speed/acceleration by motion laws
    // as expressed by specific link CH functions
    deltaC.pos.x() = motion_X->GetVal(time);
    deltaC_dt.pos.x() = motion_X->GetDer(time);
    deltaC_dtdt.pos.x() = motion_X->GetDer2(time);

    deltaC.pos.y() = motion_Y->GetVal(time);
    deltaC_dt.pos.y() = motion_Y->GetDer(time);
    deltaC_dtdt.pos.y() = motion_Y->GetDer2(time);

    deltaC.pos.z() = motion_Z->GetVal(time);
    deltaC_dt.pos.z() = motion_Z->GetDer(time);
    deltaC_dtdt.pos.z() = motion_Z->GetDer2(time);

    switch (angleset) {
        case RotRepresentation::ANGLE_AXIS:
            ang = motion_ang->GetVal(time);
            ang_dt = motion_ang->GetDer(time);
            ang_dtdt = motion_ang->GetDer2(time);

            if ((ang != 0) || (ang_dt != 0) || (ang_dtdt != 0)) {
                deltaC.rot = QuatFromAngleAxis(ang, motion_axis);
                deltaC_dt.rot = QuatDtFromAngleAxis(deltaC.rot, ang_dt, motion_axis);
                deltaC_dtdt.rot = QuatDt2FromAngleAxis(ang_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
            } else {
                deltaC.rot = QUNIT;
                deltaC_dt.rot = QNULL;
                deltaC_dtdt.rot = QNULL;
            }
            break;
        case RotRepresentation::EULER_ANGLES_ZXZ:
        case RotRepresentation::CARDAN_ANGLES_ZXY:
        case RotRepresentation::CARDAN_ANGLES_ZYX:
        case RotRepresentation::CARDAN_ANGLES_XYZ: {
            ChVector3d vangles, vangles_dt, vangles_dtdt;
            vangles.x() = motion_ang->GetVal(time);
            vangles.y() = motion_ang2->GetVal(time);
            vangles.z() = motion_ang3->GetVal(time);
            vangles_dt.x() = motion_ang->GetDer(time);
            vangles_dt.y() = motion_ang2->GetDer(time);
            vangles_dt.z() = motion_ang3->GetDer(time);
            vangles_dtdt.x() = motion_ang->GetDer2(time);
            vangles_dtdt.y() = motion_ang2->GetDer2(time);
            vangles_dtdt.z() = motion_ang3->GetDer2(time);
            deltaC.rot = QuatFromAngleSet({angleset, vangles});
            deltaC_dt.rot = QuatDtFromAngleSet({angleset, vangles_dt}, deltaC.rot);
            deltaC_dtdt.rot = QuatDt2FromAngleSet({angleset, vangles_dtdt}, deltaC.rot);
            break;
        }
        default:
            break;
    }
}

// Updates Cq1_temp, Cq2_temp, Qc_temp, etc., i.e. all LOCK-FORMULATION temp.matrices
void ChLinkLockLock::UpdateState() {
    // ----------- SOME PRECALCULATED VARIABLES, to optimize speed

    ChStarMatrix33<> P1star(marker1->GetCoordsys().pos);  // [P] star matrix of rel pos of mark1
    ChStarMatrix33<> Q2star(marker2->GetCoordsys().pos);  // [Q] star matrix of rel pos of mark2

    ChGlMatrix34<> body1Gl(m_body1->GetCoordsys().rot);
    ChGlMatrix34<> body2Gl(m_body2->GetCoordsys().rot);

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
    marker2->ComputeRotMatDt(m2_Rel_A_dt);
    ChMatrix33<> m2_Rel_A_dtdt;
    marker2->ComputeRotMatDt2(m2_Rel_A_dtdt);

    // ----------- PARTIAL DERIVATIVE Ct OF CONSTRAINT
    Ct_temp.pos = m2_Rel_A_dt.transpose() * (m_body2->GetRotMat().transpose() * PQw) +
                  marker2->GetRotMat().transpose() *
                      (m_body2->GetRotMat().transpose() * (m_body1->GetRotMat() * marker1->GetCoordsysDt().pos) -
                       marker2->GetCoordsysDt().pos);
    Ct_temp.pos -= deltaC_dt.pos;  // the deltaC contribute

    // deltaC^*(q_AD) + deltaC_dt^*q_pq
    Ct_temp.rot = Qcross(Qconjugate(deltaC.rot), q_AD) + Qcross(Qconjugate(deltaC_dt.rot), relM.rot);

    //------------ COMPLETE JACOBIANS Cq1_temp AND Cq2_temp AND Qc_temp VECTOR.
    // [Cq_temp]= [[CqxT] [CqxR]]     {Qc_temp} ={[Qcx]}
    //            [[ 0  ] [CqrR]]                {[Qcr]}

    //  JACOBIANS Cq1_temp, Cq2_temp:

    ChMatrix33<> CqxT = marker2->GetRotMat().transpose() * m_body2->GetRotMat().transpose();  // [CqxT]=[Aq]'[Ao2]'
    ChStarMatrix33<> tmpStar(m_body2->GetRotMat().transpose() * PQw);

    Cq1_temp.topLeftCorner<3, 3>() = CqxT;                                              // *- -- Cq1_temp(1-3)  =[Aqo2]
    Cq2_temp.topLeftCorner<3, 3>() = -CqxT;                                             // -- *- Cq2_temp(1-3)  =-[Aqo2]
    Cq1_temp.topRightCorner<3, 4>() = -CqxT * m_body1->GetRotMat() * P1star * body1Gl;  // -* -- Cq1_temp(4-7)
    Cq2_temp.topRightCorner<3, 4>() = CqxT * m_body2->GetRotMat() * Q2star * body2Gl +  //
                                      marker2->GetRotMat().transpose() * tmpStar * body2Gl;  // -- -* Cq2_temp(4-7)

    {
        ChStarMatrix44<> stempQ1(
            Qcross(Qconjugate(marker2->GetCoordsys().rot), Qconjugate(m_body2->GetCoordsys().rot)));
        ChStarMatrix44<> stempQ2(marker1->GetCoordsys().rot);
        ChStarMatrix44<> stempDC(Qconjugate(deltaC.rot));
        stempQ2.semiTranspose();
        Cq1_temp.bottomRightCorner<4, 4>() = stempDC * stempQ1 * stempQ2;  // =* == Cq1_temp(col 4-7, row 4-7) ... CqrR
    }

    {
        ChStarMatrix44<> stempQ1(Qconjugate(marker2->GetCoordsys().rot));
        ChStarMatrix44<> stempQ2(Qcross(m_body1->GetCoordsys().rot, marker1->GetCoordsys().rot));
        ChStarMatrix44<> stempDC(Qconjugate(deltaC.rot));
        stempQ2.semiTranspose();
        stempQ2.semiNegate();
        Cq2_temp.bottomRightCorner<4, 4>() = stempDC * stempQ1 * stempQ2;  // == =* Cq2_temp(col 4-7, row 4-7) ... CqrR
    }

    //--------- COMPLETE Qc VECTOR
    ChVector3d Qcx;
    ChQuaternion<> Qcr;
    ChVector3d vtemp1;
    ChVector3d vtemp2;

    vtemp1 = Vcross(m_body1->GetAngVelLocal(), Vcross(m_body1->GetAngVelLocal(), marker1->GetCoordsys().pos));
    vtemp1 = Vadd(vtemp1, marker1->GetCoordsysDt2().pos);
    vtemp1 = Vadd(vtemp1, Vmul(Vcross(m_body1->GetAngVelLocal(), marker1->GetCoordsysDt().pos), 2));

    vtemp2 = Vcross(m_body2->GetAngVelLocal(), Vcross(m_body2->GetAngVelLocal(), marker2->GetCoordsys().pos));
    vtemp2 = Vadd(vtemp2, marker2->GetCoordsysDt2().pos);
    vtemp2 = Vadd(vtemp2, Vmul(Vcross(m_body2->GetAngVelLocal(), marker2->GetCoordsysDt().pos), 2));

    Qcx = CqxT * (m_body1->GetRotMat() * vtemp1 - m_body2->GetRotMat() * vtemp2);

    ChStarMatrix33<> mtemp1(m_body2->GetAngVelLocal());
    ChMatrix33<> mtemp3 = m_body2->GetRotMat() * mtemp1 * mtemp1;
    vtemp2 = marker2->GetRotMat().transpose() * (mtemp3.transpose() * PQw);  // [Aq]'[[A2][w2][w2]]'*Qpq,w
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
// enclose macros in local 'ChLinkLockLock_RotRep_enum_mapper'.
class ChLinkLockLock_RotRep_enum_mapper : public ChLinkLockLock {
  public:
    typedef RotRepresentation ChRotationRepresentation;

    CH_ENUM_MAPPER_BEGIN(ChRotationRepresentation);
    CH_ENUM_VAL(RotRepresentation::ANGLE_AXIS);
    CH_ENUM_VAL(RotRepresentation::EULER_ANGLES_ZXZ);
    CH_ENUM_VAL(RotRepresentation::CARDAN_ANGLES_ZXY);
    CH_ENUM_VAL(RotRepresentation::CARDAN_ANGLES_ZYX);
    CH_ENUM_VAL(RotRepresentation::CARDAN_ANGLES_XYZ);
    CH_ENUM_VAL(RotRepresentation::RODRIGUES);
    CH_ENUM_MAPPER_END(ChRotationRepresentation);
};

void ChLinkLockLock::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLockLock>();

    // serialize parent class
    ChLinkMarkers::ArchiveOut(archive_out);

    // serialize all member data
    ////archive_out << CHNVP(mask); //// TODO: needed?

    archive_out << CHNVP(d_restlength);

    ////archive_out << CHNVP(force_D);
    ////archive_out << CHNVP(force_R);
    ////archive_out << CHNVP(force_X);
    ////archive_out << CHNVP(force_Y);
    ////archive_out << CHNVP(force_Z);
    ////archive_out << CHNVP(force_Rx);
    ////archive_out << CHNVP(force_Ry);
    ////archive_out << CHNVP(force_Rz);

    ////archive_out << CHNVP(limit_X);
    ////archive_out << CHNVP(limit_Y);
    ////archive_out << CHNVP(limit_Z);
    ////archive_out << CHNVP(limit_Rx);
    ////archive_out << CHNVP(limit_Ry);
    ////archive_out << CHNVP(limit_Rz);
    ////archive_out << CHNVP(limit_Rp);
    ////archive_out << CHNVP(limit_D);

    archive_out << CHNVP(motion_X);
    archive_out << CHNVP(motion_Y);
    archive_out << CHNVP(motion_Z);
    archive_out << CHNVP(motion_ang);
    archive_out << CHNVP(motion_ang2);
    archive_out << CHNVP(motion_ang3);
    archive_out << CHNVP(motion_axis);

    ChLinkLockLock_RotRep_enum_mapper::ChRotationRepresentation_mapper setmapper;
    archive_out << CHNVP(setmapper(angleset), "angle_set");
}

void ChLinkLockLock::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLockLock>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIn(archive_in);

    // deserialize all member data
    ////if (mask) delete (mask); archive_in >> CHNVP(mask); //// TODO: needed?

    archive_in >> CHNVP(d_restlength);

    ////archive_in >> CHNVP(force_D);
    ////archive_in >> CHNVP(force_R);
    ////archive_in >> CHNVP(force_X);
    ////archive_in >> CHNVP(force_Y);
    ////archive_in >> CHNVP(force_Z);
    ////archive_in >> CHNVP(force_Rx);
    ////archive_in >> CHNVP(force_Ry);
    ////archive_in >> CHNVP(force_Rz);

    ////archive_in >> CHNVP(limit_X);
    ////archive_in >> CHNVP(limit_Y);
    ////archive_in >> CHNVP(limit_Z);
    ////archive_in >> CHNVP(limit_Rx);
    ////archive_in >> CHNVP(limit_Ry);
    ////archive_in >> CHNVP(limit_Rz);
    ////archive_in >> CHNVP(limit_Rp);
    ////archive_in >> CHNVP(limit_D);

    archive_in >> CHNVP(motion_X);
    archive_in >> CHNVP(motion_Y);
    archive_in >> CHNVP(motion_Z);
    archive_in >> CHNVP(motion_ang);
    archive_in >> CHNVP(motion_ang2);
    archive_in >> CHNVP(motion_ang3);
    archive_in >> CHNVP(motion_axis);

    ChLinkLockLock_RotRep_enum_mapper::ChRotationRepresentation_mapper setmapper;
    archive_in >> CHNVP(setmapper(angleset), "angle_set");
}

// =======================================================================================

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockRevolute)
CH_FACTORY_REGISTER(ChLinkLockSpherical)
CH_FACTORY_REGISTER(ChLinkLockCylindrical)
CH_FACTORY_REGISTER(ChLinkLockPrismatic)
CH_FACTORY_REGISTER(ChLinkLockPointPlane)
CH_FACTORY_REGISTER(ChLinkLockPointLine)
CH_FACTORY_REGISTER(ChLinkLockPlanar)
CH_FACTORY_REGISTER(ChLinkLockOldham)
CH_FACTORY_REGISTER(ChLinkLockFree)
CH_FACTORY_REGISTER(ChLinkLockAlign)
CH_FACTORY_REGISTER(ChLinkLockParallel)
CH_FACTORY_REGISTER(ChLinkLockPerpend)
CH_FACTORY_REGISTER(ChLinkLockRevolutePrismatic)

}  // end namespace chrono
