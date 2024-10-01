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

#include "chrono/physics/ChLinkDistance.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkDistance)

class ChLinkDistance_Mode_enum_mapper : public ChLinkDistance {
  public:
    CH_ENUM_MAPPER_BEGIN(Mode);
    CH_ENUM_VAL(Mode::BILATERAL);
    CH_ENUM_VAL(Mode::UNILATERAL_MAXDISTANCE);
    CH_ENUM_VAL(Mode::UNILATERAL_MINDISTANCE);
    CH_ENUM_MAPPER_END(Mode);
};

ChLinkDistance::ChLinkDistance() : m_pos1(VNULL), m_pos2(VNULL), distance(0), curr_dist(0) {
    this->SetMode(Mode::BILATERAL);
}

ChLinkDistance::ChLinkDistance(const ChLinkDistance& other) : ChLink(other) {
    this->SetMode(other.mode);
    m_body1 = other.m_body1;
    m_body2 = other.m_body2;
    system = other.system;
    if (other.m_body1 && other.m_body2)
        Cx.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
    m_pos1 = other.m_pos1;
    m_pos2 = other.m_pos2;
    distance = other.distance;
    curr_dist = other.curr_dist;
}

int ChLinkDistance::Initialize(std::shared_ptr<ChBodyFrame> body1,
                               std::shared_ptr<ChBodyFrame> body2,
                               bool pos_are_relative,
                               ChVector3d pos1,
                               ChVector3d pos2,
                               bool auto_distance,
                               double mdistance,
                               Mode mode) {
    this->SetMode(mode);

    m_body1 = body1.get();
    m_body2 = body2.get();
    Cx.SetVariables(&m_body1->Variables(), &m_body2->Variables());

    if (pos_are_relative) {
        m_pos1 = pos1;
        m_pos2 = pos2;
    } else {
        m_pos1 = m_body1->TransformPointParentToLocal(pos1);
        m_pos2 = m_body2->TransformPointParentToLocal(pos2);
    }

    ChVector3d delta_pos = m_body1->TransformPointLocalToParent(m_pos1) - m_body2->TransformPointLocalToParent(m_pos2);
    curr_dist = delta_pos.Length();

    if (auto_distance) {
        distance = curr_dist;
    } else {
        distance = mdistance;
    }

    C[0] = mode_sign * (curr_dist - distance);

    return true;
}

ChFramed ChLinkDistance::GetFrame1Rel() const {
    // unit vector from pos1 to pos2 in absolute coordinates
    ChVector3d dir_12_abs =
        Vnorm(m_body2->TransformPointLocalToParent(m_pos2) - m_body1->TransformPointLocalToParent(m_pos1));
    ChVector3d dir_12_loc1 = m_body1->TransformDirectionParentToLocal(dir_12_abs);
    ChMatrix33<> rel_matrix;
    rel_matrix.SetFromAxisX(dir_12_loc1, VECT_Y);

    ChQuaterniond Q_loc1 = rel_matrix.GetQuaternion();
    return ChFrame<>(m_pos1, Q_loc1);
}

ChFramed ChLinkDistance::GetFrame2Rel() const {
    // unit vector from pos2 to pos1 in absolute coordinates
    ChVector3d dir_21_abs =
        Vnorm(m_body1->TransformPointLocalToParent(m_pos1) - m_body2->TransformPointLocalToParent(m_pos2));
    ChVector3d dir_21_loc2 = m_body2->TransformDirectionParentToLocal(dir_21_abs);
    ChMatrix33<> rel_matrix;
    rel_matrix.SetFromAxisX(dir_21_loc2, VECT_Y);

    ChQuaterniond Q_loc2 = rel_matrix.GetQuaternion();
    return ChFrame<>(m_pos2, Q_loc2);
}

void ChLinkDistance::SetMode(Mode mode) {
    this->mode = mode;
    mode_sign = (this->mode == Mode::UNILATERAL_MAXDISTANCE ? -1.0 : +1.0);
    Cx.SetMode(this->mode == Mode::BILATERAL ? ChConstraint::Mode::LOCK : ChConstraint::Mode::UNILATERAL);
}

void ChLinkDistance::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class (ChLink), basically doing nothing :)
    ChLink::Update(mytime, update_assets);

    // compute jacobians
    ChVector3d delta_pos = m_body1->TransformPointLocalToParent(m_pos1) - m_body2->TransformPointLocalToParent(m_pos2);
    curr_dist = delta_pos.Length();
    ChVector3d dir_F1_F2_W = Vnorm(delta_pos);
    ChVector3d dir_F1_F2_B2 = m_body2->TransformDirectionParentToLocal(dir_F1_F2_W);
    ChVector3d dir_F1_F2_B1 = m_body1->TransformDirectionParentToLocal(dir_F1_F2_W);

    ChVector3d Cq_B1_pos = dir_F1_F2_W;
    ChVector3d Cq_B2_pos = -dir_F1_F2_W;

    ChVector3d Cq_B1_rot = -Vcross(dir_F1_F2_B1, m_pos1);
    ChVector3d Cq_B2_rot = Vcross(dir_F1_F2_B2, m_pos2);

    Cx.Get_Cq_a()(0) = mode_sign * Cq_B1_pos.x();
    Cx.Get_Cq_a()(1) = mode_sign * Cq_B1_pos.y();
    Cx.Get_Cq_a()(2) = mode_sign * Cq_B1_pos.z();
    Cx.Get_Cq_a()(3) = mode_sign * Cq_B1_rot.x();
    Cx.Get_Cq_a()(4) = mode_sign * Cq_B1_rot.y();
    Cx.Get_Cq_a()(5) = mode_sign * Cq_B1_rot.z();

    Cx.Get_Cq_b()(0) = mode_sign * Cq_B2_pos.x();
    Cx.Get_Cq_b()(1) = mode_sign * Cq_B2_pos.y();
    Cx.Get_Cq_b()(2) = mode_sign * Cq_B2_pos.z();
    Cx.Get_Cq_b()(3) = mode_sign * Cq_B2_rot.x();
    Cx.Get_Cq_b()(4) = mode_sign * Cq_B2_rot.y();
    Cx.Get_Cq_b()(5) = mode_sign * Cq_B2_rot.z();

    C[0] = mode_sign * (curr_dist - distance);

    //// TODO  C_dt? C_dtdt? (may be never used..)
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkDistance::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = -react_force.x();
}

void ChLinkDistance::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react_force.x() = -L(off_L);
    react_force.y() = 0;
    react_force.z() = 0;

    react_torque = VNULL;
}

void ChLinkDistance::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  ///< the L vector
                                         const double c               ///< a scaling factor
) {
    if (!IsActive())
        return;

    Cx.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChLinkDistance::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                         ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                         const double c,            ///< a scaling factor
                                         bool do_clamp,             ///< apply clamping to c*C?
                                         double recovery_clamp      ///< value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    if (do_clamp)
        if (mode == Mode::BILATERAL)
            Qc(off_L) += std::min(std::max(c * C[0], -recovery_clamp), recovery_clamp);
        else
            Qc(off_L) += std::max(c * C[0], -recovery_clamp);
    else
        Qc(off_L) += c * C[0];
}

void ChLinkDistance::IntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    Cx.SetLagrangeMultiplier(L(off_L));

    Cx.SetRightHandSide(Qc(off_L));
}

void ChLinkDistance::IntFromDescriptor(const unsigned int off_v,
                                       ChStateDelta& v,
                                       const unsigned int off_L,
                                       ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L) = Cx.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkDistance::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&Cx);
}

void ChLinkDistance::ConstraintsBiReset() {
    Cx.SetRightHandSide(0.);
}

void ChLinkDistance::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
        return;

    if (do_clamp)
        Cx.SetRightHandSide(Cx.GetRightHandSide() + std::min(std::max(factor * C[0], -recovery_clamp), recovery_clamp));
    else
        Cx.SetRightHandSide(Cx.GetRightHandSide() + factor * C[0]);
}

void ChLinkDistance::LoadConstraintJacobians() {
    // already loaded when doing Update (which used the matrices of the scalar constraint objects)
}

void ChLinkDistance::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    react_force.x() = -Cx.GetLagrangeMultiplier() * factor;
    react_force.y() = 0;
    react_force.z() = 0;

    react_torque = VNULL;
}

void ChLinkDistance::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkDistance>();

    // serialize parent class
    ChLink::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(distance);
    archive_out << CHNVP(m_pos1);
    archive_out << CHNVP(m_pos2);

    ChLinkDistance_Mode_enum_mapper::Mode_mapper typemapper;
    archive_out << CHNVP(typemapper(mode), "ChLinkDistance__Mode");
}

/// Method to allow de serialization of transient data from archives.
void ChLinkDistance::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkDistance>();

    // deserialize parent class
    ChLink::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(distance);
    archive_in >> CHNVP(m_pos1);
    archive_in >> CHNVP(m_pos2);

    Cx.SetVariables(&m_body1->Variables(), &m_body2->Variables());

    ChLinkDistance_Mode_enum_mapper::Mode_mapper typemapper;
    Mode mode_temp;
    archive_in >> CHNVP(typemapper(mode_temp), "ChLinkDistance__Mode");
    SetMode(mode_temp);
}

}  // namespace chrono
