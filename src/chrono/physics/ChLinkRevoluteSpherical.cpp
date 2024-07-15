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
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkRevoluteSpherical.h"

namespace chrono {

// Register into the object factory.
CH_FACTORY_REGISTER(ChLinkRevoluteSpherical)

// -----------------------------------------------------------------------------
// Constructor and destructor
// -----------------------------------------------------------------------------
ChLinkRevoluteSpherical::ChLinkRevoluteSpherical()
    : m_pos1(ChVector3d(0, 0, 0)),
      m_pos2(ChVector3d(0, 0, 0)),
      m_dir1(ChVector3d(0, 0, 1)),
      m_dist(0),
      m_cur_dist(0),
      m_cur_dot(0) {
    m_multipliers[0] = 0;
    m_multipliers[1] = 0;
}

ChLinkRevoluteSpherical::ChLinkRevoluteSpherical(const ChLinkRevoluteSpherical& other) : ChLink(other) {
    m_body1 = other.m_body1;
    m_body2 = other.m_body2;
    system = other.system;

    m_pos1 = other.m_pos1;
    m_pos2 = other.m_pos2;
    m_dir1 = other.m_dir1;
    m_dist = other.m_dist;
    m_cur_dist = other.m_cur_dist;
    m_cur_dot = other.m_cur_dot;

    if (other.m_body1 && other.m_body2) {
        m_cnstr_dist.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
        m_cnstr_dot.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
    }

    m_multipliers[0] = other.m_multipliers[0];
    m_multipliers[1] = other.m_multipliers[1];
}

// -----------------------------------------------------------------------------

ChFrame<> ChLinkRevoluteSpherical::GetFrame1Rel() const {
    ChVector3d pos2_F1 = m_body1->TransformPointParentToLocal(m_body2->TransformPointLocalToParent(m_pos2));

    ChVector3d u = (pos2_F1 - m_pos1).GetNormalized();
    ChVector3d w = m_dir1;
    ChVector3d v = Vcross(w, u);
    ChMatrix33<> A(u, v, w);

    return ChFrame<>(m_pos1, A.GetQuaternion());
}

ChFrame<> ChLinkRevoluteSpherical::GetFrame2Rel() const {
    ChVector3d pos1_F2 = m_body2->TransformPointParentToLocal(m_body1->TransformPointLocalToParent(m_pos1));

    ChVector3d u = (m_pos2 - pos1_F2).GetNormalized();
    ChVector3d w = m_body2->TransformDirectionParentToLocal(m_body1->TransformDirectionLocalToParent(m_dir1));
    ChVector3d v = Vcross(w, u);
    ChMatrix33<> A(u, v, w);

    return ChFrame<>(m_pos2, A.GetQuaternion());
}

// -----------------------------------------------------------------------------
// Override the ChLink default GetReaction1 and GetReaction2
// This is necessary because here we interpret react_force and react_torque as
// as the reactions on body 1 (revolute side) expressed in link frame 1.
// -----------------------------------------------------------------------------

ChWrenchd ChLinkRevoluteSpherical::GetReaction1() const {
    return {react_force, react_torque};
}

ChWrenchd ChLinkRevoluteSpherical::GetReaction2() const {
    auto w2_abs = GetFrame1Abs().TransformWrenchLocalToParent({-react_force, -react_torque});
    return GetFrame2Abs().TransformWrenchParentToLocal(w2_abs);
}

// -----------------------------------------------------------------------------
// Link initialization functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::Initialize(std::shared_ptr<ChBody> body1,
                                         std::shared_ptr<ChBody> body2,
                                         const ChCoordsys<>& csys,
                                         double distance) {
    m_body1 = body1.get();
    m_body2 = body2.get();

    m_cnstr_dist.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_dot.SetVariables(&m_body1->Variables(), &m_body2->Variables());

    ChVector3d x_Axis = csys.rot.GetAxisX();
    ChVector3d z_axis = csys.rot.GetAxisZ();

    m_pos1 = m_body1->TransformPointParentToLocal(csys.pos);
    m_dir1 = m_body1->TransformDirectionParentToLocal(z_axis);
    m_pos2 = m_body2->TransformPointParentToLocal(csys.pos + distance * x_Axis);

    m_dist = distance;
    m_cur_dist = distance;
    m_cur_dot = 0;
}

void ChLinkRevoluteSpherical::Initialize(std::shared_ptr<ChBody> body1,
                                         std::shared_ptr<ChBody> body2,
                                         bool local,
                                         const ChVector3d& pos1,
                                         const ChVector3d& dir1,
                                         const ChVector3d& pos2,
                                         bool auto_distance,
                                         double distance) {
    m_body1 = body1.get();
    m_body2 = body2.get();

    m_cnstr_dist.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_dot.SetVariables(&m_body1->Variables(), &m_body2->Variables());

    ChVector3d pos1_abs;
    ChVector3d pos2_abs;
    ChVector3d dir1_abs;

    if (local) {
        m_pos1 = pos1;
        m_pos2 = pos2;
        m_dir1 = Vnorm(dir1);
        pos1_abs = m_body1->TransformPointLocalToParent(m_pos1);
        pos2_abs = m_body2->TransformPointLocalToParent(m_pos2);
        dir1_abs = m_body1->TransformDirectionLocalToParent(m_dir1);
    } else {
        pos1_abs = pos1;
        pos2_abs = pos2;
        dir1_abs = Vnorm(dir1);
        m_pos1 = m_body1->TransformPointParentToLocal(pos1_abs);
        m_pos2 = m_body2->TransformPointParentToLocal(pos2_abs);
        m_dir1 = m_body1->TransformDirectionParentToLocal(dir1_abs);
    }

    ChVector3d d12_abs = pos2_abs - pos1_abs;

    m_cur_dist = d12_abs.Length();
    m_dist = auto_distance ? m_cur_dist : distance;

    m_cur_dot = Vdot(d12_abs, dir1_abs);
}

// -----------------------------------------------------------------------------
// Link update function
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::Update(double time, bool update_assets) {
    // Inherit time changes of parent class (ChLink)
    ChLink::Update(time, update_assets);

    // Express the body locations and direction in absolute frame
    ChVector3d pos1_abs = m_body1->TransformPointLocalToParent(m_pos1);
    ChVector3d pos2_abs = m_body2->TransformPointLocalToParent(m_pos2);
    ChVector3d dir1_abs = m_body1->TransformDirectionLocalToParent(m_dir1);
    ChVector3d d12_abs = pos2_abs - pos1_abs;

    // Update current distance and dot product
    m_cur_dist = d12_abs.Length();
    m_cur_dot = Vdot(d12_abs, dir1_abs);

    // Calculate a unit vector in the direction d12, expressed in absolute frame
    // Then express it in the two body frames
    ChVector3d u12_abs = d12_abs / m_cur_dist;
    ChVector3d u12_loc1 = m_body1->TransformDirectionParentToLocal(u12_abs);
    ChVector3d u12_loc2 = m_body2->TransformDirectionParentToLocal(u12_abs);

    // Express the direction vector in the frame of body 2
    ChVector3d dir1_loc2 = m_body2->TransformDirectionParentToLocal(dir1_abs);

    // Cache violation of the distance constraint
    m_C(0) = m_cur_dist - m_dist;

    // Compute Jacobian of the distance constraint
    //    ||pos2_abs - pos1_abs|| - dist = 0
    {
        ChVector3d Phi_r1 = -u12_abs;
        ChVector3d Phi_pi1 = Vcross(u12_loc1, m_pos1);

        m_cnstr_dist.Get_Cq_a()(0) = Phi_r1.x();
        m_cnstr_dist.Get_Cq_a()(1) = Phi_r1.y();
        m_cnstr_dist.Get_Cq_a()(2) = Phi_r1.z();

        m_cnstr_dist.Get_Cq_a()(3) = Phi_pi1.x();
        m_cnstr_dist.Get_Cq_a()(4) = Phi_pi1.y();
        m_cnstr_dist.Get_Cq_a()(5) = Phi_pi1.z();

        ChVector3d Phi_r2 = u12_abs;
        ChVector3d Phi_pi2 = -Vcross(u12_loc2, m_pos2);

        m_cnstr_dist.Get_Cq_b()(0) = Phi_r2.x();
        m_cnstr_dist.Get_Cq_b()(1) = Phi_r2.y();
        m_cnstr_dist.Get_Cq_b()(2) = Phi_r2.z();

        m_cnstr_dist.Get_Cq_b()(3) = Phi_pi2.x();
        m_cnstr_dist.Get_Cq_b()(4) = Phi_pi2.y();
        m_cnstr_dist.Get_Cq_b()(5) = Phi_pi2.z();
    }

    // Cache violation of the dot constraint
    m_C(1) = m_cur_dot;

    // Compute Jacobian of the dot constraint
    //    dot(dir1_abs, pos2_abs - pos1_abs) = 0
    {
        ChVector3d Phi_r1 = -dir1_abs;
        ChVector3d Phi_pi1 = Vcross(m_dir1, m_pos1) - Vcross(u12_loc1, m_pos1);

        m_cnstr_dot.Get_Cq_a()(0) = Phi_r1.x();
        m_cnstr_dot.Get_Cq_a()(1) = Phi_r1.y();
        m_cnstr_dot.Get_Cq_a()(2) = Phi_r1.z();

        m_cnstr_dot.Get_Cq_a()(3) = Phi_pi1.x();
        m_cnstr_dot.Get_Cq_a()(4) = Phi_pi1.y();
        m_cnstr_dot.Get_Cq_a()(5) = Phi_pi1.z();

        ChVector3d Phi_r2 = dir1_abs;
        ChVector3d Phi_pi2 = -Vcross(dir1_loc2, m_pos2);

        m_cnstr_dot.Get_Cq_b()(0) = Phi_r2.x();
        m_cnstr_dot.Get_Cq_b()(1) = Phi_r2.y();
        m_cnstr_dot.Get_Cq_b()(2) = Phi_r2.z();

        m_cnstr_dot.Get_Cq_b()(3) = Phi_pi2.x();
        m_cnstr_dot.Get_Cq_b()(4) = Phi_pi2.y();
        m_cnstr_dot.Get_Cq_b()(5) = Phi_pi2.z();
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkRevoluteSpherical::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    L(off_L + 0) = m_multipliers[0];
    L(off_L + 1) = m_multipliers[1];
}

void ChLinkRevoluteSpherical::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    m_multipliers[0] = L(off_L + 0);
    m_multipliers[1] = L(off_L + 1);

    // Also compute 'intuitive' reactions:

    double lam_dist = m_multipliers[0];  // ||pos2_abs - pos1_abs|| - dist = 0
    double lam_dot = m_multipliers[1];   // dot(dir1_abs, pos2_abs - pos1_abs) = 0

    // For this joint, we define react_force and react_torque as the reaction force and torque on body 1 (revolute side)
    // in the link frame 1. This frame is centered at the revolute joint location, has its x axis along the rev-sph
    // joint connector and z axis aligned with the revolute axis of rotation.

    react_force.x() = -lam_dist;
    react_force.y() = 0;
    react_force.z() = -lam_dot;

    react_torque.x() = 0;
    react_torque.y() = m_cur_dist * lam_dot;
    react_torque.z() = 0;
}

void ChLinkRevoluteSpherical::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                                  ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                                  const ChVectorDynamic<>& L,  ///< the L vector
                                                  const double c               ///< a scaling factor
) {
    m_cnstr_dist.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    m_cnstr_dot.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
}

void ChLinkRevoluteSpherical::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                                  ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                                  const double c,            ///< a scaling factor
                                                  bool do_clamp,             ///< apply clamping to c*C?
                                                  double recovery_clamp      ///< value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    double cnstr_dist_violation = do_clamp
                                      ? std::min(std::max(c * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp)
                                      : c * (m_cur_dist - m_dist);

    double cnstr_dot_violation =
        do_clamp ? std::min(std::max(c * m_cur_dot, -recovery_clamp), recovery_clamp) : c * m_cur_dot;

    Qc(off_L + 0) += cnstr_dist_violation;
    Qc(off_L + 1) += cnstr_dot_violation;
}

void ChLinkRevoluteSpherical::IntToDescriptor(const unsigned int off_v,
                                              const ChStateDelta& v,
                                              const ChVectorDynamic<>& R,
                                              const unsigned int off_L,
                                              const ChVectorDynamic<>& L,
                                              const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_cnstr_dist.SetLagrangeMultiplier(L(off_L + 0));
    m_cnstr_dot.SetLagrangeMultiplier(L(off_L + 1));

    m_cnstr_dist.SetRightHandSide(Qc(off_L + 0));
    m_cnstr_dot.SetRightHandSide(Qc(off_L + 1));
}

void ChLinkRevoluteSpherical::IntFromDescriptor(const unsigned int off_v,
                                                ChStateDelta& v,
                                                const unsigned int off_L,
                                                ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_cnstr_dist.GetLagrangeMultiplier();
    L(off_L + 1) = m_cnstr_dot.GetLagrangeMultiplier();
}

// -----------------------------------------------------------------------------
// Implementation of solver interface functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_cnstr_dist);
    descriptor.InsertConstraint(&m_cnstr_dot);
}

void ChLinkRevoluteSpherical::ConstraintsBiReset() {
    m_cnstr_dist.SetRightHandSide(0.0);
    m_cnstr_dot.SetRightHandSide(0.0);
}

void ChLinkRevoluteSpherical::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
        return;

    double cnstr_dist_violation =
        do_clamp ? std::min(std::max(factor * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp)
                 : factor * (m_cur_dist - m_dist);

    double cnstr_dot_violation =
        do_clamp ? std::min(std::max(factor * m_cur_dot, -recovery_clamp), recovery_clamp) : factor * m_cur_dot;

    m_cnstr_dist.SetRightHandSide(m_cnstr_dist.GetRightHandSide() + cnstr_dist_violation);
    m_cnstr_dot.SetRightHandSide(m_cnstr_dot.GetRightHandSide() + cnstr_dot_violation);
}

void ChLinkRevoluteSpherical::LoadConstraintJacobians() {
    // Nothing to do here. Jacobians were loaded in Update().
}

void ChLinkRevoluteSpherical::ConstraintsFetch_react(double factor) {
    // Extract the Lagrange multipliers for the distance and for
    // the dot constraint.
    double lam_dist = m_cnstr_dist.GetLagrangeMultiplier();  // ||pos2_abs - pos1_abs|| - dist = 0
    double lam_dot = m_cnstr_dot.GetLagrangeMultiplier();    // dot(dir1_abs, pos2_abs - pos1_abs) = 0

    // Note that the Lagrange multipliers must be multiplied by 'factor' to
    // convert from reaction impulses to reaction forces.
    lam_dist *= factor;
    lam_dot *= factor;

    // For this joint, we define react_force and react_torque as the reaction force and torque on body 1 (revolute side)
    // in the link frame 1. This frame is centered at the revolute joint location, has its x axis along the rev-sph
    // joint connector and z axis aligned with the revolute axis of rotation.

    react_force.x() = -lam_dist;
    react_force.y() = 0;
    react_force.z() = -lam_dot;

    react_torque.x() = 0;
    react_torque.y() = m_cur_dist * lam_dot;
    react_torque.z() = 0;
}

// -----------------------------------------------------------------------------

void ChLinkRevoluteSpherical::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkRevoluteSpherical>();

    // serialize parent class
    ChLink::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_pos1);
    archive_out << CHNVP(m_pos2);
    archive_out << CHNVP(m_dir1);
    archive_out << CHNVP(m_dist);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRevoluteSpherical::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkRevoluteSpherical>();

    // deserialize parent class
    ChLink::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_pos1);
    archive_in >> CHNVP(m_pos2);
    archive_in >> CHNVP(m_dir1);
    archive_in >> CHNVP(m_dist);
}

}  // end namespace chrono
