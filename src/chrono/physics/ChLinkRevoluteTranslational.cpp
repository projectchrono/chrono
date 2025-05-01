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

#include "chrono/physics/ChLinkRevoluteTranslational.h"

namespace chrono {

// Register into the object factory.
CH_FACTORY_REGISTER(ChLinkRevoluteTranslational)

// -----------------------------------------------------------------------------
// Constructor and destructor
// -----------------------------------------------------------------------------
ChLinkRevoluteTranslational::ChLinkRevoluteTranslational()
    : m_p1(ChVector3d(0, 0, 0)),
      m_p2(ChVector3d(0, 0, 0)),
      m_z1(ChVector3d(0, 0, 1)),
      m_x2(ChVector3d(1, 0, 0)),
      m_y2(ChVector3d(0, 1, 0)),
      m_dist(0),
      m_cur_par1(0),
      m_cur_par2(0),
      m_cur_dot(0),
      m_cur_dist(0) {
    for (int i = 0; i < 4; i++) {
        m_multipliers[i] = 0;
    }
}

ChLinkRevoluteTranslational::ChLinkRevoluteTranslational(const ChLinkRevoluteTranslational& other) : ChLink(other) {
    m_body1 = other.m_body1;
    m_body2 = other.m_body2;
    system = other.system;

    m_p1 = other.m_p1;
    m_p2 = other.m_p2;
    m_z1 = other.m_z1;
    m_x2 = other.m_x2;
    m_y2 = other.m_y2;
    m_dist = other.m_dist;
    m_cur_par1 = other.m_cur_par1;
    m_cur_par2 = other.m_cur_par2;
    m_cur_dot = other.m_cur_dot;
    m_cur_dist = other.m_cur_dist;

    if (other.m_body1 && other.m_body2) {
        m_cnstr_par1.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
        m_cnstr_par2.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
        m_cnstr_dot.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
        m_cnstr_dist.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
    }

    for (int i = 0; i < 4; i++) {
        m_multipliers[i] = other.m_multipliers[i];
    }
}

// -----------------------------------------------------------------------------

ChFrame<> ChLinkRevoluteTranslational::GetFrame1Rel() const {
    auto x2 = m_body1->TransformDirectionParentToLocal(m_body2->TransformDirectionLocalToParent(m_x2));
    auto y2 = m_body1->TransformDirectionParentToLocal(m_body2->TransformDirectionLocalToParent(m_y2));

    return ChFramed(m_p1, ChMatrix33<>(x2, y2, m_z1));
}

ChFrame<> ChLinkRevoluteTranslational::GetFrame2Rel() const {
    auto z1 = m_body2->TransformDirectionParentToLocal(m_body1->TransformDirectionLocalToParent(m_z1));

    return ChFramed(m_p2, ChMatrix33<>(m_x2, m_y2, z1));
}

// -----------------------------------------------------------------------------
// Override the ChLink default GetReaction1 and GetReaction2
// This is necessary because here we interpret react_force and react_torque as
// as the reactions on body 1 (revolute side) expressed in link frame 1.
// -----------------------------------------------------------------------------

ChWrenchd ChLinkRevoluteTranslational::GetReaction1() const {
    return {react_force, react_torque};
}

ChWrenchd ChLinkRevoluteTranslational::GetReaction2() const {
    auto w2_abs = GetFrame1Abs().TransformWrenchLocalToParent({-react_force, -react_torque});
    return GetFrame2Abs().TransformWrenchParentToLocal(w2_abs);
}

// -----------------------------------------------------------------------------
// Link initialization functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteTranslational::Initialize(std::shared_ptr<ChBody> body1,
                                             std::shared_ptr<ChBody> body2,
                                             const ChCoordsys<>& csys,
                                             double distance) {
    m_body1 = body1.get();
    m_body2 = body2.get();

    m_cnstr_par1.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_par2.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_dot.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_dist.SetVariables(&m_body1->Variables(), &m_body2->Variables());

    ChVector3d x_axis = csys.rot.GetAxisX();
    ChVector3d y_axis = csys.rot.GetAxisY();
    ChVector3d z_axis = csys.rot.GetAxisZ();

    m_p1 = m_body1->TransformPointParentToLocal(csys.pos);
    m_z1 = m_body1->TransformDirectionParentToLocal(z_axis);
    m_p2 = m_body2->TransformPointParentToLocal(csys.pos + distance * x_axis);
    m_x2 = m_body2->TransformDirectionParentToLocal(x_axis);
    m_y2 = m_body2->TransformDirectionParentToLocal(y_axis);

    m_dist = distance;

    m_cur_par1 = 0;
    m_cur_par2 = 0;
    m_cur_dot = 0;
    m_cur_dist = distance;
}

void ChLinkRevoluteTranslational::Initialize(std::shared_ptr<ChBody> body1,
                                             std::shared_ptr<ChBody> body2,
                                             bool local,
                                             const ChVector3d& p1,
                                             const ChVector3d& dirZ1,
                                             const ChVector3d& p2,
                                             const ChVector3d& dirX2,
                                             const ChVector3d& dirY2,
                                             bool auto_distance,
                                             double distance) {
    m_body1 = body1.get();
    m_body2 = body2.get();

    m_cnstr_par1.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_par2.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_dot.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_dist.SetVariables(&m_body1->Variables(), &m_body2->Variables());

    ChVector3d p1_abs;
    ChVector3d p2_abs;
    ChVector3d z1_abs;
    ChVector3d x2_abs;
    ChVector3d y2_abs;

    if (local) {
        m_p1 = p1;
        m_p2 = p2;
        m_z1 = Vnorm(dirZ1);
        m_x2 = Vnorm(dirX2);
        m_y2 = Vnorm(dirY2);
        p1_abs = m_body1->TransformPointLocalToParent(m_p1);
        p2_abs = m_body2->TransformPointLocalToParent(m_p2);
        z1_abs = m_body1->TransformDirectionLocalToParent(m_z1);
        x2_abs = m_body2->TransformDirectionLocalToParent(m_x2);
        y2_abs = m_body2->TransformDirectionLocalToParent(m_y2);
    } else {
        p1_abs = p1;
        p2_abs = p2;
        z1_abs = Vnorm(dirZ1);
        x2_abs = Vnorm(dirX2);
        y2_abs = Vnorm(dirY2);
        m_p1 = m_body1->TransformPointParentToLocal(p1_abs);
        m_p2 = m_body2->TransformPointParentToLocal(p2_abs);
        m_z1 = m_body1->TransformDirectionParentToLocal(z1_abs);
        m_x2 = m_body2->TransformDirectionParentToLocal(x2_abs);
        m_y2 = m_body2->TransformDirectionParentToLocal(y2_abs);
    }

    ChVector3d d12_abs = p2_abs - p1_abs;

    m_cur_par1 = Vdot(z1_abs, x2_abs);
    m_cur_par2 = Vdot(z1_abs, y2_abs);
    m_cur_dot = Vdot(d12_abs, z1_abs);
    m_cur_dist = Vdot(d12_abs, x2_abs);

    m_dist = auto_distance ? m_cur_dist : distance;
}

// -----------------------------------------------------------------------------
// Link update function
// -----------------------------------------------------------------------------
void ChLinkRevoluteTranslational::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChLink::Update(time, update_assets);

    // Express the body locations and direction in absolute frame
    ChVector3d p1_abs = m_body1->TransformPointLocalToParent(m_p1);
    ChVector3d p2_abs = m_body2->TransformPointLocalToParent(m_p2);
    ChVector3d z1_abs = m_body1->TransformDirectionLocalToParent(m_z1);
    ChVector3d x2_abs = m_body2->TransformDirectionLocalToParent(m_x2);
    ChVector3d y2_abs = m_body2->TransformDirectionLocalToParent(m_y2);
    ChVector3d d12_abs = p2_abs - p1_abs;

    // Update current constraint quantities
    m_cur_par1 = Vdot(z1_abs, x2_abs);
    m_cur_par2 = Vdot(z1_abs, y2_abs);
    m_cur_dot = Vdot(d12_abs, z1_abs);
    m_cur_dist = Vdot(d12_abs, x2_abs);

    // Calculate a few more quantities
    // (express directions on one body in the frame ot the other body)
    ChVector3d z1_2 = m_body2->TransformDirectionParentToLocal(z1_abs);
    ChVector3d x2_1 = m_body1->TransformDirectionParentToLocal(x2_abs);
    ChVector3d y2_1 = m_body1->TransformDirectionParentToLocal(y2_abs);

    ChVector3d d12_1 = m_body1->TransformDirectionParentToLocal(d12_abs);
    ChVector3d d12_2 = m_body2->TransformDirectionParentToLocal(d12_abs);

    // First constraint (par1)
    {
        // Cache constraint violation
        m_C(0) = m_cur_par1;

        // Set Jacobian w.r.t. states of Body 1
        ChVector3d Phi_pi1 = Vcross(m_z1, x2_1);

        m_cnstr_par1.Get_Cq_a()(0) = 0;
        m_cnstr_par1.Get_Cq_a()(1) = 0;
        m_cnstr_par1.Get_Cq_a()(2) = 0;

        m_cnstr_par1.Get_Cq_a()(3) = Phi_pi1.x();
        m_cnstr_par1.Get_Cq_a()(4) = Phi_pi1.y();
        m_cnstr_par1.Get_Cq_a()(5) = Phi_pi1.z();

        // Set Jacobian w.r.t. states of Body 2
        ChVector3d Phi_pi2 = Vcross(m_x2, z1_2);

        m_cnstr_par1.Get_Cq_b()(0) = 0;
        m_cnstr_par1.Get_Cq_b()(1) = 0;
        m_cnstr_par1.Get_Cq_b()(2) = 0;

        m_cnstr_par1.Get_Cq_b()(3) = Phi_pi2.x();
        m_cnstr_par1.Get_Cq_b()(4) = Phi_pi2.y();
        m_cnstr_par1.Get_Cq_b()(5) = Phi_pi2.z();
    }

    // Second constraint (par2)
    {
        // Cache constraint violation
        m_C(1) = m_cur_par2;

        // Set Jacobian w.r.t. states of Body 1
        ChVector3d Phi_pi1 = Vcross(m_z1, y2_1);

        m_cnstr_par2.Get_Cq_a()(0) = 0;
        m_cnstr_par2.Get_Cq_a()(1) = 0;
        m_cnstr_par2.Get_Cq_a()(2) = 0;

        m_cnstr_par2.Get_Cq_a()(3) = Phi_pi1.x();
        m_cnstr_par2.Get_Cq_a()(4) = Phi_pi1.y();
        m_cnstr_par2.Get_Cq_a()(5) = Phi_pi1.z();

        // Set Jacobian w.r.t. states of Body 2
        ChVector3d Phi_pi2 = Vcross(m_y2, z1_2);

        m_cnstr_par2.Get_Cq_b()(0) = 0;
        m_cnstr_par2.Get_Cq_b()(1) = 0;
        m_cnstr_par2.Get_Cq_b()(2) = 0;

        m_cnstr_par2.Get_Cq_b()(3) = Phi_pi2.x();
        m_cnstr_par2.Get_Cq_b()(4) = Phi_pi2.y();
        m_cnstr_par2.Get_Cq_b()(5) = Phi_pi2.z();
    }

    // Third constraint (dot)
    {
        // Cache constraint violation
        m_C(2) = m_cur_dot;

        // Set Jacobian w.r.t. states of Body 1
        ChVector3d Phi_pi1 = Vcross(m_z1, d12_1) - Vcross(m_p1, m_z1);

        m_cnstr_dot.Get_Cq_a()(0) = -z1_abs.x();
        m_cnstr_dot.Get_Cq_a()(1) = -z1_abs.y();
        m_cnstr_dot.Get_Cq_a()(2) = -z1_abs.z();

        m_cnstr_dot.Get_Cq_a()(3) = Phi_pi1.x();
        m_cnstr_dot.Get_Cq_a()(4) = Phi_pi1.y();
        m_cnstr_dot.Get_Cq_a()(5) = Phi_pi1.z();

        // Set Jacobian w.r.t. states of Body 2
        ChVector3d Phi_pi2 = Vcross(m_p2, z1_2);

        m_cnstr_dot.Get_Cq_b()(0) = z1_abs.x();
        m_cnstr_dot.Get_Cq_b()(1) = z1_abs.y();
        m_cnstr_dot.Get_Cq_b()(2) = z1_abs.z();

        m_cnstr_dot.Get_Cq_b()(3) = Phi_pi2.x();
        m_cnstr_dot.Get_Cq_b()(4) = Phi_pi2.y();
        m_cnstr_dot.Get_Cq_b()(5) = Phi_pi2.z();
    }

    // Fourth constraint (dist)
    {
        // Cache constraint violation
        m_C(3) = m_cur_dist - m_dist;

        // Set Jacobian w.r.t. states of Body 1
        ChVector3d Phi_pi1 = -Vcross(m_p1, x2_1);

        m_cnstr_dist.Get_Cq_a()(0) = -x2_abs.x();
        m_cnstr_dist.Get_Cq_a()(1) = -x2_abs.y();
        m_cnstr_dist.Get_Cq_a()(2) = -x2_abs.z();

        m_cnstr_dist.Get_Cq_a()(3) = Phi_pi1.x();
        m_cnstr_dist.Get_Cq_a()(4) = Phi_pi1.y();
        m_cnstr_dist.Get_Cq_a()(5) = Phi_pi1.z();

        // Set Jacobian w.r.t. states of Body 2
        ChVector3d Phi_pi2 = Vcross(m_x2, d12_2) + Vcross(m_p2, m_x2);

        m_cnstr_dist.Get_Cq_b()(0) = x2_abs.x();
        m_cnstr_dist.Get_Cq_b()(1) = x2_abs.y();
        m_cnstr_dist.Get_Cq_b()(2) = x2_abs.z();

        m_cnstr_dist.Get_Cq_b()(3) = Phi_pi2.x();
        m_cnstr_dist.Get_Cq_b()(4) = Phi_pi2.y();
        m_cnstr_dist.Get_Cq_b()(5) = Phi_pi2.z();
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkRevoluteTranslational::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    L(off_L + 0) = m_multipliers[0];
    L(off_L + 1) = m_multipliers[1];
    L(off_L + 2) = m_multipliers[2];
    L(off_L + 3) = m_multipliers[3];
}

void ChLinkRevoluteTranslational::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    m_multipliers[0] = L(off_L + 0);
    m_multipliers[1] = L(off_L + 1);
    m_multipliers[2] = L(off_L + 2);
    m_multipliers[3] = L(off_L + 3);

    // Also compute 'intuitive' reactions:
    ////double lam_par1 = m_multipliers[0];
    ////double lam_par2 = m_multipliers[1];
    ////double lam_dot = m_multipliers[2];
    ////double lam_dist = m_multipliers[3];

    ////
    //// TODO
    ////

    // For this joint, we define react_force and react_torque as the reaction force and torque on body 1 (revolute side)
    // in the link frame 1. This frame is centered at the revolute joint location, has its x axis along the joint
    // connector and z axis aligned with the revolute axis of rotation.

    react_force.x() = 0;
    react_force.y() = 0;
    react_force.z() = 0;

    react_torque.x() = 0;
    react_torque.y() = 0;
    react_torque.z() = 0;
}

void ChLinkRevoluteTranslational::IntLoadResidual_CqL(const unsigned int off_L,
                                                      ChVectorDynamic<>& R,
                                                      const ChVectorDynamic<>& L,
                                                      const double c) {
    m_cnstr_par1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    m_cnstr_par2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    m_cnstr_dot.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
    m_cnstr_dist.AddJacobianTransposedTimesScalarInto(R, L(off_L + 3) * c);
}

void ChLinkRevoluteTranslational::IntLoadConstraint_C(const unsigned int off_L,
                                                      ChVectorDynamic<>& Qc,
                                                      const double c,
                                                      bool do_clamp,
                                                      double recovery_clamp) {
    if (!IsActive())
        return;

    double cnstr_par1_violation =
        do_clamp ? std::min(std::max(c * m_cur_par1, -recovery_clamp), recovery_clamp) : c * m_cur_par1;
    double cnstr_par2_violation =
        do_clamp ? std::min(std::max(c * m_cur_par2, -recovery_clamp), recovery_clamp) : c * m_cur_par2;
    double cnstr_dot_violation =
        do_clamp ? std::min(std::max(c * m_cur_dot, -recovery_clamp), recovery_clamp) : c * m_cur_dot;
    double cnstr_dist_violation = do_clamp
                                      ? std::min(std::max(c * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp)
                                      : c * (m_cur_dist - m_dist);

    Qc(off_L + 0) += cnstr_par1_violation;
    Qc(off_L + 1) += cnstr_par2_violation;
    Qc(off_L + 2) += cnstr_dot_violation;
    Qc(off_L + 3) += cnstr_dist_violation;
}

void ChLinkRevoluteTranslational::IntToDescriptor(const unsigned int off_v,
                                                  const ChStateDelta& v,
                                                  const ChVectorDynamic<>& R,
                                                  const unsigned int off_L,
                                                  const ChVectorDynamic<>& L,
                                                  const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_cnstr_par1.SetLagrangeMultiplier(L(off_L + 0));
    m_cnstr_par2.SetLagrangeMultiplier(L(off_L + 1));
    m_cnstr_dot.SetLagrangeMultiplier(L(off_L + 2));
    m_cnstr_dist.SetLagrangeMultiplier(L(off_L + 3));

    m_cnstr_par1.SetRightHandSide(Qc(off_L + 0));
    m_cnstr_par2.SetRightHandSide(Qc(off_L + 1));
    m_cnstr_dot.SetRightHandSide(Qc(off_L + 2));
    m_cnstr_dist.SetRightHandSide(Qc(off_L + 3));
}

void ChLinkRevoluteTranslational::IntFromDescriptor(const unsigned int off_v,
                                                    ChStateDelta& v,
                                                    const unsigned int off_L,
                                                    ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_cnstr_par1.GetLagrangeMultiplier();
    L(off_L + 1) = m_cnstr_par2.GetLagrangeMultiplier();
    L(off_L + 2) = m_cnstr_dot.GetLagrangeMultiplier();
    L(off_L + 3) = m_cnstr_dist.GetLagrangeMultiplier();
}

// -----------------------------------------------------------------------------
// Implementation of solver interface functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteTranslational::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_cnstr_par1);
    descriptor.InsertConstraint(&m_cnstr_par2);
    descriptor.InsertConstraint(&m_cnstr_dot);
    descriptor.InsertConstraint(&m_cnstr_dist);
}

void ChLinkRevoluteTranslational::ConstraintsBiReset() {
    m_cnstr_par1.SetRightHandSide(0.0);
    m_cnstr_par2.SetRightHandSide(0.0);
    m_cnstr_dot.SetRightHandSide(0.0);
    m_cnstr_dist.SetRightHandSide(0.0);
}

void ChLinkRevoluteTranslational::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
        return;

    double cnstr_par1_violation =
        do_clamp ? std::min(std::max(factor * m_cur_par1, -recovery_clamp), recovery_clamp) : factor * m_cur_par1;
    double cnstr_par2_violation =
        do_clamp ? std::min(std::max(factor * m_cur_par2, -recovery_clamp), recovery_clamp) : factor * m_cur_par2;
    double cnstr_dot_violation =
        do_clamp ? std::min(std::max(factor * m_cur_dot, -recovery_clamp), recovery_clamp) : factor * m_cur_dot;
    double cnstr_dist_violation =
        do_clamp ? std::min(std::max(factor * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp)
                 : factor * (m_cur_dist - m_dist);

    m_cnstr_par1.SetRightHandSide(m_cnstr_par1.GetRightHandSide() + cnstr_par1_violation);
    m_cnstr_par2.SetRightHandSide(m_cnstr_par2.GetRightHandSide() + cnstr_par2_violation);
    m_cnstr_dot.SetRightHandSide(m_cnstr_dot.GetRightHandSide() + cnstr_dot_violation);
    m_cnstr_dist.SetRightHandSide(m_cnstr_dist.GetRightHandSide() + cnstr_dist_violation);
}

void ChLinkRevoluteTranslational::LoadConstraintJacobians() {
    // Nothing to do here. Jacobians were loaded in Update().
}

void ChLinkRevoluteTranslational::ConstraintsFetch_react(double factor) {
    // Extract the Lagrange multipliers for the four constraints
    double lam_par1 = m_cnstr_par1.GetLagrangeMultiplier();
    double lam_par2 = m_cnstr_par2.GetLagrangeMultiplier();
    double lam_dot = m_cnstr_dot.GetLagrangeMultiplier();
    double lam_dist = m_cnstr_dist.GetLagrangeMultiplier();

    // Note that the Lagrange multipliers must be multiplied by 'factor' to
    // convert from reaction impulses to reaction forces.
    lam_par1 *= factor;
    lam_par2 *= factor;
    lam_dot *= factor;
    lam_dist *= factor;

    ////
    ////  TODO
    ////

    // For this joint, we define react_force and react_torque as the reaction force and torque on body 1 (revolute side)
    // in the link frame 1. This frame is centered at the revolute joint location, has its x axis along the joint
    // connector and z axis aligned with the revolute axis of rotation.

    react_force.x() = 0;
    react_force.y() = 0;
    react_force.z() = 0;

    react_torque.x() = 0;
    react_torque.y() = 0;
    react_torque.z() = 0;
}

// -----------------------------------------------------------------------------

void ChLinkRevoluteTranslational::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkRevoluteTranslational>();

    // serialize parent class
    ChLink::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_p1);
    archive_out << CHNVP(m_p2);
    archive_out << CHNVP(m_z1);
    archive_out << CHNVP(m_x2);
    archive_out << CHNVP(m_y2);
    archive_out << CHNVP(m_dist);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRevoluteTranslational::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkRevoluteTranslational>();

    // deserialize parent class
    ChLink::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_p1);
    archive_in >> CHNVP(m_p2);
    archive_in >> CHNVP(m_z1);
    archive_in >> CHNVP(m_x2);
    archive_in >> CHNVP(m_y2);
    archive_in >> CHNVP(m_dist);
}

}  // end namespace chrono
