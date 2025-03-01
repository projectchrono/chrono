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

#include "chrono/physics/ChLinkRevolute.h"

namespace chrono {

// Register into the object factory.
CH_FACTORY_REGISTER(ChLinkRevolute)

// -----------------------------------------------------------------------------
// Constructor and destructor
// -----------------------------------------------------------------------------
ChLinkRevolute::ChLinkRevolute() {
    for (int i = 0; i < 5; i++) {
        m_multipliers[i] = 0;
    }
}

ChLinkRevolute::ChLinkRevolute(const ChLinkRevolute& other) : ChLink(other) {
    m_body1 = other.m_body1;
    m_body2 = other.m_body2;
    system = other.system;

    m_frame1 = other.m_frame1;
    m_frame2 = other.m_frame2;

    if (other.m_body1 && other.m_body2) {
        m_cnstr_x.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
        m_cnstr_y.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
        m_cnstr_z.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
        m_cnstr_uw.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
        m_cnstr_vw.SetVariables(&other.m_body1->Variables(), &other.m_body2->Variables());
    }

    for (int i = 0; i < 5; i++) {
        m_multipliers[i] = other.m_multipliers[i];
    }
}

// -----------------------------------------------------------------------------
// Link initialization functions
// -----------------------------------------------------------------------------
void ChLinkRevolute::Initialize(std::shared_ptr<ChBody> body1, std::shared_ptr<ChBody> body2, const ChFrame<>& frame) {
    m_body1 = body1.get();
    m_body2 = body2.get();

    m_cnstr_x.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_y.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_z.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_uw.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_vw.SetVariables(&m_body1->Variables(), &m_body2->Variables());

    m_frame1 = ((ChFrame<>*)m_body1)->TransformParentToLocal(frame);
    m_frame2 = ((ChFrame<>*)m_body2)->TransformParentToLocal(frame);

    m_u1_tilde = ChStarMatrix33<>(m_frame1.GetRotMat().GetAxisX());
    m_v1_tilde = ChStarMatrix33<>(m_frame1.GetRotMat().GetAxisY());
    m_w2_tilde = ChStarMatrix33<>(m_frame2.GetRotMat().GetAxisZ());

    m_C.setZero();
}

void ChLinkRevolute::Initialize(std::shared_ptr<ChBody> body1,
                                std::shared_ptr<ChBody> body2,
                                bool local,
                                const ChFrame<>& frame1,
                                const ChFrame<>& frame2) {
    m_body1 = body1.get();
    m_body2 = body2.get();

    m_cnstr_x.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_y.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_z.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_uw.SetVariables(&m_body1->Variables(), &m_body2->Variables());
    m_cnstr_vw.SetVariables(&m_body1->Variables(), &m_body2->Variables());

    ChFrame<> frame1_abs;
    ChFrame<> frame2_abs;

    if (local) {
        m_frame1 = frame1;
        m_frame2 = frame2;
        frame1_abs = frame1 >> *m_body1;
        frame2_abs = frame2 >> *m_body2;
    } else {
        m_frame1 = ((ChFrame<>*)m_body1)->TransformParentToLocal(frame1);
        m_frame2 = ((ChFrame<>*)m_body2)->TransformParentToLocal(frame2);
        frame1_abs = frame1;
        frame2_abs = frame2;
    }

    m_u1_tilde = ChStarMatrix33<>(m_frame1.GetRotMat().GetAxisX());
    m_v1_tilde = ChStarMatrix33<>(m_frame1.GetRotMat().GetAxisY());
    m_w2_tilde = ChStarMatrix33<>(m_frame2.GetRotMat().GetAxisZ());

    m_C(0) = frame2_abs.GetPos().x() - frame1_abs.GetPos().x();
    m_C(1) = frame2_abs.GetPos().y() - frame1_abs.GetPos().y();
    m_C(2) = frame2_abs.GetPos().z() - frame1_abs.GetPos().z();
    m_C(3) = Vdot(frame1_abs.GetRotMat().GetAxisX(), frame2_abs.GetRotMat().GetAxisZ());
    m_C(4) = Vdot(frame1_abs.GetRotMat().GetAxisY(), frame2_abs.GetRotMat().GetAxisZ());
}

// -----------------------------------------------------------------------------
// Link update function
// -----------------------------------------------------------------------------
void ChLinkRevolute::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChLink::Update(time, update_assets);

    // Express the joint frames in absolute frame
    ChFrame<> frame1_abs = m_frame1 >> *m_body1;
    ChFrame<> frame2_abs = m_frame2 >> *m_body2;

    // Calculate violations and Jacobians of the spherical constraints
    //    pos2_abs - pos1_abs = 0
    m_C(0) = frame2_abs.GetPos().x() - frame1_abs.GetPos().x();
    m_C(1) = frame2_abs.GetPos().y() - frame1_abs.GetPos().y();
    m_C(2) = frame2_abs.GetPos().z() - frame1_abs.GetPos().z();

    {
        ChMatrix33<> Phi_pi1 = m_body1->GetRotMat() * ChStarMatrix33<>(m_frame1.GetPos());
        ChMatrix33<> Phi_pi2 = m_body2->GetRotMat() * ChStarMatrix33<>(m_frame2.GetPos());

        m_cnstr_x.Get_Cq_a()(0) = -1;
        m_cnstr_x.Get_Cq_b()(0) = +1;
        m_cnstr_x.Get_Cq_a()(1) = 0;
        m_cnstr_x.Get_Cq_b()(1) = 0;
        m_cnstr_x.Get_Cq_a()(2) = 0;
        m_cnstr_x.Get_Cq_b()(2) = 0;
        m_cnstr_x.Get_Cq_a()(3) = Phi_pi1(0, 0);
        m_cnstr_x.Get_Cq_b()(3) = -Phi_pi2(0, 0);
        m_cnstr_x.Get_Cq_a()(4) = Phi_pi1(0, 1);
        m_cnstr_x.Get_Cq_b()(4) = -Phi_pi2(0, 1);
        m_cnstr_x.Get_Cq_a()(5) = Phi_pi1(0, 2);
        m_cnstr_x.Get_Cq_b()(5) = -Phi_pi2(0, 2);

        m_cnstr_y.Get_Cq_a()(0) = 0;
        m_cnstr_y.Get_Cq_b()(0) = 0;
        m_cnstr_y.Get_Cq_a()(1) = -1;
        m_cnstr_y.Get_Cq_b()(1) = +1;
        m_cnstr_y.Get_Cq_a()(2) = 0;
        m_cnstr_y.Get_Cq_b()(2) = 0;
        m_cnstr_y.Get_Cq_a()(3) = Phi_pi1(1, 0);
        m_cnstr_y.Get_Cq_b()(3) = -Phi_pi2(1, 0);
        m_cnstr_y.Get_Cq_a()(4) = Phi_pi1(1, 1);
        m_cnstr_y.Get_Cq_b()(4) = -Phi_pi2(1, 1);
        m_cnstr_y.Get_Cq_a()(5) = Phi_pi1(1, 2);
        m_cnstr_y.Get_Cq_b()(5) = -Phi_pi2(1, 2);

        m_cnstr_z.Get_Cq_a()(0) = 0;
        m_cnstr_z.Get_Cq_b()(0) = 0;
        m_cnstr_z.Get_Cq_a()(1) = 0;
        m_cnstr_z.Get_Cq_b()(1) = 0;
        m_cnstr_z.Get_Cq_a()(2) = -1;
        m_cnstr_z.Get_Cq_b()(2) = +1;
        m_cnstr_z.Get_Cq_a()(3) = Phi_pi1(2, 0);
        m_cnstr_z.Get_Cq_b()(3) = -Phi_pi2(2, 0);
        m_cnstr_z.Get_Cq_a()(4) = Phi_pi1(2, 1);
        m_cnstr_z.Get_Cq_b()(4) = -Phi_pi2(2, 1);
        m_cnstr_z.Get_Cq_a()(5) = Phi_pi1(2, 2);
        m_cnstr_z.Get_Cq_b()(5) = -Phi_pi2(2, 2);
    }

    // Calculate violation and Jacobians of the dot constraint (u1,w2)
    //    dot(u1_abs, w2_abs) = 0
    ChVector3d u1 = frame1_abs.GetRotMat().GetAxisX();
    ChVector3d w2 = frame2_abs.GetRotMat().GetAxisZ();

    m_C(3) = Vdot(u1, w2);

    {
        ChMatrix33<> mat1 = m_body1->GetRotMat() * m_u1_tilde;
        ChMatrix33<> mat2 = m_body2->GetRotMat() * m_w2_tilde;
        ChVector3d Phi_pi1 = mat1.transpose() * w2;
        ChVector3d Phi_pi2 = mat2.transpose() * u1;

        m_cnstr_uw.Get_Cq_a()(0) = 0;
        m_cnstr_uw.Get_Cq_a()(1) = 0;
        m_cnstr_uw.Get_Cq_a()(2) = 0;
        m_cnstr_uw.Get_Cq_a()(3) = -Phi_pi1.x();
        m_cnstr_uw.Get_Cq_a()(4) = -Phi_pi1.y();
        m_cnstr_uw.Get_Cq_a()(5) = -Phi_pi1.z();

        m_cnstr_uw.Get_Cq_b()(0) = 0;
        m_cnstr_uw.Get_Cq_b()(1) = 0;
        m_cnstr_uw.Get_Cq_b()(2) = 0;
        m_cnstr_uw.Get_Cq_b()(3) = -Phi_pi2.x();
        m_cnstr_uw.Get_Cq_b()(4) = -Phi_pi2.y();
        m_cnstr_uw.Get_Cq_b()(5) = -Phi_pi2.z();
    }

    // Calculate violation and Jacobians of the dot constraint (v1,w2)
    //    dot(v1_abs, w2_abs) = 0
    ChVector3d v1 = frame1_abs.GetRotMat().GetAxisY();

    m_C(4) = Vdot(v1, w2);

    {
        ChMatrix33<> mat1 = m_body1->GetRotMat() * m_v1_tilde;
        ChMatrix33<> mat2 = m_body2->GetRotMat() * m_w2_tilde;
        ChVector3d Phi_pi1 = mat1.transpose() * w2;
        ChVector3d Phi_pi2 = mat2.transpose() * v1;

        m_cnstr_vw.Get_Cq_a()(0) = 0;
        m_cnstr_vw.Get_Cq_a()(1) = 0;
        m_cnstr_vw.Get_Cq_a()(2) = 0;
        m_cnstr_vw.Get_Cq_a()(3) = -Phi_pi1.x();
        m_cnstr_vw.Get_Cq_a()(4) = -Phi_pi1.y();
        m_cnstr_vw.Get_Cq_a()(5) = -Phi_pi1.z();

        m_cnstr_vw.Get_Cq_b()(0) = 0;
        m_cnstr_vw.Get_Cq_b()(1) = 0;
        m_cnstr_vw.Get_Cq_b()(2) = 0;
        m_cnstr_vw.Get_Cq_b()(3) = -Phi_pi2.x();
        m_cnstr_vw.Get_Cq_b()(4) = -Phi_pi2.y();
        m_cnstr_vw.Get_Cq_b()(5) = -Phi_pi2.z();
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkRevolute::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    L(off_L + 0) = m_multipliers[0];
    L(off_L + 1) = m_multipliers[1];
    L(off_L + 2) = m_multipliers[2];
    L(off_L + 3) = m_multipliers[3];
    L(off_L + 4) = m_multipliers[4];
}

void ChLinkRevolute::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    m_multipliers[0] = L(off_L + 0);
    m_multipliers[1] = L(off_L + 1);
    m_multipliers[2] = L(off_L + 2);
    m_multipliers[3] = L(off_L + 3);
    m_multipliers[4] = L(off_L + 4);

    // Also compute 'intuitive' reactions:
    ChVector3d lam_sph(m_multipliers[0], m_multipliers[1], m_multipliers[2]);
    double lam_uw = m_multipliers[3];
    double lam_vw = m_multipliers[4];

    // Calculate the reaction force and torque acting on the 2nd body at the joint
    // location, expressed in the joint reference frame.  Taking into account the
    // sign with which Lagrange multipliers show up in the EOM in Chrono, we get:
    //   F = C^T * A_2^T * Phi_r2^T * lam
    //   T = C^T * ( Phi_pi2^T - tilde(s2') * A_2^T * Phi_r2^T ) * lam
    // For the revolute joint, after some manipulations, we have:
    //   F = C^T * A_2^T * lam_sph
    //   T = C^T * tilde(w2') *A_2^T * (u1 * lam_uw + v1 * lam_vw)
    //     = -C^T * [A_2 * tilde(w2')]^T * (u1 * lam_uw + v1 * lam_vw)

    // Reaction force
    ChVector3d F2 = m_body2->GetRotMat().transpose() * lam_sph;
    react_force = m_frame2.GetRotMat().transpose() * F2;

    // Reaction torque
    ChVector3d u1 = m_body1->TransformDirectionLocalToParent(m_frame1.GetRotMat().GetAxisX());
    ChVector3d v1 = m_body1->TransformDirectionLocalToParent(m_frame1.GetRotMat().GetAxisY());
    ChMatrix33<> mat2 = m_body2->GetRotMat() * m_w2_tilde;
    ChVector3d T2 = mat2.transpose() * (lam_uw * u1 + lam_vw * v1);
    react_torque = -(m_frame2.GetRotMat().transpose() * T2);
}

void ChLinkRevolute::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                         ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  ///< the L vector
                                         const double c               ///< a scaling factor
) {
    if (!IsActive())
        return;

    m_cnstr_x.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    m_cnstr_y.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
    m_cnstr_z.AddJacobianTransposedTimesScalarInto(R, L(off_L + 2) * c);
    m_cnstr_uw.AddJacobianTransposedTimesScalarInto(R, L(off_L + 3) * c);
    m_cnstr_vw.AddJacobianTransposedTimesScalarInto(R, L(off_L + 4) * c);
}

void ChLinkRevolute::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                         ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                         const double c,            ///< a scaling factor
                                         bool do_clamp,             ///< apply clamping to c*C?
                                         double recovery_clamp      ///< value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    double cnstr_x_violation = c * m_C(0);
    double cnstr_y_violation = c * m_C(1);
    double cnstr_z_violation = c * m_C(2);
    double cnstr_uw_violation = c * m_C(3);
    double cnstr_vw_violation = c * m_C(4);

    if (do_clamp) {
        cnstr_x_violation = std::min(std::max(cnstr_x_violation, -recovery_clamp), recovery_clamp);
        cnstr_y_violation = std::min(std::max(cnstr_y_violation, -recovery_clamp), recovery_clamp);
        cnstr_z_violation = std::min(std::max(cnstr_z_violation, -recovery_clamp), recovery_clamp);
        cnstr_uw_violation = std::min(std::max(cnstr_uw_violation, -recovery_clamp), recovery_clamp);
        cnstr_vw_violation = std::min(std::max(cnstr_vw_violation, -recovery_clamp), recovery_clamp);
    }
    Qc(off_L + 0) += cnstr_x_violation;
    Qc(off_L + 1) += cnstr_y_violation;
    Qc(off_L + 2) += cnstr_z_violation;
    Qc(off_L + 3) += cnstr_uw_violation;
    Qc(off_L + 4) += cnstr_vw_violation;
}

void ChLinkRevolute::IntToDescriptor(const unsigned int off_v,
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_cnstr_x.SetLagrangeMultiplier(L(off_L + 0));
    m_cnstr_y.SetLagrangeMultiplier(L(off_L + 1));
    m_cnstr_z.SetLagrangeMultiplier(L(off_L + 2));
    m_cnstr_uw.SetLagrangeMultiplier(L(off_L + 3));
    m_cnstr_vw.SetLagrangeMultiplier(L(off_L + 4));

    m_cnstr_x.SetRightHandSide(Qc(off_L + 0));
    m_cnstr_y.SetRightHandSide(Qc(off_L + 1));
    m_cnstr_z.SetRightHandSide(Qc(off_L + 2));
    m_cnstr_uw.SetRightHandSide(Qc(off_L + 3));
    m_cnstr_vw.SetRightHandSide(Qc(off_L + 4));
}

void ChLinkRevolute::IntFromDescriptor(const unsigned int off_v,
                                       ChStateDelta& v,
                                       const unsigned int off_L,
                                       ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_cnstr_x.GetLagrangeMultiplier();
    L(off_L + 1) = m_cnstr_y.GetLagrangeMultiplier();
    L(off_L + 2) = m_cnstr_z.GetLagrangeMultiplier();
    L(off_L + 3) = m_cnstr_uw.GetLagrangeMultiplier();
    L(off_L + 4) = m_cnstr_vw.GetLagrangeMultiplier();
}

// -----------------------------------------------------------------------------
// Implementation of solver interface functions
// -----------------------------------------------------------------------------
void ChLinkRevolute::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_cnstr_x);
    descriptor.InsertConstraint(&m_cnstr_y);
    descriptor.InsertConstraint(&m_cnstr_z);
    descriptor.InsertConstraint(&m_cnstr_uw);
    descriptor.InsertConstraint(&m_cnstr_vw);
}

void ChLinkRevolute::ConstraintsBiReset() {
    m_cnstr_x.SetRightHandSide(0.0);
    m_cnstr_y.SetRightHandSide(0.0);
    m_cnstr_z.SetRightHandSide(0.0);
    m_cnstr_uw.SetRightHandSide(0.0);
    m_cnstr_vw.SetRightHandSide(0.0);
}

void ChLinkRevolute::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
        return;

    double cnstr_x_violation = factor * m_C(0);
    double cnstr_y_violation = factor * m_C(1);
    double cnstr_z_violation = factor * m_C(2);
    double cnstr_uw_violation = factor * m_C(3);
    double cnstr_vw_violation = factor * m_C(4);

    if (do_clamp) {
        cnstr_x_violation = std::min(std::max(cnstr_x_violation, -recovery_clamp), recovery_clamp);
        cnstr_y_violation = std::min(std::max(cnstr_y_violation, -recovery_clamp), recovery_clamp);
        cnstr_z_violation = std::min(std::max(cnstr_z_violation, -recovery_clamp), recovery_clamp);
        cnstr_uw_violation = std::min(std::max(cnstr_uw_violation, -recovery_clamp), recovery_clamp);
        cnstr_vw_violation = std::min(std::max(cnstr_vw_violation, -recovery_clamp), recovery_clamp);
    }

    m_cnstr_x.SetRightHandSide(m_cnstr_x.GetRightHandSide() + cnstr_x_violation);
    m_cnstr_y.SetRightHandSide(m_cnstr_y.GetRightHandSide() + cnstr_y_violation);
    m_cnstr_z.SetRightHandSide(m_cnstr_z.GetRightHandSide() + cnstr_z_violation);
    m_cnstr_uw.SetRightHandSide(m_cnstr_uw.GetRightHandSide() + cnstr_uw_violation);
    m_cnstr_vw.SetRightHandSide(m_cnstr_vw.GetRightHandSide() + cnstr_vw_violation);
}

void ChLinkRevolute::LoadConstraintJacobians() {
    // Nothing to do here. Jacobians were loaded in Update().
}

void ChLinkRevolute::ConstraintsFetch_react(double factor) {
    // Extract the Lagrange multipliers for the 3 spherical constraints and for
    // the 2 dot constraint.
    ChVector3d lam_sph(m_cnstr_x.GetLagrangeMultiplier(), m_cnstr_y.GetLagrangeMultiplier(), m_cnstr_z.GetLagrangeMultiplier());
    double lam_uw = m_cnstr_uw.GetLagrangeMultiplier();
    double lam_vw = m_cnstr_vw.GetLagrangeMultiplier();

    // Note that the Lagrange multipliers must be multiplied by 'factor' to
    // convert from reaction impulses to reaction forces.
    lam_sph *= factor;
    lam_uw *= factor;
    lam_vw *= factor;

    // Calculate the reaction force and torque acting on the 2nd body at the joint
    // location, expressed in the joint reference frame.  Taking into account the
    // sign with which Lagrange multipliers show up in the EOM in Chrono, we get:
    //   F = C^T * A_2^T * Phi_r2^T * lam
    //   T = C^T * ( Phi_pi2^T - tilde(s2') * A_2^T * Phi_r2^T ) * lam
    // For the revolute joint, after some manipulations, we have:
    //   F = C^T * A_2^T * lam_sph
    //   T = C^T * tilde(w2') *A_2^T * (u1 * lam_uw + v1 * lam_vw)
    //     = -C^T * [A_2 * tilde(w2')]^T * (u1 * lam_uw + v1 * lam_vw)

    // Reaction force
    ChVector3d F2 = m_body2->GetRotMat().transpose() * lam_sph;
    react_force = m_frame2.GetRotMat().transpose() * F2;

    // Reaction torque
    ChVector3d u1 = m_body1->TransformDirectionLocalToParent(m_frame1.GetRotMat().GetAxisX());
    ChVector3d v1 = m_body1->TransformDirectionLocalToParent(m_frame1.GetRotMat().GetAxisY());
    ChMatrix33<> mat2 = m_body2->GetRotMat() * m_w2_tilde;
    ChVector3d T2 = mat2.transpose() * (lam_uw * u1 + lam_vw * v1);
    react_torque = -(m_frame2.GetRotMat().transpose() * T2);
}

void ChLinkRevolute::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkRevolute>();

    // serialize parent class
    ChLink::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_frame1);
    archive_out << CHNVP(m_frame2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRevolute::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkRevolute>();

    // deserialize parent class
    ChLink::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_frame1);
    archive_in >> CHNVP(m_frame2);
}

}  // end namespace chrono
