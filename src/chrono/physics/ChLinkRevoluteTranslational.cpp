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

#include "chrono/physics/ChLinkRevoluteTranslational.h"

namespace chrono {

// Register into the object factory.
CH_FACTORY_REGISTER(ChLinkRevoluteTranslational)

// -----------------------------------------------------------------------------
// Constructor and destructor
// -----------------------------------------------------------------------------
ChLinkRevoluteTranslational::ChLinkRevoluteTranslational()
    : m_p1(ChVector<>(0, 0, 0)),
      m_p2(ChVector<>(0, 0, 0)),
      m_z1(ChVector<>(0, 0, 1)),
      m_x2(ChVector<>(1, 0, 0)),
      m_y2(ChVector<>(0, 1, 0)),
      m_dist(0),
      m_cur_par1(0),
      m_cur_par2(0),
      m_cur_dot(0),
      m_cur_dist(0) {
    m_C = new ChMatrixDynamic<>(4, 1);

    for (int i = 0; i < 4; i++) {
        m_multipliers[i] = 0;
    }
}

ChLinkRevoluteTranslational::ChLinkRevoluteTranslational(const ChLinkRevoluteTranslational& other) : ChLink(other) {
    Body1 = other.Body1;
    Body2 = other.Body2;
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

    m_cnstr_par1.SetVariables(&other.Body1->Variables(), &other.Body2->Variables());
    m_cnstr_par2.SetVariables(&other.Body1->Variables(), &other.Body2->Variables());
    m_cnstr_dot.SetVariables(&other.Body1->Variables(), &other.Body2->Variables());
    m_cnstr_dist.SetVariables(&other.Body1->Variables(), &other.Body2->Variables());

    for (int i = 0; i < 4; i++) {
        m_multipliers[i] = other.m_multipliers[i];
    }
}

ChLinkRevoluteTranslational::~ChLinkRevoluteTranslational() {
    delete m_C;
}

// -----------------------------------------------------------------------------
// Link initialization functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteTranslational::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                             std::shared_ptr<ChBodyFrame> body2,
                                             const ChCoordsys<>& csys,
                                             double distance) {
    Body1 = body1.get();
    Body2 = body2.get();

    m_cnstr_par1.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_par2.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_dist.SetVariables(&Body1->Variables(), &Body2->Variables());

    ChVector<> x_axis = csys.rot.GetXaxis();
    ChVector<> y_axis = csys.rot.GetYaxis();
    ChVector<> z_axis = csys.rot.GetZaxis();

    m_p1 = Body1->TransformPointParentToLocal(csys.pos);
    m_z1 = Body1->TransformDirectionParentToLocal(z_axis);
    m_p2 = Body2->TransformPointParentToLocal(csys.pos + distance * x_axis);
    m_x2 = Body2->TransformDirectionParentToLocal(x_axis);
    m_y2 = Body2->TransformDirectionParentToLocal(y_axis);

    m_dist = distance;

    m_cur_par1 = 0;
    m_cur_par2 = 0;
    m_cur_dot = 0;
    m_cur_dist = distance;
}

void ChLinkRevoluteTranslational::Initialize(std::shared_ptr<ChBodyFrame> body1,
                                             std::shared_ptr<ChBodyFrame> body2,
                                             bool local,
                                             const ChVector<>& p1,
                                             const ChVector<>& dirZ1,
                                             const ChVector<>& p2,
                                             const ChVector<>& dirX2,
                                             const ChVector<>& dirY2,
                                             bool auto_distance,
                                             double distance) {
    Body1 = body1.get();
    Body2 = body2.get();

    m_cnstr_par1.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_par2.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_dist.SetVariables(&Body1->Variables(), &Body2->Variables());

    ChVector<> p1_abs;
    ChVector<> p2_abs;
    ChVector<> z1_abs;
    ChVector<> x2_abs;
    ChVector<> y2_abs;

    if (local) {
        m_p1 = p1;
        m_p2 = p2;
        m_z1 = Vnorm(dirZ1);
        m_x2 = Vnorm(dirX2);
        m_y2 = Vnorm(dirY2);
        p1_abs = Body1->TransformPointLocalToParent(m_p1);
        p2_abs = Body2->TransformPointLocalToParent(m_p2);
        z1_abs = Body1->TransformDirectionLocalToParent(m_z1);
        x2_abs = Body2->TransformDirectionLocalToParent(m_x2);
        y2_abs = Body2->TransformDirectionLocalToParent(m_y2);
    } else {
        p1_abs = p1;
        p2_abs = p2;
        z1_abs = Vnorm(dirZ1);
        x2_abs = Vnorm(dirX2);
        y2_abs = Vnorm(dirY2);
        m_p1 = Body1->TransformPointParentToLocal(p1_abs);
        m_p2 = Body2->TransformPointParentToLocal(p2_abs);
        m_z1 = Body1->TransformDirectionParentToLocal(z1_abs);
        m_x2 = Body2->TransformDirectionParentToLocal(x2_abs);
        m_y2 = Body2->TransformDirectionParentToLocal(y2_abs);
    }

    ChVector<> d12_abs = p2_abs - p1_abs;

    m_cur_par1 = Vdot(z1_abs, x2_abs);
    m_cur_par2 = Vdot(z1_abs, y2_abs);
    m_cur_dot = Vdot(d12_abs, z1_abs);
    m_cur_dist = Vdot(d12_abs, x2_abs);

    m_dist = auto_distance ? m_cur_dist : distance;
}

// -----------------------------------------------------------------------------
// Form and return the joint reference frame (expressed in frame of Body 2)
// -----------------------------------------------------------------------------
ChCoordsys<> ChLinkRevoluteTranslational::GetLinkRelativeCoords() {
    // Origin at P1 (center of revolute side) and orientation formed using
    // the mutually orthogonal direction x2 and y2.
    ChVector<> p = Body2->TransformPointParentToLocal(Body1->TransformPointLocalToParent(m_p1));
    ChMatrix33<> A;
    A.Set_A_axis(m_x2, m_y2, Vcross(m_x2, m_y2));

    return ChCoordsys<>(p, A.Get_A_quaternion());
}

// -----------------------------------------------------------------------------
// Link update function
// -----------------------------------------------------------------------------
void ChLinkRevoluteTranslational::Update(double time, bool update_assets) {
    // Inherit time changes of parent class (ChLink)
    ChLink::UpdateTime(time);

    // Express the body locations and direction in absolute frame
    ChVector<> p1_abs = Body1->TransformPointLocalToParent(m_p1);
    ChVector<> p2_abs = Body2->TransformPointLocalToParent(m_p2);
    ChVector<> z1_abs = Body1->TransformDirectionLocalToParent(m_z1);
    ChVector<> x2_abs = Body2->TransformDirectionLocalToParent(m_x2);
    ChVector<> y2_abs = Body2->TransformDirectionLocalToParent(m_y2);
    ChVector<> d12_abs = p2_abs - p1_abs;

    // Update current constraint quantities
    m_cur_par1 = Vdot(z1_abs, x2_abs);
    m_cur_par2 = Vdot(z1_abs, y2_abs);
    m_cur_dot = Vdot(d12_abs, z1_abs);
    m_cur_dist = Vdot(d12_abs, x2_abs);

    // Calculate a few more quantities
    // (express directions on one body in the frame ot the other body)
    ChVector<> z1_2 = Body2->TransformDirectionParentToLocal(z1_abs);
    ChVector<> x2_1 = Body1->TransformDirectionParentToLocal(x2_abs);
    ChVector<> y2_1 = Body1->TransformDirectionParentToLocal(y2_abs);

    ChVector<> d12_1 = Body1->TransformDirectionParentToLocal(d12_abs);
    ChVector<> d12_2 = Body2->TransformDirectionParentToLocal(d12_abs);

    // First constraint (par1)
    {
        // Cache constraint violation
        m_C->SetElement(0, 0, m_cur_par1);

        // Set Jacobian w.r.t. states of Body 1
        ChVector<> Phi_pi1 = Vcross(m_z1, x2_1);

        m_cnstr_par1.Get_Cq_a()->ElementN(0) = 0;
        m_cnstr_par1.Get_Cq_a()->ElementN(1) = 0;
        m_cnstr_par1.Get_Cq_a()->ElementN(2) = 0;

        m_cnstr_par1.Get_Cq_a()->ElementN(3) = Phi_pi1.x();
        m_cnstr_par1.Get_Cq_a()->ElementN(4) = Phi_pi1.y();
        m_cnstr_par1.Get_Cq_a()->ElementN(5) = Phi_pi1.z();

        // Set Jacobian w.r.t. states of Body 2
        ChVector<> Phi_pi2 = Vcross(m_x2, z1_2);

        m_cnstr_par1.Get_Cq_b()->ElementN(0) = 0;
        m_cnstr_par1.Get_Cq_b()->ElementN(1) = 0;
        m_cnstr_par1.Get_Cq_b()->ElementN(2) = 0;

        m_cnstr_par1.Get_Cq_b()->ElementN(3) = Phi_pi2.x();
        m_cnstr_par1.Get_Cq_b()->ElementN(4) = Phi_pi2.y();
        m_cnstr_par1.Get_Cq_b()->ElementN(5) = Phi_pi2.z();
    }

    // Second constraint (par2)
    {
        // Cache constraint violation
        m_C->SetElement(1, 0, m_cur_par2);

        // Set Jacobian w.r.t. states of Body 1
        ChVector<> Phi_pi1 = Vcross(m_z1, y2_1);

        m_cnstr_par2.Get_Cq_a()->ElementN(0) = 0;
        m_cnstr_par2.Get_Cq_a()->ElementN(1) = 0;
        m_cnstr_par2.Get_Cq_a()->ElementN(2) = 0;

        m_cnstr_par2.Get_Cq_a()->ElementN(3) = Phi_pi1.x();
        m_cnstr_par2.Get_Cq_a()->ElementN(4) = Phi_pi1.y();
        m_cnstr_par2.Get_Cq_a()->ElementN(5) = Phi_pi1.z();

        // Set Jacobian w.r.t. states of Body 2
        ChVector<> Phi_pi2 = Vcross(m_y2, z1_2);

        m_cnstr_par2.Get_Cq_b()->ElementN(0) = 0;
        m_cnstr_par2.Get_Cq_b()->ElementN(1) = 0;
        m_cnstr_par2.Get_Cq_b()->ElementN(2) = 0;

        m_cnstr_par2.Get_Cq_b()->ElementN(3) = Phi_pi2.x();
        m_cnstr_par2.Get_Cq_b()->ElementN(4) = Phi_pi2.y();
        m_cnstr_par2.Get_Cq_b()->ElementN(5) = Phi_pi2.z();
    }

    // Third constraint (dot)
    {
        // Cache constraint violation
        m_C->SetElement(2, 0, m_cur_dot);

        // Set Jacobian w.r.t. states of Body 1
        ChVector<> Phi_pi1 = Vcross(m_z1, d12_1) - Vcross(m_p1, m_z1);

        m_cnstr_dot.Get_Cq_a()->ElementN(0) = -z1_abs.x();
        m_cnstr_dot.Get_Cq_a()->ElementN(1) = -z1_abs.y();
        m_cnstr_dot.Get_Cq_a()->ElementN(2) = -z1_abs.z();

        m_cnstr_dot.Get_Cq_a()->ElementN(3) = Phi_pi1.x();
        m_cnstr_dot.Get_Cq_a()->ElementN(4) = Phi_pi1.y();
        m_cnstr_dot.Get_Cq_a()->ElementN(5) = Phi_pi1.z();

        // Set Jacobian w.r.t. states of Body 2
        ChVector<> Phi_pi2 = Vcross(m_p2, z1_2);

        m_cnstr_dot.Get_Cq_b()->ElementN(0) = z1_abs.x();
        m_cnstr_dot.Get_Cq_b()->ElementN(1) = z1_abs.y();
        m_cnstr_dot.Get_Cq_b()->ElementN(2) = z1_abs.z();

        m_cnstr_dot.Get_Cq_b()->ElementN(3) = Phi_pi2.x();
        m_cnstr_dot.Get_Cq_b()->ElementN(4) = Phi_pi2.y();
        m_cnstr_dot.Get_Cq_b()->ElementN(5) = Phi_pi2.z();
    }

    // Fourth constraint (dist)
    {
        // Cache constraint violation
        m_C->SetElement(3, 0, m_cur_dist - m_dist);

        // Set Jacobian w.r.t. states of Body 1
        ChVector<> Phi_pi1 = -Vcross(m_p1, x2_1);

        m_cnstr_dist.Get_Cq_a()->ElementN(0) = -x2_abs.x();
        m_cnstr_dist.Get_Cq_a()->ElementN(1) = -x2_abs.y();
        m_cnstr_dist.Get_Cq_a()->ElementN(2) = -x2_abs.z();

        m_cnstr_dist.Get_Cq_a()->ElementN(3) = Phi_pi1.x();
        m_cnstr_dist.Get_Cq_a()->ElementN(4) = Phi_pi1.y();
        m_cnstr_dist.Get_Cq_a()->ElementN(5) = Phi_pi1.z();

        // Set Jacobian w.r.t. states of Body 2
        ChVector<> Phi_pi2 = Vcross(m_x2, d12_2) + Vcross(m_p2, m_x2);

        m_cnstr_dist.Get_Cq_b()->ElementN(0) = x2_abs.x();
        m_cnstr_dist.Get_Cq_b()->ElementN(1) = x2_abs.y();
        m_cnstr_dist.Get_Cq_b()->ElementN(2) = x2_abs.z();

        m_cnstr_dist.Get_Cq_b()->ElementN(3) = Phi_pi2.x();
        m_cnstr_dist.Get_Cq_b()->ElementN(4) = Phi_pi2.y();
        m_cnstr_dist.Get_Cq_b()->ElementN(5) = Phi_pi2.z();
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
    double lam_par1 = m_multipliers[0];
    double lam_par2 = m_multipliers[1];
    double lam_dot = m_multipliers[2];
    double lam_dist = m_multipliers[3];

    ////
    //// TODO
    ////

    // Calculate the reaction torques and forces on Body 2 in the joint frame
    // (Note: origin of the joint frame is at the center of the revolute joint
    //  which is defined on body 1, the x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    react_force.x() = 0;
    react_force.y() = 0;
    react_force.z() = 0;

    react_torque.x() = 0;
    react_torque.y() = 0;
    react_torque.z() = 0;
}

void ChLinkRevoluteTranslational::IntLoadResidual_CqL(const unsigned int off_L,  ///< offset in L multipliers
                                                      ChVectorDynamic<>& R,  ///< result: the R residual, R += c*Cq'*L
                                                      const ChVectorDynamic<>& L,  ///< the L vector
                                                      const double c               ///< a scaling factor
                                                      ) {
    m_cnstr_par1.MultiplyTandAdd(R, L(off_L + 0) * c);
    m_cnstr_par2.MultiplyTandAdd(R, L(off_L + 1) * c);
    m_cnstr_dot.MultiplyTandAdd(R, L(off_L + 2) * c);
    m_cnstr_dist.MultiplyTandAdd(R, L(off_L + 3) * c);
}

void ChLinkRevoluteTranslational::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                                      ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                                      const double c,            ///< a scaling factor
                                                      bool do_clamp,             ///< apply clamping to c*C?
                                                      double recovery_clamp      ///< value for min/max clamping of c*C
                                                      ) {
    if (!IsActive())
        return;

    double cnstr_par1_violation =
        do_clamp ? ChMin(ChMax(c * m_cur_par1, -recovery_clamp), recovery_clamp) : c * m_cur_par1;
    double cnstr_par2_violation =
        do_clamp ? ChMin(ChMax(c * m_cur_par2, -recovery_clamp), recovery_clamp) : c * m_cur_par2;
    double cnstr_dot_violation =
        do_clamp ? ChMin(ChMax(c * m_cur_dot, -recovery_clamp), recovery_clamp) : c * m_cur_dot;
    double cnstr_dist_violation =
        do_clamp ? ChMin(ChMax(c * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp) : c * (m_cur_dist - m_dist);

    Qc(off_L + 0) += cnstr_par1_violation;
    Qc(off_L + 1) += cnstr_par2_violation;
    Qc(off_L + 2) += cnstr_dot_violation;
    Qc(off_L + 3) += cnstr_dist_violation;
}

void ChLinkRevoluteTranslational::IntToDescriptor(const unsigned int off_v,  ///< offset in v, R
                                                  const ChStateDelta& v,
                                                  const ChVectorDynamic<>& R,
                                                  const unsigned int off_L,  ///< offset in L, Qc
                                                  const ChVectorDynamic<>& L,
                                                  const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_cnstr_par1.Set_l_i(L(off_L + 0));
    m_cnstr_par2.Set_l_i(L(off_L + 1));
    m_cnstr_dot.Set_l_i(L(off_L + 2));
    m_cnstr_dist.Set_l_i(L(off_L + 3));

    m_cnstr_par1.Set_b_i(Qc(off_L + 0));
    m_cnstr_par2.Set_b_i(Qc(off_L + 1));
    m_cnstr_dot.Set_b_i(Qc(off_L + 2));
    m_cnstr_dist.Set_b_i(Qc(off_L + 3));
}

void ChLinkRevoluteTranslational::IntFromDescriptor(const unsigned int off_v,  ///< offset in v
                                                    ChStateDelta& v,
                                                    const unsigned int off_L,  ///< offset in L
                                                    ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_cnstr_par1.Get_l_i();
    L(off_L + 1) = m_cnstr_par2.Get_l_i();
    L(off_L + 2) = m_cnstr_dot.Get_l_i();
    L(off_L + 3) = m_cnstr_dist.Get_l_i();
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
    m_cnstr_par1.Set_b_i(0.0);
    m_cnstr_par2.Set_b_i(0.0);
    m_cnstr_dot.Set_b_i(0.0);
    m_cnstr_dist.Set_b_i(0.0);
}

void ChLinkRevoluteTranslational::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
        return;

    double cnstr_par1_violation =
        do_clamp ? ChMin(ChMax(factor * m_cur_par1, -recovery_clamp), recovery_clamp) : factor * m_cur_par1;
    double cnstr_par2_violation =
        do_clamp ? ChMin(ChMax(factor * m_cur_par2, -recovery_clamp), recovery_clamp) : factor * m_cur_par2;
    double cnstr_dot_violation =
        do_clamp ? ChMin(ChMax(factor * m_cur_dot, -recovery_clamp), recovery_clamp) : factor * m_cur_dot;
    double cnstr_dist_violation = do_clamp
                                      ? ChMin(ChMax(factor * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp)
                                      : factor * (m_cur_dist - m_dist);

    m_cnstr_par1.Set_b_i(m_cnstr_par1.Get_b_i() + cnstr_par1_violation);
    m_cnstr_par2.Set_b_i(m_cnstr_par2.Get_b_i() + cnstr_par2_violation);
    m_cnstr_dot.Set_b_i(m_cnstr_dot.Get_b_i() + cnstr_dot_violation);
    m_cnstr_dist.Set_b_i(m_cnstr_dist.Get_b_i() + cnstr_dist_violation);
}

void ChLinkRevoluteTranslational::ConstraintsLoadJacobians() {
    // Nothing to do here. Jacobians were loaded in Update().
}

void ChLinkRevoluteTranslational::ConstraintsFetch_react(double factor) {
    // Extract the Lagrange multipliers for the four constraints
    double lam_par1 = m_cnstr_par1.Get_l_i();
    double lam_par2 = m_cnstr_par2.Get_l_i();
    double lam_dot = m_cnstr_dot.Get_l_i();
    double lam_dist = m_cnstr_dist.Get_l_i();

    // Note that the Lagrange multipliers must be multiplied by 'factor' to
    // convert from reaction impulses to reaction forces.
    lam_par1 *= factor;
    lam_par2 *= factor;
    lam_dot *= factor;
    lam_dist *= factor;

    ////
    ////  TODO
    ////

    // Calculate the reaction torques and forces on Body 2 in the joint frame
    // (Note: origin of the joint frame is at the center of the revolute joint
    //  which is defined on body 1, the x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    react_force.x() = 0;
    react_force.y() = 0;
    react_force.z() = 0;

    react_torque.x() = 0;
    react_torque.y() = 0;
    react_torque.z() = 0;
}

// -----------------------------------------------------------------------------
// Additional reaction force and torque calculations due to the odd definition
//  of the standard output for this joint style
// -----------------------------------------------------------------------------

ChVector<> ChLinkRevoluteTranslational::Get_react_force_body1() {
    ////
    ////  TODO
    ////

    // Calculate the reaction forces on Body 1 in the joint frame
    // (Note: origin of the joint frame is at the center of the revolute joint
    //  which is defined on body 1, the x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    //  react_force = (-lam_dist,0,-lam_dot)

    return -react_force;
}

ChVector<> ChLinkRevoluteTranslational::Get_react_torque_body1() {
    ////
    ////  TODO
    ////

    // Calculate the reaction forces on Body 1 in the joint frame
    // (Note: origin of the joint frame is at the center of the revolute joint
    //  which is defined on body 1, the x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    //  react_torque = (0,m_cur_dist*lam_dot,0)

    return -react_torque;
}

ChVector<> ChLinkRevoluteTranslational::Get_react_force_body2() {
    ////
    ////  TODO
    ////

    // Calculate the reaction torques on Body 2 in the joint frame at the spherical joint
    // (Note: the joint frame x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    //  react_force = (lam_dist,0,lam_dot)
    return react_force;
}

ChVector<> ChLinkRevoluteTranslational::Get_react_torque_body2() {
    ////
    ////  TODO
    ////

    // Calculate the reaction torques on Body 2 in the joint frame at the spherical joint
    // (Note: the joint frame x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    //  react_torque = (0,0,0)
    return VNULL;
}

void ChLinkRevoluteTranslational::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkRevoluteTranslational>();

    // serialize parent class
    ChLink::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(m_p1);
    marchive << CHNVP(m_p2);
    marchive << CHNVP(m_z1);
    marchive << CHNVP(m_x2);
    marchive << CHNVP(m_y2);
    marchive << CHNVP(m_dist);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRevoluteTranslational::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkRevoluteTranslational>();

    // deserialize parent class
    ChLink::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(m_p1);
    marchive >> CHNVP(m_p2);
    marchive >> CHNVP(m_z1);
    marchive >> CHNVP(m_x2);
    marchive >> CHNVP(m_y2);
    marchive >> CHNVP(m_dist);
}

}  // end namespace chrono
