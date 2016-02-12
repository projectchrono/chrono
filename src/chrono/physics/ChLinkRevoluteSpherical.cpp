//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "physics/ChLinkRevoluteSpherical.h"

namespace chrono {

// Register into the object factory.
ChClassRegister<ChLinkRevoluteSpherical> a_registration_ChLinkRevoluteSpherical;

// -----------------------------------------------------------------------------
// Constructor and destructor
// -----------------------------------------------------------------------------
ChLinkRevoluteSpherical::ChLinkRevoluteSpherical()
    : m_pos1(ChVector<>(0, 0, 0)),
      m_pos2(ChVector<>(0, 0, 0)),
      m_dir1(ChVector<>(0, 0, 1)),
      m_dist(0),
      m_cur_dist(0),
      m_cur_dot(0) {
    m_C = new ChMatrixDynamic<>(2, 1);

    m_cache_speed[0] = 0;
    m_cache_speed[1] = 0;

    m_cache_pos[0] = 0;
    m_cache_pos[1] = 0;
}

ChLinkRevoluteSpherical::~ChLinkRevoluteSpherical() {
    delete m_C;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::Copy(ChLinkRevoluteSpherical* source) {
    // first copy the parent class data...
    ChLink::Copy(source);

    Body1 = source->Body1;
    Body2 = source->Body2;
    system = source->system;

    m_pos1 = source->m_pos1;
    m_pos2 = source->m_pos2;
    m_dir1 = source->m_dir1;
    m_dist = source->m_dist;
    m_cur_dist = source->m_cur_dist;
    m_cur_dot = source->m_cur_dot;

    m_cnstr_dist.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

    m_cache_speed[0] = source->m_cache_speed[0];
    m_cache_speed[1] = source->m_cache_speed[1];

    m_cache_pos[0] = source->m_cache_pos[0];
    m_cache_pos[1] = source->m_cache_pos[1];
}

ChLink* ChLinkRevoluteSpherical::new_Duplicate() {
    ChLinkRevoluteSpherical* link = new ChLinkRevoluteSpherical;
    link->Copy(this);
    return (link);
}

// -----------------------------------------------------------------------------
// Link initialization functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::Initialize(std::shared_ptr<ChBodyFrame> body1,  // first frame (revolute side)
                                         std::shared_ptr<ChBodyFrame> body2,  // second frame (spherical side)
                                         const ChCoordsys<>& csys,        // joint coordinate system (in absolute frame)
                                         double distance)                 // imposed distance
{
    Body1 = body1.get();
    Body2 = body2.get();

    m_cnstr_dist.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

    ChVector<> x_Axis = csys.rot.GetXaxis();
    ChVector<> z_axis = csys.rot.GetZaxis();

    m_pos1 = Body1->TransformPointParentToLocal(csys.pos);
    m_dir1 = Body1->TransformDirectionParentToLocal(z_axis);
    m_pos2 = Body2->TransformPointParentToLocal(csys.pos + distance * x_Axis);

    m_dist = distance;
    m_cur_dist = distance;
    m_cur_dot = 0;
}

void ChLinkRevoluteSpherical::Initialize(std::shared_ptr<ChBodyFrame> body1,  // first frame (revolute side)
                                         std::shared_ptr<ChBodyFrame> body2,  // second frame (spherical side)
                                         bool local,                      // true if data given in body local frames
                                         const ChVector<>& pos1,          // point on first frame
                                         const ChVector<>& dir1,          // direction of revolute on first frame
                                         const ChVector<>& pos2,          // point on second frame
                                         bool auto_distance,  // true if imposed distance equal to |pos1 - po2|
                                         double distance)     // imposed distance (used only if auto_distance = false)
{
    Body1 = body1.get();
    Body2 = body2.get();

    m_cnstr_dist.SetVariables(&Body1->Variables(), &Body2->Variables());
    m_cnstr_dot.SetVariables(&Body1->Variables(), &Body2->Variables());

    ChVector<> pos1_abs;
    ChVector<> pos2_abs;
    ChVector<> dir1_abs;

    if (local) {
        m_pos1 = pos1;
        m_pos2 = pos2;
        m_dir1 = Vnorm(dir1);
        pos1_abs = Body1->TransformPointLocalToParent(m_pos1);
        pos2_abs = Body2->TransformPointLocalToParent(m_pos2);
        dir1_abs = Body1->TransformDirectionLocalToParent(m_dir1);
    } else {
        pos1_abs = pos1;
        pos2_abs = pos2;
        dir1_abs = Vnorm(dir1);
        m_pos1 = Body1->TransformPointParentToLocal(pos1_abs);
        m_pos2 = Body2->TransformPointParentToLocal(pos2_abs);
        m_dir1 = Body1->TransformDirectionParentToLocal(dir1_abs);
    }

    ChVector<> d12_abs = pos2_abs - pos1_abs;

    m_cur_dist = d12_abs.Length();
    m_dist = auto_distance ? m_cur_dist : distance;

    m_cur_dot = Vdot(d12_abs, dir1_abs);
}

// -----------------------------------------------------------------------------
// Form and return the joint reference frame.
// -----------------------------------------------------------------------------
ChCoordsys<> ChLinkRevoluteSpherical::GetLinkRelativeCoords() {
    ChVector<> pos1 = Body2->TransformPointParentToLocal(Body1->TransformPointLocalToParent(m_pos1));
    ChMatrix33<> A;

    ChVector<> u = (m_pos2 - pos1).GetNormalized();
    ChVector<> w = Body2->TransformDirectionParentToLocal(Body1->TransformDirectionLocalToParent(m_dir1));
    ChVector<> v = Vcross(w, u);
    A.Set_A_axis(u, v, w);

    return ChCoordsys<>(pos1, A.Get_A_quaternion());
}

// -----------------------------------------------------------------------------
// Link update function
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::Update(double time, bool update_assets) {
    // Inherit time changes of parent class (ChLink)
    ChLink::UpdateTime(time);

    // Express the body locations and direction in absolute frame
    ChVector<> pos1_abs = Body1->TransformPointLocalToParent(m_pos1);
    ChVector<> pos2_abs = Body2->TransformPointLocalToParent(m_pos2);
    ChVector<> dir1_abs = Body1->TransformDirectionLocalToParent(m_dir1);
    ChVector<> d12_abs = pos2_abs - pos1_abs;

    // Update current distance and dot product
    m_cur_dist = d12_abs.Length();
    m_cur_dot = Vdot(d12_abs, dir1_abs);

    // Calculate a unit vector in the direction d12, expressed in absolute frame
    // Then express it in the two body frames
    ChVector<> u12_abs = d12_abs / m_cur_dist;
    ChVector<> u12_loc1 = Body1->TransformDirectionParentToLocal(u12_abs);
    ChVector<> u12_loc2 = Body2->TransformDirectionParentToLocal(u12_abs);

    // Express the direction vector in the frame of body 2
    ChVector<> dir1_loc2 = Body2->TransformDirectionParentToLocal(dir1_abs);

    // Cache violation of the distance constraint
    m_C->SetElement(0, 0, m_cur_dist - m_dist);

    // Compute Jacobian of the distance constraint
    //    ||pos2_abs - pos1_abs|| - dist = 0
    {
        ChVector<> Phi_r1 = -u12_abs;
        ChVector<> Phi_pi1 = Vcross(u12_loc1, m_pos1);

        m_cnstr_dist.Get_Cq_a()->ElementN(0) = (float)Phi_r1.x;
        m_cnstr_dist.Get_Cq_a()->ElementN(1) = (float)Phi_r1.y;
        m_cnstr_dist.Get_Cq_a()->ElementN(2) = (float)Phi_r1.z;

        m_cnstr_dist.Get_Cq_a()->ElementN(3) = (float)Phi_pi1.x;
        m_cnstr_dist.Get_Cq_a()->ElementN(4) = (float)Phi_pi1.y;
        m_cnstr_dist.Get_Cq_a()->ElementN(5) = (float)Phi_pi1.z;

        ChVector<> Phi_r2 = u12_abs;
        ChVector<> Phi_pi2 = -Vcross(u12_loc2, m_pos2);

        m_cnstr_dist.Get_Cq_b()->ElementN(0) = (float)Phi_r2.x;
        m_cnstr_dist.Get_Cq_b()->ElementN(1) = (float)Phi_r2.y;
        m_cnstr_dist.Get_Cq_b()->ElementN(2) = (float)Phi_r2.z;

        m_cnstr_dist.Get_Cq_b()->ElementN(3) = (float)Phi_pi2.x;
        m_cnstr_dist.Get_Cq_b()->ElementN(4) = (float)Phi_pi2.y;
        m_cnstr_dist.Get_Cq_b()->ElementN(5) = (float)Phi_pi2.z;
    }

    // Cache violation of the dot constraint
    m_C->SetElement(1, 0, m_cur_dot);

    // Compute Jacobian of the dot constraint
    //    dot(dir1_abs, pos2_abs - pos1_abs) = 0
    {
        ChVector<> Phi_r1 = -dir1_abs;
        ChVector<> Phi_pi1 = Vcross(m_dir1, m_pos1) - Vcross(u12_loc1, m_pos1);

        m_cnstr_dot.Get_Cq_a()->ElementN(0) = (float)Phi_r1.x;
        m_cnstr_dot.Get_Cq_a()->ElementN(1) = (float)Phi_r1.y;
        m_cnstr_dot.Get_Cq_a()->ElementN(2) = (float)Phi_r1.z;

        m_cnstr_dot.Get_Cq_a()->ElementN(3) = (float)Phi_pi1.x;
        m_cnstr_dot.Get_Cq_a()->ElementN(4) = (float)Phi_pi1.y;
        m_cnstr_dot.Get_Cq_a()->ElementN(5) = (float)Phi_pi1.z;

        ChVector<> Phi_r2 = dir1_abs;
        ChVector<> Phi_pi2 = -Vcross(dir1_loc2, m_pos2);

        m_cnstr_dot.Get_Cq_b()->ElementN(0) = (float)Phi_r2.x;
        m_cnstr_dot.Get_Cq_b()->ElementN(1) = (float)Phi_r2.y;
        m_cnstr_dot.Get_Cq_b()->ElementN(2) = (float)Phi_r2.z;

        m_cnstr_dot.Get_Cq_b()->ElementN(3) = (float)Phi_pi2.x;
        m_cnstr_dot.Get_Cq_b()->ElementN(4) = (float)Phi_pi2.y;
        m_cnstr_dot.Get_Cq_b()->ElementN(5) = (float)Phi_pi2.z;
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkRevoluteSpherical::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    L(off_L) = m_cache_speed[0];
    L(off_L + 1) = m_cache_speed[1];
}

void ChLinkRevoluteSpherical::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    m_cache_speed[0] = L(off_L);
    m_cache_speed[1] = L(off_L + 1);

    // Also compute 'intuitive' reactions:

    double lam_dist = m_cache_speed[0];  // ||pos2_abs - pos1_abs|| - dist = 0
    double lam_dot = m_cache_speed[1];   // dot(dir1_abs, pos2_abs - pos1_abs) = 0

    // Calculate the reaction torques and forces on Body 2 in the joint frame
    // (Note: origin of the joint frame is at the center of the revolute joint
    //  which is defined on body 1, the x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    react_force.x = lam_dist;
    react_force.y = 0;
    react_force.z = lam_dot;

    react_torque.x = 0;
    react_torque.y = -m_cur_dist * lam_dot;
    react_torque.z = 0;
}

void ChLinkRevoluteSpherical::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                                  ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                                  const ChVectorDynamic<>& L,  ///< the L vector
                                                  const double c               ///< a scaling factor
                                                  ) {
    m_cnstr_dist.MultiplyTandAdd(R, L(off_L + 0) * c);
    m_cnstr_dot.MultiplyTandAdd(R, L(off_L + 1) * c);
}

void ChLinkRevoluteSpherical::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                                  ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                                  const double c,            ///< a scaling factor
                                                  bool do_clamp,             ///< apply clamping to c*C?
                                                  double recovery_clamp      ///< value for min/max clamping of c*C
                                                  ) {
    if (!IsActive())
        return;

    double cnstr_dist_violation =
        do_clamp ? ChMin(ChMax(c * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp) : c * (m_cur_dist - m_dist);

    double cnstr_dot_violation =
        do_clamp ? ChMin(ChMax(c * m_cur_dot, -recovery_clamp), recovery_clamp) : c * m_cur_dot;

    Qc(off_L + 0) += cnstr_dist_violation;
    Qc(off_L + 1) += cnstr_dot_violation;
}

void ChLinkRevoluteSpherical::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                                       const ChStateDelta& v,
                                       const ChVectorDynamic<>& R,
                                       const unsigned int off_L,  ///< offset in L, Qc
                                       const ChVectorDynamic<>& L,
                                       const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_cnstr_dist.Set_l_i(L(off_L + 0));
    m_cnstr_dot.Set_l_i(L(off_L + 1));

    m_cnstr_dist.Set_b_i(Qc(off_L + 0));
    m_cnstr_dot.Set_b_i(Qc(off_L + 1));
}

void ChLinkRevoluteSpherical::IntFromLCP(const unsigned int off_v,  ///< offset in v
                                         ChStateDelta& v,
                                         const unsigned int off_L,  ///< offset in L
                                         ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    L(off_L + 0) = m_cnstr_dist.Get_l_i();
    L(off_L + 1) = m_cnstr_dot.Get_l_i();
}

// -----------------------------------------------------------------------------
// Implementation of solver interface functions
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::InjectConstraints(ChLcpSystemDescriptor& descriptor) {
    if (!IsActive())
        return;

    descriptor.InsertConstraint(&m_cnstr_dist);
    descriptor.InsertConstraint(&m_cnstr_dot);
}

void ChLinkRevoluteSpherical::ConstraintsBiReset() {
    m_cnstr_dist.Set_b_i(0.0);
    m_cnstr_dot.Set_b_i(0.0);
}

void ChLinkRevoluteSpherical::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!IsActive())
        return;

    double cnstr_dist_violation = do_clamp
                                      ? ChMin(ChMax(factor * (m_cur_dist - m_dist), -recovery_clamp), recovery_clamp)
                                      : factor * (m_cur_dist - m_dist);

    double cnstr_dot_violation =
        do_clamp ? ChMin(ChMax(factor * m_cur_dot, -recovery_clamp), recovery_clamp) : factor * m_cur_dot;

    m_cnstr_dist.Set_b_i(m_cnstr_dist.Get_b_i() + cnstr_dist_violation);
    m_cnstr_dot.Set_b_i(m_cnstr_dot.Get_b_i() + cnstr_dot_violation);
}

void ChLinkRevoluteSpherical::ConstraintsLoadJacobians() {
    // Nothing to do here. Jacobians were loaded in Update().
}

void ChLinkRevoluteSpherical::ConstraintsFetch_react(double factor) {
    // Extract the Lagrange multipliers for the distance and for
    // the dot constraint.
    double lam_dist = m_cnstr_dist.Get_l_i();  // ||pos2_abs - pos1_abs|| - dist = 0
    double lam_dot = m_cnstr_dot.Get_l_i();    // dot(dir1_abs, pos2_abs - pos1_abs) = 0

    // Note that the Lagrange multipliers must be multiplied by 'factor' to
    // convert from reaction impulses to reaction forces.
    lam_dist *= factor;
    lam_dot *= factor;

    // Calculate the reaction torques and forces on Body 2 in the joint frame
    // (Note: origin of the joint frame is at the center of the revolute joint
    //  which is defined on body 1, the x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    react_force.x = lam_dist;
    react_force.y = 0;
    react_force.z = lam_dot;

    react_torque.x = 0;
    react_torque.y = -m_cur_dist * lam_dot;
    react_torque.z = 0;
}

// -----------------------------------------------------------------------------
// Additional reaction force and torque calculations due to the odd definition
//  of the standard output for this joint style
// -----------------------------------------------------------------------------

ChVector<> ChLinkRevoluteSpherical::Get_react_force_body1() {
    // Calculate the reaction forces on Body 1 in the joint frame
    // (Note: origin of the joint frame is at the center of the revolute joint
    //  which is defined on body 1, the x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    //  react_force = (-lam_dist,0,-lam_dot)

    return -react_force;
}

ChVector<> ChLinkRevoluteSpherical::Get_react_torque_body1() {
    // Calculate the reaction forces on Body 1 in the joint frame
    // (Note: origin of the joint frame is at the center of the revolute joint
    //  which is defined on body 1, the x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    //  react_torque = (0,m_cur_dist*lam_dot,0)

    return -react_torque;
}

ChVector<> ChLinkRevoluteSpherical::Get_react_force_body2() {
    // Calculate the reaction torques on Body 2 in the joint frame at the spherical joint
    // (Note: the joint frame x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    //  react_force = (lam_dist,0,lam_dot)
    return react_force;
}

ChVector<> ChLinkRevoluteSpherical::Get_react_torque_body2() {
    // Calculate the reaction torques on Body 2 in the joint frame at the spherical joint
    // (Note: the joint frame x-axis is along the vector from the
    //  point on body 1 to the point on body 2.  The z axis is along the revolute
    //  axis defined for the joint)
    //  react_torque = (0,0,0)
    return VNULL;
}

// -----------------------------------------------------------------------------
// Load and store multipliers (caching to allow warm starting)
// -----------------------------------------------------------------------------
void ChLinkRevoluteSpherical::ConstraintsLiLoadSuggestedSpeedSolution() {
    // Set multipliers to those cached at previous step.
    m_cnstr_dist.Set_l_i(m_cache_speed[0]);
    m_cnstr_dot.Set_l_i(m_cache_speed[1]);
}

void ChLinkRevoluteSpherical::ConstraintsLiLoadSuggestedPositionSolution() {
    // Set multipliers to those cached at previous step.
    m_cnstr_dist.Set_l_i(m_cache_pos[0]);
    m_cnstr_dot.Set_l_i(m_cache_pos[1]);
}

void ChLinkRevoluteSpherical::ConstraintsLiFetchSuggestedSpeedSolution() {
    // Cache current multipliers.
    m_cache_speed[0] = m_cnstr_dist.Get_l_i();
    m_cache_speed[1] = m_cnstr_dot.Get_l_i();
}

void ChLinkRevoluteSpherical::ConstraintsLiFetchSuggestedPositionSolution() {
    // Cache current multipliers.
    m_cache_pos[0] = m_cnstr_dist.Get_l_i();
    m_cache_pos[1] = m_cnstr_dot.Get_l_i();
}


void ChLinkRevoluteSpherical::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLink::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(m_pos1);
    marchive << CHNVP(m_pos2);
    marchive << CHNVP(m_dir1);
    marchive << CHNVP(m_dist);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRevoluteSpherical::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLink::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(m_pos1);
    marchive >> CHNVP(m_pos2);
    marchive >> CHNVP(m_dir1);
    marchive >> CHNVP(m_dist);
}


}  // END_OF_NAMESPACE____
