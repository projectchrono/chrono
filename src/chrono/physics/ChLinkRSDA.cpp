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

#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/solver/ChSystemDescriptor.h"

namespace chrono {

// Register into the object factory.
CH_FACTORY_REGISTER(ChLinkRSDA)

// Perturbation for finite-difference Jacobian approximation
const double ChLinkRSDA::m_FD_delta = 1e-8;

// -----------------------------------------------------------------------------

ChLinkRSDA::ChLinkRSDA()
    : m_turns(0),
      m_auto_rest_angle(true),
      m_rest_angle(0),
      m_torque(0),
      m_raw_angle(0),
      m_k(0),
      m_r(0),
      m_t(0),
      m_stiff(false),
      m_jacobians(nullptr) {}

ChLinkRSDA::ChLinkRSDA(const ChLinkRSDA& other) : ChLink(other), m_turns(0), m_torque(0), m_raw_angle(0), m_jacobians(nullptr) {
    m_body1 = other.m_body1;
    m_body2 = other.m_body2;
    system = other.system;

    m_csys1 = other.m_csys1;
    m_csys2 = other.m_csys2;

    m_turns = other.m_turns;
    m_auto_rest_angle = other.m_auto_rest_angle;
    m_rest_angle = other.m_rest_angle;

    m_angle = other.m_angle;
    m_angle_dt = other.m_angle_dt;

    m_k = other.m_k;
    m_r = other.m_r;
    m_t = other.m_t;

    m_torque_fun = other.m_torque_fun;

    m_stiff = other.m_stiff;
}

ChLinkRSDA::~ChLinkRSDA() {
    delete m_jacobians;
}

// -----------------------------------------------------------------------------
// Link initialization functions

void ChLinkRSDA::Initialize(std::shared_ptr<ChBody> body1, std::shared_ptr<ChBody> body2, const ChFrame<>& frame) {
    m_body1 = body1.get();
    m_body2 = body2.get();

    m_csys1 = m_body1->GetCoordsys().TransformParentToLocal(frame.GetCoordsys());
    m_csys2 = m_body2->GetCoordsys().TransformParentToLocal(frame.GetCoordsys());

    CalcAngle();
    m_last_angle = m_angle;
    if (m_auto_rest_angle)
        m_rest_angle = m_angle;
}

void ChLinkRSDA::Initialize(std::shared_ptr<ChBody> body1,
                            std::shared_ptr<ChBody> body2,
                            bool local,
                            const ChFrame<>& frame1,
                            const ChFrame<>& frame2) {
    m_body1 = body1.get();
    m_body2 = body2.get();

    if (local) {
        m_csys1 = frame1.GetCoordsys();
        m_csys2 = frame2.GetCoordsys();
    } else {
        m_csys1 = m_body1->GetCoordsys().TransformParentToLocal(frame1.GetCoordsys());
        m_csys2 = m_body2->GetCoordsys().TransformParentToLocal(frame2.GetCoordsys());
    }

    CalcAngle();
    m_last_angle = m_angle;
    if (m_auto_rest_angle)
        m_rest_angle = m_angle;
}

// -----------------------------------------------------------------------------

void ChLinkRSDA::SetRestAngle(double rest_angle) {
    m_auto_rest_angle = false;
    m_rest_angle = rest_angle;
}

void ChLinkRSDA::SetNumInitRevolutions(int n) {
    m_turns = n;
}

// -----------------------------------------------------------------------------

double ChLinkRSDA::GetRestAngle() const {
    return m_rest_angle;
}

double ChLinkRSDA::GetAngle() const {
    return CH_2PI * m_turns + m_angle;
}

double ChLinkRSDA::GetDeformation() const {
    return CH_2PI * m_turns + m_angle - m_rest_angle;
}

double ChLinkRSDA::GetVelocity() const {
    return m_angle_dt;
}

double ChLinkRSDA::GetTorque() const {
    return m_torque;
}

ChFrame<> ChLinkRSDA::GetVisualModelFrame(unsigned int nclone) const {
    return ChFrame<>(m_csys1 >> m_body1->GetCoordsys());
}

// -----------------------------------------------------------------------------

void ChLinkRSDA::CalcAngle() {
    // Express the RSDA frames in absolute frame
    ChQuaternion<> rot1 = m_body1->GetRot() * m_csys1.rot;
    ChQuaternion<> rot2 = m_body2->GetRot() * m_csys2.rot;

    // Extract unit vectors
    auto f1 = rot1.GetAxisX();
    auto g1 = rot1.GetAxisY();
    auto f2 = rot2.GetAxisX();

    // Calculate sine and cosine of rotation angle
    double s = Vdot(g1, f2);
    double c = Vdot(f1, f2);

    // Get angle (in [-pi , +pi])
    double a = std::asin(s);
    if (c >= 0) {
        m_angle = a;
    } else {
        m_angle = (s >= 0) ? CH_PI - a : -CH_PI - a;
    }

    // Get angle rate of change
    m_axis = rot1.GetAxisZ();
    m_angle_dt = Vdot(m_axis, m_body2->GetAngVelParent() - m_body1->GetAngVelParent());
}

void ChLinkRSDA::AdjustAngle() {
    // Check cross at +- pi
    if (m_last_angle > CH_PI_2 && m_angle < 0)
        m_angle += CH_2PI;
    if (m_last_angle < -CH_PI_2 && m_angle > 0)
        m_angle -= CH_2PI;

    // Accumulate full turns
    if (m_last_angle - m_angle > CH_PI)
        m_turns++;
    if (m_last_angle - m_angle < -CH_PI)
        m_turns--;

    if (m_angle < 0) {
        while (m_turns > 0) {
            m_angle += CH_2PI;
            m_turns--;
        }
    }
    if (m_angle > 0) {
        while (m_turns < 0) {
            m_angle -= CH_2PI;
            m_turns++;
        }
    }

    // Update last angle
    m_last_angle = m_angle;
}

// -----------------------------------------------------------------------------
// Link update function
// -----------------------------------------------------------------------------
void ChLinkRSDA::Update(double time, UpdateFlags update_flags) {
    ChLink::Update(time, update_flags);

    // Calculate current angle and angle rate
    CalcAngle();
    m_raw_angle = m_angle;
    AdjustAngle();

    // Calculate torque along RSDA axis
    m_torque = CalcTorque(time, m_angle, m_angle_dt);

    // Compute Jacobians (if needed)
    if (m_stiff) {
        if (!m_jacobians)
            CreateJacobianMatrices();

        // Pack states and state derivatives for the two connected bodies
        ChState state_x(14, nullptr);
        ChStateDelta state_w(12, nullptr);

        static_cast<ChBody*>(m_body1)->LoadableGetStateBlockPosLevel(0, state_x);
        static_cast<ChBody*>(m_body2)->LoadableGetStateBlockPosLevel(7, state_x);

        static_cast<ChBody*>(m_body1)->LoadableGetStateBlockVelLevel(0, state_w);
        static_cast<ChBody*>(m_body2)->LoadableGetStateBlockVelLevel(6, state_w);

        ComputeQ(time, state_x, state_w, m_Qforce);
        ComputeJacobians(time, state_x, state_w);
    }
}

double ChLinkRSDA::CalcTorque(double time, double angle, double angle_dt) const {
    if (m_torque_fun)
        return m_torque_fun->evaluate(time, m_rest_angle, angle, angle_dt, *this);

    return m_t - m_k * (CH_2PI * m_turns + angle - m_rest_angle) - m_r * angle_dt;
}

// -----------------------------------------------------------------------------
// Jacobians for a stiff RSDA
// -----------------------------------------------------------------------------

void ChLinkRSDA::ComputeQ(double time,                  // current time
                          const ChState& state_x,       // state position to evaluate Q
                          const ChStateDelta& state_w,  // state speed to evaluate Q
                          ChVectorDynamic<>& Qforce     // output forcing vector
) {
    // Extract orientations and angular velocities (local frames) of the two connected bodies
    ChQuaternion<> brot1(state_x(3), state_x(4), state_x(5), state_x(6));
    ChQuaternion<> brot2(state_x(10), state_x(11), state_x(12), state_x(13));
    ChVector3d bwloc1(state_w(3), state_w(4), state_w(5));
    ChVector3d bwloc2(state_w(9), state_w(10), state_w(11));

    // Express the RSDA frames in absolute frame and calculate the sine and cosine of the rotation angle
    ChQuaternion<> rot1 = brot1 * m_csys1.rot;
    ChQuaternion<> rot2 = brot2 * m_csys2.rot;
    auto f1 = rot1.GetAxisX();
    auto g1 = rot1.GetAxisY();
    auto f2 = rot2.GetAxisX();
    double s = Vdot(g1, f2);
    double c = Vdot(f1, f2);

    // Angle in [-pi, +pi], then the angle relative to the current configuration (accounting for the adjustments
    // and full turns applied to the current angle)
    double a = std::asin(s);
    if (c < 0)
        a = (s >= 0) ? CH_PI - a : -CH_PI - a;
    double delta = a - m_raw_angle;
    if (delta > CH_PI)
        delta -= CH_2PI;
    if (delta < -CH_PI)
        delta += CH_2PI;
    double angle = m_angle + delta;

    // Angle rate of change
    auto axis = rot1.GetAxisZ();
    double angle_dt = Vdot(axis, brot2.Rotate(bwloc2) - brot1.Rotate(bwloc1));

    // Torque in absolute frame and generalized forces (torques in local frames)
    ChVector3d torque = CalcTorque(time, angle, angle_dt) * axis;
    Qforce.setZero();
    Qforce.segment(3, 3) = -brot1.RotateBack(torque).eigen();
    Qforce.segment(9, 3) = brot2.RotateBack(torque).eigen();
}

void ChLinkRSDA::CreateJacobianMatrices() {
    m_jacobians = new SpringJacobians;

    // Collect all variables associated with this element
    std::vector<ChVariables*> variables_list;
    static_cast<ChBody*>(m_body1)->LoadableGetVariables(variables_list);
    static_cast<ChBody*>(m_body2)->LoadableGetVariables(variables_list);

    // Set the associated variables in the underlying KRM block
    m_jacobians->m_KRM.SetVariables(variables_list);

    // Resize matrices
    m_jacobians->m_K.resize(12, 12);
    m_jacobians->m_R.resize(12, 12);
    m_Qforce.resize(12);
}

void ChLinkRSDA::ComputeJacobians(double time,                 // current time
                                  const ChState& state_x,      // state position to evaluate jacobians
                                  const ChStateDelta& state_w  // state speed to evaluate jacobians
) {
    ChVectorDynamic<> Qforce1(12);  // forcing vector after perturbation

    // Compute Jacobian w.r.t. states of the connected bodies (load columns of the m_K matrix).
    // Note: we must "bake in" the velocity transform: perturb at velocity level and let the body increment state.
    ChState state_x_perturbed(14, nullptr);
    ChStateDelta state_delta(12, nullptr);
    state_delta.setZero(12, nullptr);

    for (int i = 0; i < 12; i++) {
        state_delta(i) += m_FD_delta;
        static_cast<ChBody*>(m_body1)->LoadableStateIncrement(0, state_x_perturbed, state_x, 0, state_delta);
        static_cast<ChBody*>(m_body2)->LoadableStateIncrement(7, state_x_perturbed, state_x, 6, state_delta);
        ComputeQ(time, state_x_perturbed, state_w, Qforce1);
        state_delta(i) -= m_FD_delta;
        m_jacobians->m_K.col(i) = (Qforce1 - m_Qforce) * (1 / m_FD_delta);
    }

    // Compute Jacobian w.r.t. state derivatives of the connected bodies (load columns of the m_R matrix)
    ChStateDelta state_w_perturbed = state_w;

    for (int i = 0; i < 12; i++) {
        state_w_perturbed(i) += m_FD_delta;
        ComputeQ(time, state_x, state_w_perturbed, Qforce1);
        state_w_perturbed(i) -= m_FD_delta;
        m_jacobians->m_R.col(i) = (Qforce1 - m_Qforce) * (1 / m_FD_delta);
    }
}

void ChLinkRSDA::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (m_jacobians && IsActive()) {
        descriptor.InsertKRMBlock(&m_jacobians->m_KRM);
    }
}

void ChLinkRSDA::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (m_jacobians) {
        // Recall to flip sign to load K = -dQ/dx and R = -dQ/dv
        m_jacobians->m_KRM.GetMatrix() = -Kfactor * m_jacobians->m_K - Rfactor * m_jacobians->m_R;
    }
}

void ChLinkRSDA::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    if (!IsActive())
        return;

    // Applied torque in absolute frame
    ChVector3d torque = m_torque * m_axis;

    // Load torques in 'R' vector accumulator (torques in local coords)
    if (m_body1->Variables().IsActive()) {
        R.segment(m_body1->Variables().GetOffset() + 3, 3) -=
            c * m_body1->TransformDirectionParentToLocal(torque).eigen();
    }
    if (m_body2->Variables().IsActive()) {
        R.segment(m_body2->Variables().GetOffset() + 3, 3) +=
            c * m_body2->TransformDirectionParentToLocal(torque).eigen();
    }
}

void ChLinkRSDA::ConstraintsFbLoadForces(double factor) {
    if (!IsActive())
        return;

    // Applied torque in absolute frame
    ChVector3d torque = m_torque * m_axis;

    // Load torques in 'fb' vector accumulator of body variables (torques in local coords)
    m_body1->Variables().Force().segment(3, 3) -= factor * m_body1->TransformDirectionParentToLocal(torque).eigen();
    m_body2->Variables().Force().segment(3, 3) += factor * m_body2->TransformDirectionParentToLocal(torque).eigen();
}

void ChLinkRSDA::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkRSDA>();

    // serialize parent class
    ChLink::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_csys1);
    archive_out << CHNVP(m_csys2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRSDA::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkRSDA>();

    // deserialize parent class
    ChLink::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_csys1);
    archive_in >> CHNVP(m_csys2);
}

}  // end namespace chrono
