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

namespace chrono {

// Register into the object factory.
CH_FACTORY_REGISTER(ChLinkRSDA)

// -----------------------------------------------------------------------------

ChLinkRSDA::ChLinkRSDA() : m_turns(0), m_auto_rest_angle(true), m_rest_angle(0), m_torque(0), m_k(0), m_r(0), m_t(0) {}

ChLinkRSDA::ChLinkRSDA(const ChLinkRSDA& other) : ChLink(other), m_turns(0), m_torque(0) {
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
        m_angle -= CH_PI_2;

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
void ChLinkRSDA::Update(double time, bool update_assets) {
    ChLink::Update(time, update_assets);

    // Calculate current angle and angle rate
    CalcAngle();
    AdjustAngle();

    // Calculate torque along RSDA axis
    double angle = CH_2PI * m_turns + m_angle;
    if (m_torque_fun) {
        m_torque = m_torque_fun->evaluate(time, m_rest_angle, m_angle, m_angle_dt, *this);
    } else {
        m_torque = m_t - m_k * (angle - m_rest_angle) - m_r * m_angle_dt;
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
