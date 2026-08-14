// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/utils/ChUtils.h"
#include "chrono/physics/ChActuator.h"

namespace chrono {

ChActuator::ChActuator() : is_attached(false), calculate_consistent_IC(false) {}

double ChActuator::GetInput(double t) const {
    return ChClamp(ref_fun->GetVal(t), -1.0, +1.0);
}

void ChActuator::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    if (!IsActive())
        return;

    // Load dynamics of the actuator (if any)
    ChExternalDynamicsODE::IntLoadResidual_F(off, R, c);

    if (is_attached) {
        // Add forces to connected bodies (calculated in Update)
        if (m_body1->Variables().IsActive()) {
            R.segment(m_body1->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(0, 3);
            R.segment(m_body1->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(3, 3);
        }
        if (m_body2->Variables().IsActive()) {
            R.segment(m_body2->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(6, 3);
            R.segment(m_body2->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(9, 3);
        }
    }
}

void ChActuator::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    // Insert the optional coupling KRM block (if used)
    if (m_KRM_coupling.GetNumVariables() > 0)
        descriptor.InsertKRMBlock(&m_KRM_coupling);

    // Let the base class insert its own KRM block for actuator dynamics (if any)
    ChExternalDynamicsODE::InjectKRMMatrices(descriptor);
}

void ChActuator::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    // Let derived classes load the coupling KRM block (if used)
    if (m_KRM_coupling.GetNumVariables() > 0)
        ComputeCouplingKRM(m_KRM_coupling.GetMatrix(), Kfactor, Rfactor, Mfactor);

    // Let the base class load its own KRM block for actuator dynamics (if any)
    ChExternalDynamicsODE::LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
}

// -----------------------------------------------------------------------------

ChLinearActuator::ChLinearActuator() : ChActuator() {}

void ChLinearActuator::SetActuatorInitialLength(double len) {
    s_0 = len;
    s = len;
    sd = 0;
}

void ChLinearActuator::SetActuatorLength(double len, double vel) {
    // Do nothing if not attached to bodies
    if (is_attached)
        return;

    s = len;
    sd = vel;
}

void ChLinearActuator::SetInitialLoad(double initial_load) {
    calculate_consistent_IC = true;
    F0 = initial_load;
}

void ChLinearActuator::Connect(std::shared_ptr<ChBody> body1, std::shared_ptr<ChBody> body2, bool local, ChVector3d loc1, ChVector3d loc2) {
    is_attached = true;

    // Call base initialization (external dynamics)
    Initialize();

    // Cache connected bodies and body local connection points
    m_body1 = body1.get();
    m_body2 = body2.get();

    if (local) {
        m_loc1 = loc1;
        m_loc2 = loc2;
        m_aloc1 = body1->TransformPointLocalToParent(loc1);
        m_aloc2 = body2->TransformPointLocalToParent(loc2);
    } else {
        m_loc1 = body1->TransformPointParentToLocal(loc1);
        m_loc2 = body2->TransformPointParentToLocal(loc2);
        m_aloc1 = loc1;
        m_aloc2 = loc2;
    }

    s_0 = (m_aloc1 - m_aloc2).Length();

    // Resize temporary vector of generalized body forces
    m_Qforce.resize(12);

    // If enabled, set up the coupling KRM block
    if (EnableCouplingKRM()) {
        std::vector<ChVariables*> vars;
        vars.push_back(&m_body1->Variables());
        vars.push_back(&m_body2->Variables());
        m_KRM_coupling.SetVariables(vars);
    }
}

void ChLinearActuator::Update(double time, UpdateFlags update_flags) {
    // Update the dynamics of this actuator (if any)
    ChExternalDynamicsODE::Update(time, update_flags);

    // If the actuator is attached to bodies
    // - update its length and rate from the body states
    // - apply the generated force to the two bodies
    if (is_attached) {
        m_aloc1 = m_body1->TransformPointLocalToParent(m_loc1);
        m_aloc2 = m_body2->TransformPointLocalToParent(m_loc2);

        auto avel1 = m_body1->PointSpeedLocalToParent(m_loc1);
        auto avel2 = m_body2->PointSpeedLocalToParent(m_loc2);

        ChVector3d dir = (m_aloc1 - m_aloc2).GetNormalized();

        s = (m_aloc1 - m_aloc2).Length();
        sd = Vdot(dir, avel1 - avel2);

        ////std::cout << "time = " << time << "    s=" << s << "   sd=" << sd << std::endl;

        // Actuator force
        auto f = GetActuatorForce(time);
        ChVector3d force = f * dir;

        // Force and moment acting on body 1
        auto atorque1 = Vcross(m_aloc1 - m_body1->GetPos(), force);          // applied torque (absolute frame)
        auto ltorque1 = m_body1->TransformDirectionParentToLocal(atorque1);  // applied torque (local frame)
        m_Qforce.segment(0, 3) = force.eigen();
        m_Qforce.segment(3, 3) = ltorque1.eigen();

        // Force and moment acting on body 2
        auto atorque2 = Vcross(m_aloc2 - m_body2->GetPos(), -force);         // applied torque (absolute frame)
        auto ltorque2 = m_body2->TransformDirectionParentToLocal(atorque2);  // applied torque (local frame)
        m_Qforce.segment(6, 3) = -force.eigen();
        m_Qforce.segment(9, 3) = ltorque2.eigen();
    }
}

// -----------------------------------------------------------------------------

ChRotationalActuator::ChRotationalActuator() : ChActuator() {}

void ChRotationalActuator::SetActuatorInitialAngle(double angle) {
    a_0 = angle;
    a = angle;
    ad = 0;
}

void ChRotationalActuator::SetActuatorAngle(double angle, double ang_vel) {
    // Do nothing if not attached to bodies
    if (is_attached)
        return;

    a = angle;
    ad = ang_vel;
}

void ChRotationalActuator::SetInitialLoad(double initial_load) {
    calculate_consistent_IC = true;
    T0 = initial_load;
}

void ChRotationalActuator::CalcAngle() {
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
    a = std::asin(s);
    if (c < 0) {
        a = (s >= 0) ? CH_PI - a : -CH_PI - a;
    }

    // Get angle rate of change
    m_axis = rot1.GetAxisZ();
    ad = Vdot(m_axis, m_body2->GetAngVelParent() - m_body1->GetAngVelParent());
}

void ChRotationalActuator::AdjustAngle() {
    // Check cross at +- pi
    if (m_last_angle > CH_PI_2 && a < 0)
        a += CH_2PI;
    if (m_last_angle < -CH_PI_2 && a > 0)
        a -= CH_PI_2;

    // Accumulate full turns
    if (m_last_angle - a > CH_PI)
        m_turns++;
    if (m_last_angle - a < -CH_PI)
        m_turns--;

    if (a < 0) {
        while (m_turns > 0) {
            a += CH_2PI;
            m_turns--;
        }
    }
    if (a > 0) {
        while (m_turns < 0) {
            a -= CH_2PI;
            m_turns++;
        }
    }

    // Update last angle
    m_last_angle = a;
}

void ChRotationalActuator::Connect(std::shared_ptr<ChBody> body1, std::shared_ptr<ChBody> body2, const ChFrame<>& frame) {
    is_attached = true;

    // Call base initialization (external dynamics)
    Initialize();

    // Cache connected bodies
    m_body1 = body1.get();
    m_body2 = body2.get();

    // Coordinate frames on the two bodies
    m_csys1 = m_body1->GetCoordsys().TransformParentToLocal(frame.GetCoordsys());
    m_csys2 = m_body2->GetCoordsys().TransformParentToLocal(frame.GetCoordsys());

    // Calculate initial angle
    CalcAngle();
    a_0 = a;
    m_last_angle = a;

    // Resize temporary vector of generalized body forces
    m_Qforce.resize(12);

    // If enabled, set up the coupling KRM block
    if (EnableCouplingKRM()) {
        std::vector<ChVariables*> vars;
        vars.push_back(&m_body1->Variables());
        vars.push_back(&m_body2->Variables());
        m_KRM_coupling.SetVariables(vars);
    }
}

void ChRotationalActuator::Update(double time, UpdateFlags update_flags) {
    // Update the dynamics of this actuator (if any)
    ChExternalDynamicsODE::Update(time, update_flags);

    // If the actuator is attached to bodies
    // - update its angle and rate from the body states
    // - apply the generated torque to the two bodies
    if (is_attached) {
        // Calculate current angle and angle rate
        CalcAngle();
        AdjustAngle();

        // Calculate torque along actuator axis
        double angle = CH_2PI * m_turns + a;
        auto t = GetActuatorTorque(time);
        ChVector3d torque = t * m_axis;

        // Force and moment acting on body 1
        auto ltorque1 = m_body1->TransformDirectionParentToLocal(torque);
        m_Qforce.segment(0, 3).setZero();
        m_Qforce.segment(3, 3) = ltorque1.eigen();

        // Force and moment acting on body 2
        auto ltorque2 = m_body2->TransformDirectionParentToLocal(torque);
        m_Qforce.segment(6, 3).setZero();
        m_Qforce.segment(9, 3) = ltorque2.eigen();
    }
}

}  // end namespace chrono
