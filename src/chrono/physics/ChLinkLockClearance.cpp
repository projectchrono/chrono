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

#include "chrono/physics/ChLinkLockClearance.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLockClearance)

ChLinkLockClearance::ChLinkLockClearance() {
    type = LinkType::CLEARANCE;

    clearance = 0.1;
    c_friction = 0.;
    c_viscous = 0.;
    c_restitution = 0.9;
    c_tang_restitution = 0.9;
    diameter = 0.8;

    contact_F_abs = VNULL;
    contact_V_abs = VNULL;

    limit_X->SetActive(true);
    limit_X->SetMax(clearance);
    limit_X->SetMaxElastic(c_restitution);
    limit_X->SetMin(-1000.0);

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    mask.SetLockMask(false, false, false, false, true, true, false);
    BuildLink();
}

ChLinkLockClearance::ChLinkLockClearance(const ChLinkLockClearance& other) : ChLinkLockLock(other) {
    clearance = other.clearance;
    c_friction = other.c_friction;
    c_restitution = other.c_restitution;
    c_tang_restitution = other.c_tang_restitution;
    c_viscous = other.c_viscous;
    diameter = other.diameter;

    contact_F_abs = other.contact_F_abs;
    contact_V_abs = other.contact_V_abs;
}

double ChLinkLockClearance::Get_axis_eccentricity() {
    return GetDist();
}

double ChLinkLockClearance::Get_axis_phase() {
    if (!GetMarker2())
        return 0;
    double angle;
    ChVector3d axis;
    GetMarker2()->GetCoordsys().rot.GetAngleAxis(angle, axis);
    if (axis.z() < 0.0) {
        axis = Vmul(axis, -1.0);
        angle = CH_C_2PI - angle;
    }
    return angle;
}

double ChLinkLockClearance::Get_rotation_angle() {
    return GetRelAngle();
}

ChVector3d ChLinkLockClearance::Get_contact_P_abs() {
    if (!GetMarker2())
        return VNULL;
    return GetMarker2()->GetAbsCoordsys().pos - (clearance + diameter / 2) * Get_contact_N_abs();
}

ChVector3d ChLinkLockClearance::Get_contact_N_abs() {
    if (!GetMarker2())
        return VECT_X;
    return GetMarker2()->GetAbsFrame().TransformDirectionLocalToParent(-VECT_X);
}

ChVector3d ChLinkLockClearance::Get_contact_F_abs() {
    return contact_F_abs;
}

double ChLinkLockClearance::Get_contact_F_n() {
    if (!GetMarker2())
        return 0;
    return GetMarker2()->GetAbsFrame().TransformDirectionParentToLocal(contact_F_abs).x();
}

double ChLinkLockClearance::Get_contact_F_t() {
    if (!GetMarker2())
        return 0;
    return GetMarker2()->GetAbsFrame().TransformDirectionParentToLocal(contact_F_abs).y();
}

double ChLinkLockClearance::Get_contact_V_t() {
    if (!GetMarker2())
        return 0;
    return GetMarker2()->GetAbsFrame().TransformDirectionParentToLocal(contact_V_abs).y();
}

void ChLinkLockClearance::UpdateForces(double mytime) {
    // May avoid inheriting parent class force computation, since not needed
    ////LinkLock::UpdateForces(mytime);

    ChVector3d m_friction_F_abs = VNULL;
    double m_norm_force = -react_force.x();

    // Add Coulomb kinematic friction

    if (mask.Constr_X().IsActive()) {
        ChVector3d temp = Get_contact_P_abs();
        ChVector3d pb1 = Body1->TransformPointParentToLocal(temp);
        ChVector3d pb2 = Body2->TransformPointParentToLocal(temp);
        ChVector3d m_V1_abs = Body1->PointSpeedLocalToParent(pb1);
        ChVector3d m_V2_abs = Body2->PointSpeedLocalToParent(pb2);
        contact_V_abs = Vsub(m_V1_abs, m_V2_abs);
        ChVector3d m_tang_V_abs =
            Vsub(contact_V_abs, Vmul(Get_contact_N_abs(), Vdot(contact_V_abs, Get_contact_N_abs())));

        // absolute friction force, as applied in contact point
        m_friction_F_abs = Vmul(Vnorm(m_tang_V_abs), Get_c_friction() * (-m_norm_force));

        // transform the friction force in link master coords ***TO CHECK*** (new version!)
        C_force = marker2->GetAbsFrame().TransformDirectionParentToLocal(m_friction_F_abs);
    }

    // update internal data: the abs. vector of all contact forces, is a sum of reaction and friction
    contact_F_abs = Vadd(Vmul(Get_contact_N_abs(), m_norm_force), m_friction_F_abs);
}

void ChLinkLockClearance::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkLockLock::UpdateTime(mytime);

    // Move (well, rotate...) marker 2 to align it in actuator direction

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::MotionType::EXTERNAL);

    ChMatrix33<> ma;
    ma.SetFromQuaternion(marker2->GetAbsCoordsys().rot);

    ChVector3d absdist = Vsub(marker1->GetAbsCoordsys().pos, marker2->GetAbsCoordsys().pos);

    ChVector3d mz = ma.GetAxisZ();
    ChVector3d my = Vnorm(Vcross(ma.GetAxisZ(), absdist));
    ChVector3d mx = Vnorm(Vcross(my, ma.GetAxisZ()));

    ma.SetFromDirectionAxes(mx, my, mz);

    // rotate "main" marker2 into tangent position
    marker2->ImposeAbsoluteTransform(ChFrame<>(marker2->GetAbsCoordsys().pos, ma.GetQuaternion()));  

    // imposed relative positions/speeds
    deltaC.pos = VNULL;
    deltaC.pos.x() = clearance;  // distance is always on M2 'X' axis

    deltaC_dt.pos = VNULL;
    deltaC_dt.pos.x() = 0;  // distance speed

    deltaC_dtdt.pos = VNULL;

    // add also the centripetal acceleration if distance vector's rotating,
    // as centripetal acc. of point sliding on a sphere surface.
    ChVector3d tang_speed = GetRelM_dt().pos;
    tang_speed.x() = 0;  // only z-y coords in relative tang speed vector
    double Rcurvature = Vlength(absdist);
    deltaC_dtdt.pos.x() = -pow(Vlength(tang_speed), 2) / Rcurvature;  // An =  -(Vt^2 / r)

    deltaC.rot = QUNIT;  // no relative rotations imposed!
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}

void ChLinkLockClearance::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkLockClearance>();

    // serialize parent class
    ChLinkLockLock::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(clearance);
    archive_out << CHNVP(c_friction);
    archive_out << CHNVP(c_restitution);
    archive_out << CHNVP(diameter);
    archive_out << CHNVP(c_tang_restitution);
    archive_out << CHNVP(c_viscous);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkLockClearance::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkLockClearance>();

    // deserialize parent class
    ChLinkLockLock::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(clearance);
    archive_in >> CHNVP(c_friction);
    archive_in >> CHNVP(c_restitution);
    archive_in >> CHNVP(diameter);
    archive_in >> CHNVP(c_tang_restitution);
    archive_in >> CHNVP(c_viscous);
}

}  // end namespace chrono
