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

#include "chrono/physics/ChLinkClearance.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkClearance)

ChLinkClearance::ChLinkClearance() {
    type = LinkType::CLEARANCE;

    clearance = 0.1;
    c_friction = 0.;
    c_viscous = 0.;
    c_restitution = 0.9;
    c_tang_restitution = 0.9;
    diameter = 0.8;

    contact_F_abs = VNULL;
    contact_V_abs = VNULL;

    this->limit_X->SetActive(true);
    this->limit_X->SetMax(clearance);
    this->limit_X->SetMaxElastic(c_restitution);
    this->limit_X->SetMin(-1000.0);

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    // It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false, false, true, true, false);

    ChangedLinkMask();
}

ChLinkClearance::ChLinkClearance(const ChLinkClearance& other) : ChLinkLockLock(other) {
    clearance = other.clearance;
    c_friction = other.c_friction;
    c_restitution = other.c_restitution;
    c_tang_restitution = other.c_tang_restitution;
    c_viscous = other.c_viscous;
    diameter = other.diameter;

    contact_F_abs = other.contact_F_abs;
    contact_V_abs = other.contact_V_abs;
}

// easy data getting

double ChLinkClearance::Get_axis_eccentricity() {
    return (this->GetDist());
}
double ChLinkClearance::Get_axis_phase() {
    if (!this->GetMarker2())
        return 0;
    double mangle;
    Vector maxis;
    Q_to_AngAxis(this->GetMarker2()->GetCoord().rot, mangle, maxis);
    if (maxis.z() < 0.0) {
        maxis = Vmul(maxis, -1.0);
        mangle = (2.0 * CH_C_PI) - mangle;
    }
    return (mangle);
}
double ChLinkClearance::Get_rotation_angle() {
    return (this->GetRelAngle());
}
Vector ChLinkClearance::Get_contact_P_abs() {
    if (!this->GetMarker2())
        return VNULL;
    return Vadd(this->GetMarker2()->GetAbsCoord().pos,
                Vmul(this->Get_contact_N_abs(), -(this->clearance + this->diameter / 2)));
}
Vector ChLinkClearance::Get_contact_N_abs() {
    if (!this->GetMarker2())
        return VECT_X;
    Vector mNrel = VECT_X;
    mNrel = Vmul(mNrel, -1);
    return (this->GetMarker2()->Dir_Ref2World(&mNrel));
}
Vector ChLinkClearance::Get_contact_F_abs() {
    return (this->contact_F_abs);
}
double ChLinkClearance::Get_contact_F_n() {
    if (!this->GetMarker2())
        return 0;
    return (this->GetMarker2()->Dir_World2Ref(&this->contact_F_abs).x());
}
double ChLinkClearance::Get_contact_F_t() {
    if (!this->GetMarker2())
        return 0;
    return (this->GetMarker2()->Dir_World2Ref(&this->contact_F_abs).y());
}
double ChLinkClearance::Get_contact_V_t() {
    if (!this->GetMarker2())
        return 0;
    return (this->GetMarker2()->Dir_World2Ref(&this->contact_V_abs).y());
}

void ChLinkClearance::UpdateForces(double mytime) {
    // May avoid inheriting parent class force computation, since not
    // needed...
    // LinkLock::UpdateForces(mytime);

    Vector m_friction_F_abs = VNULL;
    double m_norm_force = -this->react_force.x();

    // Just add Coulomb kinematic friction...

    if (((ChLinkMaskLF*)(this->GetMask()))->Constr_X().IsActive()) {
        Vector temp = Get_contact_P_abs();
        Vector pb1 = ((ChFrame<double>*)Body1)->TransformParentToLocal(temp);
        Vector pb2 = ((ChFrame<double>*)Body2)->TransformParentToLocal(temp);
        Vector m_V1_abs = Body1->PointSpeedLocalToParent(pb1);
        Vector m_V2_abs = Body2->PointSpeedLocalToParent(pb2);
        this->contact_V_abs = Vsub(m_V1_abs, m_V2_abs);
        Vector m_tang_V_abs = Vsub(contact_V_abs, Vmul(Get_contact_N_abs(), Vdot(contact_V_abs, Get_contact_N_abs())));

        // absolute friction force, as applied in contact point
        m_friction_F_abs = Vmul(Vnorm(m_tang_V_abs), Get_c_friction() * (-m_norm_force));

        // transform the friction force in link master coords ***TO CHECK*** (new version!)
        this->C_force += this->marker2->Dir_World2Ref(&m_friction_F_abs);
    }

    // update internal data: the abs. vector of all contact forces, is a sum of reaction and friction
    this->contact_F_abs = Vadd(Vmul(Get_contact_N_abs(), m_norm_force), m_friction_F_abs);
}

void ChLinkClearance::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkLockLock::UpdateTime(mytime);

    // Move (well, rotate...) marker 2 to align it in actuator direction

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

    ChMatrix33<> ma;
    ma.Set_A_quaternion(marker2->GetAbsCoord().rot);

    Vector absdist = Vsub(marker1->GetAbsCoord().pos, marker2->GetAbsCoord().pos);

    Vector mz = ma.Get_A_Zaxis();
    Vector my = Vnorm(Vcross(ma.Get_A_Zaxis(), absdist));
    Vector mx = Vnorm(Vcross(my, ma.Get_A_Zaxis()));

    ma.Set_A_axis(mx, my, mz);

    Coordsys newmarkpos;
    newmarkpos.pos = marker2->GetAbsCoord().pos;
    newmarkpos.rot = ma.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);  // rotate "main" marker2 into tangent position

    // imposed relative positions/speeds
    deltaC.pos = VNULL;
    deltaC.pos.x() = this->clearance;  // distance is always on M2 'X' axis

    deltaC_dt.pos = VNULL;
    deltaC_dt.pos.x() = 0;  // distance speed

    deltaC_dtdt.pos = VNULL;

    // add also the centripetal acceleration if distance vector's rotating,
    // as centripetal acc. of point sliding on a sphere surface.
    Vector tang_speed = GetRelM_dt().pos;
    tang_speed.x() = 0;  // only z-y coords in relative tang speed vector
    double Rcurvature = Vlength(absdist);
    deltaC_dtdt.pos.x() = -pow(Vlength(tang_speed), 2) / Rcurvature;  // An =  -(Vt^2 / r)

    deltaC.rot = QUNIT;  // no relative rotations imposed!
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}

void ChLinkClearance::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkClearance>();

    // serialize parent class
    ChLinkLockLock::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(clearance);
    marchive << CHNVP(c_friction);
    marchive << CHNVP(c_restitution);
    marchive << CHNVP(diameter);
    marchive << CHNVP(c_tang_restitution);
    marchive << CHNVP(c_viscous);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkClearance::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkClearance>();

    // deserialize parent class
    ChLinkLockLock::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(clearance);
    marchive >> CHNVP(c_friction);
    marchive >> CHNVP(c_restitution);
    marchive >> CHNVP(diameter);
    marchive >> CHNVP(c_tang_restitution);
    marchive >> CHNVP(c_viscous);
}

}  // end namespace chrono
