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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkWheel.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
ChClassRegister<ChLinkWheel> a_registration_ChLinkWheel;

ChLinkWheel::ChLinkWheel()
    : angle(0),
      angle_dt(0),
      angle_dtdt(0),
      slipping(0),
      f_slip(0),
      l_slip(0),
      derive_angle(0),
      tforce(0),
      f_tforce(0),
      l_tforce(0),
      curr_friction(0),
      loc_iters(0),
      mv(0),
      mu(0),
      malpha(0) {
    // initializes type
    type = LNK_WHEEL;

    wheel_rotation = new ChFunction_Const(0);
    wcollision = 0;
    speed_handled = 0;
    radius = 0.1;
    thickness = 0.0;
    friction = 0.7;
    fri_spe = new ChFunction_Const(1);
    fri_norm = new ChFunction_Const(1);
    allow_sticking = FALSE;
    slip_treshold = 0.01;
    static_friction = 1;
    unilateral = TRUE;
    pneus_krp = FALSE;
    rad_k = 1000;
    rad_r = 0.001;
    rad_p = 100000;  // about 1 atm..
    rad_k_def = new ChFunction_Const(1);
    pneus_h = 0.2;

    limit_Z->Set_active(TRUE);
    limit_Z->Set_max(100000000.0);
    limit_Z->Set_min(0.0);

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    ((ChLinkMaskLF*)mask)->SetLockMask(false, false, true, false, false, false, false);
    ((ChLinkMaskLF*)mask)->Constr_Z().SetMode(CONSTRAINT_UNILATERAL);

    ChangedLinkMask();
}

ChLinkWheel::ChLinkWheel(const ChLinkWheel& other) : ChLinkLock(other) {
    wcollision = other.wcollision;
    speed_handled = other.speed_handled;
    radius = other.radius;
    thickness = other.thickness;
    friction = other.friction;
    allow_sticking = other.allow_sticking;
    slip_treshold = other.slip_treshold;
    static_friction = other.static_friction;
    unilateral = other.unilateral;
    pneus_krp = other.pneus_krp;
    rad_k = other.rad_k;
    rad_r = other.rad_r;
    rad_p = other.rad_p;  // about 2 atm..
    pneus_h = other.pneus_h;

    if (wheel_rotation)
        delete wheel_rotation;
    wheel_rotation = other.wheel_rotation->Clone();
    if (fri_spe)
        delete fri_spe;
    fri_spe = other.fri_spe->Clone();
    if (fri_norm)
        delete fri_norm;
    fri_norm = other.fri_norm->Clone();
    if (rad_k_def)
        delete rad_k_def;
    rad_k_def = other.rad_k_def->Clone();

    angle = angle_dt = angle_dtdt = 0;
    slipping = f_slip = l_slip = derive_angle = tforce = f_tforce = l_tforce = curr_friction = 0;
    loc_iters = 0;
    mv = mu = malpha = 0.0;
}

ChLinkWheel::~ChLinkWheel() {
    delete wheel_rotation;
    delete fri_spe;
    delete fri_norm;
}

void ChLinkWheel::Set_radius(double mset) {
    radius = mset;
    limit_Z->Set_min(radius - thickness);
}

void ChLinkWheel::Set_thickness(double mset) {
    thickness = mset;
    limit_Z->Set_min(radius - thickness);
}

void ChLinkWheel::Set_pneus_h(double mset) {
    pneus_h = mset;
    if (pneus_h > 1)
        pneus_h = 1;
}

void ChLinkWheel::Set_unilateral(int mset) {
    unilateral = mset;
    if (unilateral) {
        limit_Z->Set_active(TRUE);
        ((ChLinkMaskLF*)mask)->Constr_Z().SetMode(CONSTRAINT_FREE);
    } else {
        limit_Z->Set_active(FALSE);
        ((ChLinkMaskLF*)mask)->Constr_Z().SetMode(CONSTRAINT_LOCK);
    }
    ChangedLinkMask();
}

void ChLinkWheel::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);

    ChVector<> m2_pos, m1_pos, vx, vy, vz, m1_relacc, m1_relvel;
    Quaternion m2_rot, m1_rot;

    // defaults: if no spindle obj. is provided, marker 1 is used instead
    // spindle_csys.Set_A_quaternion(marker1->GetAbsCoord().rot);
    // spindle_pos = marker1->GetAbsCoord().pos;
    spindle_csys.Set_A_quaternion(Body1->GetCoord().rot);
    spindle_pos = Body1->GetCoord().pos;

    // CONTACT MODE:          (also used to set defaults, and default mode)
    // -- X-Z PLANE ---

    // Default normal
    ChVector<> surf_normal = VECT_Y;
    // Radius vector from spindle to ground (default)
    ChVector<> vgroundrad = Vmul(
        Vcross(spindle_csys.Get_A_Zaxis(), Vnorm(Vcross(spindle_csys.Get_A_Zaxis(), surf_normal))), Get_RigidRadius());

    // Defaults for marker positions and rotations
    m1_pos = Vadd(spindle_pos, vgroundrad);

    ChVector<> vm1z = Vmul(Vnorm(vgroundrad), -1.0);
    ChVector<> vm1x = Vnorm(Vcross(spindle_csys.Get_A_Zaxis(), vm1z));
    ChMatrix33<> a_m1;
    a_m1.Set_A_axis(vm1x, spindle_csys.Get_A_Zaxis(), vm1z);
    m1_rot = Qnorm(a_m1.Get_A_quaternion());

    m2_pos = m1_pos;
    m2_pos.y = 0.0;

    ChMatrix33<> a_m2;
    vz = surf_normal;
    vy = spindle_csys.Get_A_Zaxis();
    vx = Vnorm(Vcross(vy, vz));
    vy = Vnorm(Vcross(vz, vx));
    a_m2.Set_A_axis(vx, vy, vz);
    m2_rot = a_m2.Get_A_quaternion();

    // Compute relative speed of reference M1
    // (hypothesis: ground is flat and steady: contact must move with same vel. as wheel spindle)
    ChVector<> vabsradius = Vsub(m1_pos, spindle_pos);
    m1_relvel = Body1->GetA().MatrT_x_Vect(Vcross(vabsradius, Body1->GetWvel_par()));
    // Compute relative acceleration of reference M1
    // (hypothesis: ground is flat and steady: contact must move with same acc. as wheel spindle)
    m1_relacc = Body1->GetA().MatrT_x_Vect(
        Vcross(Body1->GetWvel_par(), Vcross(vabsradius, Body1->GetWvel_par())));                        // cut centrip
    m1_relacc = Vadd(m1_relacc, Vmul(Vcross(Body1->GetWvel_loc(), m1_relvel), -2.0));                   // cut coriolis
    m1_relacc = Vadd(m1_relacc, Body1->GetA().MatrT_x_Vect(Vcross(vabsradius, Body1->GetWacc_par())));  // cut tang acc

    ///////  CONTACT MODE:
    ///////  -- Y-COLLISION ---
    ///////

    // MOVE "MAIN" MARKER 2 INTO UPDATED POSITION
    //
    Coordsys newmarkpos2;
    newmarkpos2.pos = m2_pos;
    newmarkpos2.rot = m2_rot;
    marker2->Impose_Abs_Coord(newmarkpos2);               // move "main" marker2 into tangent position
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);  // M_MOTION_KEYFRAMED );  // the BDF routine won't handle
                                                          // speed and acc.calculus of the point
    marker2->UpdateState();

    // MOVE "SLAVE" MARKER 1 INTO UPDATED POSITION
    //
    Coordsys newmarkpos1;
    Coordsys relmotioncsys = CSYSNULL;
    newmarkpos1.pos = m1_pos;
    newmarkpos1.rot = m1_rot;
    marker1->Impose_Abs_Coord(newmarkpos1);  // impose position to slave marker
    relmotioncsys.pos = m1_relvel;
    marker1->SetCoord_dt(relmotioncsys);  // impose rel.speed
    relmotioncsys.pos = m1_relacc;
    marker1->SetCoord_dtdt(relmotioncsys);                // impose rel.accel.
    marker1->SetMotionType(ChMarker::M_MOTION_EXTERNAL);  // M_MOTION_KEYFRAMED ); // the BDF routine won't handle speed
                                                          // and acc.calculus of the point
    marker1->UpdateState();

    // THE RELATIVE MARKER POSITION:
    //
    deltaC.pos = VNULL;
    deltaC_dt.pos = VNULL;
    deltaC_dtdt.pos = VNULL;

    deltaC.rot = QUNIT;
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}

void ChLinkWheel::UpdateForces(double mytime) {
    // First, inherit to parent class
    ChLinkLock::UpdateForces(mytime);

    // COMPUTE SLIP FORCES

    ChVector<> mvradius = Vsub(marker2->GetAbsCoord().pos, spindle_pos);

    // point of contact on Body1: absolute speed
    ChVector<> vp1 = Vadd(Body1->GetCoord_dt().pos, Body1->GetA_dt().Matr_x_Vect(marker1->GetCoord().pos));

    // point of contact on Body2: absolute speed
    ChVector<> vp2 = Vadd(Body2->GetCoord_dt().pos, Body2->GetA_dt().Matr_x_Vect(marker2->GetCoord().pos));

    ChVector<> mabs_slip = Vsub(vp1, vp2);

    ChMatrix33<> mtra1;
    mtra1.Set_A_quaternion(marker1->GetAbsCoord().rot);
    ChMatrix33<> mtra2;
    mtra2.Set_A_quaternion(marker2->GetAbsCoord().rot);

    // skip further computations if wheel do not touch ground
    //
    if (Vlength(mvradius) > radius * 1.01) {
        slipping = f_slip = l_slip = derive_angle = tforce = f_tforce = l_tforce = curr_friction = 0;
        return;  // <<<<<
    }

    // effect of imposed rotation:
    //
    if (speed_handled == 1)  // set artificial speed
    {
        angle = wheel_rotation->Get_y(ChTime);
        angle_dt = wheel_rotation->Get_y_dx(ChTime);
        angle_dtdt = wheel_rotation->Get_y_dxdx(ChTime);
    }
    if (speed_handled == 2)  // set artificial speed (function already represents dy/dx mode)
    {
        angle = 0;  // ***TO DO*** wheel_rotation->Get_y_Ix(ChTime, initial_rot);
        angle_dt = wheel_rotation->Get_y(ChTime);
        angle_dtdt = wheel_rotation->Get_y_dx(ChTime);
    }
    if (speed_handled)
        mabs_slip = Vadd(mabs_slip,                                          // s = s + w x R
                         Vcross(Vmul(spindle_csys.Get_A_Zaxis(), angle_dt),  // w
                                mvradius)                                    // R
                         );

    ChVector<> mrel_slip2 = mtra2.MatrT_x_Vect(mabs_slip);
    // ChVector<> mrel_slip1 = mtra1.MatrT_x_Vect(mabs_slip);

    slipping = Vlength(mabs_slip);

    l_slip = mrel_slip2.y;
    f_slip = mrel_slip2.x;

    double norm_force = 0.0;
    if (react)
        norm_force += GetReact()->GetElement(0, 0);  // N = constraint reaction
    // if (norm_force >0.01)
    //  R3Error("pos force %g", norm_force);

    curr_friction = friction * fri_spe->Get_y(slipping) * fri_norm->Get_y(norm_force);

    tforce = curr_friction * norm_force;  // Ft = u() *Fn
    tforce = fabs(tforce);

    ChVector<> m_force2 = Vmul(Vnorm(mrel_slip2), (tforce * -1.0));
    m_force2.z = 0;  // Ft_z should be already near zero, in m2 csys, but imposed for safety.

    ChVector<> mabs_slipforce = mtra2.Matr_x_Vect(m_force2);
    // ChVector<> mabs_slipforce = Vmul( Vnorm(mabs_slip), (tforce * -1.0) );

    l_tforce = m_force2.y;
    f_tforce = m_force2.x;

    /* ***should use only C_force and C_torque*** .... Qf1 and Qf2 are deprecated ... TO DO
        Body1->Add_as_lagrangian_force(
            mabs_slipforce,         // Force in abs space! (as seen from Body1)
            marker2->GetAbsCoord().pos,  // application point: marker 2 (the ground contact point)
            FALSE,                  // reference: absolute
            Qf1);                   // store += the resulting lagrangian force/torque;

        Body2->Add_as_lagrangian_force(
            Vmul(mabs_slipforce, -1.0), // Force in abs space! (as seen from Body2, F2=-F1)
            marker2->GetAbsCoord().pos,  // application point: marker 2 (the ground contact point)
            FALSE,                  // reference: absolute
            Qf2);                   // store += the resulting lagrangian force/torque;
    */
}

void ChLinkWheel::Set_wheel_rotation(ChFunction* m_funct) {
    if (wheel_rotation)
        delete wheel_rotation;
    wheel_rotation = m_funct;
}
void ChLinkWheel::Set_fri_spe(ChFunction* m_funct) {
    if (fri_spe)
        delete fri_spe;
    fri_spe = m_funct;
}
void ChLinkWheel::Set_fri_norm(ChFunction* m_funct) {
    if (fri_norm)
        delete fri_norm;
    fri_norm = m_funct;
}
void ChLinkWheel::Set_rad_k_def(ChFunction* m_funct) {
    if (rad_k_def)
        delete rad_k_def;
    rad_k_def = m_funct;
}

// FILE I/O
//

void ChLinkWheel::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLinkLock::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(speed_handled);
    marchive << CHNVP(radius);
    marchive << CHNVP(thickness);
    marchive << CHNVP(friction);
    marchive << CHNVP(allow_sticking);
    marchive << CHNVP(slip_treshold);
    marchive << CHNVP(static_friction);
    marchive << CHNVP(unilateral);
    marchive << CHNVP(pneus_krp);
    marchive << CHNVP(rad_k);
    marchive << CHNVP(rad_r);
    marchive << CHNVP(rad_p);
    marchive << CHNVP(pneus_h);
    marchive << CHNVP(wheel_rotation);
    marchive << CHNVP(fri_spe);
    marchive << CHNVP(fri_norm);
    marchive << CHNVP(rad_k_def);
    marchive << CHNVP(wcollision);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkWheel::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLinkLock::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(speed_handled);
    marchive >> CHNVP(radius);
    marchive >> CHNVP(thickness);
    marchive >> CHNVP(friction);
    marchive >> CHNVP(allow_sticking);
    marchive >> CHNVP(slip_treshold);
    marchive >> CHNVP(static_friction);
    marchive >> CHNVP(unilateral);
    marchive >> CHNVP(pneus_krp);
    marchive >> CHNVP(rad_k);
    marchive >> CHNVP(rad_r);
    marchive >> CHNVP(rad_p);
    marchive >> CHNVP(pneus_h);
    marchive >> CHNVP(wheel_rotation);
    marchive >> CHNVP(fri_spe);
    marchive >> CHNVP(fri_norm);
    marchive >> CHNVP(rad_k_def);
    marchive >> CHNVP(wcollision);
}

}  // end namespace chrono
