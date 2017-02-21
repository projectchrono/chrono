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

#include "chrono/physics/ChLinkLinActuator.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkLinActuator)

ChLinkLinActuator::ChLinkLinActuator()
    : learn(false),
      learn_torque_rotation(true),
      offset(0.1),
      mot_tau(1),
      mot_eta(1),
      mot_inertia(0),
      mot_rerot(0),
      mot_rerot_dt(0),
      mot_rerot_dtdt(0) {
    dist_funct = std::make_shared<ChFunction_Const>(0);
    mot_torque = std::make_shared<ChFunction_Recorder>();
    mot_rot = std::make_shared<ChFunction_Recorder>();

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(true, false, false, false, false, false, false);

    ChangedLinkMask();

    mot_rerot = mot_rerot_dt = mot_rerot_dtdt = 0;
}

ChLinkLinActuator::ChLinkLinActuator(const ChLinkLinActuator& other) : ChLinkLock(other) {
    learn = other.learn;
    learn_torque_rotation = other.learn_torque_rotation;
    offset = other.offset;

    dist_funct = std::shared_ptr<ChFunction>(other.dist_funct->Clone());
    mot_torque = std::shared_ptr<ChFunction>(other.mot_torque->Clone());
    mot_rot = std::shared_ptr<ChFunction>(other.mot_rot->Clone());

    mot_tau = other.mot_tau;
    mot_eta = other.mot_eta;
    mot_inertia = other.mot_inertia;
}

void ChLinkLinActuator::Set_learn(bool mset) {
    if (mset) {
        SetDisabled(true);  // ..just to show it as a green wireframe...
        Set_learn_torque_rotaton(false);
    } else {
        SetDisabled(false);
    }

    if (mset)
        ((ChLinkMaskLF*)mask)->Constr_X().SetMode(CONSTRAINT_FREE);
    else
        ((ChLinkMaskLF*)mask)->Constr_X().SetMode(CONSTRAINT_LOCK);

    ChangedLinkMask();

    learn = mset;
    if (dist_funct->Get_Type() != ChFunction::FUNCT_RECORDER)
        dist_funct = std::make_shared<ChFunction_Recorder>();
}

void ChLinkLinActuator::Set_learn_torque_rotaton(bool mset) {
    learn_torque_rotation = mset;
    if (mot_torque->Get_Type() != ChFunction::FUNCT_RECORDER)
        mot_torque = std::make_shared<ChFunction_Recorder>();

    if (mot_rot->Get_Type() != ChFunction::FUNCT_RECORDER)
        mot_rot = std::make_shared<ChFunction_Recorder>();
}

void ChLinkLinActuator::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);

    // If LEARN MODE, just record motion
    if (learn) {
        /*   do not change deltas, in free mode maybe that 'limit on X' changed them
        deltaC.pos = VNULL;
        deltaC_dt.pos = VNULL;
        deltaC_dtdt.pos = VNULL;
        deltaC.rot = QUNIT;
        deltaC_dt.rot = QNULL;
        deltaC_dtdt.rot = QNULL;
        */
        if (dist_funct->Get_Type() != ChFunction::FUNCT_RECORDER)
            dist_funct = std::make_shared<ChFunction_Recorder>();

        // record point
        double rec_dist = Vlength(Vsub(marker1->GetAbsCoord().pos, marker2->GetAbsCoord().pos));
        rec_dist -= offset;
        std::static_pointer_cast<ChFunction_Recorder>(dist_funct)->AddPoint(mytime, rec_dist, 1);  // (x,y,w)  x=t
    }

    // Move (well, rotate...) marker 2 to align it in actuator direction

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

    ChMatrix33<> ma;
    ma.Set_A_quaternion(marker2->GetAbsCoord().rot);

    Vector absdist = Vsub(marker1->GetAbsCoord().pos, marker2->GetAbsCoord().pos);

    Vector mx = Vnorm(absdist);

    Vector my = ma.Get_A_Yaxis();
    if (Vequal(mx, my)) {
        if (mx.x() == 1.0)
            my = VECT_Y;
        else
            my = VECT_X;
    }
    Vector mz = Vnorm(Vcross(mx, my));
    my = Vnorm(Vcross(mz, mx));

    ma.Set_A_axis(mx, my, mz);

    Coordsys newmarkpos;
    ChVector<> oldpos = marker2->GetPos();  // backup to avoid numerical err.accumulation
    newmarkpos.pos = marker2->GetAbsCoord().pos;
    newmarkpos.rot = ma.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);  // rotate "main" marker2 into tangent position (may add err.accumulation)
    marker2->SetPos(oldpos);                // backup to avoid numerical err.accumulation

    if (learn)
        return;  // no need to go on further...--->>>>

    // imposed relative positions/speeds
    deltaC.pos = VNULL;
    deltaC.pos.x() = dist_funct->Get_y(ChTime) + offset;  // distance is always on M2 'X' axis

    deltaC_dt.pos = VNULL;
    deltaC_dt.pos.x() = dist_funct->Get_y_dx(ChTime);  // distance speed

    deltaC_dtdt.pos = VNULL;
    deltaC_dtdt.pos.x() = dist_funct->Get_y_dxdx(ChTime);  // distance acceleration
    // add also the centripetal acceleration if distance vector's rotating,
    // as centripetal acc. of point sliding on a sphere surface.
    Vector tang_speed = GetRelM_dt().pos;
    tang_speed.x() = 0;                       // only z-y coords in relative tang speed vector
    double len_absdist = Vlength(absdist);  // don't divide by zero
    if (len_absdist > 1E-6)
        deltaC_dtdt.pos.x() -= pow(Vlength(tang_speed), 2) / Vlength(absdist);  // An = Adelta -(Vt^2 / r)

    deltaC.rot = QUNIT;  // no relative rotations imposed!
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;

    // Compute motor variables
    // double m_rotation;
    // double m_torque;
    mot_rerot = (deltaC.pos.x() - offset) / mot_tau;
    mot_rerot_dt = deltaC_dt.pos.x() / mot_tau;
    mot_rerot_dtdt = deltaC_dtdt.pos.x() / mot_tau;
    mot_retorque = mot_rerot_dtdt * mot_inertia + (react_force.x() * mot_tau) / mot_eta;
    //  m_rotation = (deltaC.pos.x() - offset) / mot_tau;
    //  m_torque =  (deltaC_dtdt.pos.x() / mot_tau) * mot_inertia + (react_force.x() * mot_tau) / mot_eta;

    if (learn_torque_rotation) {
        if (mot_torque->Get_Type() != ChFunction::FUNCT_RECORDER)
            mot_torque = std::make_shared<ChFunction_Recorder>();

        if (mot_rot->Get_Type() != ChFunction::FUNCT_RECORDER)
            mot_rot = std::make_shared<ChFunction_Recorder>();

        std::static_pointer_cast<ChFunction_Recorder>(mot_torque)->AddPoint(mytime, mot_retorque, 1);  // (x,y,w)  x=t
        std::static_pointer_cast<ChFunction_Recorder>(mot_rot)->AddPoint(mytime, mot_rerot, 1);        // (x,y,w)  x=t
    }
}

void ChLinkLinActuator::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkLinActuator>();

    // serialize parent class
    ChLinkLock::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(learn);
    marchive << CHNVP(learn_torque_rotation);
    marchive << CHNVP(offset);
    marchive << CHNVP(dist_funct);
    marchive << CHNVP(mot_tau);
    marchive << CHNVP(mot_eta);
    marchive << CHNVP(mot_inertia);
    marchive << CHNVP(mot_rot);
    marchive << CHNVP(mot_torque);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkLinActuator::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkLinActuator>();

    // deserialize parent class
    ChLinkLock::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(learn);
    marchive >> CHNVP(learn_torque_rotation);
    marchive >> CHNVP(offset);
    marchive >> CHNVP(dist_funct);
    marchive >> CHNVP(mot_tau);
    marchive >> CHNVP(mot_eta);
    marchive >> CHNVP(mot_inertia);
    marchive >> CHNVP(mot_rot);
    marchive >> CHNVP(mot_torque);
}

}  // end namespace chrono
