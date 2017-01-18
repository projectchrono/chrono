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

#include "chrono/physics/ChLinkPneumaticActuator.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPneumaticActuator)

ChLinkPneumaticActuator::ChLinkPneumaticActuator() {
    type = LNK_PNEUMATIC;  // initializes type

    pneuma = new pneumatics::AssePneumatico();
    offset = pneuma->Get_L() + 0.1;

    pA = pB = pneuma->Get_Ps();  // default state (initial chamber pressure = ambient pressure)
    pA_dt = pB_dt = 0;
    pneu_F = 0;
    last_force_time = 0;

    // Mask: initialize our LinkMaskLF (lock formulation mask)
    // to free. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false, false, false, false, false);

    // set upper lower limits, active
    limit_X->Set_active(true);
    limit_X->Set_min(offset);  // 0.0);
    limit_X->Set_max(pneuma->Get_L() + offset);
    limit_X->Set_maxElastic(0.0);
    limit_X->Set_minElastic(0.0);

    ChangedLinkMask();
}

ChLinkPneumaticActuator::ChLinkPneumaticActuator(const ChLinkPneumaticActuator& other) : ChLinkLock(other) {
    pneuma->Set_Ci(other.pneuma->Get_Ci());
    pneuma->Set_Co(other.pneuma->Get_Co());
    pneuma->Set_Bi(other.pneuma->Get_Bi());
    pneuma->Set_Bo(other.pneuma->Get_Bo());
    pneuma->Set_Ps(other.pneuma->Get_Ps());
    pneuma->Set_Pma(other.pneuma->Get_Pma());
    pneuma->Set_Pmb(other.pneuma->Get_Pmb());
    pneuma->Set_L(other.pneuma->Get_L());
    pneuma->Set_Wa(other.pneuma->Get_Wa());
    pneuma->Set_Wb(other.pneuma->Get_Wb());
    pneuma->Set_A(other.pneuma->Get_A());
    pneuma->Set_Alfa(other.pneuma->Get_Alfa());
    pneuma->Set_Gamma(other.pneuma->Get_Gamma());
    pneuma->Set_ValvA_min(other.pneuma->Get_ValvA_min());
    pneuma->Set_ValvA_max(other.pneuma->Get_ValvA_max());
    pneuma->Set_ValvA_close(other.pneuma->Get_ValvA_close());
    pneuma->Set_ValvB_min(other.pneuma->Get_ValvB_min());
    pneuma->Set_ValvB_max(other.pneuma->Get_ValvB_max());
    pneuma->Set_ValvB_close(other.pneuma->Get_ValvB_close());
    pneuma->SetupAssePneumatico();  // setup into sub objects
    offset = other.offset;
    pA = other.pA;
    pB = other.pB;
    pA_dt = other.pA_dt;
    pB_dt = other.pB_dt;
    pneuma->Set_P(other.pA, other.pB);  // this also copies state into internal structures
    pneuma->Set_Pos(other.Get_pneu_pos(), other.Get_pneu_pos_dt());
    pneu_F = other.pneu_F;
    last_force_time = other.last_force_time;
}

ChLinkPneumaticActuator::~ChLinkPneumaticActuator() {
    if (pneuma)
        delete pneuma;
    pneuma = NULL;
}

void ChLinkPneumaticActuator::Set_lin_offset(double mset) {
    offset = mset;
    limit_X->Set_min(offset);
    limit_X->Set_max(pneuma->Get_L() + offset);
}

void ChLinkPneumaticActuator::Set_pneu_L(double mset) {
    pneuma->Set_L(mset);
    limit_X->Set_max(pneuma->Get_L() + offset);
}

void ChLinkPneumaticActuator::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);

    // Move (well, rotate...) marker 2 to align it in actuator direction

    // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

    ChMatrix33<> ma;
    ma.Set_A_quaternion(marker2->GetAbsCoord().rot);

    Vector absdist = Vsub(marker1->GetAbsCoord().pos, marker2->GetAbsCoord().pos);

    Vector mx = Vnorm(absdist);

    Vector my = ma.Get_A_Yaxis();
    if (Vequal(mx, my)) {
        if (mx.x == 1.0) {
            my = VECT_Y;
        } else {
            my = VECT_X;
        }
    }
    Vector mz = Vnorm(Vcross(mx, my));
    my = Vnorm(Vcross(mz, mx));

    ma.Set_A_axis(mx, my, mz);

    Coordsys newmarkpos;
    newmarkpos.pos = marker2->GetAbsCoord().pos;
    newmarkpos.rot = ma.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);  // rotate "main" marker2 into tangent position
}

void ChLinkPneumaticActuator::UpdateForces(double mytime) {
    // First, inherit to parent class
    ChLinkLock::UpdateForces(mytime);

    // DEFAULTS set null forces
    pneu_F = 0;

    // COMPUTE PNEUMATIC FORCE!!

    // 1a - set current state (pressure A and B)
    pneuma->Set_P(pA, pB);
    // 1b - set current state (position, speed)
    pneuma->Set_Pos(Get_pneu_pos(), Get_pneu_pos_dt());

    // 2- compute new force for this state
    pneuma->Update();  // needed, so that F is computed in pneuma*
    pneu_F = pneuma->Get_F();

    // Security clamping on plausible limit, to avoid divergence
    // if (pneu_F > 100000) pneu_F = 100000;

    // 3- compute new pressures by 'local' integration, from previous
    //    values of pressures.
    pneuma->Get_P_dt(&pA_dt, &pB_dt);

    double mforce_timestep = mytime - last_force_time;
    if ((mforce_timestep < 0.1) && (mforce_timestep > 0)) {
        pA = pA + mforce_timestep * pA_dt;
        pB = pB + mforce_timestep * pB_dt;

        if (pA < 0)
            pA = 0;
        if (pB < 0)
            pB = 0;

        pneuma->Set_P(pA, pB);
    }

    last_force_time = mytime;

    // +++ADD PNEUMATIC FORCE TO LINK INTERNAL FORCE VECTOR (on x axis only)

    C_force.x = C_force.x + pneu_F;
}

void ChLinkPneumaticActuator::ArchiveOUT(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkPneumaticActuator::ArchiveIN(ChArchiveIn& marchive) {
    //// TODO
}

}  // end namespace chrono
