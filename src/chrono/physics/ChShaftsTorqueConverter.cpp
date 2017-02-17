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

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsTorqueConverter.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsTorqueConverter)

ChShaftsTorqueConverter::ChShaftsTorqueConverter()
    : shaft1(NULL),
      shaft2(NULL),
      shaft_stator(NULL),
      torque_in(0),
      torque_out(0),
      state_warning_reverseflow(false),
      state_warning_wrongimpellerdirection(false) {
    K = std::make_shared<ChFunction_Const>(0.9);
    T = std::make_shared<ChFunction_Const>(0.9);
}

ChShaftsTorqueConverter::ChShaftsTorqueConverter(const ChShaftsTorqueConverter& other) : ChPhysicsItem(other) {
    shaft1 = NULL;
    shaft2 = NULL;
    shaft_stator = NULL;

    torque_in = other.torque_in;
    torque_out = other.torque_out;

    state_warning_reverseflow = other.state_warning_reverseflow;
    state_warning_wrongimpellerdirection = other.state_warning_wrongimpellerdirection;

    K = std::shared_ptr<ChFunction>(other.K->Clone());  // deep copy
    T = std::shared_ptr<ChFunction>(other.T->Clone());  // deep copy
}

bool ChShaftsTorqueConverter::Initialize(std::shared_ptr<ChShaft> mshaft1,       // input shaft
                                         std::shared_ptr<ChShaft> mshaft2,       // output shaft
                                         std::shared_ptr<ChShaft> mshaft_stator  // stator shaft (often fixed)
                                         ) {
    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();
    ChShaft* mm_stator = mshaft_stator.get();
    assert(mm1 && mm2 && mm_stator);
    assert((mm1 != mm2) && (mm1 != mm_stator));
    assert((mm1->GetSystem() == mm2->GetSystem()) && (mm1->GetSystem() == mm_stator->GetSystem()));

    shaft1 = mm1;
    shaft2 = mm2;
    shaft_stator = mm_stator;

    SetSystem(shaft1->GetSystem());

    return true;
}

double ChShaftsTorqueConverter::GetSpeedRatio() const {
    double wrel1 = shaft1->GetPos_dt() - shaft_stator->GetPos_dt();
    double wrel2 = shaft2->GetPos_dt() - shaft_stator->GetPos_dt();

    if ((fabs(wrel1) < 10e-9) || (fabs(wrel2) < 10e-9))
        return 0;

    return wrel2 / wrel1;
}

void ChShaftsTorqueConverter::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data

    state_warning_wrongimpellerdirection = false;
    state_warning_reverseflow = false;

    // Compute actual speed ratio
    double mR = GetSpeedRatio();

    // The speed ratio must always be in the [0...1] range,
    // anyway let's correct singular cases:

    // - it should be inusual that speed ratio >1, say if the impeller
    //   outruns turbine (if there's a clutch lock-in, this should happen).
    //   If so, assume a reflection of T curve and a polar reflection of K curve, after 1.
    if (mR > 1) {
        mR = 1 - (mR - 1);
        state_warning_reverseflow = true;
    }

    // - if the output shaft is spinning a bit backward, when
    //   close to stall, maybe win/wout < 0. If so, set as stall anyway:
    if (mR < 0)
        mR = 0;

    // - if input impeller shaft is spinning in negative direction,
    //   this is assumed as an error: set all torques to zero and bail out:
    if (shaft1->GetPos_dt() - shaft_stator->GetPos_dt() < 0) {
        state_warning_wrongimpellerdirection = true;
        torque_in = 0;
        torque_out = 0;
        return;
    }

    // Compute actual capacity factor
    double mK = K->Get_y(mR);

    // Compute actual torque factor
    double mT = T->Get_y(mR);

    // compute input torque (with minus sign because applied TO input thaft)
    torque_in = -pow((shaft1->GetPos_dt() / mK), 2);

    if (state_warning_reverseflow)
        torque_in = -torque_in;

    // compute output torque (with opposite sign because
    // applied to output shaft, with same direction of input shaft)
    torque_out = -mT * torque_in;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsTorqueConverter::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                                ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                                const double c           // a scaling factor
                                                ) {
    if (shaft1->IsActive())
        R(shaft1->GetOffset_w()) += torque_in * c;
    if (shaft2->IsActive())
        R(shaft2->GetOffset_w()) += torque_out * c;
    if (shaft_stator->IsActive())
        R(shaft_stator->GetOffset_w()) += GetTorqueReactionOnStator() * c;
}

// SOLVER INTERFACES

void ChShaftsTorqueConverter::VariablesFbLoadForces(double factor) {
    // Apply torques to the three connected 1D variables:
    shaft1->Variables().Get_fb().ElementN(0) += torque_in * factor;
    shaft2->Variables().Get_fb().ElementN(0) += torque_out * factor;
    shaft_stator->Variables().Get_fb().ElementN(0) += GetTorqueReactionOnStator() * factor;
}

// FILE I/O

void ChShaftsTorqueConverter::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsTorqueConverter>();

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(K);
    marchive << CHNVP(T);
    // marchive << CHNVP(shaft1); //***TODO*** serialize with shared ptr
    // marchive << CHNVP(shaft2); //***TODO*** serialize with shared ptr
    // marchive << CHNVP(shaft_stator); //***TODO*** serialize with shared ptr
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorqueConverter::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsTorqueConverter>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(K);
    marchive >> CHNVP(T);
    // marchive >> CHNVP(shaft1); //***TODO*** serialize with shared ptr
    // marchive >> CHNVP(shaft2); //***TODO*** serialize with shared ptr
    // marchive >> CHNVP(shaft_stator); //***TODO*** serialize with shared ptr
}

}  // end namespace chrono