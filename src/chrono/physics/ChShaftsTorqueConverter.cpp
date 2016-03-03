//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "physics/ChShaftsTorqueConverter.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsTorqueConverter> a_registration_ChShaftsTorqueConverter;

//////////////////////////////////////
//////////////////////////////////////

ChShaftsTorqueConverter::ChShaftsTorqueConverter() {
    this->shaft1 = 0;
    this->shaft2 = 0;
    this->shaft_stator = 0;
    this->torque_in = 0;
    this->torque_out = 0;
    this->K = std::make_shared<ChFunction_Const>(0.9);
    this->T = std::make_shared<ChFunction_Const>(0.9);
    this->state_warning_reverseflow = false;
    this->state_warning_wrongimpellerdirection = false;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

ChShaftsTorqueConverter::~ChShaftsTorqueConverter() {
}

void ChShaftsTorqueConverter::Copy(ChShaftsTorqueConverter* source) {
    // copy the parent class data...
    ChPhysicsItem::Copy(source);

    // copy class data
    shaft1 = 0;
    shaft2 = 0;
    shaft_stator = 0;

    torque_in = source->torque_in;
    torque_out = source->torque_out;

    state_warning_reverseflow = source->state_warning_reverseflow;
    state_warning_wrongimpellerdirection = source->state_warning_wrongimpellerdirection;

    this->K = std::shared_ptr<ChFunction>(source->K->new_Duplicate());  // deep copy
    this->T = std::shared_ptr<ChFunction>(source->T->new_Duplicate());  // deep copy
}

int ChShaftsTorqueConverter::Initialize(std::shared_ptr<ChShaft> mshaft1,
                                        std::shared_ptr<ChShaft> mshaft2,
                                        std::shared_ptr<ChShaft> mshaft_stator) {
    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();
    ChShaft* mm_stator = mshaft_stator.get();
    assert(mm1 && mm2 && mm_stator);
    assert((mm1 != mm2) && (mm1 != mm_stator));
    assert((mm1->GetSystem() == mm2->GetSystem()) && (mm1->GetSystem() == mm_stator->GetSystem()));

    this->shaft1 = mm1;
    this->shaft2 = mm2;
    this->shaft_stator = mm_stator;

    this->SetSystem(this->shaft1->GetSystem());

    return true;
}

double ChShaftsTorqueConverter::GetSpeedRatio() const {
    double wrel1 = this->shaft1->GetPos_dt() - this->shaft_stator->GetPos_dt();
    double wrel2 = this->shaft2->GetPos_dt() - this->shaft_stator->GetPos_dt();

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
    double mR = this->GetSpeedRatio();

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
    if (this->shaft1->GetPos_dt() - this->shaft_stator->GetPos_dt() < 0) {
        state_warning_wrongimpellerdirection = true;
        this->torque_in = 0;
        this->torque_out = 0;
        return;
    }

    // Compute actual capacity factor
    double mK = K->Get_y(mR);

    // Compute actual torque factor
    double mT = T->Get_y(mR);

    // compute input torque (with minus sign because applied TO input thaft)
    this->torque_in = -pow((this->shaft1->GetPos_dt() / mK), 2);

    if (state_warning_reverseflow)
        this->torque_in = -this->torque_in;

    // compute output torque (with opposite sign because
    // applied to output shaft, with same direction of input shaft)
    this->torque_out = -mT * this->torque_in;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsTorqueConverter::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                                ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                                const double c           ///< a scaling factor
                                                ) {
    if (shaft1->IsActive())
        R(shaft1->GetOffset_w()) += this->torque_in * c;
    if (shaft2->IsActive())
        R(shaft2->GetOffset_w()) += this->torque_out * c;
    if (shaft_stator->IsActive())
        R(shaft_stator->GetOffset_w()) += GetTorqueReactionOnStator() * c;
}

////////// LCP INTERFACES ////

void ChShaftsTorqueConverter::VariablesFbLoadForces(double factor) {
    // Apply torques to the three connected 1D variables:
    shaft1->Variables().Get_fb().ElementN(0) += torque_in * factor;
    shaft2->Variables().Get_fb().ElementN(0) += torque_out * factor;
    shaft_stator->Variables().Get_fb().ElementN(0) += GetTorqueReactionOnStator() * factor;
}

//////// FILE I/O


void ChShaftsTorqueConverter::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(K);
    marchive << CHNVP(T);
    //marchive << CHNVP(shaft1); //***TODO*** serialize with shared ptr
    //marchive << CHNVP(shaft2); //***TODO*** serialize with shared ptr
    //marchive << CHNVP(shaft_stator); //***TODO*** serialize with shared ptr
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorqueConverter::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(K);
    marchive >> CHNVP(T);
    //marchive >> CHNVP(shaft1); //***TODO*** serialize with shared ptr
    //marchive >> CHNVP(shaft2); //***TODO*** serialize with shared ptr
    //marchive >> CHNVP(shaft_stator); //***TODO*** serialize with shared ptr
} 


}  // END_OF_NAMESPACE____

/////////////////////
