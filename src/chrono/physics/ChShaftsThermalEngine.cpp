//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "physics/ChShaftsThermalEngine.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsThermalEngine> a_registration_ChShaftsThermalEngine;

//////////////////////////////////////
//////////////////////////////////////

ChShaftsThermalEngine::ChShaftsThermalEngine() {
    this->throttle = 1;
    this->error_backward = false;

    // default torque curve= constant zero. User will provide better fx.
    this->Tw = std::make_shared<ChFunction_Const>(0);

    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

ChShaftsThermalEngine::~ChShaftsThermalEngine() {
}

void ChShaftsThermalEngine::Copy(ChShaftsThermalEngine* source) {
    // copy the parent class data...
    ChShaftsTorqueBase::Copy(source);

    // copy class data
    throttle = source->throttle;
    error_backward = source->error_backward;
    this->Tw = std::shared_ptr<ChFunction>(source->Tw->new_Duplicate());  // deep copy
}

double ChShaftsThermalEngine::ComputeTorque() {
    // COMPUTE THE TORQUE HERE!
    double mw = this->GetRelativeRotation_dt();

    if (mw < 0)
        this->error_backward = true;
    else
        this->error_backward = false;

    // get the actual torque from torque curve
    double mT = Tw->Get_y(mw);

    // modulate it with throttle
    double modulated_T = mT * this->throttle;

    return modulated_T;
}

//////// FILE I/O



void ChShaftsThermalEngine::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChShaftsTorqueBase::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(Tw);
    marchive << CHNVP(throttle);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsThermalEngine::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChShaftsTorqueBase::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(Tw);
    marchive >> CHNVP(throttle);
} 



}  // END_OF_NAMESPACE____

/////////////////////
