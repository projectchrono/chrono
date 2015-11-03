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

#include "physics/ChShaftsTorsionSpring.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsTorsionSpring> a_registration_ChShaftsTorsionSpring;

//////////////////////////////////////
//////////////////////////////////////

ChShaftsTorsionSpring::ChShaftsTorsionSpring() {
    this->stiffness = 0;
    this->damping = 0;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

ChShaftsTorsionSpring::~ChShaftsTorsionSpring() {
}

void ChShaftsTorsionSpring::Copy(ChShaftsTorsionSpring* source) {
    // copy the parent class data...
    ChShaftsTorqueBase::Copy(source);

    // copy class data
    stiffness = source->stiffness;
    damping = source->damping;
}

double ChShaftsTorsionSpring::ComputeTorque() {
    // COMPUTE THE TORQUE HERE!
    return -(this->GetRelativeRotation() * this->stiffness     // the torsional spring term
             + this->GetRelativeRotation_dt() * this->damping  // the torsional damper term
             );
}

//////// FILE I/O

void ChShaftsTorsionSpring::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChShaftsTorqueBase::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(stiffness);
    marchive << CHNVP(damping);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorsionSpring::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChShaftsTorqueBase::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(stiffness);
    marchive >> CHNVP(damping);
} 


}  // END_OF_NAMESPACE____

/////////////////////
