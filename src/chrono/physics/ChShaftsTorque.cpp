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

#include "physics/ChShaftsTorque.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaftsTorque> a_registration_ChShaftsTorque;

//////////////////////////////////////
//////////////////////////////////////

ChShaftsTorque::ChShaftsTorque() {
    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

ChShaftsTorque::~ChShaftsTorque() {
}

void ChShaftsTorque::Copy(ChShaftsTorque* source) {
    // copy the parent class data...
    ChShaftsTorqueBase::Copy(source);
}

double ChShaftsTorque::ComputeTorque() {
    // COMPUTE THE TORQUE HERE!
    // Nothing, just leave the user-set   this->torque
    return this->torque;
}

//////// FILE I/O

void ChShaftsTorque::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChShaftsTorqueBase::ArchiveOUT(marchive);

    // serialize all member data:

}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorque::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChShaftsTorqueBase::ArchiveIN(marchive);

    // deserialize all member data:

} 


}  // END_OF_NAMESPACE____

/////////////////////
