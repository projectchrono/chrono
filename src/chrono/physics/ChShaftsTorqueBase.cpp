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

#include "physics/ChShaftsTorqueBase.h"
#include "physics/ChSystem.h"
#include "physics/ChShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChShaftsTorqueBase> a_registration_ChShaftsTorqueBase;

//////////////////////////////////////
//////////////////////////////////////

ChShaftsTorqueBase::ChShaftsTorqueBase() {
    this->torque = 0;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID
}

ChShaftsTorqueBase::~ChShaftsTorqueBase() {
}

void ChShaftsTorqueBase::Copy(ChShaftsTorqueBase* source) {
    // copy the parent class data...
    ChShaftsCouple::Copy(source);

    // copy class data
    torque = source->torque;
}

void ChShaftsTorqueBase::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(mytime, update_assets);

    // update class data
    this->torque = ComputeTorque();
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsTorqueBase::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                           ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                           const double c           ///< a scaling factor
                                           ) {
    if (shaft1->IsActive())
        R(shaft1->GetOffset_w()) += this->torque * c;
    if (shaft2->IsActive())
        R(shaft2->GetOffset_w()) += -this->torque * c;
}

////////// LCP INTERFACES ////

void ChShaftsTorqueBase::VariablesFbLoadForces(double factor) {
    // add applied torques to 'fb' vector
    this->shaft1->Variables().Get_fb().ElementN(0) += this->torque * factor;
    this->shaft2->Variables().Get_fb().ElementN(0) += -this->torque * factor;
}

//////// FILE I/O

void ChShaftsTorqueBase::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChShaftsCouple::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(torque);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsTorqueBase::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(torque);
} 

}  // END_OF_NAMESPACE____

/////////////////////
