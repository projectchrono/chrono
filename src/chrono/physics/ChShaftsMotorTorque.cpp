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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChShaftsMotorTorque.h"

namespace chrono {


// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsMotorTorque)

ChShaftsMotorTorque::ChShaftsMotorTorque()  {

   this->f_torque = chrono_types::make_shared<ChFunction_Const>(0.0);
}

ChShaftsMotorTorque::ChShaftsMotorTorque(const ChShaftsMotorTorque& other) : ChShaftsMotorBase(other) {

    this->f_torque = other.f_torque;
}


void ChShaftsMotorTorque::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsMotorBase::Update(mytime, update_assets);

    // update class data

    this->f_torque->Update(mytime); // call callbacks if any

}

//// STATE BOOKKEEPING FUNCTIONS


void ChShaftsMotorTorque::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                const double c           // a scaling factor
                                ) {
    double imposed_torque = this->f_torque->Get_y(this->GetChTime());
    if (shaft1->IsActive())
        R(shaft1->GetOffset_w()) +=  imposed_torque * c;
    if (shaft2->IsActive())
        R(shaft2->GetOffset_w()) += -imposed_torque * c;
}

void ChShaftsMotorTorque::VariablesFbLoadForces(double factor) {

    double imposed_torque = this->f_torque->Get_y(this->GetChTime());
    shaft1->Variables().Get_fb()(0) +=  imposed_torque * factor;
    shaft2->Variables().Get_fb()(0) += -imposed_torque * factor;
}

//////// FILE I/O

void ChShaftsMotorTorque::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsMotorTorque>();

    // serialize parent class
    ChShaftsMotorBase::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(f_torque);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsMotorTorque::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChShaftsMotorTorque>();

    // deserialize parent class:
    ChShaftsMotorBase::ArchiveIn(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_torque);
}





}  // end namespace chrono
