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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChShaftsMotorLoad.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsMotorLoad)

ChShaftsMotorLoad::ChShaftsMotorLoad() {
    motor_function = chrono_types::make_shared<ChFunctionConst>(0.0);
}

ChShaftsMotorLoad::ChShaftsMotorLoad(const ChShaftsMotorLoad& other) : ChShaftsMotor(other) {
    motor_function = other.motor_function;
}

void ChShaftsMotorLoad::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsMotor::Update(time, update_assets);

    // update class data

    motor_function->Update(time);  // call callbacks if any
}

void ChShaftsMotorLoad::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                          ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                          const double c           // a scaling factor
) {
    double imposed_torque = motor_function->GetVal(GetChTime());
    if (shaft1->IsActive())
        R(shaft1->GetOffset_w()) += imposed_torque * c;
    if (shaft2->IsActive())
        R(shaft2->GetOffset_w()) += -imposed_torque * c;
}

void ChShaftsMotorLoad::VariablesFbLoadForces(double factor) {
    double imposed_torque = motor_function->GetVal(GetChTime());
    shaft1->Variables().Force()(0) += imposed_torque * factor;
    shaft2->Variables().Force()(0) += -imposed_torque * factor;
}

void ChShaftsMotorLoad::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsMotorLoad>();

    // serialize parent class
    ChShaftsMotor::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(motor_function);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsMotorLoad::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsMotorLoad>();

    // deserialize parent class:
    ChShaftsMotor::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(motor_function);
}

}  // end namespace chrono
