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

#include "chrono/physics/ChLinkMotorRotationTorque.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorRotationTorque)


ChLinkMotorRotationTorque::ChLinkMotorRotationTorque() {
    
    this->c_rz = false;
    SetupLinkMask();

    this->f_torque = std::make_shared<ChFunction_Const>(0.0);
}

ChLinkMotorRotationTorque::ChLinkMotorRotationTorque(const ChLinkMotorRotationTorque& other) : ChLinkMotorRotation(other) {
   this->f_torque = other.f_torque;
}

ChLinkMotorRotationTorque::~ChLinkMotorRotationTorque() {
    
}

void ChLinkMotorRotationTorque::Update(double mytime, bool update_assets) {

     // Inherit parent class:
    ChLinkMotorRotation::Update(mytime, update_assets);

    this->f_torque->Update(mytime); // call callbacks if any
}

void ChLinkMotorRotationTorque::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // compute instant torque
    double mT = this->f_torque->Get_y( this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_torque = aframe2.GetA().Matr_x_Vect(ChVector<>(  0,0,mT ) );

    if (Body2->Variables().IsActive()) {
        R.PasteSumVector(Body2->TransformDirectionParentToLocal(m_abs_torque) * -c,
                            Body2->Variables().GetOffset() + 3, 0);
    }

    if (Body1->Variables().IsActive()) {
        R.PasteSumVector(Body1->TransformDirectionParentToLocal(m_abs_torque) * c,
                            Body1->Variables().GetOffset() + 3, 0);
    }
}


void ChLinkMotorRotationTorque::ConstraintsFbLoadForces(double factor) {
    // compute instant torque
    double mT = this->f_torque->Get_y( this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_torque = aframe2.GetA().Matr_x_Vect(ChVector<>(  0,0,mT ) );

    Body2->Variables().Get_fb().PasteSumVector(Body2->TransformDirectionParentToLocal(m_abs_torque) * -factor, 3, 0);

    Body1->Variables().Get_fb().PasteSumVector(Body1->TransformDirectionParentToLocal(m_abs_torque) * factor, 3, 0);
}

void ChLinkMotorRotationTorque::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationTorque>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(f_torque);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationTorque::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorRotationTorque>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_torque);
}




}  // end namespace chrono
