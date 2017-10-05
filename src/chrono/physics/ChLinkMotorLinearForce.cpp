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

#include "chrono/physics/ChLinkMotorLinearForce.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorLinearForce)


ChLinkMotorLinearForce::ChLinkMotorLinearForce() {
    
    this->c_x = false;
    SetupLinkMask();

    this->f_force = std::make_shared<ChFunction_Const>(0.0);
}

ChLinkMotorLinearForce::ChLinkMotorLinearForce(const ChLinkMotorLinearForce& other) : ChLinkMotorLinear(other) {
   this->f_force = other.f_force;
}

ChLinkMotorLinearForce::~ChLinkMotorLinearForce() {
    
}

void ChLinkMotorLinearForce::Update(double mytime, bool update_assets) {

     // Inherit parent class:
    ChLinkMotorLinear::Update(mytime, update_assets);

    this->f_force->Update(mytime); // call callbacks if any
}

void ChLinkMotorLinearForce::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // compute instant force
    double mF = this->f_force->Get_y( this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_force = aframe2.GetA().Matr_x_Vect(ChVector<>(  mF,0,0 ) );
    Vector mbody_force;
    Vector mbody_torque;

    if (Body2->Variables().IsActive()) {
        Body2->To_abs_forcetorque(m_abs_force,
                                    aframe1.GetPos(),            // absolute application point is always marker1
                                    false,                       // from abs. space
                                    mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
        R.PasteSumVector(- mbody_force * c, Body2->Variables().GetOffset(), 0);
        R.PasteSumVector(Body2->TransformDirectionParentToLocal(mbody_torque) * -c,
                            Body2->Variables().GetOffset() + 3, 0);
    }

    if (Body1->Variables().IsActive()) {
        Body1->To_abs_forcetorque(m_abs_force,
                                    aframe1.GetPos(),            // absolute application point is always marker1
                                    false,                       // from abs. space
                                    mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
        R.PasteSumVector(  mbody_force * c, Body1->Variables().GetOffset(), 0);
        R.PasteSumVector(Body1->TransformDirectionParentToLocal(mbody_torque) * c,
                            Body1->Variables().GetOffset() + 3, 0);
    }
}


void ChLinkMotorLinearForce::ConstraintsFbLoadForces(double factor) {
    // compute instant force
    double mF = this->f_force->Get_y( this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_force = aframe2.GetA().Matr_x_Vect(ChVector<>(  mF,0,0 ) );
    Vector mbody_force;
    Vector mbody_torque;
     Body2->To_abs_forcetorque(m_abs_force,
                                    aframe1.GetPos(),            // absolute application point is always marker1
                                    false,                       // from abs. space
                                    mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
    Body2->Variables().Get_fb().PasteSumVector(- mbody_force * factor, 0, 0);
    Body2->Variables().Get_fb().PasteSumVector(Body2->TransformDirectionParentToLocal(mbody_torque) * -factor, 3, 0);

    Body1->To_abs_forcetorque(m_abs_force,
                                    aframe1.GetPos(),            // absolute application point is always marker1
                                    false,                       // from abs. space
                                    mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
    Body1->Variables().Get_fb().PasteSumVector( mbody_force * factor, 0, 0);
    Body1->Variables().Get_fb().PasteSumVector(Body1->TransformDirectionParentToLocal(mbody_torque) * factor, 3, 0);
}

void ChLinkMotorLinearForce::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorLinearForce>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(f_force);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearForce::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorLinearForce>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_force);
}





}  // end namespace chrono
