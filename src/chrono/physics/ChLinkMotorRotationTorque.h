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

#ifndef CHLINKMOTORROTATIONTORQUE_H
#define CHLINKMOTORROTATIONTORQUE_H


#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/motion_functions/ChFunction.h"


namespace chrono {

/// A motor that applies a torque between
/// two frames on two bodies.
/// Differently from the ChLinkMotorRotationAngle and
/// ChLinkMotorRotationSpeed, this does not enforce precise
/// motion via constraint. 
/// Example of application:
/// - mimic a PID controlled system with 
///   some feedback (that is up to you too implement)
/// - force that is updated by a cosimulation
/// - force from a man-in-the-loop setpoint
/// Use SetTorqueFunction() to change to other torque function (by
/// default is no torque), possibly introduce some custom ChFunction
/// of yours that is updated at each time step.

class ChApi ChLinkMotorRotationTorque : public ChLinkMotorRotation {

    std::shared_ptr<ChFunction> f_torque;

  public:
    ChLinkMotorRotationTorque();
    ChLinkMotorRotationTorque(const ChLinkMotorRotationTorque& other);
    virtual ~ChLinkMotorRotationTorque();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationTorque* Clone() const override { return new ChLinkMotorRotationTorque(*this); }


    /// Sets the torque function T(t). In [Nm]. It is a function of time. 
    void SetTorqueFunction(const std::shared_ptr<ChFunction> mf) {f_torque = mf;}

    /// Gets the torque function F(t).
    std::shared_ptr<ChFunction> GetTorqueFunction() {return f_torque;}
    

    /// Get the current actuator reaction torque [Nm]
    virtual double GetMotorTorque() const { return this->f_torque->Get_y(this->GetChTime());}

    
    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //
    virtual void ConstraintsFbLoadForces(double factor = 1) override;


    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorRotationTorque,0)




}  // end namespace chrono

#endif
