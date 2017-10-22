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

#ifndef CHLINKMOTORLINEARFORCE_H
#define CHLINKMOTORLINEARFORCE_H


#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/motion_functions/ChFunction.h"


namespace chrono {


/// A linear motor that applies a force between
/// two frames on two bodies.
/// Differently from the ChLinkMotorLinearPosition and
/// ChLinkMotorLinearSpeed, this does not enforce precise
/// motion via constraint. 
/// Example of application:
/// - mimic a PID controlled system with 
///   some feedback (that is up to you too implement)
/// - force that is updated by a cosimulation
/// - force from a man-in-the-loop setpoint
/// Use SetForceFunction() to change to other force function (by
/// default is no force), possibly introduce some custom ChFunction
/// of yours that is updated at each time step.

class ChApi ChLinkMotorLinearForce : public ChLinkMotorLinear {

    std::shared_ptr<ChFunction> f_force;

  public:
    ChLinkMotorLinearForce();
    ChLinkMotorLinearForce(const ChLinkMotorLinearForce& other);
    virtual ~ChLinkMotorLinearForce();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearForce* Clone() const override { return new ChLinkMotorLinearForce(*this); }


    /// Sets the force function F(t). It is a function of time. 
    void SetForceFunction(const std::shared_ptr<ChFunction> mf) {f_force = mf;}

    /// Gets the force function F(t).
    std::shared_ptr<ChFunction> GetForceFunction() {return f_force;}
    

    /// Get the current actuator reaction force [N]
    virtual double GetMotorForce() const { return this->f_force->Get_y(this->GetChTime());}

    
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

CH_CLASS_VERSION(ChLinkMotorLinearForce,0)




}  // end namespace chrono

#endif
