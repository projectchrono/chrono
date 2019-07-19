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

#include "chrono/physics/ChShaftsLoads.h"


namespace chrono {


// -----------------------------------------------------------------------------
// ChShaftsLoad
// -----------------------------------------------------------------------------

ChShaftsLoad::ChShaftsLoad(std::shared_ptr<ChShaft> shaftA,  ///< shaft A
								   std::shared_ptr<ChShaft> shaftB   ///< shaft B
    )
    : ChLoadCustomMultiple(shaftA, shaftB) {
    this->torque = 0;
}

void ChShaftsLoad::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mshaftA = std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
    auto mshaftB = std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);

    double mrotA, mrotB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        mrotA = state_x->GetElementN(0);
        mrotB = state_x->GetElementN(1);
    } else {
        mrotA = mshaftA->GetPos();
        mrotB = mshaftB->GetPos();
    }

	double mrotA_dt, mrotB_dt;
    if (state_w) {
        // the numerical jacobian algo might change state_w
        mrotA_dt = state_w->GetElementN(0);
        mrotB_dt = state_w->GetElementN(1);
    } else {
        mrotA_dt = mshaftA->GetPos_dt();
        mrotB_dt = mshaftB->GetPos_dt();
    }

	double rel_rot    = mrotA    - mrotB;
    double rel_rot_dt = mrotA_dt - mrotB_dt;

    // COMPUTE THE TORQUE

    ComputeShaftShaftTorque(rel_rot, rel_rot_dt, this->torque);

    // Compute Q

    this->load_Q(0) =  this->torque;
    this->load_Q(1) = -this->torque;
}

std::shared_ptr<ChShaft> ChShaftsLoad::GetShaftA() const {
    return std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
}

std::shared_ptr<ChShaft> ChShaftsLoad::GetShaftB() const {
    return std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);
}


// -----------------------------------------------------------------------------
// ChShaftsTorsionSpringDamper
// -----------------------------------------------------------------------------


chrono::ChShaftsTorsionSpringDamper::ChShaftsTorsionSpringDamper(std::shared_ptr<ChShaft> mshaftA,
                                                             std::shared_ptr<ChShaft> mshaftB,
                                                             const double mstiffness,
                                                             const double mdamping) :
    ChShaftsLoad(mshaftA, mshaftB), stiffness(mstiffness), damping(mdamping), rest_phase(0.0) {

}


void ChShaftsTorsionSpringDamper::ComputeShaftShaftTorque(const double rel_rot, const double rel_rot_dt, double& result_torque) {
    result_torque = - (rel_rot-rest_phase) * stiffness    
                    - rel_rot_dt * damping;  
}



// -----------------------------------------------------------------------------
// ChShaftsElasticGear
// -----------------------------------------------------------------------------

chrono::ChShaftsElasticGear::ChShaftsElasticGear(
    std::shared_ptr<ChShaft> mshaftA,  ///< shaft A
    std::shared_ptr<ChShaft> mshaftB,  ///< shaft B
    const double mstiffness,          ///< normal stiffness at teeth contact, tangent direction to primitive
    const double mdamping,            ///< normal damping at teeth contact, tangent direction to primitive
    const double mRa,                 ///< primitive radius of the gear on shaft A (the radius of B is not needed)
    const double mratio)
    : ChLoadCustomMultiple(mshaftA, mshaftB),
      stiffness(mstiffness),
      damping(mdamping),
      rest_phase(0.0),
      Ra(mRa),
      ratio(mratio) {
    contact_force = 0.0;
}


void ChShaftsElasticGear::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mshaftA = std::dynamic_pointer_cast<ChShaft>(this->loadables[0]);
    auto mshaftB = std::dynamic_pointer_cast<ChShaft>(this->loadables[1]);

    double mrotA, mrotB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        mrotA = state_x->GetElementN(0);
        mrotB = state_x->GetElementN(1);
    } else {
        mrotA = mshaftA->GetPos();
        mrotB = mshaftB->GetPos();
    }

    double mrotA_dt, mrotB_dt;
    if (state_w) {
        // the numerical jacobian algo might change state_w
        mrotA_dt = state_w->GetElementN(0);
        mrotB_dt = state_w->GetElementN(1);
    } else {
        mrotA_dt = mshaftA->GetPos_dt();
        mrotB_dt = mshaftB->GetPos_dt();
    }
	double invratio = 1.0 / ratio;

    double rel_compression	   = Ra * (mrotA - this->rest_phase) - invratio * Ra * mrotB;
    double rel_compression_dt  = Ra *  mrotA_dt                  - invratio * Ra * mrotB_dt;

	// Compute contact force
	contact_force = -rel_compression * this->stiffness - rel_compression_dt * this->damping;

    // Compute resulting torques on the two shafts and store them in Q

    this->load_Q(0) =		      Ra * contact_force;
    this->load_Q(1) = -invratio * Ra * contact_force;
}




}  // end namespace chrono