// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Simple brake created with constant torque opposing wheel rotation.
//
// =============================================================================

#ifndef CH_BRAKESIMPLE_H
#define CH_BRAKESIMPLE_H


#include "subsys/ChBrake.h"
#include "physics/ChLinkBrake.h"

namespace chrono {


	/// Simple brake created with constant torque opposing 
	/// wheel rotation.
	/// It just uses a speed-dependant torque, so it fits in ODEs
	/// because it does not use DVI set valued constraints (the 
	/// drawback is that it cannot simulate sticking brakes).

class CH_SUBSYS_API ChBrakeSimple : public ChBrake {
public:

  ChBrakeSimple (ChSharedPtr<ChLinkLockRevolute> mhub);

  virtual ~ChBrakeSimple() {}

		/// Get the current brake angular speed, relative between disc and caliper  [rad/s]
  virtual double GetBrakeSpeed();

		/// Set the brake modulation, in 0..1 range, 
		/// when = 0 it is completely free,
		/// when = 1 it should provide the max braking torque
		/// This function can be called to modulate braking in realtime simulation loops.
  virtual void ApplyBrakeModulation(double modulation);
		
		/// Get the current brake modulation
  virtual double GetBrakeModulation() {return modulation;}

		/// Get the current brake torque, as a result of simulation,
		/// so it might change from time to time
  virtual double GetBrakeTorque();

		/// Set the max braking torque (for modulation =1)
  virtual void SetMaxBrakingTorque(double maxt);
		
		/// Get the max braking torque (for modulation =1)
  virtual double GetMaxBrakingTorque() {return maxtorque;}

protected:

	double modulation;
	double maxtorque;

	ChSharedPtr<ChLinkBrake> mbrake;
};


} // end namespace chrono


#endif
