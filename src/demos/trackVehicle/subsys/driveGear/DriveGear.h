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
// Authors: Justin Madsen
// =============================================================================
//
// The drive gear propels the tracked vehicle
//
// =============================================================================

#ifndef DRIVEGEAR_H
#define DRIVEGEAR_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChShaft.h"
#include "physics/ChShaftsBody.h"
#include "ModelDefs.h"

namespace chrono {


/// Drive gear class, a single rigid body. Attached to the chassis via revolute joint.
/// Torque applied by the driveline.
class CH_SUBSYS_API DriveGear : public ChShared
{
public:

  DriveGear(const std::string& name,
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES);

  /// override static values for mass, inertia
  DriveGear(const std::string& name,
    double gear_mass,
    const ChVector<>& gear_Ixx,
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES);

  ~DriveGear() {}

  /// init the gear with the initial pos. and rot., w.r.t. the chassis c-sys
  void Initialize(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const ChCoordsys<>& local_Csys);

  // accessors
  ChSharedPtr<ChBody> GetBody() const { return m_gear; }

  ChSharedPtr<ChShaft> GetAxle() const { return m_axle; }

  double GetRadius() { return m_radius; }

private:
  // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }

  void AddVisualization();
  void AddCollisionGeometry(double mu = 0.7,
                            double mu_sliding = 0.6,
                            double mu_roll = 0,
                            double mu_spin = 0);
  
  // private variables
  ChSharedPtr<ChBody> m_gear;
  ChSharedPtr<ChShaft> m_axle;                  ///< handle to axle shaft
  ChSharedPtr<ChShaftsBody>  m_axle_to_gear;    ///< handle to gear-shaft connector
  ChSharedPtr<ChLinkLockRevolute>  m_revolute;  ///< handle to revolute joint

  VisualizationType m_vis;    // visual asset geometry type
  CollisionType m_collide;    // collision geometry type

  const std::string m_meshName;
  const std::string m_meshFile;

  // static variables
  static const ChVector<> m_inertia;
  static const double m_mass;
  static const double m_radius;
  static const double m_width;
  static const double m_widthGap; // inner distance between cydliners
  static const double m_shaft_inertia;
  
};


// Differently from friction, that has always a default value that 
// is computed as the average of two friction values of the two rigid 
// bodies, the 'cohesion' has no body-specific setting (at least in 
// this Chrono::Engine release) so it is always zero by default. 
// Therefore it is up to the user to set it when each contact is created, 
// by instancing a callback as in the following example:

class MyContactCallback : public ChSystem::ChCustomCollisionPointCallback
{
	public:	virtual void ContactCallback(
							const collision::ChCollisionInfo& mcontactinfo, ///< get info about contact (cannot change it)				
							ChMaterialCouple&  material,
							double friction0 = 0.3,
							double compliance0 = 0.0,
							double cohesion0 = 0.0,
							double dampingf0 = 0.1)			  		///< you can modify this!	
	{
		// set the ICs
		this->friction = friction0;
		this->compliance = compliance0;
		this->cohesion = cohesion0;
		this->dampingf = dampingf0;
		// Set friction according to user setting:
		material.static_friction = this->friction;
		// Set compliance (normal and tangential at once)
		material.compliance  =  (float)this->compliance;
		material.complianceT =  (float)this->compliance;
		material.dampingf	 =  (float)this->dampingf;

		// Set cohesion according to user setting:
		// Note that we must scale the cohesion force value by time step, because 
		// the material 'cohesion' value has the dimension of an impulse.
		double my_cohesion_force =  cohesion;
		material.cohesion = (float)(msystem->GetStep() * my_cohesion_force); //<- all contacts will have this cohesion!

		if (mcontactinfo.distance>0.12)
			material.cohesion = 0;
		// Note that here you might decide to modify the cohesion 
		// depending on object sizes, type, time, position, etc. etc.
		// For example, after some time disable cohesion at all, just
		// add here:  
		//    if (msystem->GetChTime() > 10) material.cohesion = 0;
	};

	ChSystem* msystem;
	// this will be modified by the user
	double friction;
	double cohesion;
	double compliance;
	double dampingf;

};

} // end namespace chrono


#endif
