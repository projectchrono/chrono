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
#include "subsys/base/ChTrackVehicle.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChShaft.h"
#include "physics/ChShaftsBody.h"
#include "ModelDefs.h"
// collision callback function
#include "subsys/collision/TrackCollisionCallback.h"


namespace chrono {

/// Drive gear class, a single rigid body. Attached to the chassis via revolute joint.
/// Torque applied by the driveline.
class CH_SUBSYS_API DriveGear : public ChShared
{
public:

  /// override static values for mass, inertia
  DriveGear(const std::string& name,
    VisualizationType::Enum vis = VisualizationType::Primitives,
    CollisionType::Enum collide = CollisionType::Primitives,
    size_t chainSys_idx = 0, ///< what chain system is this gear associated with?
    double gear_mass = 436.7,
    const ChVector<>& gear_Ixx = ChVector<>(12.22, 12.22, 13.87) );

  ~DriveGear();

  /// init the gear with the initial pos. and rot., w.r.t. the chassis c-sys
  void Initialize(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const ChCoordsys<>& local_Csys,
    const std::vector<ChSharedPtr<ChBody> >& shoes,
    ChTrackVehicle* vehicle);

  // accessors
  ChSharedPtr<ChBody> GetBody() const { return m_gear; }

  ChSharedPtr<ChShaft> GetAxle() const { return m_axle; }

  double GetRadius() const { return m_radius; }

  // log constraint violations to console
  void LogConstraintViolations();

  /// write constraint violations to ostream, which will be written to the output file
  void SaveConstraintViolations(std::stringstream& ss);
  
  /// write headers for the output data file to the input ostream
  //  Gear Constraint Violation, (x,y,z,rx,ry)
  const std::string getFileHeader_ConstraintViolations(size_t idx) const;

private:
  // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }

  void AddVisualization();
  void AddCollisionGeometry(const std::vector<ChSharedPtr<ChBody> >& shoes,
    ChTrackVehicle* vehicle,
    VehicleSide side = RIGHTSIDE,
    double mu = 0.6,
    double mu_sliding = 0.5,
    double mu_roll = 0,
    double mu_spin = 0);
  
  // private variables
  ChSharedPtr<ChBody> m_gear;
  ChSharedPtr<GearPinGeometry> m_gearPinGeom;  ///< gear and pin geometry info

  ChSharedPtr<ChShaft> m_axle;                  ///< handle to axle shaft
  ChSharedPtr<ChShaftsBody>  m_axle_to_gear;    ///< handle to gear-shaft connector
  ChSharedPtr<ChLinkLockRevolute>  m_revolute;  ///< handle to revolute joint

  VisualizationType::Enum m_vis;    // visual asset geometry type
  CollisionType::Enum m_collide;    // collision geometry type
  const size_t m_chainSys_idx; ///< if there are multiple chain systems 
  // (e.g., on the M113, the subsystem knows which it is a part of for collision family purposes)

  ChVector<> m_inertia;
  double m_mass;

  const std::string m_meshName;
  const std::string m_meshFile;

  // most basic gear geometry: two concentric cylinders.
  const double m_radius;
  const double m_width;
  const double m_widthGap; // inner distance between cylinders

  // static variables
  static const double m_shaft_inertia;
  
};



} // end namespace chrono


#endif
