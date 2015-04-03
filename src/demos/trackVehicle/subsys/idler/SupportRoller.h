// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// A support roller, with no suspension, connects to the hull via revolute constraint.
//
// =============================================================================

#ifndef SUPPORTROLLER_H
#define SUPPORTROLLER_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChShaft.h"
#include "physics/ChShaftsBody.h"
#include "ModelDefs.h"

namespace chrono {


/// SupportRoller class, has a single rigid body. Attaches to the chassis via revolute joint.
class CH_SUBSYS_API SupportRoller : public ChShared
{
public:

  SupportRoller(const std::string& name,
    VisualizationType::Enum vis = VisualizationType::Primitives,
    CollisionType::Enum collide = CollisionType::Primitives,
    size_t chainSys_idx = 0,  ///< what chain system is this gear associated with?
    double mass = 100.0,
    const ChVector<>& Ixx = ChVector<>(3.82, 3.82, 5.06) );

  ~SupportRoller() {}

  /// init the gear with the initial pos. and rot., w.r.t. the chassis c-sys
  void Initialize(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const ChCoordsys<>& local_Csys);

  // log constraint violations of any bilateral constraints
  void LogConstraintViolations();

  /// write constraint violations to ostream, which will be written to the output file
  void SaveConstraintViolations(std::stringstream& ss);

   /// write headers for the output data file to the input ostream
  const std::string getFileHeader_ConstraintViolations(size_t idx);

  // accessors
  ChSharedPtr<ChBody> GetBody() const { return m_roller; }

  double GetRadius() { return m_radius; }

  double GetWidth() { return m_width; }

private:

  // private functions
  void AddVisualization();
  void AddCollisionGeometry(VehicleSide side = RIGHTSIDE,
    double mu = 0.4,
    double mu_sliding = 0.3,
    double mu_roll = 0.0,
    double mu_spin = 0.0);
  
  // private variables
  ChSharedPtr<ChBody> m_roller;
  ChSharedPtr<ChLinkLockRevolute>  m_revolute;  ///< handle to revolute joint

  VisualizationType::Enum m_vis;    // visual asset geometry type
  CollisionType::Enum m_collide;    // collision geometry type
  const size_t m_chainSys_idx; ///< if there are multiple chain systems 
  // (e.g., on the M113, the subsystem knows which it is a part of for collision family purposes)

  double     m_mass;
  ChVector<> m_inertia;

  // static variables
  static const double     m_radius;
  static const double     m_width;
  static const double     m_widthGap;

};


} // end namespace chrono


#endif
