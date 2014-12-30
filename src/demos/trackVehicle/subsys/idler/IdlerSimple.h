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
// A simple Idler system that keeps track chain tension by pre-loading a 
//	spring/damper element
//
// =============================================================================

#ifndef IDLERSIMPLE_H
#define IDLERSIMPLE_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "ModelDefs.h"

namespace chrono {


/// An idler system that includes the chain tensioning system.
/// The simplest case of this is a prismatic (translational) constraint
/// between the idler and chassis, and mount a pre-loaded spring-damper
/// along the DOF axis
class CH_SUBSYS_API IdlerSimple : public ChShared
{
public:

  IdlerSimple(const std::string& name,
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES);

  ~IdlerSimple();

  /// init the idler with the initial pos. and rot., w.r.t. the chassis c-sys
  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                  const ChCoordsys<>& local_Csys);
  
  
  double getSpringCoefficient() const { return m_springK; }
  double getDampingCoefficient() const { return m_springC; }
  double getSpringRestLength() const { return m_springRestLength; }
  ChSharedPtr<ChBody> GetBody() { return m_idler; }
  double GetRadius() { return m_radius; }

private:
  // private functions
  void AddVisualization();
  void AddCollisionGeometry();
    // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }
  
  // private variables
  ChSharedPtr<ChBody> m_idler;  ///< handle to idler body
  ChSharedPtr<ChLinkLockRevolutePrismatic> m_idler_joint; ///< connetion to chassis
  ChSharedPtr<ChLinkSpring> m_shock;  ///< handle to spring-damper;
  // ChSpringForceCallback* m_shockCB;   ///< shock callback function
  // ChSpringForceCallback* m_springCB;  ///< spring callback function

  VisualizationType m_vis;
  CollisionType m_collide;

  // static variables
  static const double m_mass;
  static const ChVector<> m_inertia;
  static const double m_springK;  ///< shock linear spring coefficient
  static const double m_springC;  ///< shock linear damping coefficient
  static const double m_springRestLength; ///< shock rest length
  static const double m_width;
  static const double m_radius;

  static const std::string m_meshName;
  static const std::string m_meshFile;

};


} // end namespace chrono


#endif
