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
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES);

  ~SupportRoller() {}

  /// init the gear with the initial pos. and rot., w.r.t. the chassis c-sys
  void Initialize(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const ChCoordsys<>& local_Csys);

  // accessors
  ChSharedPtr<ChBody> GetBody() const { return m_roller; }

  double GetRadius() { return m_radius; }

private:

  // private functions
  void AddVisualization();
  void AddCollisionGeometry(double mu = 0.4,
                            double mu_sliding = 0.3,
                            double mu_roll = 0.0,
                            double mu_spin = 0.0);
  
  // private variables
  ChSharedPtr<ChBody> m_roller;
  ChSharedPtr<ChLinkLockRevolute>  m_revolute;  ///< handle to revolute joint

  VisualizationType m_vis;    // visual asset geometry type
  CollisionType m_collide;    // collision geometry type

  // static variables
  static const double     m_mass;
  static const ChVector<> m_inertia;
  static const double     m_radius;
  static const double     m_width;
  static const double     m_widthGap;

};


} // end namespace chrono


#endif
