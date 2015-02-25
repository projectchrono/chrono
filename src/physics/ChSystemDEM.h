//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSYSTEMDEM_H
#define CHSYSTEMDEM_H

#include "physics/ChSystem.h"
#include "physics/ChContactDEM.h"


namespace chrono {


/// Class for a physical system in which contact is modeled using a
/// Penalty Method (aka DEM)
class ChApi ChSystemDEM : public ChSystem
{
  CH_RTTI(ChSystemDEM, ChSystem);

public:
  /// Constructor
  /// Note, in case you will use collision detection, the values of
  /// 'max_objects' and 'scene_size' can be used to initialize the broadphase
  /// collision algorithm in an optimal way. Scene size should be approximately 
  /// the radius of the expected area where colliding objects will move.
  ChSystemDEM(
    bool use_material_properties = true,
    bool use_history = true,
    unsigned int max_objects = 16000,
    double scene_size = 500);

  virtual ~ChSystemDEM() {}

  virtual void SetLcpSolverType(eCh_lcpSolver mval);
  virtual void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);
  virtual void ChangeContactContainer(ChContactContainerBase* newcontainer);

  bool UseMaterialProperties() const                               { return m_use_mat_props; }
  bool UseContactHistory() const                                   { return m_use_history; }
  void SetContactForceModel(ChContactDEM::ContactForceModel model) { m_contact_model = model; }
  ChContactDEM::ContactForceModel GetContactForceModel() const     { return m_contact_model; }

private:
  bool                            m_use_mat_props;
  bool                            m_use_history;
  ChContactDEM::ContactForceModel m_contact_model;
};


} // end namespace chrono


#endif
