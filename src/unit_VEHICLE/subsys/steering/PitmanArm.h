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
// Authors: Radu Serban
// =============================================================================
//
// Pitman arm steering model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef PITMAN_ARM_H
#define PITMAN_ARM_H

#include "subsys/ChApiSubsys.h"
#include "subsys/steering/ChPitmanArm.h"

namespace chrono {


class CH_SUBSYS_API PitmanArm : public ChPitmanArm {
public:
  PitmanArm(const std::string& filename);
  ~PitmanArm() {}

  virtual double getSteeringLinkMass() const { return m_steeringLinkMass; }
  virtual double getPitmanArmMass() const    { return m_pitmanArmMass; }

  virtual double getSteeringLinkRadius() const { return m_steeringLinkRadius; }
  virtual double getPitmanArmRadius() const    { return m_pitmanArmRadius; }

  virtual const ChVector<>& getSteeringLinkInertia() const { return m_steeringLinkInertia; }
  virtual const ChVector<>& getPitmanArmInertia() const    { return m_pitmanArmInertia; }

  virtual const ChVector<> getLocation(PointId which)      { return m_points[which]; }
  virtual const ChVector<> getDirection(DirectionId which) { return m_dirs[which]; }

private:
  ChVector<>  m_points[NUM_POINTS];
  ChVector<>  m_dirs[NUM_DIRS];

  double      m_steeringLinkMass;
  double      m_pitmanArmMass;

  double      m_steeringLinkRadius;
  double      m_pitmanArmRadius;

  ChVector<>  m_steeringLinkInertia;
  ChVector<>  m_pitmanArmInertia;
};


} // end namespace chrono


#endif
