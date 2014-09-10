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
// HMMWV Pitman arm steering model.
//
// =============================================================================

#ifndef HMMWV_PITMAN_ARM_H
#define HMMWV_PITMAN_ARM_H

#include "subsys/ChApiSubsys.h"
#include "subsys/steering/ChPitmanArm.h"

namespace hmmwv {


class HMMWV_PitmanArm : public chrono::ChPitmanArm {
public:
  HMMWV_PitmanArm(const std::string& name);
  ~HMMWV_PitmanArm() {}

  virtual double getSteeringLinkMass() const { return m_steeringLinkMass; }
  virtual double getPitmanArmMass() const    { return m_pitmanArmMass; }

  virtual double getSteeringLinkRadius() const { return m_steeringLinkRadius; }
  virtual double getPitmanArmRadius() const    { return m_pitmanArmRadius; }

  virtual const chrono::ChVector<>& getSteeringLinkInertia() const { return m_steeringLinkInertia; }
  virtual const chrono::ChVector<>& getPitmanArmInertia() const    { return m_pitmanArmInertia; }

  virtual const chrono::ChVector<> getLocation(PointId which);
  virtual const chrono::ChVector<> getDirection(DirectionId which);

private:
  static const double      m_steeringLinkMass;
  static const double      m_pitmanArmMass;

  static const double      m_steeringLinkRadius;
  static const double      m_pitmanArmRadius;

  static const chrono::ChVector<>  m_steeringLinkInertia;
  static const chrono::ChVector<>  m_pitmanArmInertia;
};


} // end namespace hmmwv


#endif
