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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Front and Rear multi-link suspension subsystems.
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origins at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#ifndef GENERIC_MULTILINK_H
#define GENERIC_MULTILINK_H


#include "subsys/suspension/ChMultiLink.h"


class Generic_MultiLinkFront : public chrono::ChMultiLink
{
public:

  Generic_MultiLinkFront(const std::string& name);
  ~Generic_MultiLinkFront() {}

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUpperArmMass() const { return m_upperArmMass; }
  virtual double getLateralMass() const { return m_lateralMass; }
  virtual double getTrailingLinkMass() const { return m_trailingLinkMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUpperArmRadius() const { return m_upperArmRadius; }
  virtual double getLateralRadius() const { return m_lateralRadius; }
  virtual double getTrailingLinkRadius() const { return m_trailingLinkRadius; }
  virtual double getUprightRadius() const { return m_uprightRadius; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getUpperArmInertia() const { return m_upperArmInertia; }
  virtual const chrono::ChVector<>& getLateralInertia() const { return m_lateralInertia; }
  virtual const chrono::ChVector<>& getTrailingLinkInertia() const { return m_trailingLinkInertia; }
  virtual const chrono::ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);
  virtual const chrono::ChVector<> getDirection(DirectionId which);

  static const double      m_spindleMass;
  static const double      m_upperArmMass;
  static const double      m_lateralMass;
  static const double      m_trailingLinkMass;
  static const double      m_uprightMass;

  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_upperArmRadius;
  static const double      m_lateralRadius;
  static const double      m_trailingLinkRadius;
  static const double      m_uprightRadius;

  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_upperArmInertia;
  static const chrono::ChVector<>  m_lateralInertia;
  static const chrono::ChVector<>  m_trailingLinkInertia;
  static const chrono::ChVector<>  m_uprightInertia;

  static const double      m_axleInertia;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};

// -----------------------------------------------------------------------------

class Generic_MultiLinkRear : public chrono::ChMultiLink
{
public:

  Generic_MultiLinkRear(const std::string& name);
  ~Generic_MultiLinkRear() {}

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUpperArmMass() const { return m_upperArmMass; }
  virtual double getLateralMass() const { return m_lateralMass; }
  virtual double getTrailingLinkMass() const { return m_trailingLinkMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUpperArmRadius() const { return m_upperArmRadius; }
  virtual double getLateralRadius() const { return m_lateralRadius; }
  virtual double getTrailingLinkRadius() const { return m_trailingLinkRadius; }
  virtual double getUprightRadius() const { return m_uprightRadius; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getUpperArmInertia() const { return m_upperArmInertia; }
  virtual const chrono::ChVector<>& getLateralInertia() const { return m_lateralInertia; }
  virtual const chrono::ChVector<>& getTrailingLinkInertia() const { return m_trailingLinkInertia; }
  virtual const chrono::ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);
  virtual const chrono::ChVector<> getDirection(DirectionId which);

  static const double      m_spindleMass;
  static const double      m_upperArmMass;
  static const double      m_lateralMass;
  static const double      m_trailingLinkMass;
  static const double      m_uprightMass;

  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_upperArmRadius;
  static const double      m_lateralRadius;
  static const double      m_trailingLinkRadius;
  static const double      m_uprightRadius;

  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_upperArmInertia;
  static const chrono::ChVector<>  m_lateralInertia;
  static const chrono::ChVector<>  m_trailingLinkInertia;
  static const chrono::ChVector<>  m_uprightInertia;

  static const double      m_axleInertia;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};


#endif
