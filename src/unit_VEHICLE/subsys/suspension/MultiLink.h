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
// Multi-link suspension constructed with data from file.
//
// =============================================================================

#ifndef MULTILINK_H
#define MULTILINK_H

#include "subsys/ChApiSubsys.h"
#include "subsys/suspension/ChMultiLink.h"

#include "rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API MultiLink : public ChMultiLink
{
public:

  MultiLink(const std::string& filename,
                 bool               driven);
  MultiLink(const rapidjson::Document& d,
                 bool                       driven);
  virtual ~MultiLink() {}

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

  virtual const ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const ChVector<>& getUpperArmInertia() const { return m_upperArmInertia; }
  virtual const ChVector<>& getLateralInertia() const { return m_lateralInertia; }
  virtual const ChVector<>& getTrailingLinkInertia() const { return m_trailingLinkInertia; }
  virtual const ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const ChVector<> getLocation(PointId which) { return m_points[which]; }
  virtual const ChVector<> getDirection(DirectionId which) { return m_directions[which]; }

  void Create(const rapidjson::Document& d);

  ChVector<>  m_points[NUM_POINTS];
  ChVector<>  m_directions[NUM_DIRS];

  double      m_spindleMass;
  double      m_upperArmMass;
  double      m_lateralMass;
  double      m_trailingLinkMass;
  double      m_uprightMass;

  double      m_spindleRadius;
  double      m_spindleWidth;
  double      m_upperArmRadius;
  double      m_lateralRadius;
  double      m_trailingLinkRadius;
  double      m_uprightRadius;

  ChVector<>  m_spindleInertia;
  ChVector<>  m_upperArmInertia;
  ChVector<>  m_lateralInertia;
  ChVector<>  m_trailingLinkInertia;
  ChVector<>  m_uprightInertia;

  double      m_axleInertia;

  double      m_springCoefficient;
  double      m_dampingCoefficient;
  double      m_springRestLength;
};


} // end namespace chrono


#endif
