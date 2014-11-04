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
// Solid axle suspension constructed with data from file.
//
// =============================================================================

#ifndef SOLIDAXLE_H
#define SOLIDAXLE_H

#include "subsys/ChApiSubsys.h"
#include "subsys/suspension/ChSolidAxle.h"

#include "rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API SolidAxle : public ChSolidAxle
{
public:

  SolidAxle(const std::string& filename);
  SolidAxle(const rapidjson::Document& d);
  virtual ~SolidAxle() {}

  virtual double getAxleTubeMass() const { return m_axleTubeMass; }
  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getULMass() const { return m_ULMass; }
  virtual double getLLMass() const { return m_LLMass; }
  virtual double getKnuckleMass() const { return m_knuckleMass; }

  virtual double getAxleTubeRadius() const { return m_axleTubeRadius; }
  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getULRadius() const { return m_ULRadius; }
  virtual double getLLRadius() const { return m_LLRadius; }
  virtual double getKnuckleRadius() const { return m_knuckleRadius; }

  virtual const ChVector<>& getAxleTubeInertia() const { return m_axleTubeInertia; }
  virtual const ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const ChVector<>& getULInertia() const { return m_ULInertia; }
  virtual const ChVector<>& getLLInertia() const { return m_LLInertia; }
  virtual const ChVector<>& getKnuckleInertia() const { return m_knuckleInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

  virtual const ChVector<> getAxleTubeCOM() const { return m_axleTubeCOM; }

private:

  virtual const ChVector<> getLocation(PointId which) { return m_points[which]; }
  virtual const ChVector<> getDirection(DirectionId which) { return m_directions[which]; }

  void Create(const rapidjson::Document& d);

  ChVector<>  m_points[NUM_POINTS];
  ChVector<>  m_directions[NUM_DIRS];

  double      m_axleTubeMass;
  double      m_spindleMass;
  double      m_ULMass;
  double      m_LLMass;
  double      m_knuckleMass;

  double      m_axleTubeRadius;
  double      m_spindleRadius;
  double      m_spindleWidth;
  double      m_ULRadius;
  double      m_LLRadius;
  double      m_knuckleRadius;

  ChVector<>  m_axleTubeCOM;

  ChVector<>  m_axleTubeInertia;
  ChVector<>  m_spindleInertia;
  ChVector<>  m_ULInertia;
  ChVector<>  m_LLInertia;
  ChVector<>  m_knuckleInertia;

  double      m_axleInertia;

  double      m_springCoefficient;
  double      m_dampingCoefficient;
  double      m_springRestLength;
};


} // end namespace chrono


#endif
