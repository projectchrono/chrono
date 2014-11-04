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
// Front and Rear solid axle suspension subsystems.
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origins at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#ifndef GENERIC_SOLIDAXLE_H
#define GENERIC_SOLIDAXLE_H


#include "subsys/suspension/ChSolidAxle.h"


class Generic_SolidAxleFront : public chrono::ChSolidAxle
{
public:

  Generic_SolidAxleFront(const std::string& name);
  ~Generic_SolidAxleFront() {}

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

  virtual const chrono::ChVector<>& getAxleTubeInertia() const { return m_axleTubeInertia; }
  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getULInertia() const { return m_ULInertia; }
  virtual const chrono::ChVector<>& getLLInertia() const { return m_LLInertia; }
  virtual const chrono::ChVector<>& getKnuckleInertia() const { return m_knuckleInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

  virtual const chrono::ChVector<> getAxleTubeCOM() const { return m_axleTubeCOM; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);
  virtual const chrono::ChVector<> getDirection(DirectionId which);

  static const double      m_axleTubeMass;
  static const double      m_spindleMass;
  static const double      m_ULMass;
  static const double      m_LLMass;
  static const double      m_knuckleMass;

  static const double      m_axleTubeRadius;
  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_ULRadius;
  static const double      m_LLRadius;
  static const double      m_knuckleRadius;

  static const chrono::ChVector<>  m_axleTubeCOM;

  static const chrono::ChVector<>  m_axleTubeInertia;
  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_ULInertia;
  static const chrono::ChVector<>  m_LLInertia;
  static const chrono::ChVector<>  m_knuckleInertia;

  static const double      m_axleInertia;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};

// -----------------------------------------------------------------------------

class Generic_SolidAxleRear : public chrono::ChSolidAxle
{
public:

  Generic_SolidAxleRear(const std::string& name);
  ~Generic_SolidAxleRear() {}

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

  virtual const chrono::ChVector<>& getAxleTubeInertia() const { return m_axleTubeInertia; }
  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getULInertia() const { return m_ULInertia; }
  virtual const chrono::ChVector<>& getLLInertia() const { return m_LLInertia; }
  virtual const chrono::ChVector<>& getKnuckleInertia() const { return m_knuckleInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

  virtual const chrono::ChVector<> getAxleTubeCOM() const { return m_axleTubeCOM; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);
  virtual const chrono::ChVector<> getDirection(DirectionId which);

  static const double      m_axleTubeMass;
  static const double      m_spindleMass;
  static const double      m_ULMass;
  static const double      m_LLMass;
  static const double      m_knuckleMass;

  static const double      m_axleTubeRadius;
  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_ULRadius;
  static const double      m_LLRadius;
  static const double      m_knuckleRadius;

  static const chrono::ChVector<>  m_axleTubeCOM;

  static const chrono::ChVector<>  m_axleTubeInertia;
  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_ULInertia;
  static const chrono::ChVector<>  m_LLInertia;
  static const chrono::ChVector<>  m_knuckleInertia;

  static const double      m_axleInertia;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};


#endif
