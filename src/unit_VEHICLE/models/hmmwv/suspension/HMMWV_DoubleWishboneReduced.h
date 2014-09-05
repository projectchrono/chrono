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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Front and Rear HMMWV suspension subsystems (reduced double A-arm)
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames having X pointing towards the rear, Y to the right, and Z up (as
// imposed by the base class ChDoubleWishboneReduced) and origins at the 
// midpoint between the lower control arms' connection points to the chassis.
//
// =============================================================================

#ifndef HMMWV9_DOUBLEWISHBONE_REDUCED_H
#define HMMWV9_DOUBLEWISHBONE_REDUCED_H


#include "subsys/suspension/ChDoubleWishboneReduced.h"

namespace hmmwv {

class HMMWV_DoubleWishboneReducedFront : public chrono::ChDoubleWishboneReduced
{
public:
  HMMWV_DoubleWishboneReducedFront(const std::string& name,
                                   bool               driven = false);
  ~HMMWV_DoubleWishboneReducedFront() {}

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUprightRadius() const { return m_uprightRadius; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);

  static const double      m_spindleMass;
  static const double      m_uprightMass;

  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_uprightRadius;

  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_uprightInertia;

  static const double      m_axleInertia;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};

// -----------------------------------------------------------------------------

class HMMWV_DoubleWishboneReducedRear : public chrono::ChDoubleWishboneReduced
{
public:
  HMMWV_DoubleWishboneReducedRear(const std::string& name,
                                  bool               driven = false);
  ~HMMWV_DoubleWishboneReducedRear() {}

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUprightRadius() const { return m_uprightRadius; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);

  static const double      m_spindleMass;
  static const double      m_uprightMass;

  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_uprightRadius;

  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_uprightInertia;

  static const double      m_axleInertia;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};


} // end namespace hmmwv


#endif
