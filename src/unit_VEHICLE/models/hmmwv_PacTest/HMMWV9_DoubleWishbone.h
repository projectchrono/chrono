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
// midpoint between the lower control arm's connection points to the chassis.
//
// =============================================================================

#ifndef HMMWV9_DOUBLEWISHBONE_H
#define HMMWV9_DOUBLEWISHBONE_H


#include "subsys/suspension/ChDoubleWishboneReduced.h"

namespace pactest {

class HMMWV9_DoubleWishboneFront : public chrono::ChDoubleWishboneReduced
{
public:
  HMMWV9_DoubleWishboneFront(const std::string&         name,
                             chrono::ChSuspension::Side side,
                             bool                       driven = false);
  ~HMMWV9_DoubleWishboneFront() {}

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);

  virtual void OnInitializeUpright();


  static const double      m_spindleMass;
  static const double      m_uprightMass;
  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_uprightInertia;

  static const double      m_axleInertia;

  static const chrono::ChVector<>  m_uprightDims;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};

// -----------------------------------------------------------------------------

class HMMWV9_DoubleWishboneRear : public chrono::ChDoubleWishboneReduced
{
public:
  HMMWV9_DoubleWishboneRear(const std::string&         name,
                            chrono::ChSuspension::Side side,
                            bool                       driven = false);
  ~HMMWV9_DoubleWishboneRear() {}

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);

  virtual void OnInitializeUpright();


  static const double      m_spindleMass;
  static const double      m_uprightMass;
  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_uprightInertia;

  static const double      m_axleInertia;

  static const chrono::ChVector<>  m_uprightDims;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};


} // end namespace pactest


#endif