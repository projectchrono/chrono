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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear HMMWV suspension subsystems (double A-arm)
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames having X pointing towards the rear, Y to the right, and Z up (as
// imposed by the base class ChDoubleWishbone) and origins at the 
// midpoint between the lower control arm's connection points to the chassis.
//
// =============================================================================

#ifndef HMMWV_DOUBLEWISHBONE_H
#define HMMWV_DOUBLEWISHBONE_H


#include "subsys/suspension/ChDoubleWishbone.h"

namespace hmmwv {

class HMMWV_DoubleWishboneFront : public chrono::ChDoubleWishbone
{
public:
  HMMWV_DoubleWishboneFront(const std::string&         name,
                            chrono::ChSuspension::Side side,
                            bool                       driven = false);
  ~HMMWV_DoubleWishboneFront() {}

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUCAMass() const { return m_UCAMass; }
  virtual double getLCAMass() const { return m_LCAMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUCARadius() const { return m_UCARadius; }
  virtual double getLCARadius() const { return m_LCARadius; }
  virtual double getUprightRadius() const { return m_uprightRadius; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getUCAInertia() const { return m_UCAInertia; }
  virtual const chrono::ChVector<>& getLCAInertia() const { return m_LCAInertia; }
  virtual const chrono::ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);

  static const double      m_spindleMass;
  static const double      m_UCAMass;
  static const double      m_LCAMass;
  static const double      m_uprightMass;

  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_UCARadius;
  static const double      m_LCARadius;
  static const double      m_uprightRadius;

  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_UCAInertia;
  static const chrono::ChVector<>  m_LCAInertia;
  static const chrono::ChVector<>  m_uprightInertia;

  static const double      m_axleInertia;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};

// -----------------------------------------------------------------------------

class HMMWV_DoubleWishboneRear : public chrono::ChDoubleWishbone
{
public:
  HMMWV_DoubleWishboneRear(const std::string&         name,
                           chrono::ChSuspension::Side side,
                           bool                       driven = false);
  ~HMMWV_DoubleWishboneRear() {}

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUCAMass() const { return m_UCAMass; }
  virtual double getLCAMass() const { return m_LCAMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUCARadius() const { return m_UCARadius; }
  virtual double getLCARadius() const { return m_LCARadius; }
  virtual double getUprightRadius() const { return m_uprightRadius; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getUCAInertia() const { return m_UCAInertia; }
  virtual const chrono::ChVector<>& getLCAInertia() const { return m_LCAInertia; }
  virtual const chrono::ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringCoefficient() const { return m_springCoefficient; }
  virtual double getDampingCoefficient() const { return m_dampingCoefficient; }
  virtual double getSpringRestLength() const { return m_springRestLength; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);

  static const double      m_spindleMass;
  static const double      m_UCAMass;
  static const double      m_LCAMass;
  static const double      m_uprightMass;

  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_UCARadius;
  static const double      m_LCARadius;
  static const double      m_uprightRadius;

  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_UCAInertia;
  static const chrono::ChVector<>  m_LCAInertia;
  static const chrono::ChVector<>  m_uprightInertia;

  static const double      m_axleInertia;

  static const double      m_springCoefficient;
  static const double      m_dampingCoefficient;
  static const double      m_springRestLength;
};


} // end namespace hmmwv


#endif
