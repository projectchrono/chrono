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
// Generic concrete Hendrickson PRIMAXX suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChHendricksonPRIMAXX) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#ifndef GENERIC_HENDRICKSON_PRIMAXX_H
#define GENERIC_HENDRICKSON_PRIMAXX_H


#include "subsys/suspension/ChHendricksonPRIMAXX.h"

class Generic_HendricksonPRIMAXX : public chrono::ChHendricksonPRIMAXX
{
public:

  // Constructor takes as argument the name of the subsystem instance.
  Generic_HendricksonPRIMAXX(const std::string& name);

  // Destructor
  ~Generic_HendricksonPRIMAXX();

  // Implementation of virtual methods imposed by the base class ChHendricksonPRIMAXX

  virtual const chrono::ChVector<> getLocation(PointId which);

  virtual double getSpindleMass() const { return m_spindleMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const  { return m_spindleWidth; }

  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

private:

  static const double      m_spindleMass;

  static const double      m_spindleRadius;
  static const double      m_spindleWidth;

  static const chrono::ChVector<>  m_spindleInertia;

  static const double      m_axleInertia;
};



#endif
