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


#include "chrono_vehicle/suspension/ChHendricksonPRIMAXX.h"

class Generic_HendricksonPRIMAXX : public chrono::ChHendricksonPRIMAXX
{
public:

  Generic_HendricksonPRIMAXX(const std::string& name);
  ~Generic_HendricksonPRIMAXX();

  virtual const chrono::ChVector<> getAxlehousingCOM() const { return m_axlehousingCOM; }
  virtual const chrono::ChVector<> getTransversebeamCOM() const { return m_transversebeamCOM; }

  virtual double getAxlehousingMass() const { return m_axlehousingMass; }
  virtual double getKnuckleMass() const { return m_knuckleMass; }
  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getTorquerodMass() const { return m_torquerodMass; }
  virtual double getLowerbeamMass() const { return m_lowerbeamMass; }
  virtual double getTransversebeamMass() const { return m_transversebeamMass; }

  virtual double getAxlehousingRadius() const { return m_axlehousingRadius; }
  virtual double getKnuckleRadius() const { return m_knuckleRadius; }
  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const  { return m_spindleWidth; }
  virtual double getTorquerodRadius() const { return m_torquerodRadius; }
  virtual double getLowerbeamRadius() const { return m_lowerbeamRadius; }
  virtual double getTransversebeamRadius() const { return m_transversebeamRadius; }

  virtual const chrono::ChVector<>& getAxlehousingInertia() const { return m_axlehousingInertia; }
  virtual const chrono::ChVector<>& getKnuckleInertia() const { return m_knuckleInertia; }
  virtual const chrono::ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const chrono::ChVector<>& getTorquerodInertia() const { return m_torquerodInertia; }
  virtual const chrono::ChVector<>& getLowerbeamInertia() const { return m_lowerbeamInertia; }
  virtual const chrono::ChVector<>& getTransversebeamInertia() const { return m_transversebeamInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getShockAHRestLength() const { return m_shockAH_restLength; }
  virtual chrono::ChSpringForceCallback* getShockAHForceCallback()  const { return m_shockAHForceCB; }

  virtual double getShockLBRestLength() const { return m_shockLB_restLength; }
  virtual chrono::ChSpringForceCallback* getShockLBForceCallback()  const { return m_shockLBForceCB; }

private:

  virtual const chrono::ChVector<> getLocation(PointId which);
  virtual const chrono::ChVector<> getDirection(DirectionId which);

  chrono::ChSpringForceCallback* m_shockAHForceCB;
  chrono::ChSpringForceCallback* m_shockLBForceCB;

  static const double      m_axlehousingMass;
  static const double      m_knuckleMass;
  static const double      m_spindleMass;
  static const double      m_torquerodMass;
  static const double      m_lowerbeamMass;
  static const double      m_transversebeamMass;

  static const double      m_axlehousingRadius;
  static const double      m_knuckleRadius;
  static const double      m_spindleRadius;
  static const double      m_spindleWidth;
  static const double      m_torquerodRadius;
  static const double      m_lowerbeamRadius;
  static const double      m_transversebeamRadius;

  static const chrono::ChVector<>  m_axlehousingCOM;
  static const chrono::ChVector<>  m_transversebeamCOM;

  static const chrono::ChVector<>  m_axlehousingInertia;
  static const chrono::ChVector<>  m_knuckleInertia;
  static const chrono::ChVector<>  m_spindleInertia;
  static const chrono::ChVector<>  m_torquerodInertia;
  static const chrono::ChVector<>  m_lowerbeamInertia;
  static const chrono::ChVector<>  m_transversebeamInertia;

  static const double      m_axleInertia;

  static const double      m_shockAH_springCoefficient;
  static const double      m_shockAH_dampingCoefficient;
  static const double      m_shockAH_restLength;

  static const double      m_shockLB_springCoefficient;
  static const double      m_shockLB_dampingCoefficient;
  static const double      m_shockLB_restLength;
};



#endif
