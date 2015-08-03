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
// Reduced double-A arm suspension constructed with data from file.
//
// =============================================================================

#ifndef DOUBLEWISHBONEREDUCED_H
#define DOUBLEWISHBONEREDUCED_H

#include "chrono_vehicle/ChApiSubsys.h"
#include "chrono_vehicle/suspension/ChDoubleWishboneReduced.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_SUBSYS_API DoubleWishboneReduced : public ChDoubleWishboneReduced
{
public:

  DoubleWishboneReduced(const std::string& filename);
  DoubleWishboneReduced(const rapidjson::Document& d);
  ~DoubleWishboneReduced();

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUprightRadius() const { return m_uprightRadius; }

  virtual const ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringRestLength() const { return m_springRestLength; }
  virtual ChSpringForceCallback* getShockForceCallback() const { return m_shockForceCB; }

private:

  virtual const ChVector<> getLocation(PointId which) { return m_points[which]; }

  void Create(const rapidjson::Document& d);

  ChSpringForceCallback* m_shockForceCB;

  ChVector<>  m_points[NUM_POINTS];

  double      m_spindleMass;
  double      m_uprightMass;

  double      m_spindleRadius;
  double      m_spindleWidth;
  double      m_uprightRadius;

  ChVector<>  m_spindleInertia;
  ChVector<>  m_uprightInertia;

  double      m_axleInertia;

  double      m_springRestLength;
};


} // end namespace chrono


#endif
