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
// Double-A arm suspension constructed with data from file.
//
// =============================================================================

#ifndef DOUBLEWISHBONE_H
#define DOUBLEWISHBONE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/suspension/ChDoubleWishbone.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_VEHICLE_API DoubleWishbone : public ChDoubleWishbone
{
public:

  DoubleWishbone(const std::string& filename);
  DoubleWishbone(const rapidjson::Document& d);
  ~DoubleWishbone();

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUCAMass() const { return m_UCAMass; }
  virtual double getLCAMass() const { return m_LCAMass; }
  virtual double getUprightMass() const { return m_uprightMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUCARadius() const { return m_UCARadius; }
  virtual double getLCARadius() const { return m_LCARadius; }
  virtual double getUprightRadius() const { return m_uprightRadius; }

  virtual const ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const ChVector<>& getUCAInertia() const { return m_UCAInertia; }
  virtual const ChVector<>& getLCAInertia() const { return m_LCAInertia; }
  virtual const ChVector<>& getUprightInertia() const { return m_uprightInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringRestLength() const { return m_springRestLength; }
  virtual ChSpringForceCallback* getSpringForceCallback() const { return m_springForceCB; }
  virtual ChSpringForceCallback* getShockForceCallback()  const { return m_shockForceCB; }

private:

  virtual const ChVector<> getLocation(PointId which) { return m_points[which]; }

  void Create(const rapidjson::Document& d);

  ChSpringForceCallback* m_springForceCB;
  ChSpringForceCallback* m_shockForceCB;

  ChVector<>  m_points[NUM_POINTS];

  double      m_spindleMass;
  double      m_UCAMass;
  double      m_LCAMass;
  double      m_uprightMass;

  double      m_spindleRadius;
  double      m_spindleWidth;
  double      m_UCARadius;
  double      m_LCARadius;
  double      m_uprightRadius;

  ChVector<>  m_spindleInertia;
  ChVector<>  m_UCAInertia;
  ChVector<>  m_LCAInertia;
  ChVector<>  m_uprightInertia;

  double      m_axleInertia;

  double      m_springRestLength;
};


} // end namespace chrono


#endif
