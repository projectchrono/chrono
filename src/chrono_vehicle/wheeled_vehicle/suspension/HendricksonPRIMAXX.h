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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Hendrickson PRIMAXX suspension constructed with data from file.
//
// =============================================================================

#ifndef HENDRICKSON_PRIMAXX_H
#define HENDRICKSON_PRIMAXX_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChHendricksonPRIMAXX.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_VEHICLE_API HendricksonPRIMAXX : public ChHendricksonPRIMAXX
{
public:

  HendricksonPRIMAXX(const std::string& filename);
  HendricksonPRIMAXX(const rapidjson::Document& d);
  ~HendricksonPRIMAXX();

  virtual double getSpindleMass() const { return m_spindleMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }

  virtual const ChVector<>& getSpindleInertia() const { return m_spindleInertia; }

  virtual double getAxleInertia() const { return m_axleInertia; }

private:

  virtual const ChVector<> getLocation(PointId which) { return m_points[which]; }

  void Create(const rapidjson::Document& d);

  ChVector<>  m_points[NUM_POINTS];

  double      m_spindleMass;

  double      m_spindleRadius;
  double      m_spindleWidth;

  ChVector<>  m_spindleInertia;

  double      m_axleInertia;
};


} // end namespace chrono


#endif
