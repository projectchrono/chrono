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
// Vehicle simple brake model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef BRAKE_SIMPLE_H
#define BRAKE_SIMPLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/brake/ChBrakeSimple.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_VEHICLE_API BrakeSimple : public ChBrakeSimple
{
public:

  BrakeSimple(const std::string& filename);
  BrakeSimple(const rapidjson::Document& d);
  ~BrakeSimple() {}

  virtual double GetMaxBrakingTorque() { return m_maxtorque; }

private:

  void Create(const rapidjson::Document& d);

  double      m_maxtorque;
};


} // end namespace chrono


#endif
