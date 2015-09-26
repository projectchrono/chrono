// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Fiala tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef FIALA_TIRE_H
#define FIALA_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tire/ChFialaTire.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_VEHICLE_API FialaTire : public ChFialaTire
{
public:

  FialaTire(const std::string&       filename,
            const chrono::ChTerrain& terrain);
  FialaTire(const rapidjson::Document& d,
            const chrono::ChTerrain&   terrain);
  ~FialaTire();

  virtual double getNormalStiffness(double depth) const      { return m_normalStiffness; }
  virtual double getNormalDamping(double depth) const        { return m_normalDamping; }

  virtual void SetFialaParams() {}


private:

  void Create(const rapidjson::Document& d);

  double   m_normalStiffness;
  double   m_normalDamping;
};


} // end namespace chrono


#endif
