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
#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Fiala tire constructed with data from file (JSON format).
class CH_VEHICLE_API FialaTire : public ChFialaTire {
  public:
    FialaTire(const std::string& filename);
    FialaTire(const rapidjson::Document& d);
    ~FialaTire();

    virtual double GetNormalStiffnessForce(double depth) const override { return m_normalStiffness * depth; }
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_normalDamping * velocity;
    }

    virtual void SetFialaParams() override {}

  private:
    void Create(const rapidjson::Document& d);

    double m_normalStiffness;
    double m_normalDamping;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
