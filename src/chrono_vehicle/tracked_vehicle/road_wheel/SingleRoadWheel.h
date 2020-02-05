// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Single road-wheel model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef SINGLE_ROAD_WHEEL_H
#define SINGLE_ROAD_WHEEL_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChSingleRoadWheel.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Single road-wheel model constructed with data from file (JSON format).
class CH_VEHICLE_API SingleRoadWheel : public ChSingleRoadWheel {
  public:
    SingleRoadWheel(const std::string& filename);
    SingleRoadWheel(const rapidjson::Document& d);
    ~SingleRoadWheel() {}

    virtual double GetWheelRadius() const override { return m_wheel_radius; }
    virtual double GetWheelWidth() const override { return m_wheel_width; }

    virtual double GetWheelMass() const override { return m_wheel_mass; }
    virtual const ChVector<>& GetWheelInertia() override { return m_wheel_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_wheel_radius;
    double m_wheel_width;

    double m_wheel_mass;
    ChVector<> m_wheel_inertia;

    bool m_has_mesh;
    std::string m_meshName;
    std::string m_meshFile;
};

/// @} vehicle_tracked_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
