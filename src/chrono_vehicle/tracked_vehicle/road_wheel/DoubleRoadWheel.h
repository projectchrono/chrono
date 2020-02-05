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
// Double road-wheel model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef DOUBLE_ROAD_WHEEL_H
#define DOUBLE_ROAD_WHEEL_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/road_wheel/ChDoubleRoadWheel.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_suspension
/// @{

/// Double road-wheel model constructed with data from file (JSON format).
class CH_VEHICLE_API DoubleRoadWheel : public ChDoubleRoadWheel {
  public:
    DoubleRoadWheel(const std::string& filename);
    DoubleRoadWheel(const rapidjson::Document& d);
    ~DoubleRoadWheel() {}

    virtual double GetWheelRadius() const override { return m_wheel_radius; }
    virtual double GetWheelWidth() const override { return m_wheel_width; }
    virtual double GetWheelGap() const override { return m_wheel_gap; }

    virtual double GetWheelMass() const override { return m_wheel_mass; }
    virtual const ChVector<>& GetWheelInertia() override { return m_wheel_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_wheel_radius;
    double m_wheel_width;
    double m_wheel_gap;

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
