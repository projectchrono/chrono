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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Vehicle wheel constructed with data from file (JSON format).
//
// =============================================================================

#ifndef WHEEL_H
#define WHEEL_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_wheel
/// @{

/// Vehicle wheel constructed with data from file (JSON format).
class CH_VEHICLE_API Wheel : public ChWheel {
  public:
    Wheel(const std::string& filename);
    Wheel(const rapidjson::Document& d);
    ~Wheel() {}

    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }

    void SetRadius(double rad) { m_radius = rad; }
    void SetWidth(double width) { m_width = width; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override;

  private:
    void Create(const rapidjson::Document& d);

    double m_mass;
    ChVector<> m_inertia;

    double m_radius;
    double m_width;
    bool m_has_mesh;
    std::string m_meshName;
    std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_wheeled_wheel

}  // end namespace vehicle
}  // end namespace chrono

#endif
