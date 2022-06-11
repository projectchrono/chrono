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
// TMeasyTire tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef TMEASY_TIRE_H
#define TMEASY_TIRE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// TMeasy tire constructed with data from file (JSON format).
class CH_VEHICLE_API TMeasyTire : public ChTMeasyTire {
  public:
    TMeasyTire(const std::string& filename);
    TMeasyTire(const rapidjson::Document& d);
    ~TMeasyTire() {}

    virtual void SetTMeasyParams() override {}
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_mass;
    ChVector<> m_inertia;

    bool m_has_mesh;
    std::string m_meshFile_left;
    std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
