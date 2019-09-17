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
// Pacejka tire constructed with data from file (JSON format).
//
// =============================================================================

#ifndef PACEJKA_TIRE_H
#define PACEJKA_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"
//
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"
//
#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Pacejka tire constructed with data from file (JSON format).
class CH_VEHICLE_API PacejkaTire : public ChPacejkaTire {
  public:
    PacejkaTire(const std::string& filename);
    PacejkaTire(const rapidjson::Document& d);
    ~PacejkaTire() {}

    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_mass;
    ChVector<> m_inertia;

    bool m_has_mesh;
    std::string m_meshName;
    std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
