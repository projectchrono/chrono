// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All rights reserved.
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

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

#include "chrono_thirdparty/rapidjson/document.h"

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
    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_visualization_width; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_normalStiffness;
    double m_normalDamping;

    double m_mass;
    ChVector<> m_inertia;

    bool m_has_mesh;
    std::string m_meshName;
    std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;

    double m_visualization_width;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
