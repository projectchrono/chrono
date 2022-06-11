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
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// CityBus PAC02 tire subsystem
//
// =============================================================================

#ifndef CITYBUS_PAC02_TIRE_H
#define CITYBUS_PAC02_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace citybus {

/// @addtogroup vehicle_models_citybus
/// @{

/// PAC89 tire model for the CityBus vehicle.
class CH_MODELS_API CityBus_Pac02Tire : public ChPac02Tire {
  public:
    CityBus_Pac02Tire(const std::string& name);
    ~CityBus_Pac02Tire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_PacCoeff.Kz * velocity;
    }

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_PacCoeff.width; }

    virtual void SetPac02Params() override;

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    static const double m_mass;
    static const ChVector<> m_inertia;
    ChFunction_Recorder m_vert_map;
    bool m_use_vert_map;

    static const std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_citybus

}  // namespace citybus
}  // end namespace vehicle
}  // end namespace chrono

#endif
