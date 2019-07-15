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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Shuo He
// =============================================================================
//
// CityBus rigid tire subsystem
//
// =============================================================================

#ifndef CITYBUS_RIGID_TIRE_H
#define CITYBUS_RIGID_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace citybus {

/// @addtogroup vehicle_models_citybus
/// @{

/// Rigid tire model for the CityBus vehicle.
class CH_MODELS_API CityBus_RigidTire : public ChRigidTire {
  public:
    CityBus_RigidTire(const std::string& name, bool use_mesh = false);
    ~CityBus_RigidTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector<> m_inertia;

    static const std::string m_meshName;
    static const std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_citybus

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono

#endif
