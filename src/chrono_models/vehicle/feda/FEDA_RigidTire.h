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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// FEDA rigid tire subsystem
//
// =============================================================================

#ifndef FEDA_RIGID_TIRE_H
#define FEDA_RIGID_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace feda {

/// @addtogroup vehicle_models_feda
/// @{

/// Rigid tire model for the FEDA vehicle.
class CH_MODELS_API FEDA_RigidTire : public ChRigidTire {
  public:
    FEDA_RigidTire(const std::string& name, bool use_mesh = false);
    ~FEDA_RigidTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

  private:
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;
    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector<> m_inertia;

    static const std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_feda

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono

#endif
