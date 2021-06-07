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
// mrole rigid tire subsystem
//
// =============================================================================

#ifndef MROLE_RIGID_TIRE_H
#define MROLE_RIGID_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Rigid tire model for the mrole vehicle.
class CH_MODELS_API mrole_RigidTire : public ChRigidTire {
  public:
    mrole_RigidTire(const std::string& name, bool use_mesh = false);
    ~mrole_RigidTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }
    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

  private:
    virtual void CreateContactMaterial(ChContactMethod contact_method) override;
    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    static const double m_radius;
    static const double m_width;
    static const double m_mass;
    static const ChVector<> m_inertia;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;

    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_hmmwv

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
