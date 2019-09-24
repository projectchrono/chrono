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
// HMMWV Pacejka 2002 tire subsystem
//
// =============================================================================

#ifndef HMMWV_PACEJKA_TIRE_H
#define HMMWV_PACEJKA_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Pacejka 2002 tire model for the HMMWV vehicle.
class CH_MODELS_API HMMWV_PacejkaTire : public ChPacejkaTire {
  public:
    HMMWV_PacejkaTire(const std::string& name);
    ~HMMWV_PacejkaTire() {}

    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    static const double m_mass;
    static const ChVector<> m_inertia;

    static const std::string m_pacTireFile;

    static const std::string m_meshName;
    static const std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
