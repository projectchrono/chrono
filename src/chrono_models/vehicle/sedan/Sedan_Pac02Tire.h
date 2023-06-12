// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Sedan Magic Formula tire subsystem
//
// =============================================================================

#ifndef SEDAN_PAC02_TIRE_H
#define SEDAN_PAC02_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace sedan {

/// @addtogroup vehicle_models_feda
/// @{

/// MF tire model for the FEDA vehicle.
class CH_MODELS_API Sedan_Pac02Tire : public ChPac02Tire {
   public:
    Sedan_Pac02Tire(const std::string& name, unsigned int pressure_level = 2);
    ~Sedan_Pac02Tire() {}

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_par.WIDTH; }

    virtual void SetMFParams() override;

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

   private:
    static const double m_mass;
    static const ChVector<> m_inertia;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_hmmwv

}  // namespace sedan
}  // end namespace vehicle
}  // end namespace chrono

#endif
