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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Generic PAC89 tire subsystem
//
// =============================================================================

#ifndef GENERIC_PAC89_TIRE_H
#define GENERIC_PAC89_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChPac89Tire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// PAC89 tire model for the Generic vehicle vehicle.
class CH_MODELS_API Generic_Pac89Tire : public ChPac89Tire {
  public:
    Generic_Pac89Tire(const std::string& name);
    ~Generic_Pac89Tire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_normalDamping * velocity;
    }

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetPac89Params() override;

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    static const double m_normalDamping;
    static const double m_mass;
    static const ChVector<> m_inertia;
    ChFunction_Recorder m_vert_map;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
