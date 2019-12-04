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
// HMMWV Fiala tire subsystem
//
// =============================================================================

#ifndef HMMWV_FIALA_TIRE_H
#define HMMWV_FIALA_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Fiala tire model for the HMMWV vehicle.
class CH_MODELS_API HMMWV_FialaTire : public ChFialaTire {
  public:
    HMMWV_FialaTire(const std::string& name);
    ~HMMWV_FialaTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override;

    virtual double GetVisualizationWidth() const override { return 0.25; }

    virtual void SetFialaParams() override;

    virtual double GetMass() const override { return m_mass; }
    virtual ChVector<> GetInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    ChFunction_Recorder m_vert_map;
    double m_max_depth;
    double m_max_val;
    double m_slope;

    static const double m_normalDamping;
    static const double m_mass;
    static const ChVector<> m_inertia;

    static const std::string m_meshName;
    static const std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
