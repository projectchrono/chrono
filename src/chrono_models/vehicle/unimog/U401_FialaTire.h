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
// U401 Fiala tire subsystem
//
// =============================================================================

#ifndef U401_FIALA_TIRE_H
#define U401_FIALA_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace unimog {

/// @addtogroup vehicle_models_U401unimgo
/// @{

/// Fiala tire model for the U401unimgo vehicle.
class CH_MODELS_API U401_FialaTire : public ChFialaTire {
  public:
    U401_FialaTire(const std::string& name);
    ~U401_FialaTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override;

    virtual double GetVisualizationWidth() const override { return 0.25; }

    virtual void SetFialaParams() override;

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

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

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_U401

}  // end namespace unimog
}  // end namespace vehicle
}  // end namespace chrono

#endif

