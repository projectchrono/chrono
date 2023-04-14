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
// MAN5T TMsimple tire subsystem
//
// =============================================================================

#ifndef MAN5T_TMsimple_TIRE_H
#define MAN5T_TMsimple_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMsimpleTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// TMsimple tire model for the HMMWV vehicle.
class CH_MODELS_API MAN_5t_TMsimpleTire : public ChTMsimpleTire {
  public:
    MAN_5t_TMsimpleTire(const std::string& name);
    ~MAN_5t_TMsimpleTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override;

    virtual double GetVisualizationWidth() const override { return 0.25; }

    virtual void SetTMsimpleParams() override;

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

/// @} vehicle_models_man

}  // end namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif


