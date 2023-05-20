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
// FEDA Magic Formula tire subsystem
//
// =============================================================================

#ifndef FEDA_MF_TIRE_H
#define FEDA_MF_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChMFTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace feda {

/// @addtogroup vehicle_models_feda
/// @{

/// MF tire model for the FEDA vehicle.
class CH_MODELS_API FEDA_MFTire : public ChMFTire {
  public:
    FEDA_MFTire(const std::string& name, unsigned int pressure_level = 2);
    ~FEDA_MFTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_par.VERTICAL_DAMPING * velocity;
    }

    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual double GetVisualizationWidth() const override { return m_par.WIDTH; }

    virtual void SetMFParams() override;

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void SetPressureLevel(unsigned int p_level) { m_tire_inflation_pressure_level = p_level; }

  private:
    static const double m_mass;
    static const ChVector<> m_inertia;
    ChFunction_Recorder m_vert_map;
    bool m_use_vert_map;
    ChFunction_Recorder m_bott_map;
    bool m_use_bott_map;
    unsigned int m_tire_inflation_pressure_level;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;

    void SetParametersLevel1();
    void SetParametersLevel2();
    void SetParametersLevel3();
    void SetParametersLevel4();
};

/// @} vehicle_models_hmmwv

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono

#endif

