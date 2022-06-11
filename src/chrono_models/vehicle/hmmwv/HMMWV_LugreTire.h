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
// HMMWV LuGre tire subsystem
//
// =============================================================================

#ifndef HMMWV_LUGRE_TIRE_H
#define HMMWV_LUGRE_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChLugreTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Lugre tire model for the HMMWV vehicle.
class CH_MODELS_API HMMWV_LugreTire : public ChLugreTire {
  public:
    HMMWV_LugreTire(const std::string& name);
    ~HMMWV_LugreTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }
    virtual int GetNumDiscs() const override { return m_numDiscs; }
    virtual const double* GetDiscLocations() const override { return m_discLocs; }

    virtual double GetNormalStiffnessForce(double depth) const override { return m_normalStiffness * depth; }
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_normalDamping * velocity;
    }

    virtual void SetLugreParams() override;

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    static const double m_radius;
    static const double m_mass;
    static const ChVector<> m_inertia;
    static const int m_numDiscs = 3;
    static const double m_discLocs[m_numDiscs];

    static const double m_normalStiffness;
    static const double m_normalDamping;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
