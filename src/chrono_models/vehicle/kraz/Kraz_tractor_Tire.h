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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Kraz 64431 TMeasy tire subsystem 12.00R20 150
//
// =============================================================================

#ifndef KRAZ_TRACTOR_TIRE_H
#define KRAZ_TRACTOR_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// @addtogroup vehicle_models_kraz
/// @{

/// TMeasy tire model for the Kraz tractor.
class CH_MODELS_API Kraz_tractor_Tire : public ChTMeasyTire {
  public:
    Kraz_tractor_Tire(const std::string& name);
    ~Kraz_tractor_Tire() {}

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetTMeasyParams() override;
    virtual double GetTireMass() const override { return m_mass; }
    virtual chrono::ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void GenerateCharacteristicPlots(const std::string& dirname);

  private:
    static const double m_mass;
    static const chrono::ChVector<> m_inertia;
    ChFunction_Recorder m_stiffnessMap;

    static const std::string m_meshFile;
    std::shared_ptr<chrono::ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_kraz

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
