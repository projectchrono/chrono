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
// Authors: Rainer Gericke, Asher Elmquist, Evan Hoerl, Shuo He
// =============================================================================
//
// CityBus TMeasy tire subsystem
//
// =============================================================================

#ifndef CITYBUS_TMEASY_TIRE_H
#define CITYBUS_TMEASY_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace citybus {

/// @addtogroup vehicle_models_citybus
/// @{

/// TMeasy tire model for the Citybus vehicle.
class CH_MODELS_API CityBus_TMeasyTire : public ChTMeasyTire {
  public:
    CityBus_TMeasyTire(const std::string& name);
    ~CityBus_TMeasyTire() {}

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetTMeasyParams() override;
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector3d GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void GenerateCharacteristicPlots(const std::string& dirname);

  private:
    static const std::string m_meshFile;
    static const double m_mass;
    static const ChVector3d m_inertia;

    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
    ChFunctionInterp m_stiffnessMap;
};

/// @} vehicle_models_citybus

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono

#endif
