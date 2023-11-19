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
// Authors: Rainer Gericke
// =============================================================================
//
// G500 TMeasy tire subsystem
//
// Updated: 2018-02-24
// =============================================================================

#ifndef G500_TMEASY_TIRE_H
#define G500_TMEASY_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace gclass {

/// @addtogroup vehicle_models_gclass
/// @{

/// TMeasy tire model for the UAZBUS (front)
class CH_MODELS_API G500_TMeasyTireFront : public ChTMeasyTire {
  public:
    G500_TMeasyTireFront(const std::string& name);
    ~G500_TMeasyTireFront() {}

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetTMeasyParams() override;
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void GenerateCharacteristicPlots(const std::string& dirname);

  private:
    static const double m_mass;
    static const ChVector<> m_inertia;

    ChFunction_Recorder m_stiffnessMap;

    static const std::string m_meshFile;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// TMeasy tire model for the G500 (rear)
class CH_MODELS_API G500_TMeasyTireRear : public ChTMeasyTire {
  public:
    G500_TMeasyTireRear(const std::string& name);
    ~G500_TMeasyTireRear() {}

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetTMeasyParams() override;
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void GenerateCharacteristicPlots(const std::string& dirname);

  private:
    static const double m_mass;
    static const ChVector<> m_inertia;

    ChFunction_Recorder m_stiffnessMap;

    static const std::string m_meshFile;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_models_gclass

}  // end namespace gclass
}  // end namespace vehicle
}  // end namespace chrono

#endif
