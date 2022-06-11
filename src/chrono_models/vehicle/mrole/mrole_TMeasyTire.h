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
// mrole TMeasy tire subsystem
//
// Updated: 2018-02-24
// =============================================================================

#ifndef MROLE_TMEASY_TIRE_H
#define MROLE_TMEASY_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_mrole
/// @{

/// TMeasy tire model for the mrole, for ON ROAD operation
class CH_MODELS_API mrole_TMeasyTire : public ChTMeasyTire {
  public:
    mrole_TMeasyTire(const std::string& name);
    ~mrole_TMeasyTire() {}

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
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// TMeasy tire model for the mrole, for OFFROAD operation on deformable soils
class CH_MODELS_API mrole_TMeasyTireSoil : public ChTMeasyTire {
  public:
    mrole_TMeasyTireSoil(const std::string& name);
    ~mrole_TMeasyTireSoil() {}

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
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// TMeasy tire model for the mrole, for OFFROAD operation on deformable sand
class CH_MODELS_API mrole_TMeasyTireSand : public ChTMeasyTire {
  public:
    mrole_TMeasyTireSand(const std::string& name);
    ~mrole_TMeasyTireSand() {}

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
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_mrole

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
