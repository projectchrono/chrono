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
// U401 TMeasy tire subsystem
//
// Updated: 2018-02-24
// =============================================================================

#ifndef HMMWV_TMSIMPLE_TIRE_H
#define HMMWV_TMSIMPLE_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMsimpleTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_U401
/// @{

/// TMeasy tire model for the U401.
class CH_MODELS_API HMMWV_TMsimpleTire : public ChTMsimpleTire {
   public:
    HMMWV_TMsimpleTire(const std::string& name);
    ~HMMWV_TMsimpleTire() {}

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetTMsimpleParams() override;
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector<> GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void GenerateCharacteristicPlots(const std::string& dirname);

   private:
    static const double m_mass;
    static const ChVector<> m_inertia;

    ChFunction_Recorder m_stiffnessMap;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_U401

}  // namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
