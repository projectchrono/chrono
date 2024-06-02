// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// BMW E90 (330i 2006) TMsimple front tire subsystem 225/40 R18 92W
// Parameters calculated from NADS Tire Model Data Set
//
// =============================================================================

#ifndef BMW_E90_TMSIMPLE_TIRE_FRONT_H
#define BMW_E90_TMSIMPLE_TIRE_FRONT_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChTMsimpleTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// TMeasy tire model for the BMW 330i.
class CH_MODELS_API BMW_E90_TMsimpleTireFront : public ChTMsimpleTire {
  public:
    BMW_E90_TMsimpleTireFront(const std::string& name);
    ~BMW_E90_TMsimpleTireFront() {}

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetTMsimpleParams() override;
    virtual double GetTireMass() const override { return m_mass; }
    virtual ChVector3d GetTireInertia() const override { return m_inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

    void GenerateCharacteristicPlots(const std::string& dirname);

  private:
    static const double m_mass;
    static const ChVector3d m_inertia;

    ChFunctionInterp m_stiffnessMap;

    static const std::string m_meshFile_left;
    static const std::string m_meshFile_right;
    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
