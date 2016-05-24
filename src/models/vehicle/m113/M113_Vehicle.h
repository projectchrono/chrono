// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// M113 vehicle model.
//
// =============================================================================

#ifndef M113_VEHICLE_H
#define M113_VEHICLE_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurfaceBase.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

class CH_MODELS_API M113_Vehicle : public ChTrackedVehicle {
  public:
    M113_Vehicle(bool fixed,
                 ChMaterialSurfaceBase::ContactMethod contactMethod = ChMaterialSurfaceBase::DVI);

    M113_Vehicle(bool fixed, ChSystem* system);

    ~M113_Vehicle() {}

    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    virtual void Initialize(const ChCoordsys<>& chassisPos) override;

    void SetChassisVisType(VisualizationType vis);
    void SetSprocketVisType(VisualizationType vis);
    void SetIdlerVisType(VisualizationType vis);
    void SetRoadWheelVisType(VisualizationType vis);
    void SetTrackShoeVisType(VisualizationType vis);

    void ExportMeshPovray(const std::string& out_dir);

  private:
    void Create(bool fixed);

    // Chassis visualization mesh
    static const std::string m_chassisMeshName;
    static const std::string m_chassisMeshFile;

    // Chassis mass properties
    static const double m_chassisMass;
    static const ChVector<> m_chassisCOM;
    static const ChVector<> m_chassisInertia;

    // Driver local coordinate system
    static const ChCoordsys<> m_driverCsys;

    VisualizationType m_chassisVisType;
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
