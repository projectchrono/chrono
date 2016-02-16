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
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurfaceBase.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace m113 {

class M113_Vehicle : public chrono::vehicle::ChTrackedVehicle {
  public:
    M113_Vehicle(bool fixed,
                 chrono::ChMaterialSurfaceBase::ContactMethod contactMethod = chrono::ChMaterialSurfaceBase::DVI);

    M113_Vehicle(bool fixed, chrono::ChSystem* system);

    ~M113_Vehicle() {}

    virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos) override;

    void SetChassisVisType(chrono::vehicle::VisualizationType vis);
    void SetSprocketVisType(chrono::vehicle::VisualizationType vis);
    void SetIdlerVisType(chrono::vehicle::VisualizationType vis);
    void SetRoadWheelVisType(chrono::vehicle::VisualizationType vis);
    void SetTrackShoeVisType(chrono::vehicle::VisualizationType vis);

    void ExportMeshPovray(const std::string& out_dir);

  private:
    void Create(bool fixed);

    // Chassis visualization mesh
    static const std::string m_chassisMeshName;
    static const std::string m_chassisMeshFile;

    // Chassis mass properties
    static const double m_chassisMass;
    static const chrono::ChVector<> m_chassisCOM;
    static const chrono::ChVector<> m_chassisInertia;

    // Driver local coordinate system
    static const chrono::ChCoordsys<> m_driverCsys;

    chrono::vehicle::VisualizationType m_chassisVisType;
};

}  // end namespace m113

#endif
