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
    M113_Vehicle(const bool fixed,
                 chrono::vehicle::VisualizationType chassisVis,
                 chrono::vehicle::VisualizationType trackVis,
                 chrono::ChMaterialSurfaceBase::ContactMethod contactMethod = chrono::ChMaterialSurfaceBase::DVI);

    ~M113_Vehicle() {}

    virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos) override;

    void ExportMeshPovray(const std::string& out_dir);

  private:
    // Chassis visualization mesh
    static const std::string m_chassisMeshName;
    static const std::string m_chassisMeshFile;

    // Chassis mass properties
    static const double m_chassisMass;
    static const chrono::ChVector<> m_chassisCOM;
    static const chrono::ChVector<> m_chassisInertia;

    // Driver local coordinate system
    static const chrono::ChCoordsys<> m_driverCsys;
};

}  // end namespace m113

#endif
