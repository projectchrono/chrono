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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// HMMWV full vehicle model...
//
// =============================================================================

#ifndef HMMWV_VEHICLE_H
#define HMMWV_VEHICLE_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurfaceBase.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "models/ChApiModels.h"
#include "models/vehicle/hmmwv/HMMWV_Wheel.h"
#include "models/vehicle/hmmwv/HMMWV_DoubleWishbone.h"
#include "models/vehicle/hmmwv/HMMWV_PitmanArm.h"
#include "models/vehicle/hmmwv/HMMWV_Driveline2WD.h"
#include "models/vehicle/hmmwv/HMMWV_Driveline4WD.h"
#include "models/vehicle/hmmwv/HMMWV_BrakeSimple.h"

namespace hmmwv {

class CH_MODELS_API HMMWV_Vehicle : public chrono::vehicle::ChWheeledVehicle {
  public:
    HMMWV_Vehicle(const bool fixed = false,
                  chrono::vehicle::DrivelineType driveType = chrono::vehicle::AWD,
                  chrono::vehicle::VisualizationType chassisVis = chrono::vehicle::NONE,
                  chrono::vehicle::VisualizationType wheelVis = chrono::vehicle::PRIMITIVES,
                  chrono::ChMaterialSurfaceBase::ContactMethod contactMethod = chrono::ChMaterialSurfaceBase::DVI);

    HMMWV_Vehicle(chrono::ChSystem* system,
                  const bool fixed = false,
                  chrono::vehicle::DrivelineType driveType = chrono::vehicle::AWD,
                  chrono::vehicle::VisualizationType chassisVis = chrono::vehicle::NONE,
                  chrono::vehicle::VisualizationType wheelVis = chrono::vehicle::PRIMITIVES);

    ~HMMWV_Vehicle();

    virtual int GetNumberAxles() const override { return 2; }

    virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    double GetSpringForce(const chrono::vehicle::WheelID& wheel_id) const;
    double GetSpringLength(const chrono::vehicle::WheelID& wheel_id) const;
    double GetSpringDeformation(const chrono::vehicle::WheelID& wheel_id) const;

    double GetShockForce(const chrono::vehicle::WheelID& wheel_id) const;
    double GetShockLength(const chrono::vehicle::WheelID& wheel_id) const;
    double GetShockVelocity(const chrono::vehicle::WheelID& wheel_id) const;

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos) override;

    void ExportMeshPovray(const std::string& out_dir);

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    void Create(bool fixed, chrono::vehicle::VisualizationType chassisVis, chrono::vehicle::VisualizationType wheelVis);

    chrono::vehicle::DrivelineType m_driveType;

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

}  // end namespace hmmwv

#endif
