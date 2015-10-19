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

#include "chrono_vehicle/ChVehicle.h"

#include "ModelDefs.h"
#include "hmmwv/wheel/HMMWV_Wheel.h"
#include "hmmwv/suspension/HMMWV_DoubleWishbone.h"
#include "hmmwv/steering/HMMWV_PitmanArm.h"
#include "hmmwv/driveline/HMMWV_Driveline2WD.h"
#include "hmmwv/driveline/HMMWV_Driveline4WD.h"
#include "hmmwv/brake/HMMWV_BrakeSimple.h"

namespace hmmwv {

class HMMWV_Vehicle : public chrono::ChVehicle
{
public:
  HMMWV_Vehicle(const bool fixed = false,
                DrivelineType driveType = AWD,
                VisualizationType chassisVis = NONE,
                VisualizationType wheelVis = PRIMITIVES,
                chrono::ChMaterialSurfaceBase::ContactMethod contactMethod = chrono::ChMaterialSurfaceBase::DVI);

  ~HMMWV_Vehicle();

  virtual int GetNumberAxles() const { return 2; }

  virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

  double GetSpringForce(const chrono::ChWheelID& wheel_id) const;
  double GetSpringLength(const chrono::ChWheelID& wheel_id) const;
  double GetSpringDeformation(const chrono::ChWheelID& wheel_id) const;

  double GetShockForce(const chrono::ChWheelID& wheel_id) const;
  double GetShockLength(const chrono::ChWheelID& wheel_id) const;
  double GetShockVelocity(const chrono::ChWheelID& wheel_id) const;

  virtual void Initialize(const chrono::ChCoordsys<>& chassisPos);

  void ExportMeshPovray(const std::string& out_dir);

  // Log debugging information
  void LogHardpointLocations(); /// suspension hardpoints at design
  void DebugLog(int what);      /// shock forces and lengths, constraints, etc.

private:

  DrivelineType m_driveType;

  // Chassis visualization mesh
  static const std::string m_chassisMeshName;
  static const std::string m_chassisMeshFile;

  // Chassis mass properties
  static const double             m_chassisMass;
  static const chrono::ChVector<> m_chassisCOM;
  static const chrono::ChVector<> m_chassisInertia;

  // Driver local coordinate system
  static const chrono::ChCoordsys<> m_driverCsys;
};


} // end namespace hmmwv


#endif
