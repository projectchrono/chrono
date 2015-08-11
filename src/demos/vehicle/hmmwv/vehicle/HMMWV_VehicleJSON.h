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
// Uses JSON input files for subsystem templates
//
// =============================================================================

#ifndef HMMWV_VEHICLE_JSON_H
#define HMMWV_VEHICLE_JSON_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/driveline/ShaftsDriveline2WD.h"
#include "chrono_vehicle/wheel/Wheel.h"
#include "chrono_vehicle/brake/BrakeSimple.h"

#include "ModelDefs.h"

namespace hmmwv {

class HMMWV_VehicleJSON : public chrono::ChVehicle
{
public:

  HMMWV_VehicleJSON(const bool        fixed = false,
                    VisualizationType chassisVis = NONE);

  ~HMMWV_VehicleJSON();

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
