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
// Vehicle model constructed from a JSON specification file
//
// =============================================================================

#ifndef VEHICLE_H
#define VEHICLE_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChVehicle.h"
#include "subsys/ChSuspension.h"
#include "subsys/ChSteering.h"
#include "subsys/ChDriveline.h"
#include "subsys/ChWheel.h"
#include "subsys/ChBrake.h"

namespace chrono {

class CH_SUBSYS_API Vehicle : public chrono::ChVehicle
{
public:

  Vehicle(const std::string& filename,
          bool               fixed = false);

  ~Vehicle();

  virtual int GetNumberAxles() const { return m_num_axles; }

  virtual ChSharedPtr<ChBody> GetWheelBody(const ChWheelID& wheelID) const;

  virtual const ChVector<>& GetWheelPos(const ChWheelID& wheel_id) const;
  virtual const ChQuaternion<>& GetWheelRot(const ChWheelID& wheel_id) const;
  virtual const ChVector<>& GetWheelLinVel(const ChWheelID& wheel_id) const;
  virtual ChVector<> GetWheelAngVel(const ChWheelID& wheel_id) const;
  virtual double GetWheelOmega(const ChWheelID& wheel_id) const;

  virtual ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

  virtual void Initialize(const ChCoordsys<>& chassisPos);
  virtual void Update(double              time,
                      double              steering,
                      double              powertrain_torque,
                      double              braking,
                      const ChTireForces& tire_forces);

  virtual void LogConstraintViolations();

  void ExportMeshPovray(const std::string& out_dir);

private:

  void LoadSteering(const std::string& filename);
  void LoadDriveline(const std::string& filename);
  void LoadSuspension(const std::string& filename, int axle, bool driven);
  void LoadWheel(const std::string& filename, int axle, int side);
  void LoadBrake(const std::string& filename, int axle, int side);

private:

  int                      m_num_axles;       // number of axles for this vehicle

  ChSuspensionList         m_suspensions;     // list of handles to suspension subsystems
  std::vector<ChVector<> > m_suspLocations;   // locations of the suspensions relative to chassis

  ChSharedPtr<ChSteering>  m_steering;        // handle to the steering subsystem
  ChVector<>               m_steeringLoc;     // location of the steering relative to chassis
  ChQuaternion<>           m_steeringRot;     // orientation of the steering relative to chassis
  int                      m_steer_susp;      // index of the steered suspension

  ChSharedPtr<ChDriveline> m_driveline;       // handle to the driveline subsystem
  std::vector<int>         m_driven_susp;     // indexes of the driven suspensions

  ChWheelList              m_wheels;          // list of handles to wheel subsystems
  ChBrakeList              m_brakes;          // list of handles to brake subsystems

  bool        m_chassisUseMesh;               // true if using a mesh for chassis visualization
  std::string m_chassisMeshName;              // name of the chassis visualization mesh
  std::string m_chassisMeshFile;              // name of the Waveform file with the chassis mesh

  double     m_chassisMass;                   // chassis mass
  ChVector<> m_chassisCOM;                    // location of the chassis COM in the chassis reference frame
  ChVector<> m_chassisInertia;                // moments of inertia of the chassis

  ChCoordsys<> m_driverCsys;                  // driver position and orientation relative to chassis
};


} // end namespace chrono


#endif
