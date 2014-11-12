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
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//
// Suspension tester from a JSON specification file
//
// =============================================================================

#ifndef SUSPENSIONTEST_H
#define SUSPENSIONTEST_H

#include "core/ChCoordsys.h"
#include "physics/ChSystem.h"

#include "subsys/ChSuspensionTest.h"

namespace chrono {

class CH_SUBSYS_API SuspensionTest : public ChSuspensionTest
{
public:
  /// This takes a vehicle JSON file, and only keeps data
  ///  pertinent to suspension testing rig (e.g., things on the front)
  SuspensionTest(const std::string& filename);

  ~SuspensionTest();

  virtual ChCoordsys<> GetLocalDriverCoordsys() const { return m_driverCsys; }

  virtual void Initialize(const ChCoordsys<>& chassisPos);
  /// each timestep, specify the steer angle, displacement of posts and tire forces.
  /// NOTE: not using the tire_forces, constraints result in kinematically driven test rig.
  virtual void Update(double              time,
                      double              steering,
                      double              disp_L,
                      double              disp_R,
                      const ChTireForces& tire_forces);

  /// Log info to console
  void DebugLog(int console_what);

  /// Log info to data file. data types to be saved should already set in Save_DebugLog() 
  void SaveLog();

  /// setup class to save the log to a file for python postprocessing.
  /// Usage: call after construction & Initialize(), else no data is saved.
  void Save_DebugLog(int what,
                     const std::string& out_filename = "log_SuspensionTest.csv");

  // Accessors
  double GetSpringForce(const ChWheelID& wheel_id) const;
  double GetSpringLength(const ChWheelID& wheel_id) const;
  double GetSpringDeformation(const ChWheelID& wheel_id) const;

  double GetShockForce(const chrono::ChWheelID& wheel_id) const;
  double GetShockLength(const chrono::ChWheelID& wheel_id) const;
  double GetShockVelocity(const chrono::ChWheelID& wheel_id) const;

  double GetActuatorDisp(const chrono::ChWheelID& wheel_id)const;
  double GetActuatorForce(const chrono::ChWheelID& wheel_id)const;
  double GetActuatorMarkerDist(const chrono::ChWheelID& wheel_id)const;

  // also sets local vars for re-use
  double Get_KingpinAng(const chrono::ChVehicleSide side);
  double Get_KingpinOffset(const chrono::ChVehicleSide side);
  double Get_CasterAng(const chrono::ChVehicleSide side);
  double Get_CasterOffset(const chrono::ChVehicleSide side);
  double Get_ToeAng(const chrono::ChVehicleSide side);
  double Get_LCArollAng();

  ChVector<> GetPostSurfacePos(const chrono::ChVehicleSide& side);

private:

  int                      m_num_axles;       // number of axles on test rig always 1

  std::vector<ChVector<> > m_suspLocations;   // locations of the suspensions relative to chassis

  ChVector<>               m_steeringLoc;     // location of the steering relative to chassis
  ChQuaternion<>           m_steeringRot;     // orientation of the steering relative to chassis
  int                      m_steer_susp;      // index of the steered suspension

  std::vector<int>         m_driven_susp;     // indexes of the driven suspensions

  double     m_chassisMass;                   // chassis mass
  ChVector<> m_chassisCOM;                    // location of the chassis COM in the chassis reference frame
  ChVector<> m_chassisInertia;                // moments of inertia of the chassis

  ChCoordsys<> m_driverCsys;                  // driver position and orientation relative to chassis
  double m_post_height;                       // post height, for visuals
  double m_post_rad;                          // post radius, for visuals
  bool m_save_log_to_file;                    // save the DebugLog() info to file? default false
  bool m_log_file_exists;                     // written the headers for log file yet?
  std::string m_log_file_name;
  int m_log_what;

  // rig/steer inputs 
  double m_steer;
  double m_postDisp[2];

  // suspension measurement data
  double m_KA[2]; // kingpin angle [rad] 
  double m_Koffset[2]; // kingpin offset
  double m_CA[2]; // wheel caster angle [rad]
  double m_Coffset[2];  // wheel caster offset
  double m_TA[2]; // wheel toe angle
  double m_suspensionRoll;

  // Private functions

  void LoadSteering(const std::string& filename);
  void LoadSuspension(const std::string& filename, int axle, bool driven);
  void LoadWheel(const std::string& filename, int axle, int side);


  static void AddVisualize_post(ChSharedBodyPtr post_body, 
                                ChSharedBodyPtr ground_body,
                                double height,
                                double rad,
                                const ChColor& color = ChColor(0.1f, 0.8f, 0.15f) );

  void create_fileHeader(int what);
};


} // end namespace chrono


#endif
