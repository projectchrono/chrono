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

  /// Log info to console or file
  void DebugLog(int console_what);

  /// save the stuff printed to the log in a file
  void Save_DebugLog(int what,
                     const std::string& out_filename = "log_SuspensionTest.csv");

  // Accessors
  double GetSpringForce(const ChWheelID& wheel_id) const;
  double GetSpringLength(const ChWheelID& wheel_id) const;
  double GetSpringDeformation(const ChWheelID& wheel_id) const;

  double GetShockForce(const chrono::ChWheelID& wheel_id) const;
  double GetShockLength(const chrono::ChWheelID& wheel_id) const;
  double GetShockVelocity(const chrono::ChWheelID& wheel_id) const;

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

  // Private functions

  void LoadSteering(const std::string& filename);
  void LoadSuspension(const std::string& filename, int axle, bool driven);
  void LoadWheel(const std::string& filename, int axle, int side);


  static void AddVisualize_post(ChSharedBodyPtr post_body,
                                double height,
                                double rad);

  void create_fileHeader(const std::string& name, int what);
};


} // end namespace chrono


#endif
