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

private:

  void LoadSteering(const std::string& filename);
  void LoadSuspension(const std::string& filename, int axle, bool driven);
  void LoadWheel(const std::string& filename, int axle, int side);

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
};


} // end namespace chrono


#endif
