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
// A simple test program to verify the Pacejka tire Update function.
//
// =============================================================================

#include <iostream>

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

#include "subsys/tire/ChPacejkaTire.h"
#include "subsys/terrain/FlatTerrain.h"

#include "ChronoT_config.h"

using namespace chrono;
using std::cout;
using std::endl;

static const double rad2deg = 180 / CH_C_PI;

const std::string pacParamFile = utils::GetModelDataFile("hmmwv/pactest.tir");

// -----------------------------------------------------------------------------
// Process the specified wheel state using two different ChPacejkaTire functions
// -----------------------------------------------------------------------------
void processState(ChSharedPtr<ChPacejkaTire> tire,
                  const ChWheelState&        state)
{
  cout << "--------------------------------------------------" << endl;
  cout << "Position:     " << state.pos.x << "  " << state.pos.y << "  " << state.pos.z << endl;
  cout << "Orientation:  " << state.rot.e0 << "  " << state.rot.e1 << "  " << state.rot.e2 << "  " << state.rot.e3 << endl;
  cout << "Linear vel.:  " << state.lin_vel.x << "  " << state.lin_vel.y << "  " << state.lin_vel.z << endl;
  cout << "Angular vel.: " << state.ang_vel.x << "  " << state.ang_vel.y << "  " << state.ang_vel.z << endl;
  cout << "Wheel omega:  " << state.omega << endl << endl;

  tire->Update(0, state);
  double kappa = tire->get_kappa();
  double alpha = tire->get_alpha();
  double gamma = tire->get_gamma();
  double R_eff = tire->get_tire_rolling_rad();
  cout << "Update()" << endl;
  cout << "  alpha = " << alpha << " rad = " << alpha * rad2deg << " deg" << endl;
  cout << "  gamma = " << gamma << " rad = " << gamma * rad2deg << " deg" << endl << endl;
  cout << "  R_eff = " << R_eff << endl;
  cout << "  kappa = " << kappa << endl;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  SetChronoDataPath(CHRONO_DATA_DIR);

  // Flat rigid terrain, height = 0 for all (x,y)
  FlatTerrain flat_terrain(0);

  // Create a Pacejka tire
  ChSharedPtr<ChPacejkaTire> tire(new ChPacejkaTire(pacParamFile, flat_terrain));

  // Create different wheel state and let the tire process them
  {
    // Wheel with heading = -30 deg and camber = 0 deg
    ChQuaternion<> rz(0.965926, 0, 0, -0.25882);   // rot_z(-30)
    ChQuaternion<> rx(1, 0, 0, 0);                 // rot_x(0)
    ChQuaternion<> rot = rz * rx;

    ChWheelState state;
    state.pos = ChVector<>(2, 1, 0);
    state.rot = rot;
    state.lin_vel = ChVector<>(6.427876, -7.660444, 0);    // ||V|| = 10, alpha = -20 degrees
    state.ang_vel = 5.0 * rot.GetYaxis();                  // ||omega|| = 5
    state.omega = +5;                                      // omega > 0, wheel moves forward

    processState(tire, state);
  }

  {
    // Test the utility function getState_from_KAG
    ChWheelState state = tire->getState_from_KAG(
      tire->get_kappa(),
      tire->get_alpha(),
      tire->get_gamma(),
      10
      );

    processState(tire, state);
  }

  {
    // Wheel with heading = 135 deg and camber = 10 deg
    ChQuaternion<> rz(0.3826834, 0, 0, 0.9238795);   // rot_z(135)
    ChQuaternion<> rx(0.9961947, 0.08715574, 0, 0);  // rot_x(10)
    ChQuaternion<> rot = rz * rx;

    ChWheelState state;
    state.pos = ChVector<>(2, 1, 0);
    state.rot = rot;
    state.lin_vel = ChVector<>(-10, 0, 0);                 // ||V|| = 10, alpha = 45 degrees
    state.ang_vel = 5.0 * rot.GetYaxis();                  // ||omega|| = 5
    state.omega = +5;                                      // omega > 0, wheel moves forward

    processState(tire, state);
  }

  {
    // Test the utility function getState_from_KAG
    ChWheelState state = tire->getState_from_KAG(
      tire->get_kappa(),
      tire->get_alpha(),
      tire->get_gamma(),
      10
      );

    processState(tire, state);
  }


  return 0;
}
