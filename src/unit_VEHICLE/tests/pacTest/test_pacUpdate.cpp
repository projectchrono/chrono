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
  cout << "Position:     " << state.pos.x << "  " << state.pos.y << "  " << state.pos.z << endl;
  cout << "Orientation:  " << state.rot.e0 << "  " << state.rot.e1 << "  " << state.rot.e2 << "  " << state.rot.e3 << endl;
  cout << "Linear vel.:  " << state.lin_vel.x << "  " << state.lin_vel.y << "  " << state.lin_vel.z << endl;
  cout << "Angular vel.: " << state.ang_vel.x << "  " << state.ang_vel.y << "  " << state.ang_vel.z << endl;
  cout << "Wheel omega:  " << state.omega << endl << endl;

  ChVector<> kag = tire->getKAG_from_State(state);
  cout << "getKAG_from_State()" << endl;
  cout << "  kappa = " << kag.x << endl;
  cout << "  alpha = " << kag.y << " rad = " << kag.y * rad2deg << " deg" << endl;
  cout << "  gamma = " << kag.z << " rad = " << kag.z * rad2deg << " deg" << endl << endl;

  tire->Update(0, state);
  double kappa = tire->get_kappa();
  double alpha = tire->get_alpha();
  double gamma = tire->get_gamma();
  cout << "Update()" << endl;
  cout << "  kappa = " << kappa << endl;
  cout << "  alpha = " << alpha << " rad = " << alpha * rad2deg << " deg" << endl;
  cout << "  gamma = " << gamma << " rad = " << gamma * rad2deg << " deg" << endl << endl;
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
    ChWheelState state;
    state.pos = ChVector<>(2, 1, 0);
    state.rot = ChQuaternion<>(0.25882, 0, 0, 0.965926);  // rotate 150 deg. about Z, no camber
    state.lin_vel = ChVector<>(6.427876, -7.660444, 0);   // ||V|| = 10, away 20 deg from heading direction
    state.ang_vel = ChVector<>(2.5, 4.33013, 0);          // ||omega|| = 5
    state.omega = -5;                                     // omega < 0, wheel moves forward

    processState(tire, state);
  }

  return 0;
}
