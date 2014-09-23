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
// Authors: Justin Madsen
// =============================================================================
//
// test the pacTire for 3 distinct slip cases, then compare force/moment output
//	v. slip (kappa, alpha) to Adams/Car testrig output
//  1) pure longitudinal slip (V_yc = tan(alpha) = 0)
// 2) pure lateral slip, (kappa ~= 0)
// 3) combined slip

#include <vector>

#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"
// the most important include file:
#include "models/hmmwv/tire/HMMWV_PacejkaTire.h"
#include "subsys/terrain/FlatTerrain.h"

// we will run our vehicle with default rigid tires, but calculate the output
// for the pacjeka tire in the background
#include "ChronoT_config.h"

using namespace chrono;
using namespace hmmwv;

const std::string out_dir = "../PACTEST";
const std::string pov_dir = out_dir + "/POVRAY";

// ============== PacTire settings go here
const int num_pts = 801;	// # of data points in the slip ranges
const double step_size = 0.01;	// seconds, arbitrary unless calculating transient slip
const	double alpha_lim = CH_C_PI_4/3.0;	// slip angle in range [-lim,lim] or [0,lim]
const	double kappa_lim = 1;	// slip rate in range [-lim,lim] or [0,lim]
const double F_z = 8000;	// vertical force, [N]
const double time_end = (num_pts - 1)*step_size;

// for the transient model
const bool use_transient_slip = true;	// use kinematic or transient contact slips?

const std::string pacParamFile = utils::GetModelDataFile("hmmwv/pactest.tir");

std::string outData_filename_long = "test_pacTire_pureLongSlip.csv";
std::string outData_filename_lat = "test_pacTire_pureLatSlip.csv";
std::string outData_filename_combined = "test_pacTire_combinedSlip.csv";


int main(int argc, char* argv[])
{
  SetChronoDataPath(CHRONO_DATA_DIR);
	// flat rigid terrain, height = 0 for all (x,y)
	FlatTerrain flat_terrain(0);

	// update body state based on varying input variables to pacTire:
	// alpha, kappa and gamma
	ChBodyState long_state;
	ChBodyState lat_state;
	ChBodyState combined_state;
	// only omega_y matters for IC (for R_eff calc)
	lat_state.ang_vel.y = 51.4;

	// Create the Pac tires, try to open param file and load empirical constants
  HMMWV_PacejkaTire tire_long(pacParamFile, flat_terrain, long_state, step_size, use_transient_slip, F_z);
  HMMWV_PacejkaTire tire_lat(pacParamFile, flat_terrain, lat_state, step_size, use_transient_slip, F_z);
  HMMWV_PacejkaTire tire_combined(pacParamFile, flat_terrain, combined_state, step_size, use_transient_slip, F_z);

	// record pacTire output for each of the 3 slip cases
  ChTireForces long_forces(1);
	ChTireForces lat_forces(1);
	ChTireForces combined_forces(1);

	// keep track of the input variable values
	std::vector<double> latSlip_range;
	std::vector<double> longSlip_range;
	latSlip_range.resize(num_pts);
	longSlip_range.resize(num_pts);
	
	double time = 0.0;

	// get the bounds of the slip angles from the input *.tire file
	double k_min, k_max, a_min, a_max;
	k_min = -kappa_lim;
	k_max = kappa_lim;
	a_min = -alpha_lim;
	a_max = alpha_lim;

	// at IC, go straight ahead at zero slip
	double kappa_t = k_min;
	double alpha_t = 0;
	double tanalpha_t = tan(alpha_t);
	double gamma_t = 0.1 * alpha_t;

	// unless we're only using steady-state EQs, then just sweep min to max
	if( !use_transient_slip )
	{
		alpha_t = a_min;
		tanalpha_t = tan(alpha_t);
		gamma_t = 0.1 * alpha_t;
	} else {
		outData_filename_long = "test_pacTire_pureLongSlip_transient.csv";
		outData_filename_lat = "test_pacTire_pureLatSlip_transient.csv";
		outData_filename_combined = "test_pacTire_combinedSlip_transient.csv";
	}

	// calculate the increments for each slip case (assuming constant per step)
	double kappa_incr = (k_max - k_min) / double(num_pts);
	double alpha_incr = (a_max - a_min) / double(num_pts);

	// horizontal velocity (x,y components) is constant throughout all runs
	double vel_xy = tire_long.get_longvl();

	// time step is arbitrary, steps determined by slip array length
	for(size_t step = 0; step < num_pts; step++)
  {
		// tire input slip quantities
		tanalpha_t = tan(alpha_t);

		// DEBUGGING
		gamma_t = 0;	// 0.1 * alpha_t;



		latSlip_range[step] = tanalpha_t;
		longSlip_range[step] = kappa_t;
		
		// **** longitudinal states
		long_state = tire_long.getState_from_KAG(kappa_t, 0, 0, vel_xy);

		// **** lateral states
		lat_state = tire_lat.getState_from_KAG(0, alpha_t, gamma_t, vel_xy);

		// ****  combined states
		combined_state = tire_combined.getState_from_KAG(kappa_t, alpha_t, gamma_t, vel_xy);

		// check states calculated for this step
		// ChBodyState kag_long = tire_long.getState_from_KAG(kappa_t, 0, gamma_t, vel_xy);
		// ChBodyState kag_lat = tire_lat.getState_from_KAG(0, alpha_t, gamma_t, vel_xy;
		// ChBodyState kag_combined = tire_combined.getState_from_KAG(kappa_t, alpha_t, gamma_t, vel_xy);

		// DEBUGGING
		if( step== 360)
			int arg = 2;



		// update all 3 types of tires, with current wheel state data
    tire_long.Update(time, long_state);
    tire_lat.Update(time, lat_state);
		tire_combined.Update(time, combined_state);

		// advance the slip displacements, calculate reactions
		tire_long.Advance(step_size);
    tire_lat.Advance(step_size);
		tire_combined.Advance(step_size);
   
		// see what the calculated reactions are
    long_forces[FRONT_LEFT] = tire_long.GetTireForce();
    lat_forces[FRONT_LEFT] = tire_lat.GetTireForce();
		combined_forces[FRONT_LEFT] = tire_combined.GetTireForce();
 

		// output any data at the end of the step
		tire_long.WriteOutData(time,outData_filename_long);
		tire_lat.WriteOutData(time,outData_filename_lat);
		tire_combined.WriteOutData(time,outData_filename_combined);

		// std::cout << "step: " << step << ", kappa = " << kappa_t << ", alpha = " << alpha_t << ", tan(alpha) = " << tanalpha_t << std::endl;

		// increment time, update long. & lat. slips for the next time step
		time += step_size;
		// constant increments. sine waves would be better
		if( use_transient_slip) 
		{
			// do a powered lane change
			// Careful !!!!
			// need to properly mimic Adams/Car kappa, alpha vs. time
			// kappa is linear
			kappa_t += kappa_incr;
			// lateral angle is a sine wave
			alpha_t = abs(a_max) * sin( 2.0 * chrono::CH_C_PI * time / time_end);
		} else {
			kappa_t += kappa_incr;
			alpha_t += alpha_incr;
		}

	
	}

	// clean up anything


  return 0;
}
