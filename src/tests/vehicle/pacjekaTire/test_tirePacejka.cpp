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
// v. slip (kappa, alpha) to Adams/Car testrig output
// 1) pure longitudinal slip (V_yc = tan(alpha) = 0)
// 2) pure lateral slip, (kappa ~= 0)
// 3) combined slip
//
// we will run our vehicle with default rigid tires, but calculate the output
// for the pacjeka tire in the background
//
// =============================================================================

#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkDistance.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

using namespace chrono;
using namespace chrono::vehicle;

int main(int argc, char* argv[]) {
    // Output directory names
    const std::string out_dir = "../PACTEST";
    const std::string pov_dir = out_dir + "/POVRAY";

    // ============== PacTire settings go here
    const int num_pts = 801;                   // # of data points in the slip ranges
    const double step_size = 0.01;             // seconds, arbitrary unless calculating transient slip
    const double alpha_lim = CH_C_PI_4 / 3.0;  // slip angle in range [-lim,lim] or [0,lim]
    const double kappa_lim = 1;                // slip rate in range [-lim,lim] or [0,lim]
    const double F_z = 8000;                   // vertical force, [N]
    const VehicleSide m_side = LEFT;
    const double gamma = 10.0 * CH_C_PI / 180.0;  // gamma, in radians

    // for the transient model
    const bool use_transient_slip = true;  // use kinematic or transient contact slips?

    const std::string pacParamFile = vehicle::GetDataFile("hmmwv/pactest.tir");

    // output data filenames
    std::string out_name_long = "test_pacTire_pureLongSlip.csv";
    std::string out_name_lat = "test_pacTire_pureLatSlip.csv";
    std::string out_name_latGamma = "test_pacTire_pureLatSlipGamma.csv";
    std::string out_name_combined = "test_pacTire_combinedSlip.csv";

    const double time_end = (num_pts - 1) * step_size;

    // flat rigid terrain, height = 0 for all (x,y)
    FlatTerrain flat_terrain(0);

    // Create the Pac tires, try to open param file and load empirical constants
    ChPacejkaTire tire_long("LONGITUDINAL", pacParamFile, F_z, use_transient_slip);
    ChPacejkaTire tire_lat("LATERAL", pacParamFile, F_z, use_transient_slip);
    ChPacejkaTire tire_lat_gamma("LATERAL_GAMMA", pacParamFile, F_z, use_transient_slip);
    ChPacejkaTire tire_combined("COMBINED", pacParamFile, F_z, use_transient_slip);

    // initialize the tire on the left or right side
    tire_long.Initialize(m_side, true);
    tire_lat.Initialize(m_side, true);
    tire_lat_gamma.Initialize(m_side, true);
    tire_combined.Initialize(m_side, true);

    // record pacTire output for each of the 3 slip cases
    TireForces long_forces(1);
    TireForces lat_forces(1);
    TireForces latGamma_forces(1);
    TireForces combined_forces(1);

    // update body state based on varying input variables to pacTire:
    // alpha, kappa and gamma
    WheelState long_state;
    WheelState lat_state;
    WheelState latGamma_state;
    WheelState combined_state;

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
    if (!use_transient_slip) {
        alpha_t = a_min;
        tanalpha_t = tan(alpha_t);
        gamma_t = 0.1 * alpha_t;
    } else {
        out_name_long = "test_pacTire_pureLongSlip_transient.csv";
        out_name_lat = "test_pacTire_pureLatSlip_transient.csv";
        out_name_latGamma = "test_pacTire_pacLatSlipGamma_transient.csv";
        out_name_combined = "test_pacTire_combinedSlip_transient.csv";
    }

    // calculate the increments for each slip case (assuming constant per step)
    double kappa_incr = (k_max - k_min) / double(num_pts);
    double alpha_incr = (a_max - a_min) / double(num_pts);

    // horizontal velocity (x,y components) is constant throughout all runs
    double vel_xy = tire_long.get_longvl();

    // time step is arbitrary, steps determined by slip array length
    for (size_t step = 0; step < num_pts; step++) {
        // tire input slip quantities
        tanalpha_t = tan(alpha_t);

        latSlip_range[step] = tanalpha_t;
        longSlip_range[step] = kappa_t;

        // **** longitudinal states
        long_state = tire_long.getState_from_KAG(kappa_t, 0, 0, vel_xy);

        // **** lateral states
        lat_state = tire_lat.getState_from_KAG(0, alpha_t, 0, vel_xy);

        // ***** lateral w/ specified gamma
        latGamma_state = tire_lat_gamma.getState_from_KAG(0, alpha_t, gamma, vel_xy);

        // ****  combined states
        combined_state = tire_combined.getState_from_KAG(kappa_t, alpha_t, gamma_t, vel_xy);

        // DEBUGGING
        if (step == 200)
            int arg = 2;

        // update all 4 types of tires, with current wheel state data
        tire_long.Synchronize(time, long_state, flat_terrain);
        tire_lat.Synchronize(time, lat_state, flat_terrain);
        tire_lat_gamma.Synchronize(time, latGamma_state, flat_terrain);
        tire_combined.Synchronize(time, combined_state, flat_terrain);

        // advance the slip displacements, calculate reactions
        tire_long.Advance(step_size);
        tire_lat.Advance(step_size);
        tire_lat_gamma.Advance(step_size);
        tire_combined.Advance(step_size);

        // see what the calculated reactions are
        long_forces[0] = tire_long.GetTireForce();
        lat_forces[0] = tire_lat.GetTireForce();
        latGamma_forces[0] = tire_lat_gamma.GetTireForce();
        combined_forces[0] = tire_combined.GetTireForce();

        // output any data at the end of the step
        tire_long.WriteOutData(time, out_name_long);
        tire_lat.WriteOutData(time, out_name_lat);
        tire_lat_gamma.WriteOutData(time, out_name_latGamma);
        tire_combined.WriteOutData(time, out_name_combined);

        // std::cout << "step: " << step << ", kappa = " << kappa_t << ", alpha = " << alpha_t << ", tan(alpha) = " <<
        // tanalpha_t << std::endl;

        // increment time, update long. & lat. slips for the next time step
        time += step_size;
        // constant increments. sine waves would be better
        if (use_transient_slip) {
            // do a powered lane change
            // Careful !!!!
            // need to properly mimic Adams/Car kappa, alpha vs. time
            // kappa is linear
            kappa_t += kappa_incr;
            // lateral angle is a sine wave
            alpha_t = abs(a_max) * sin(2.0 * chrono::CH_C_PI * time / time_end);
        } else {
            kappa_t += kappa_incr;
            alpha_t += alpha_incr;
        }
    }

    // clean up anything

    return 0;
}
