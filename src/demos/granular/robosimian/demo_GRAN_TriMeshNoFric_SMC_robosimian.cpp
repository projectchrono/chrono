// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================
//
// Chrono::Granular demo using SMC method. A body who's geometry is described
// by an OBJ file is time-integrated in Chrono and interacts with a Granular
// wave tank in Chrono::Granular via the co-simulation framework.
// =============================================================================
/*! \file */

#include <iostream>
#include <string>
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "../ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;

void ShowUsage() {
    cout << "usage: ./demo_GRAN_TriMeshNoFric_SMC_ballcosim <json_file>" << endl;
}

// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// There is no friction. The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    double iteration_step = 1e-4;

    // Mesh values
    std::vector<string> mesh_filenames;
    string mesh_filename = string("robosim_cyl.obj");

    std::vector<float3> mesh_scalings;
    float3 scaling;
    scaling.x = 100;
    scaling.y = 100;
    scaling.z = 100;

    // starting positions of mesh-based balls
    std::vector<ChVector<double>> wheel_positions;
    wheel_positions.push_back({-20, -20, 12});
    wheel_positions.push_back({-20, 20, 12});
    wheel_positions.push_back({20, -20, 12});
    wheel_positions.push_back({20, 20, 12});
    unsigned int num_mesh_balls = wheel_positions.size();

    // add mesh to granular system
    for (unsigned int i = 0; i < num_mesh_balls; i++) {
        mesh_scalings.push_back(scaling);
    }
    for (unsigned int i = 0; i < num_mesh_balls; i++) {
        mesh_filenames.push_back(mesh_filename);
    }

    sim_param_holder params;
    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    // Setup granular simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys_gran(params.sphere_radius, params.sphere_density);
    m_sys_gran.setBOXdims(params.box_X, params.box_Y, params.box_Z);

    chrono::utils::PDSampler<float> sampler(2.1 * params.sphere_radius);
    ChVector<> pos(0, 0, -params.box_Z / 4);
    ChVector<> hdims(params.box_X / 2 - 2 * params.sphere_radius, params.box_Y / 2 - 2 * params.sphere_radius,
                     params.box_Z / 4 - 2 * params.sphere_radius);
    auto points = sampler.SampleBox(pos, hdims);
    m_sys_gran.setParticlePositions(points);

    m_sys_gran.set_BD_Fixed(true);
    std::function<double(double)> pos_func_still = [](double t) { return -0.5; };
    std::function<double(double)> pos_func_wave = [](double t) {
        double t0 = 0.5;
        double freq = 1.25 * M_PI;

        if (t < t0) {
            return -0.5;
        } else {
            return (-0.5 + 0.25 * std::sin((t - t0) * freq));
        }
    };

    m_sys_gran.setBDPositionFunction(pos_func_wave, pos_func_still, pos_func_still);

    m_sys_gran.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys_gran.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys_gran.set_K_n_SPH2MESH(params.normalStiffS2M);
    m_sys_gran.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys_gran.set_Gamma_n_SPH2WALL(params.normalDampS2S);
    m_sys_gran.set_Gamma_n_SPH2MESH(params.normalDampS2M);
    m_sys_gran.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys_gran.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys_gran.set_timeStepping(GRN_TIME_STEPPING::FIXED);
    m_sys_gran.set_timeIntegrator(GRN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys_gran.set_fixed_stepSize(params.step_size);

    m_sys_gran.load_meshes(mesh_filenames, mesh_scalings);

    /// output preferences
    m_sys_gran.setOutputDirectory(params.output_dir);
    m_sys_gran.setOutputMode(params.write_mode);
    m_sys_gran.setVerbose(params.verbose);
    ChFileutils::MakeDirectory(params.output_dir.c_str());

    unsigned int nSoupFamilies = m_sys_gran.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshSoupLocOri = new double[7 * nSoupFamilies];

    m_sys_gran.initialize();
    int currframe = 0;

    // Create rigid ball simulation
    ChSystemSMC m_sys_robo;
    m_sys_robo.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    m_sys_robo.SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
    m_sys_robo.Set_G_acc(ChVector<>(0, 0, -980));

    double wheel_mass = 1.499326 * 1000;

    double h = 12.3;
    double r = 24;
    double Ix = wheel_mass * (3 * r * r + h * h) / 12.;
    double Iy = Ix;
    double Iz = .5 * wheel_mass * r * r;

    std::vector<std::shared_ptr<ChBody>> chrono_bodies;
    for (unsigned int i = 0; i < num_mesh_balls; i++) {
        std::shared_ptr<ChBody> wheel(m_sys_robo.NewBody());
        wheel->SetMass(wheel_mass);
        wheel->SetInertiaXX(ChVector<>(0.006378, 0.006377, 0.009155) * 1e7);
        // wheel->SetInertiaXX(ChVector<>(0.0, 0.0, 2.29));
        // wheel->SetInertiaXY(ChVector<>(63.78, 63.77, 91.55));
        wheel->SetPos(wheel_positions[i]);
        wheel->SetRot(Q_from_AngX(CH_C_PI / 2));
        // wheel->SetWvel_par({0, 2 * 2 * CH_C_PI, 0});
        m_sys_robo.AddBody(wheel);
        // add to list to keep track of
        chrono_bodies.push_back(wheel);
    }
    unsigned int out_fps = 50;
    unsigned int out_steps = 1 / (out_fps * iteration_step);

    cout << "out_steps " << out_steps << endl;
    unsigned int curr_step = 0;
    for (float t = 0; t < params.time_end; t += iteration_step, curr_step++) {
        for (unsigned int i = 0; i < num_mesh_balls; i++) {
            std::shared_ptr<ChBody> ball = chrono_bodies[i];
            auto ball_pos = ball->GetPos();
            auto ball_rot = ball->GetRot();

            unsigned int body_family_offset = i * 7;

            meshSoupLocOri[body_family_offset + 0] = ball_pos.x();
            meshSoupLocOri[body_family_offset + 1] = ball_pos.y();
            meshSoupLocOri[body_family_offset + 2] = ball_pos.z();
            meshSoupLocOri[body_family_offset + 3] = ball_rot[0];
            meshSoupLocOri[body_family_offset + 4] = ball_rot[1];
            meshSoupLocOri[body_family_offset + 5] = ball_rot[2];
            meshSoupLocOri[body_family_offset + 6] = ball_rot[3];
        }
        m_sys_gran.meshSoup_applyRigidBodyMotion(meshSoupLocOri);  // Apply the mesh orientation data to the mesh

        m_sys_gran.advance_simulation(iteration_step);
        if (currframe >= 20) {
            float ball_force[6 * num_mesh_balls];
            m_sys_gran.collectGeneralizedForcesOnMeshSoup(ball_force);
            // Apply forces to the ball for the duration of the iteration
            for (unsigned int i = 0; i < num_mesh_balls; i++) {
                std::shared_ptr<ChBody> ball = chrono_bodies[i];

                auto ball_pos = ball->GetPos();
                auto ball_rot = ball->GetRot();

                unsigned int body_family_offset = i * 6;

                ball->Accumulate_force(
                    ChVector<>(ball_force[body_family_offset + 0], ball_force[body_family_offset + 1],
                               ball_force[body_family_offset + 2]),
                    ball_pos, false);
                ball->Accumulate_torque(
                    ChVector<>(ball_force[body_family_offset + 3], ball_force[body_family_offset + 4], ball_force[5]),
                    false);
                cout << "pos(" << ball_pos.x() << ", " << ball_pos.y() << ", " << ball_pos.z() << ") " << endl;
                cout << "force (" << ball_force[0] << ", " << ball_force[1] << ", " << ball_force[2] << "); torque ("
                     << ball_force[3] << ", " << ball_force[4] << ", " << ball_force[5] << ")" << endl;
            }

            m_sys_robo.DoStepDynamics(iteration_step);
            // empty forces on each ball
            for (unsigned int i = 0; i < num_mesh_balls; i++) {
                std::shared_ptr<ChBody> ball = chrono_bodies[i];
                ball->Empty_forces_accumulators();
            }
        }
        if (curr_step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            m_sys_gran.writeFileUU(string(filename));
            m_sys_gran.write_meshes(string(filename));
        }
    }

    delete[] meshSoupLocOri;

    return 0;
}