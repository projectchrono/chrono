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
// by a trinagle mesh is initialized under settling granular material. No friction present.
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
// =============================================================================
/*! \file */

#include <iostream>
#include <string>
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"

#include "ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;

void ShowUsage() {
    cout << "usage: ./demo_GRAN_TriMeshNoFric_SMC_boatcosim <json_file>" << endl;
}
// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// There is no friction. The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    float iteration_step = 1e-3;

    // Mesh values
    std::vector<string> mesh_filenames;
    string mesh_filename = string("boat.obj");

    std::vector<float3> mesh_scalings;
    float3 scaling;
    scaling.x = 20;
    scaling.y = 20;
    scaling.z = 20;
    mesh_scalings.push_back(scaling);

    sim_param_holder params;
    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    mesh_filenames.push_back(mesh_filename);

    // Setup granular simulation
    ChSystemGranularMonodisperse_SMC_Frictionless_trimesh m_sys_gran(params.sphere_radius, params.sphere_density);
    m_sys_gran.setBOXdims(params.box_X, params.box_Y, params.box_Z);
    m_sys_gran.set_BD_Fixed(true);
    m_sys_gran.setFillBounds(-1.f, -1.f, -1.f, 1.f, 1.f, 0.f);
    m_sys_gran.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys_gran.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys_gran.set_K_n_SPH2MESH(params.normalStiffS2M);
    m_sys_gran.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys_gran.set_Gamma_n_SPH2MESH(params.normalDampS2M);
    m_sys_gran.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys_gran.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys_gran.set_timeStepping(GRN_TIME_STEPPING::FIXED);
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

    // Create rigid boat simulation
    ChSystemSMC m_sys_boat;
    m_sys_boat.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    m_sys_boat.SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
    m_sys_boat.Set_G_acc(ChVector<>(0, 0, -980));
    double chrono_dt = 1e-4;

    double mass = 1;
    // ChMatrix33<double> inertia;
    ChVector<double> start_pos(-params.box_X / 2 + scaling.x, 0, scaling.z);

    std::shared_ptr<ChBody> boat(m_sys_boat.NewBody());
    boat->SetMass(mass);
    // boat->SetInertia(inertia); // TODO inertia
    boat->SetPos(start_pos);

    m_sys_boat.AddBody(boat);

    unsigned int chrono_steps = iteration_step / chrono_dt;  // TODO beware of drift...
    cout << "chrono_steps: " << chrono_steps << endl;
    for (float t = 0; t < params.time_end; t += iteration_step) {
        auto boat_pos = boat->GetPos();
        auto boat_rot = boat->GetRot();

        meshSoupLocOri[0] = boat_pos.x();
        meshSoupLocOri[1] = boat_pos.y();
        meshSoupLocOri[2] = boat_pos.z();
        meshSoupLocOri[3] = boat_rot[0];
        meshSoupLocOri[4] = boat_rot[1];
        meshSoupLocOri[5] = boat_rot[2];
        meshSoupLocOri[6] = boat_rot[3];

        m_sys_gran.meshSoup_applyRigidBodyMotion(meshSoupLocOri);  // Apply the mesh orientation data to the mesh

        m_sys_gran.advance_simulation(iteration_step);

        // Apply forces to the boat for the duration of the iteration
        float boat_force[6];
        m_sys_gran.collectGeneralizedForcesOnMeshSoup(boat_force);
        boat->Accumulate_force(ChVector<>(boat_force[0], boat_force[1], boat_force[2]), boat_pos, false);
        // boat->Accumulate_torque(ChVector<>(boat_force[3], boat_force[4], boat_force[5]), false);
        cout << "pos (" << boat_pos.x() << ", " << boat_pos.y() << ", " << boat_pos.z() << ")" << endl;
        cout << "force (" << boat_force[0] << ", " << boat_force[1] << ", " << boat_force[2] << "); torque ("
             << boat_force[3] << ", " << boat_force[4] << ", " << boat_force[5] << ")" << endl;
        for (unsigned int i = 0; i < chrono_steps; i++) {
            m_sys_boat.DoStepDynamics(chrono_dt);
        }

        boat->Empty_forces_accumulators();

        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        m_sys_gran.writeFileUU(string(filename));
        m_sys_gran.write_meshes(string(filename));
    }

    delete[] meshSoupLocOri;

    return 0;
}