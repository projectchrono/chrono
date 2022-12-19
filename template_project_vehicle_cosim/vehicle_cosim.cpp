// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Simple example that can be used as a template for an external project using
// the Chrono::Vehicle co-simulation framework.
//
// =============================================================================

#include <iostream>
#include <string>
#include <limits>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"
#include "chrono_vehicle/cosim/ChVehicleCosimOtherNode.h"
#include "chrono_vehicle/cosim/mbs/ChVehicleCosimRigNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeBypass.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Example of a custom terrain node.
// This simple demonstration terrain node uses a simple rigid plate as the terrain mode and creates rigid cylindrical
// tires. This terrain node works can work with a tire node of type BYPASS.
class MyTerrain : public ChVehicleCosimTerrainNode {
  public:
    MyTerrain(double length, double width);
    ~MyTerrain();

    // This terrain type does not support the MESH communication interface.
    virtual bool SupportsMeshInterface() const override { return false; }

    // Return the terrain initial height.
    virtual double GetInitHeight() const override { return 0; }

    // Initialize this Chrono terrain node.
    // Construct the terrain system, the tire material, and the proxy bodies.
    // Use information from the following vectors (of size equal to the number of tires):
    // - radius for each tire (through m_tire_radius)
    // - vertical load on each tire (through m_load_mass)
    virtual void OnInitialize(unsigned int num_tires) override final;

    // Advance simulation.
    virtual void OnAdvance(double step_size) override;

    // Render simulation.
    virtual void Render(double time) override;

    /// Update the state of the i-th proxy rigid.
    virtual void UpdateRigidProxy(unsigned int i, BodyState& rigid_state) override;

    /// Collect cumulative contact force and torque on the i-th proxy rigid.
    virtual void GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) override;

  private:
    ChSystemSMC* m_system;                          // containing Chrono system
    std::vector<std::shared_ptr<ChBody>> m_bodies;  // proxy tire bodies

#ifdef CHRONO_IRRLICHT
    irrlicht::ChVisualSystemIrrlicht* m_vis;  // Irrlicht run-time visualization
#endif
};

MyTerrain::MyTerrain(double length, double width) : ChVehicleCosimTerrainNode(length, width) {
    m_system = new ChSystemSMC;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
    m_system->SetNumThreads(1);
    m_system->SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
#ifdef CHRONO_IRRLICHT
    m_vis = nullptr;
#endif
}

MyTerrain::~MyTerrain() {
    delete m_system;
#ifdef CHRONO_IRRLICHT
    delete m_vis;
#endif
}

void MyTerrain::OnInitialize(unsigned int num_tires) {
    // Create the rigid terrain box with its top surface at init height = 0
    auto ground = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(ground);
    ground->SetMass(1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    auto mat_terrain = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat_terrain->SetFriction(0.9f);
    mat_terrain->SetRestitution(0);
    mat_terrain->SetYoungModulus(8e5f);
    mat_terrain->SetPoissonRatio(0.3f);
    mat_terrain->SetKn(1e6f);
    mat_terrain->SetGn(6e1f);
    mat_terrain->SetKt(4e5f);
    mat_terrain->SetGt(4e1f);

    ground->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(ground.get(), mat_terrain, ChVector<>(m_hdimX, m_hdimY, 0.1), ChVector<>(0, 0, -0.1),
                          ChQuaternion<>(1, 0, 0, 0), true);
    ground->GetCollisionModel()->BuildModel();

    // Shared proxy contact material
    auto mat_proxy = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mat_proxy->SetFriction(0.9f);
    mat_proxy->SetRestitution(0);
    mat_proxy->SetYoungModulus(2e7f);
    mat_proxy->SetPoissonRatio(0.3f);
    mat_proxy->SetKn(2e5f);
    mat_proxy->SetGn(4e1f);
    mat_proxy->SetKt(2e5f);
    mat_proxy->SetGt(4e1f);

    // Create the proxy bodies with cylindrical shapes
    // Create the proxy bodies with cylindrical shapes
    double tire_radius = m_aabb[0].m_dims.x() / 2;
    double tire_width = m_aabb[0].m_dims.y();

    m_bodies.resize(num_tires);
    for (unsigned int i = 0; i < num_tires; i++) {
        m_bodies[i] = chrono_types::make_shared<ChBody>();
        m_bodies[i]->SetMass(m_load_mass[0]);
        m_bodies[i]->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
        m_bodies[i]->SetCollide(true);

        m_bodies[i]->GetCollisionModel()->ClearModel();
        utils::AddCylinderGeometry(m_bodies[i].get(), mat_proxy, tire_radius, tire_width / 2);
        m_bodies[i]->GetCollisionModel()->BuildModel();

        m_bodies[i]->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/greenwhite.png"));

        m_system->AddBody(m_bodies[i]);
    }

    // Reset system time to 0.
    m_system->SetChTime(0);

    // Create the Irrlicht visualization system
#ifdef CHRONO_IRRLICHT
    if (m_render) {
        m_vis = new irrlicht::ChVisualSystemIrrlicht;
        m_vis->SetWindowSize(1280, 720);
        m_vis->SetWindowTitle("Custom terrain node");
        m_vis->SetCameraVertical(CameraVerticalDir::Z);
        m_vis->Initialize();
        m_vis->AddLogo();
        m_vis->AddSkyBox();
        m_vis->AddTypicalLights();
        m_vis->AddCamera(ChVector<>(2, 1.4, 1), ChVector<>(0, 0, 0));
        m_vis->AttachSystem(m_system);
    }
#endif
}

void MyTerrain::OnAdvance(double step_size) {
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
}

void MyTerrain::Render(double time) {
#ifdef CHRONO_IRRLICHT
    if (!m_vis->Run()) {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
    m_vis->BeginScene();
    m_vis->Render();
    m_vis->EndScene();
#endif
}

void MyTerrain::UpdateRigidProxy(unsigned int i, BodyState& rigid_state) {
    m_bodies[i]->SetPos(rigid_state.pos);
    m_bodies[i]->SetPos_dt(rigid_state.lin_vel);
    m_bodies[i]->SetRot(rigid_state.rot);
    m_bodies[i]->SetWvel_par(rigid_state.ang_vel);
}

void MyTerrain::GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) {
    rigid_contact.point = ChVector<>(0, 0, 0);
    rigid_contact.force = m_bodies[i]->GetContactForce();
    rigid_contact.moment = m_bodies[i]->GetContactTorque();
}

// =============================================================================

int main(int argc, char** argv) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Initialize MPI.
    int num_procs;
    int rank;
    int name_len;
    char procname[MPI_MAX_PROCESSOR_NAME];

    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Get_processor_name(procname, &name_len);

#ifdef _DEBUG
    if (rank == 0) {
        int foo;
        cout << "Enter something to continue..." << endl;
        cin >> foo;
    }
    MPI_Barrier(MPI_COMM_WORLD);
#endif

    if (num_procs < 3) {
        if (rank == 0)
            std::cout << "\n\nSingle wheel cosimulation code must be run on at least 3 ranks!\n\n" << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    // Simulation parameters
    double step_size = 1e-4;
    double sim_time = 10;
    double output_fps = 100;
    double render_fps = 100;
    bool render = true;
    std::string suffix = "";
    bool verbose = true;

    // Prepare output directory.
    std::string out_dir = "RIG_COSIM_CUSTOM";
    if (rank == 0) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
    }
    MPI_Barrier(MPI_COMM_WORLD);

    // Number of simulation steps between miscellaneous events.
    int sim_steps = (int)std::ceil(sim_time / step_size);
    int output_steps = (int)std::ceil(1 / (output_fps * step_size));

    // Create the node (a rig, tire, or terrain node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    if (rank == MBS_NODE_RANK) {
        auto act_type = ChVehicleCosimDBPRigImposedSlip::ActuationType::SET_ANG_VEL;
        double base_vel = 1.0;
        double slip = 0;
        auto dbp_rig = chrono_types::make_shared<ChVehicleCosimDBPRigImposedSlip>(act_type, base_vel, slip);
        dbp_rig->SetDBPFilterWindow(0.1);

        auto mbs = new ChVehicleCosimRigNode();
        mbs->SetVerbose(verbose);
        mbs->SetStepSize(step_size);
        mbs->SetNumThreads(1);
        mbs->SetTotalMass(100);
        mbs->SetOutDir(out_dir, suffix);
        mbs->AttachDrawbarPullRig(dbp_rig);

        node = mbs;
    } else if (rank == TIRE_NODE_RANK(0)) {
        auto tire = new ChVehicleCosimTireNodeBypass(0, 37.6, 0.47, 0.25);
        tire->SetVerbose(verbose);
        tire->SetStepSize(step_size);
        tire->SetNumThreads(1);
        tire->SetOutDir(out_dir, suffix);

        node = tire;
    } else if (rank == TERRAIN_NODE_RANK) {
        auto terrain = new MyTerrain(4.0, 1.0);
        terrain->SetVerbose(verbose);
        terrain->SetStepSize(step_size);
        terrain->SetOutDir(out_dir, suffix);
        terrain->EnableRuntimeVisualization(render, render_fps);

        node = terrain;
    } else {
        node = new ChVehicleCosimOtherNode();
    }

    // Initialize systems.
    node->Initialize();

    if (verbose) {
        if (rank == 0)
            cout << "---------------------------- " << endl;
        for (int i = 0; i < num_procs; i++) {
            if (rank == i) {
                cout << "rank: " << rank << " running on: " << procname << endl;
                cout << "   node type:    " << node->GetNodeTypeString() << endl;
                cout << "   cosim node:   " << (node->IsCosimNode() ? "yes" : "no") << endl;
                cout << "   output dir:   " << node->GetOutDirName() << endl;
            }
            MPI_Barrier(MPI_COMM_WORLD);
        }
    }

    // Perform co-simulation.
    int output_frame = 0;

    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;

        if (verbose && rank == 0)
            cout << is << " ---------------------------- " << endl;
        MPI_Barrier(MPI_COMM_WORLD);

        if (node->IsCosimNode()) {
            node->Synchronize(is, time);
            node->Advance(step_size);
            if (verbose)
                cout << "Node" << rank << " sim time = " << node->GetStepExecutionTime() << "  ["
                     << node->GetTotalExecutionTime() << "]" << endl;

            if (is % output_steps == 0) {
                node->OutputData(output_frame);
                output_frame++;
            }
        }
    }

    // Cleanup.
    delete node;
    MPI_Finalize();
    return 0;
}
