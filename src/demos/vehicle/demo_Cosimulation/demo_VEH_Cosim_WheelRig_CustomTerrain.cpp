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
// Demo for single-wheel rig cosimulation framework using a custom terrain node.
// Illustration of connecting a terrain simulation potentially done oustside Chrono.
//
// Global reference frame: Z up, X towards the front, and Y pointing to the left
//
// =============================================================================

#include <iostream>
#include <string>
#include <limits>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"
#include "chrono_vehicle/cosim/mbs/ChVehicleCosimRigNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChIrrApp.h"
#endif

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Example of a custom terrain node.
// This simple illustration class can only interact with a single wheel.
class MyTerrain : public ChVehicleCosimTerrainNode {
  public:
    MyTerrain(double length, double width);
    ~MyTerrain();

    /// This terrain type does not support the MESH communication interface.
    virtual bool SupportsMeshInterface() const override { return false; }

    /// Return the terrain initial height.
    virtual double GetInitHeight() const override { return 0; }

    /// Initialize this Chrono terrain node.
    /// Construct the terrain system, the tire material, and the proxy bodies.
    /// Use information from the following vectors (of size equal to the number of tires):
    /// - radius for each tire (through m_tire_radius)
    /// - mesh information for each tire (through m_mesh_data)
    /// - contact material for each tire (through m_mat_props)
    /// - vertical load on each tire (through m_load_mass)
    virtual void OnInitialize(unsigned int num_tires) override final;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void OnAdvance(double step_size) override;

    /// Render simulation.
    virtual void Render(double time) override;

    /// Update the state of the wheel proxy body for the i-th tire.
    virtual void UpdateWheelProxy(unsigned int i, const BodyState& spindle_state) override;

    /// Collect cumulative contact force and torque on the wheel proxy body for the i-th tire.
    virtual void GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) override;

  private:
    ChSystemSMC* m_system;           ///< containing Chrono system
    std::shared_ptr<ChBody> m_body;  ///< proxy body for the tire
#ifdef CHRONO_IRRLICHT
    irrlicht::ChIrrApp* m_irrapp;  ///< Irrlicht run-time visualizatino
#endif
};

MyTerrain::MyTerrain(double length, double width) : ChVehicleCosimTerrainNode(length, width) {
    m_system = new ChSystemSMC;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));
    m_system->SetNumThreads(1);
    m_system->SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
#ifdef CHRONO_IRRLICHT
    m_irrapp = nullptr;
#endif
}

MyTerrain::~MyTerrain() {
    delete m_system;
#ifdef CHRONO_IRRLICHT
    delete m_irrapp;
#endif
}

void MyTerrain::OnInitialize(unsigned int num_tires) {
    assert(num_tires == 1);

    // Create the Irrlicht visualization system
#ifdef CHRONO_IRRLICHT
    if (m_render) {
        m_irrapp = new irrlicht::ChIrrApp(m_system, L"Custom terrain node", irr::core::dimension2d<irr::u32>(1280, 720),
                                          irrlicht::VerticalDir::Z);
        m_irrapp->AddTypicalLogo();
        m_irrapp->AddTypicalSky();
        m_irrapp->AddTypicalLights(irr::core::vector3df(30.f, +30.f, 100.f), irr::core::vector3df(30.f, -30.f, 100.f));
        m_irrapp->AddTypicalCamera(irr::core::vector3df(2.0f, 1.4f, 1.0f), irr::core::vector3df(0, 0, 0));
    }
#endif

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

    // Create the proxy body with a cylindrical shape
    m_body = chrono_types::make_shared<ChBody>();
    m_body->SetMass(m_load_mass[0]);
    m_body->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

    auto mat_proxy = m_mat_props[0].CreateMaterial(ChContactMethod::SMC);
    m_body->SetCollide(true);

    m_body->GetCollisionModel()->ClearModel();
    utils::AddCylinderGeometry(m_body.get(), mat_proxy, m_tire_radius[0], m_tire_width[0] / 2);
    m_body->GetCollisionModel()->BuildModel();

    m_system->AddBody(m_body);

    // Reset system time to 0.
    m_system->SetChTime(0);

#ifdef CHRONO_IRRLICHT
    // Bind Irrlicht assets
    if (m_render) {
        m_irrapp->AssetBindAll();
        m_irrapp->AssetUpdateAll();
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
    if (!m_irrapp->GetDevice()->run()) {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
    m_irrapp->BeginScene();
    m_irrapp->DrawAll();
    m_irrapp->EndScene();
#endif
}

void MyTerrain::UpdateWheelProxy(unsigned int i, const BodyState& spindle_state) {
    assert(i == 0);

    m_body->SetPos(spindle_state.pos);
    m_body->SetPos_dt(spindle_state.lin_vel);
    m_body->SetRot(spindle_state.rot);
    m_body->SetWvel_par(spindle_state.ang_vel);
}

void MyTerrain::GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) {
    assert(i == 0);

    wheel_contact.point = ChVector<>(0, 0, 0);
    wheel_contact.force = m_body->GetContactForce();
    wheel_contact.moment = m_body->GetContactTorque();
}

// =============================================================================

int main(int argc, char** argv) {
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

    if (num_procs != 3) {
        if (rank == 0)
            std::cout << "\n\nSingle wheel cosimulation code must be run on exactly 3 ranks!\n\n" << std::endl;
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
    std::string out_dir = GetChronoOutputPath() + "RIG_COSIM_CUSTOM";
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

    // Create the node (a rig node or a terrain node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    if (rank == MBS_NODE_RANK) {
        if (verbose)
            cout << "[Rig node    ] rank = " << rank << " running on: " << procname << endl;

        auto act_type = ChVehicleCosimDBPRigImposedSlip::ActuationType::SET_ANG_VEL;
        double total_mass = 100;
        double base_vel = 1.0;
        double slip = 0;
        double dbp_filter_window = 0.1;

        auto dbp_rig = chrono_types::make_shared<ChVehicleCosimDBPRigImposedSlip>(act_type, base_vel, slip);
        dbp_rig->SetDBPfilterWindow(dbp_filter_window);

        auto mbs = new ChVehicleCosimRigNode();
        mbs->SetVerbose(verbose);
        mbs->SetStepSize(step_size);
        mbs->SetNumThreads(1);
        mbs->SetTotalMass(total_mass);
        mbs->SetOutDir(out_dir, suffix);
        mbs->AttachDrawbarPullRig(dbp_rig);
        if (verbose)
            cout << "[Rig node    ] output directory: " << mbs->GetOutDirName() << endl;

        node = mbs;
    }

    if (rank == TIRE_NODE_RANK(0)) {
        if (verbose)
            cout << "[Tire node   ] rank = " << rank << " running on: " << procname << endl;

        auto tire = new ChVehicleCosimTireNodeRigid(0);
        tire->SetTireFromSpecfile(vehicle::GetDataFile("hmmwv/tire/HMMWV_RigidMeshTire_Coarse.json"));
        tire->SetVerbose(verbose);
        tire->SetStepSize(step_size);
        tire->SetNumThreads(1);
        tire->SetOutDir(out_dir, suffix);

        node = tire;
    }

    if (rank == TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;

        auto terrain = new MyTerrain(4.0, 1.0);
        terrain->SetVerbose(verbose);
        terrain->SetStepSize(step_size);
        terrain->SetOutDir(out_dir, suffix);
        terrain->EnableRuntimeVisualization(render, render_fps);
        if (verbose)
            cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

        node = terrain;
    }

    // Initialize systems.
    node->Initialize();

    // Perform co-simulation.
    int output_frame = 0;

    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;

        if (verbose && rank == 0)
            cout << is << " ---------------------------- " << endl;
        MPI_Barrier(MPI_COMM_WORLD);

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

    // Cleanup.
    delete node;
    MPI_Finalize();
    return 0;
}
