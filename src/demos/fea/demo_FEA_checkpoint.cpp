// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Checkpointing example for a FEA Chrono simulation
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChCheckpointASCII.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::vsg3d;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

void ConstructModel(ChSystem& sys, int id) {
    // Mesh properties
    double length = 5;       // m
    double width = 0.1;      // m
    double thickness = 0.1;  // m
    double rho = 8000;       // kg/m^3
    double E = 4e8;          // Pa
    double nu = 0;           // Poisson effect neglected for this model
    // Timoshenko shear correction coefficients for a rectangular cross-section
    double k1 = 10 * (1 + nu) / (12 + 11 * nu);
    double k2 = k1;

    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(rho, E, nu, k1, k2);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->SetAutomaticGravity(false);
    sys.Add(mesh);

    // Populate the mesh container with the nodes and elements for the meshed beam
    int num_elements = 8;
    int num_nodes = num_elements + 1;
    double dx = length / (num_nodes - 1);

    // Create the first node and fix it completely to ground (Cantilever constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(0, 0, 0.0));
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);

    auto last_element = chrono_types::make_shared<ChElementBeamANCF_3243>();

    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector3d(dx * i, 0, 0));
        mesh->AddNode(nodeB);

        auto element = chrono_types::make_shared<ChElementBeamANCF_3243>();
        element->SetNodes(nodeA, nodeB);
        element->SetDimensions(dx, thickness, width);
        element->SetMaterial(material);
        element->SetAlphaDamp(0.0);
        mesh->AddElement(element);

        nodeA = nodeB;
        last_element = element;
    }

    // Define a custom point load with a time-dependent force
    class MyLoaderTimeDependentTipLoad : public ChLoaderUatomic {
      public:
        MyLoaderTimeDependentTipLoad(std::shared_ptr<ChLoadableU> loadable)
            : ChLoaderUatomic(loadable), m_sys(nullptr) {}

        // Compute F=F(U). The load is a 6-row vector, i.e. a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ.
        virtual void ComputeF(double U,                    // normalized position along the beam axis [-1...1]
                              ChVectorDynamic<>& F,        // load at U (set to zero on entry)
                              ChVectorDynamic<>* state_x,  // if non-null, first update state (position) to this
                              ChVectorDynamic<>* state_w   // if non-null, fiurst update state (speed) to this
                              ) override {
            double t = m_sys->GetChTime();
            double Fmax = -500;
            double tc = 2;
            double Fz = Fmax;
            if (t < tc) {
                Fz = 0.5 * Fmax * (1 - std::cos(CH_PI * t / tc));
            }

            F(2) = Fz;  // Apply the force along the global Z axis
        }

        void SetSystem(ChSystem* sys) { m_sys = sys; }

      private:
        ChSystem* m_sys;
    };

    // Create the load container and add to the current system
    auto loadcontainer = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(loadcontainer);

    // Create a custom load that uses the custom loader above.
    auto loader = chrono_types::make_shared<MyLoaderTimeDependentTipLoad>(last_element);
    loader->SetSystem(&sys);    
    loader->SetApplication(1.0); 

    auto load = chrono_types::make_shared<ChLoad>(loader);
    loadcontainer->Add(load); 

    // Set up mesh visualization
    auto vis_surf = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_surf->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    vis_surf->SetWireframe(id == 2);
    vis_surf->SetDrawInUndeformedReference(true);
    mesh->AddVisualShapeFEA(vis_surf);

    sys.Setup();
    sys.Update(true);
}

// -----------------------------------------------------------------------------

void ConfigureSystem(ChSystem& sys) {
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    // Set up integrator
    auto integrator = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    integrator->SetAlpha(-0.2);
    integrator->SetMaxIters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetVerbose(false);
    integrator->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_STEP);
    sys.SetTimestepper(integrator);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create output directory
    std::string out_dir = GetChronoOutputPath() + "FEA_CHECKPOINT";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create containing systems
    ChSystemSMC sys1;
    ChSystemSMC sys2;
    ConfigureSystem(sys1);
    ConfigureSystem(sys2);

    // Construct systems
    ConstructModel(sys1, 1);
    ConstructModel(sys2, 2);

    // Create the run-time visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys1);
    vis->AttachSystem(&sys2);
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetWindowTitle("FEA checkpointing");
    vis->EnableSkyBox();
    vis->AddCamera(ChVector3d(-0.8, 0.8, 0.8), ChVector3d());
    vis->SetCameraAngleDeg(50);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->SetTargetRenderFPS(60);
    vis->Initialize();

    // Checkpoint setup
    std::string cp_filename = out_dir + "/checkpoint.txt";
    double cp_time = 7.0;
    bool cp_created = false;

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double step_size = 1e-3;
    double t = 0;

    while (true) {
        if (!vis->Run())
            break;

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (t < cp_time) {
            // Simulate 1st system
            sys1.DoStepDynamics(step_size);
        } else {
            // Checkpoint 1st system and initialize 2nd system
            if (!cp_created) {
                {
                    ChCheckpointASCII cp(ChCheckpoint::Type::SYSTEM);
                    cp.WriteState(&sys1);
                    cp.WriteFile(cp_filename);
                }
                {
                    ChCheckpointASCII cp(ChCheckpoint::Type::SYSTEM);
                    cp.OpenFile(cp_filename);
                    cp.ReadState(&sys2);
                }
                cp_created = true;
            }
            // Simulate 2nd system
            sys2.DoStepDynamics(step_size);
            // Force time of sys1 (for display purposes)
            sys1.SetChTime(sys2.GetChTime());
        }

        t += step_size;
        rt_timer.Spin(step_size);
    }

    return 0;
}
