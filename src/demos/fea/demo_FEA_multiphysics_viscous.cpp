// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// FEA multiphysics, 3D viscoelastic structural damping
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/multiphysics/ChFEModelDeformation.h"
#include "chrono/fea/multiphysics/ChMaterial3DStressNeoHookean.h"
#include "chrono/fea/multiphysics/ChMaterial3DStressParallel.h"
#include "chrono/fea/multiphysics/ChMaterial3DStressViscoLinear.h"
#include "chrono/fea/multiphysics/ChMaterial3DStressViscoNewton.h"
#include "chrono/fea/multiphysics/ChDrawer.h"
#include "chrono/fea/multiphysics/ChSurfaceOfModel.h"
#include "chrono/fea/multiphysics/ChBuilderVolume.h"
#include "chrono/fea/multiphysics/ChLinkFieldNode.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "FEAvisualization.h"

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

using namespace chrono;
using namespace chrono::fea;


void CreateTestDampedCantilever(ChSystem& sys, double damping, double z_offset) {

    // FIELD
    auto displacement_field = chrono_types::make_shared<ChFieldDisplacement3D>();
    sys.Add(displacement_field);

    // MODEL
    auto deformation_model = chrono_types::make_shared<ChFEModelDeformation>(displacement_field);
    sys.Add(deformation_model);

    deformation_model->SetAutomaticGravity(true);

    // MATERIAL
    //
    // This demo shows how to add structural damping. An option is to use a viscoelastic material,
    // that is a parallel composition of an elastic material and a viscous material.
    // This can be achieved using the ChMaterial3DStressParallel, that allows to compose in parallel
    // any two ChMaterial3DStress; namely a material A that can be whatever elastic material (in our case
    // a typical hyperelastic material) and a material B that can be whatever viscous material (in our
    // case a simple linear viscous material).

    auto elastic_material = chrono_types::make_shared<ChMaterial3DStressNeoHookean>();
    elastic_material->SetDensity(2500);
    elastic_material->SetYoungModulus(3e6);
    elastic_material->SetPoissonRatio(0.39);

    auto viscous_material = chrono_types::make_shared<ChMaterial3DStressViscoLinear>();
    viscous_material->SetDeviatoricDamping(damping);
    viscous_material->SetVolumetricDamping(damping*0.3);  // for demo purposes, use very high damping, to see the effect clearly

    auto viscoelastic_material = chrono_types::make_shared<ChMaterial3DStressParallel>();
    viscoelastic_material->SetMaterialA(elastic_material);
    viscoelastic_material->SetMaterialB(viscous_material);

    deformation_model->material = viscoelastic_material;  // set the material for model

    // FINITE ELEMENTS AND NODES
    // Build a test volume discretized with a regular grid of finite elements.
    double vol_size_x = 3;
    ChBuilderVolumeBox builder;
    builder.BuildVolume(ChFrame<>(ChVector3d(0, 0, z_offset)), 
                        8, 3, 2,   // N of elements in x,y,z direction
                        vol_size_x, 0.3, 0.2);  // width in x,y,z direction
    builder.AddToModel(deformation_model);

    // Set some node to fixed:
    for (auto mnode : builder.nodes.list()) {
        if (mnode->x() <= 0)
            displacement_field->NodeData(mnode).SetFixed(true);
    }

    // Create a crank and a rod that drive the end of the bar
    double rad = 0.2;
    double len = 1.0;
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);
    sys.Add(floor);

    auto crank = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, rad, 0.1, 1000);
    crank->SetPos(ChVector3d(vol_size_x - rad, -len, z_offset));
    sys.Add(crank);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    auto motor_func1 = chrono_types::make_shared<ChFunctionConst>(8);
    auto motor_func2 = chrono_types::make_shared<ChFunctionConst>(0);
    auto motor_funcseq = chrono_types::make_shared<ChFunctionSequence>();
    motor_funcseq->InsertFunct(motor_func1, 1.0);
    motor_funcseq->InsertFunct(motor_func2, 2.0);
    auto motor_funcrepeat = chrono_types::make_shared<ChFunctionRepeat>(motor_funcseq, 0, 3.0);
    motor->SetSpeedFunction(motor_funcrepeat);
    motor->Initialize(crank, floor, ChFrame<>(crank->GetPos()));
    sys.Add(motor);

    auto rod = chrono_types::make_shared<ChBodyEasyBox>(0.1, len, 0.1, 1000);
    rod->SetPos(ChVector3d(vol_size_x, -0.5 * len, 0.1 + z_offset));
    sys.Add(rod);

    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(rod, crank, ChFrame<>(ChVector3d(vol_size_x, -len, z_offset)));
    sys.Add(revolute);

    // pick 1st node of 1st element of face at X end of volume, just for demo connection to rod
    auto endnode = builder.faces_x_hi[0]->GetNode(0);
    auto noderodlink = chrono_types::make_shared<ChLinkFieldFrame>();
    noderodlink->Initialize(endnode, displacement_field, rod);
    sys.Add(noderodlink);

    // POSTPROCESSING & VISUALIZATION (optional)

    // show mesh painted with solid color
    auto visual_mesh2 = chrono_types::make_shared<ChVisualModelMesh>(deformation_model);
    visual_mesh2->AddPositionExtractor(ExtractPos());
    visual_mesh2->SetColor(ChColor(0.9f, 0.4f, 0.0f));
    deformation_model->AddVisualShape(visual_mesh2);
}




int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    if (true) {
        // Simulate a vibrating structure with structural damping.
        // The structural damping in the 3d full lagrangian nonlinear framework requires more
        // advanced formulations than conventional Rayleigh damping. Here is a demo. 

        // Create a Chrono physical system
        ChSystemNSC sys;

        CreateTestDampedCantilever(sys,
                                   15000,   // deviatoric damping
                                   1.5);

        CreateTestDampedCantilever(sys,
                                   100000,  // deviatoric damping
                                   3.0);
        

        // Create the Irrlicht visualization system
        auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("Multiphysics example: structural damping");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(0, 2, -4));
        vis->AddTypicalLights();

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sys.SetSolver(mkl_solver);

        //sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

        // Simulation loop
        double timestep = 0.01;

        while (vis->Run()) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            sys.DoStepDynamics(timestep);
        }
    }

    

    return 0;
}
