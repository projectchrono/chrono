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
// FEA advanced demo:
//     - loading an Abaqus tetrahedron mesh
//     - using it as a wheel with contacts to ground
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoadContainer.h"

#include "chrono/core/ChRandom.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector3d tire_center(0, 0.02 + tire_rad, 0.5);
    ChMatrix33<> tire_alignment(QuatFromAngleY(CH_PI));  // create rotated 180 deg on y

    double tire_w0 = tire_vel_z0 / tire_rad;

    // Create a Chrono physical system and set the associated collision system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    // Create the surface material
    auto mysurfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(10e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    auto mysurfmaterial2 = chrono_types::make_shared<ChContactMaterialSMC>();
    mysurfmaterial->SetYoungModulus(30e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // RIGID BODIES
    // Create some rigid bodies, for instance a floor:
    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.2, 6, 2700, true, true, mysurfmaterial);
    mfloor->SetFixed(true);
    mfloor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
    sys.Add(mfloor);

    // Create a step
    if (false) {
        auto mfloor_step = chrono_types::make_shared<ChBodyEasyBox>(1, 0.2, 0.5, 2700, true, true, mysurfmaterial);
        mfloor_step->SetPos(ChVector3d(0, 0.1, -0.2));
        mfloor_step->SetFixed(true);
        sys.Add(mfloor_step);
    }

    // Create some bent rectangular fixed slabs
    if (false) {
        for (int i = 0; i < 50; ++i) {
            auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.25, 0.2, 0.25, 2700, true, true, mysurfmaterial);
            ChQuaternion<> vrot;
            vrot.SetFromAngleY(ChRandom::Get() * CH_2PI);
            mcube->Move(ChCoordsys<>(VNULL, vrot));
            vrot.SetFromAngleAxis((ChRandom::Get() - 0.5) * 2 * CH_DEG_TO_RAD * 20,
                                  ChVector3d(ChRandom::Get() - 0.5, 0, ChRandom::Get() - 0.5).Normalize());
            mcube->Move(ChCoordsys<>(VNULL, vrot));
            mcube->SetPos(
                ChVector3d((ChRandom::Get() - 0.5) * 1.8, ChRandom::Get() * 0.1, -ChRandom::Get() * 3.2 + 0.9));
            mcube->SetFixed(true);
            sys.Add(mcube);
        }
    }

    // Create some stones / obstacles on the ground
    if (true) {
        for (int i = 0; i < 150; ++i) {
            auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.18, 0.04, 0.18, 2700, true, true, mysurfmaterial2);
            ChQuaternion<> vrot;
            vrot.SetFromAngleY(ChRandom::Get() * CH_2PI);
            mcube->Move(ChCoordsys<>(VNULL, vrot));
            mcube->SetPos(
                ChVector3d((ChRandom::Get() - 0.5) * 1.4, ChRandom::Get() * 0.2 + 0.05, -ChRandom::Get() * 2.6 + 0.2));
            sys.Add(mcube);
        }
    }

    // FINITE ELEMENT MESH
    // Create a mesh, that is a container for groups
    // of FEA elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->SetYoungModulus(0.016e9);  // rubber 0.01e9, steel 200e9
    mmaterial->SetPoissonRatio(0.4);
    mmaterial->SetRayleighDampingBeta(0.004);
    mmaterial->SetDensity(1000);

    // Load an ABAQUS .INP tetrahedron mesh file from disk, defining a tetrahedron mesh.
    // Note that not all features of INP files are supported. Also, quadratic tetrahedrons are promoted to linear.
    // This is much easier than creating all nodes and elements via C++ programming.
    // Ex. you can generate these .INP files using Abaqus or exporting from the SolidWorks simulation tool.

    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets;

    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh,
                                         GetChronoDataFile("models/tractor_wheel/tractor_wheel_coarse.INP").c_str(),
                                         mmaterial, node_sets, tire_center, tire_alignment);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 0;
    }

    // Create the contact surface(s).
    // In this case it is a ChContactSurfaceNodeCloud, so just pass
    // all nodes to it.
    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mysurfmaterial);
    my_mesh->AddContactSurface(mcontactsurf);
    mcontactsurf->AddAllNodes();

    // Apply initial speed and angular speed
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); ++i) {
        ChVector3d node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->GetPos();
        ChVector3d tang_vel = Vcross(ChVector3d(tire_w0, 0, 0), node_pos - tire_center);
        std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))
            ->SetPosDt(ChVector3d(0, 0, tire_vel_z0) + tang_vel);
    }

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Add a rim
    auto mwheel_rim = chrono_types::make_shared<ChBody>();
    mwheel_rim->SetMass(80);
    mwheel_rim->SetInertiaXX(ChVector3d(60, 60, 60));
    mwheel_rim->SetPos(tire_center);
    mwheel_rim->SetRot(tire_alignment);
    mwheel_rim->SetPosDt(ChVector3d(0, 0, tire_vel_z0));
    mwheel_rim->SetAngVelParent(ChVector3d(tire_w0, 0, 0));
    sys.Add(mwheel_rim);

    auto mobjmesh = chrono_types::make_shared<ChVisualShapeModelFile>();
    mobjmesh->SetFilename(GetChronoDataFile("models/tractor_wheel/tractor_wheel_rim.obj"));
    mwheel_rim->AddVisualShape(mobjmesh);

    // Connect rim and tire using constraints.
    // the BC_RIMTIRE nodeset, in the Abaqus INP file, lists the nodes involved
    auto nodeset_sel = "BC_RIMTIRE";
    for (auto i = 0; i < node_sets.at(nodeset_sel).size(); ++i) {
        auto mlink = chrono_types::make_shared<ChLinkNodeFrame>();
        mlink->Initialize(std::dynamic_pointer_cast<ChNodeFEAxyz>(node_sets[nodeset_sel][i]), mwheel_rim);
        sys.Add(mlink);
    }

    // Create a mesh surface, for applying loads:
    auto mmeshsurf = chrono_types::make_shared<ChMeshSurface>();
    my_mesh->AddMeshSurface(mmeshsurf);

    // Nodes of the load surface are those of the nodeset with label BC_SURF:
    nodeset_sel = "BC_SURF";
    mmeshsurf->AddFacesFromNodeSet(node_sets[nodeset_sel]);

    // Apply load to all surfaces in the mesh surface
    auto mloadcontainer = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(mloadcontainer);

    for (const auto& face : mmeshsurf->GetFaces()) {
        auto face_loader = chrono_types::make_shared<ChLoaderPressure>(face);
        face_loader->SetPressure(10000);  // low pressure... the tire has no ply!
        auto face_load = chrono_types::make_shared<ChLoad>(face_loader);
        mloadcontainer->Add(face_load);
    }

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChVisualShapeTriangleMesh).
    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 10);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    /*
        auto mvisualizemeshB = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
        mvisualizemeshB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizemeshB->SetWireframe(true);
        my_mesh->AddVisualShapeFEA(mvisualizemeshB);
    */
    /*
        auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
        mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        mvisualizemeshC->SetSymbolsThickness(0.006);
        my_mesh->AddVisualShapeFEA(mvisualizemeshC);
     */

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Abaqus wheel",
                                         ChVector3d(1.0, 1.4, -1.2), ChVector3d(0, tire_rad, 0));

    // SIMULATION LOOP

    // Change solver to Pardiso from Chrono::PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sys.SetSolver(mkl_solver);
    sys.Update();

    // Change type of integrator:
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    // sys.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxIters(2);
        mystepper->SetAbsTolerances(1e-6);
    }

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.001);
    }

    return 0;
}
