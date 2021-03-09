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

#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(0, 0.02 + tire_rad, 0.5);
    ChMatrix33<> tire_alignment(Q_from_AngAxis(CH_C_PI, VECT_Y));  // create rotated 180° on y

    double tire_w0 = tire_vel_z0 / tire_rad;

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"FEA contacts", core::dimension2d<u32>(1280, 720), VerticalDir::Y, false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3dfCH(ChVector<>(1, 1.4, -1.2)),
                                 core::vector3dfCH(ChVector<>(0, tire_rad, 0)));
    // application.SetContactsDrawMode(irr::tools::CONTACT_DISTANCES);

    application.AddLightWithShadow(core::vector3dfCH(ChVector<>(1.5, 5.5, -2.5)), core::vector3df(0, 0, 0), 3, 2.2, 7.2,
                                   40, 512, video::SColorf((f32)0.8, (f32)0.8, (f32)1.0));

    //
    // CREATE THE PHYSICAL SYSTEM
    //

    // Create the surface material, containing information
    // about friction etc.
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(10e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    auto mysurfmaterial2 = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(30e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // RIGID BODIES
    // Create some rigid bodies, for instance a floor:
    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.2, 6, 2700, true, true, mysurfmaterial);
    mfloor->SetBodyFixed(true);
    my_system.Add(mfloor);

    auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    mfloor->AddAsset(mtexture);

    // Create a step
    if (false) {
        auto mfloor_step = chrono_types::make_shared<ChBodyEasyBox>(1, 0.2, 0.5, 2700, true, true, mysurfmaterial);
        mfloor_step->SetPos(ChVector<>(0, 0.1, -0.2));
        mfloor_step->SetBodyFixed(true);
        my_system.Add(mfloor_step);
    }

    // Create some bent rectangular fixed slabs
    if (false) {
        for (int i = 0; i < 50; ++i) {
            auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.25, 0.2, 0.25, 2700, true, true, mysurfmaterial);
            ChQuaternion<> vrot;
            vrot.Q_from_AngAxis(ChRandom() * CH_C_2PI, VECT_Y);
            mcube->Move(ChCoordsys<>(VNULL, vrot));
            vrot.Q_from_AngAxis((ChRandom() - 0.5) * 2 * CH_C_DEG_TO_RAD * 20,
                                ChVector<>(ChRandom() - 0.5, 0, ChRandom() - 0.5).Normalize());
            mcube->Move(ChCoordsys<>(VNULL, vrot));
            mcube->SetPos(ChVector<>((ChRandom() - 0.5) * 1.8, ChRandom() * 0.1, -ChRandom() * 3.2 + 0.9));
            mcube->SetBodyFixed(true);
            my_system.Add(mcube);
            auto mcubecol = chrono_types::make_shared<ChColorAsset>();
            mcubecol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            mcube->AddAsset(mcubecol);
        }
    }

    // Create some stones / obstacles on the ground
    if (true) {
        for (int i = 0; i < 150; ++i) {
            auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.18, 0.04, 0.18, 2700, true, true, mysurfmaterial2);
            ChQuaternion<> vrot;
            vrot.Q_from_AngAxis(ChRandom() * CH_C_2PI, VECT_Y);
            mcube->Move(ChCoordsys<>(VNULL, vrot));
            mcube->SetPos(ChVector<>((ChRandom() - 0.5) * 1.4, ChRandom() * 0.2 + 0.05, -ChRandom() * 2.6 + 0.2));
            my_system.Add(mcube);
            auto mcubecol = chrono_types::make_shared<ChColorAsset>();
            mcubecol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            mcube->AddAsset(mcubecol);
        }
    }

    // FINITE ELEMENT MESH
    // Create a mesh, that is a container for groups
    // of FEA elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(0.016e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.4);
    mmaterial->Set_RayleighDampingK(0.004);
    mmaterial->Set_density(1000);

    // Load an ABAQUS .INP tetrahedron mesh file from disk, defining a tetrahedron mesh.
    // Note that not all features of INP files are supported. Also, quadratic tetrahedrons are promoted to linear.
    // This is much easier than creating all nodes and elements via C++ programming.
    // Ex. you can generate these .INP files using Abaqus or exporting from the SolidWorks simulation tool.

    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets;

    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh,
                                         GetChronoDataFile("models/tractor_wheel/tractor_wheel_coarse.INP").c_str(),
                                         mmaterial, node_sets, tire_center, tire_alignment);
    } catch (ChException myerr) {
        GetLog() << myerr.what();
        return 0;
    }

    // Create the contact surface(s).
    // In this case it is a ChContactSurfaceNodeCloud, so just pass
    // all nodes to it.
    auto mcontactsurf = chrono_types::make_shared<ChContactSurfaceNodeCloud>(mysurfmaterial);
    my_mesh->AddContactSurface(mcontactsurf);
    mcontactsurf->AddAllNodes();

    // Apply initial speed and angular speed
    double speed_x0 = 0.5;
    for (unsigned int i = 0; i < my_mesh->GetNnodes(); ++i) {
        ChVector<> node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->GetPos();
        ChVector<> tang_vel = Vcross(ChVector<>(tire_w0, 0, 0), node_pos - tire_center);
        std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))
            ->SetPos_dt(ChVector<>(0, 0, tire_vel_z0) + tang_vel);
    }

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Add a rim
    auto mwheel_rim = chrono_types::make_shared<ChBody>();
    mwheel_rim->SetMass(80);
    mwheel_rim->SetInertiaXX(ChVector<>(60, 60, 60));
    mwheel_rim->SetPos(tire_center);
    mwheel_rim->SetRot(tire_alignment);
    mwheel_rim->SetPos_dt(ChVector<>(0, 0, tire_vel_z0));
    mwheel_rim->SetWvel_par(ChVector<>(tire_w0, 0, 0));
    application.GetSystem()->Add(mwheel_rim);

    auto mobjmesh = chrono_types::make_shared<ChObjShapeFile>();
    mobjmesh->SetFilename(GetChronoDataFile("models/tractor_wheel/tractor_wheel_rim.obj"));
    mwheel_rim->AddAsset(mobjmesh);

    // Connect rim and tire using constraints.
    // the BC_RIMTIRE nodeset, in the Abaqus INP file, lists the nodes involved
    auto nodeset_sel = "BC_RIMTIRE";
    for (auto i = 0; i < node_sets.at(nodeset_sel).size(); ++i) {
        auto mlink = chrono_types::make_shared<ChLinkPointFrame>();
        mlink->Initialize(std::dynamic_pointer_cast<ChNodeFEAxyz>(node_sets[nodeset_sel][i]), mwheel_rim);
        my_system.Add(mlink);
    }

    // Create a mesh surface, for applying loads:
    auto mmeshsurf = chrono_types::make_shared<ChMeshSurface>();
    my_mesh->AddMeshSurface(mmeshsurf);

    // Nodes of the load surface are those of the nodeset with label BC_SURF:
    nodeset_sel = "BC_SURF";
    mmeshsurf->AddFacesFromNodeSet(node_sets[nodeset_sel]);

    // Apply load to all surfaces in the mesh surface
    auto mloadcontainer = chrono_types::make_shared<ChLoadContainer>();
    my_system.Add(mloadcontainer);

    for (auto i = 0; i < mmeshsurf->GetFacesList().size(); ++i) {
        auto aface = std::shared_ptr<ChLoadableUV>(mmeshsurf->GetFacesList()[i]);
        auto faceload = chrono_types::make_shared<ChLoad<ChLoaderPressure>>(aface);
        faceload->loader.SetPressure(10000);  // low pressure... the tire has no ply!
        mloadcontainer->Add(faceload);
    }

    //
    // Optional...  visualization
    //

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!
    auto mvisualizemesh = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 10);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    /*
        auto mvisualizemeshB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
        mvisualizemeshB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
        mvisualizemeshB->SetWireframe(true);
        my_mesh->AddAsset(mvisualizemeshB);
    */
    /*
        auto mvisualizemeshC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
        mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
        mvisualizemeshC->SetSymbolsThickness(0.006);
        my_mesh->AddAsset(mvisualizemeshC);
     */

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();

    //
    // SIMULATION LOOP
    //

    // Change solver to Pardiso from Chrono::PardisoMKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    my_system.SetSolver(mkl_solver);
    my_system.Update();

    // Change type of integrator:
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    // my_system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(2);
        mystepper->SetAbsTolerances(1e-6);
    }

    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
