//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
//
//   Demo code (advanced), about
//
//     - loading an Abaqus tetahedron mesh
//     - apply a load to the mesh using Chrono::Parallel

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadBodyMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChMeshFileLoader.h"
#include "chrono_fea/ChLoadContactSurfaceMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGenerators.h"

// comment the following line out to see opengl view
#undef CHRONO_OPENGL

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::collision;

using namespace irr;

// Utility to draw some triangles that are affected by cosimulation.
// Also plot forces as vectors.
// Mostly for debugging. 
void draw_affected_triangles(ChIrrApp& application, std::vector<ChVector<>>& vert_pos, std::vector<ChVector<int>>& triangles, std::vector<int>& vert_indexes, std::vector<ChVector<>>& vert_forces, double forcescale=0.01) {
  for (int it= 0;it < triangles.size(); ++it) {
    bool vert_hit = false;
    for (int io = 0; io < vert_indexes.size(); ++io) {
      if (triangles[it].x() == vert_indexes[io] || triangles[it].y() == vert_indexes[io] || triangles[it].z() == vert_indexes[io])
        vert_hit = true;
    }
    if (vert_hit == true) {
      std::vector<chrono::ChVector<> > fourpoints = { vert_pos[triangles[it].x()],
          vert_pos[triangles[it].y()],
          vert_pos[triangles[it].z()],
          vert_pos[triangles[it].x()]};
      ChIrrTools::drawPolyline(application.GetVideoDriver(), fourpoints, irr::video::SColor(255,240,200,0), true);
    }
  }
  if (forcescale>0)
    for (int io = 0; io < vert_indexes.size(); ++io) {
      std::vector<chrono::ChVector<> > forceline =  { vert_pos[vert_indexes[io]],
          vert_pos[vert_indexes[io]]+vert_forces[io]*forcescale};
      ChIrrTools::drawPolyline(application.GetVideoDriver(), forceline, irr::video::SColor(100,240,0,0), true);
    }
}

int main(int argc, char* argv[]) {
  double time_step = 0.005;
  int max_iteration = 30;
  double tolerance = 1e-3;
  double time_end = 2.0;

  // Frequency for visualization output
  bool saveData = true;
  int out_fps = 60;

  // Global parameter for tire:
  double tire_rad = 0.8;
  double tire_vel_z0 = -3;
  ChVector<> tire_center(0, 1+0.02+tire_rad, 0);
  ChMatrix33<> tire_alignment(Q_from_AngAxis(CH_C_PI, VECT_Y)); // create rotated 180� on y

  double tire_w0 = tire_vel_z0/tire_rad;

  // Create a Chrono::Engine physical system
  ChSystemDEM my_system;
#ifndef CHRONO_OPENGL
  // Create the Irrlicht visualization (open the Irrlicht device,
  // bind a simple user interface, etc. etc.)
  ChIrrApp application(&my_system, L"FEA contacts", core::dimension2d<u32>(1280, 720), false, true);

  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
  application.AddTypicalLogo();
  application.AddTypicalSky();
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3dfCH(ChVector<>(3, 1.4, -3.2)),
                               core::vector3dfCH(ChVector<>(0, 0, 0)));

  application.AddLightWithShadow(core::vector3dfCH(ChVector<>(1.5, 5.5, -2.5)), core::vector3df(0, 0, 0), 3, 2.2, 7.2,
                                 40, 512, video::SColorf((f32)0.8, (f32)0.8, (f32)1.0));
#endif
  //
  // CREATE A FINITE ELEMENT MESH
  //

  // Create the surface material, containing information
  // about friction etc.

  auto mysurfmaterial = std::make_shared<ChMaterialSurfaceDEM>();
  mysurfmaterial->SetYoungModulus(10e4);
  mysurfmaterial->SetFriction(0.3f);
  mysurfmaterial->SetRestitution(0.2f);
  mysurfmaterial->SetAdhesion(0);

  // Create a mesh, that is a container for groups
  // of FEA elements and their referenced nodes.
  auto my_mesh = std::make_shared<ChMesh>();
  my_system.Add(my_mesh);

  // Create a material, that must be assigned to each solid element in the mesh,
  // and set its parameters
  auto mmaterial = std::make_shared<ChContinuumElastic>();
  mmaterial->Set_E(0.003e9);  // rubber 0.01e9, steel 200e9
  mmaterial->Set_v(0.4);
  mmaterial->Set_RayleighDampingK(0.004);
  mmaterial->Set_density(1000);

  // Load an ABAQUS .INP tetahedron mesh file from disk, defining a tetahedron mesh.
  // Note that not all features of INP files are supported. Also, quadratic tetahedrons are promoted to linear.
  // This is much easier than creating all nodes and elements via C++ programming.
  // Ex. you can generate these .INP files using Abaqus or exporting from the SolidWorks simulation tool.
  std::vector<std::vector<std::shared_ptr<ChNodeFEAbase> > > node_sets;
  try {
      ChMeshFileLoader::FromAbaqusFile(my_mesh, GetChronoDataFile("fea/tractor_wheel_coarse.INP").c_str(), mmaterial,
                                       node_sets, tire_center, tire_alignment);
  } catch (ChException myerr) {
      GetLog() << myerr.what();
      return 0;
  }

  // Create the contact surface(s).
  // Use the AddFacesFromBoundary() to select automatically the outer skin of the tetrahedron mesh:
  auto mcontactsurf = std::make_shared<ChContactSurfaceMesh>();
  my_mesh->AddContactSurface(mcontactsurf);
  mcontactsurf->AddFacesFromBoundary();
  mcontactsurf->SetMaterialSurface(mysurfmaterial); // by the way it is not needed because contacts will be emulated by cosimulation

  /// Create a mesh load for cosimulation, acting on the contact surface above
  /// (forces on nodes will be computed by an external procedure)
  auto mloadcontainer = std::make_shared<ChLoadContainer>();
  my_system.Add(mloadcontainer);
  auto mrigidmeshload = std::make_shared<ChLoadContactSurfaceMesh>(mcontactsurf);
  mloadcontainer->Add(mrigidmeshload);

  // ==Asset== attach a visualization of the FEM mesh.
  // This will automatically update a triangle mesh (a ChTriangleMeshShape
  // asset that is internally managed) by setting  proper
  // coordinates and vertex colours as in the FEM elements.
  auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
  mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
  mvisualizemesh->SetColorscaleMinMax(0.0, 10);
  mvisualizemesh->SetSmoothFaces(true);
  my_mesh->AddAsset(mvisualizemesh);

  //
  // END CREATE A FINITE ELEMENT MESH
  //

  /*
  //
  // CREATE A RIGID BODY WITH A MESH
  //

  // Create also a rigid body with a rigid mesh that will be used for the cosimulation,
  // this time the ChLoadContactSurfaceMesh cannot be used as in the FEA case, so we
  // will use the ChLoadBodyMesh class:

  auto mrigidbody = std::make_shared<ChBody>();
  my_system.Add(mrigidbody);
  mrigidbody->SetMass(200);
  mrigidbody->SetInertiaXX(ChVector<>(20,20,20));
  mrigidbody->SetPos(tire_center);

  auto mrigidmesh = std::make_shared<ChTriangleMeshShape>();
  mrigidmesh->GetMesh().LoadWavefrontMesh(GetChronoDataFile("tractor_wheel_fine.obj"));
  mrigidmesh->GetMesh().Transform(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y) );
  mrigidbody->AddAsset(mrigidmesh);

  auto mcol = std::make_shared<ChColorAsset>();
  mcol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
  mrigidbody->AddAsset(mcol);

  /// Create a mesh load for cosimulation, acting on the contact surface above
  /// (forces on nodes will be computed by an external procedure)

  auto mloadcontainer = std::make_shared<ChLoadContainer>();
  my_system.Add(mloadcontainer);

  // this is used to use the mesh in cosimulation!
  auto mrigidmeshload = std::make_shared<ChLoadBodyMesh>(mrigidbody, mrigidmesh->GetMesh());
  mloadcontainer->Add(mrigidmeshload);

  //
  // END CREATE A RIGID BODY WITH A MESH
  //
   *
  */

#ifndef CHRONO_OPENGL
  // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
  application.AssetBindAll();

  // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
  application.AssetUpdateAll();

  // Use shadows in realtime view
  application.AddShadowAll();
#endif

  // ==IMPORTANT!== Mark completion of system construction
  my_system.SetupInitial();

  //
  // THE SOFT-REAL-TIME CYCLE
  //

  // Change solver to embedded MINRES
  // NOTE! it is strongly advised that you compile the optional MKL module
  // if you need higher precision, and switch to its MKL solver - see demos for FEA & MKL.
  my_system.SetSolverType(ChSolver::Type::MINRES);
  my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
  my_system.SetMaxItersSolverSpeed(40);
  my_system.SetTolForce(1e-10);

  // Change type of integrator:
  my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise

#ifndef CHRONO_OPENGL
  application.SetTimestep(time_step);
#endif

  // BEGIN PARALLEL SYSTEM INITIALIZATION
  ChSystemParallelDVI* systemG = new ChSystemParallelDVI();

  // Set gravitational acceleration
  systemG->Set_G_acc(my_system.Get_G_acc());

  // Set solver parameters
  systemG->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
  systemG->GetSettings()->solver.max_iteration_normal = max_iteration / 3;
  systemG->GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
  systemG->GetSettings()->solver.max_iteration_spinning = 0;
  systemG->GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
  systemG->GetSettings()->solver.tolerance = tolerance;
  systemG->GetSettings()->solver.alpha = 0;
  systemG->GetSettings()->solver.contact_recovery_speed = 10000;
  systemG->ChangeSolverType(SolverType::APGD);
  systemG->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

  systemG->GetSettings()->collision.collision_envelope = 0.01;
  systemG->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

  auto triMat = std::make_shared<ChMaterialSurface>();
  triMat->SetFriction(0.4f);

  // Create the triangles for the tire geometry
  ChVector<> pos(0, 0, 0);
  ChVector<> vel(0, 0, 0);

  std::vector<ChVector<>> vert_pos;
  std::vector<ChVector<>> vert_vel;
  std::vector<ChVector<int>> triangles;
  std::vector<ChVector<>> vert_forces;
  std::vector<int> vert_indexes;
  vert_forces.clear();
  vert_indexes.clear();
  std::vector<ChVector<>> vert_forcesVisualization;
  std::vector<int> vert_indexesVisualization;
  vert_forcesVisualization.clear();
  vert_indexesVisualization.clear();
  mrigidmeshload->OutputSimpleMesh(vert_pos, vert_vel, triangles);
  for(int i=0; i<vert_pos.size(); i++) {
    vert_forces.push_back(ChVector<>(0,0,0));
    vert_indexes.push_back(i);
  }

  double mass = 2;//mrigidbody->GetMass()/((double) triangles.size());
  double radius = 0.005;
  ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

  int triId = 0;
  for (int i = 0; i < triangles.size(); i++) {
      auto triangle = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
      triangle->SetMaterialSurface(triMat);
      triangle->SetIdentifier(triId++);
      triangle->SetMass(mass);
      triangle->SetInertiaXX(inertia);
      pos = (vert_pos[triangles[i].x()] + vert_pos[triangles[i].y()] + vert_pos[triangles[i].z()]) / 3.0;
      vel = (vert_vel[triangles[i].x()] + vert_vel[triangles[i].y()] + vert_vel[triangles[i].z()]) / 3.0;
      triangle->SetPos(pos);
      triangle->SetPos_dt(vel);
      triangle->SetRot(ChQuaternion<>(1, 0, 0, 0));
      triangle->SetCollide(true);
      triangle->SetBodyFixed(true);

      triangle->GetCollisionModel()->ClearModel();
      // utils::AddSphereGeometry(triangle.get(), radius);
      std::string name = "tri" + std::to_string(triId);
      utils::AddTriangle(triangle.get(), vert_pos[triangles[i].x()] - pos, vert_pos[triangles[i].y()] - pos,
                         vert_pos[triangles[i].z()] - pos, name);
      triangle->GetCollisionModel()->SetFamily(1);
      triangle->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
      triangle->GetCollisionModel()->BuildModel();

      systemG->AddBody(triangle);
  }

  // Add the terrain, MUST BE ADDED AFTER TIRE GEOMETRY (for index assumptions)
  utils::CreateBoxContainer(systemG,-2,triMat,ChVector<>(1,1,1),0.1,ChVector<>(0,-1,0),QUNIT,true,true,true,false);

  double r = 0.1;//0.02;//
  double shapeRatio = 0.4;
  utils::Generator gen(systemG);
  auto m1 = gen.AddMixtureIngredient(utils::ELLIPSOID, 1.0);
  m1->setDefaultMaterial(triMat);
  m1->setDefaultDensity(2500);
  m1->setDefaultSize(ChVector<>(r,r*shapeRatio,r));

  gen.setBodyIdentifier(triId);
  ChVector<> hdims(1 - r*1.01, 0.5, 1 - r*1.01);
  ChVector<> center(0, 0, 0);
  gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);

#ifdef CHRONO_OPENGL
  // Initialize OpenGL
  opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
  gl_window.Initialize(1280, 720, "DEMO TTI", systemG);
  gl_window.SetCamera(ChVector<>(1, 1.4, -1.2), ChVector<>(0, tire_rad, 0), ChVector<>(0, 1, 0));
  gl_window.SetRenderMode(opengl::WIREFRAME);
#endif
  // END PARALLEL SYSTEM INITIALIZATION

  // Begin time loop
  int out_steps = (int) std::ceil((1.0 / time_step) / out_fps);
  int timeIndex = 0;
  double time = 0;
  int frameIndex = 0;
#ifdef CHRONO_OPENGL
  while (true) {
#else
    while (application.GetDevice()->run()) {
#endif
      //while (time<time_end) {

      // STEP 1: ADVANCE DYNAMICS OF GRANULAR SYSTEM
#ifdef CHRONO_OPENGL
      if (gl_window.Active()) {
        gl_window.DoStepDynamics(time_step);
        gl_window.Render();
      } else
        break;
#else
      systemG->DoStepDynamics(time_step);

      if(timeIndex%out_steps==0 && saveData) {
        char filename[100];
        sprintf(filename, "../POVRAY/data_%d.dat", frameIndex);

        utils::WriteShapesPovray(systemG, filename, false);
        std::string delim = ",";
        utils::CSV_writer csv(delim);
        csv << triangles.size() << std::endl;
        for(int i=0; i<triangles.size();i++) {
          csv << systemG->Get_bodylist()->at(i)->GetPos() << vert_pos[triangles[i].x()] << vert_pos[triangles[i].y()] << vert_pos[triangles[i].z()] << std::endl;
        }
        sprintf(filename, "../POVRAY/triangles_%d.dat", frameIndex);
        csv.write_to_file(filename);
      }
#endif
      // END STEP 1

      // STEP 2: APPLY CONTACT FORCES FROM GRANULAR TO TIRE SYSTEM
      real3 force(0, 0, 0);
      real3 torque(0, 0, 0);
      systemG->CalculateContactForces();

      vert_forces.clear();
      for(int i=0; i<vert_pos.size(); i++) {
        vert_forces.push_back(ChVector<>(0,0,0));
      }

      for(int i=0; i<triangles.size(); i++) {
        force = systemG->GetBodyContactForce(i);
        torque = systemG->GetBodyContactTorque(i);

        // TODO: Calculate force based on the position in the triangle
        vert_forces[triangles[i].x()] += ChVector<>(force.x, force.y, force.z)/3;
        vert_forces[triangles[i].y()] += ChVector<>(force.x, force.y, force.z)/3;
        vert_forces[triangles[i].z()] += ChVector<>(force.x, force.y, force.z)/3;
      }
      mrigidmeshload->InputSimpleForces(vert_forces, vert_indexes);
      // END STEP 2

      // STEP 3: ADVANCE DYNAMICS OF TIRE SYSTEM
#ifdef CHRONO_OPENGL
      my_system.DoStepDynamics(time_step);
#else
      application.BeginScene();

      application.DrawAll();

      application.DoStep();

      if(timeIndex%out_steps==0 && saveData) {
        //takeScreenshot(application.GetDevice(),frameIndex);
        frameIndex++;
      }
#endif
      // END STEP 3

      // STEP 4: UPDATE THE POSITION/VELOCITY OF THE TIRE GEOMETRY IN GRANULAR SYSTEM
      vert_pos.clear();
      vert_vel.clear();
      triangles.clear();

      mrigidmeshload->OutputSimpleMesh(vert_pos, vert_vel, triangles);

      for(int i=0; i<triangles.size(); i++) {
        std::shared_ptr<ChBody> triBody = systemG->Get_bodylist()->at(i);
        pos = (vert_pos[triangles[i].x()]+vert_pos[triangles[i].y()]+vert_pos[triangles[i].z()])/3.0;
        triBody->SetPos(pos);
        vel = (vert_vel[triangles[i].x()]+vert_vel[triangles[i].y()]+vert_vel[triangles[i].z()])/3.0;
        triBody->SetPos_dt(vel);

        //            // Update visual assets TODO: chrono_opengl cannot handle dynamic meshes yet
        //            for (int j = 0; j < triBody->GetAssets().size(); j++) {
        //              std::shared_ptr<ChAsset> asset = triBody->GetAssets()[j];
        //              if (std::dynamic_pointer_cast<ChTriangleMeshShape>(asset)) {
        //                //std::cout << j << std::endl;
        //                //std::cout << vert_pos[triangles[i].x()].x() << " " << vert_pos[triangles[i].x()].y() << " " << vert_pos[triangles[i].x()].z() << std::endl;
        //                ((ChTriangleMeshShape*)(asset.get()))->GetMesh().m_vertices[0] = vert_pos[triangles[i].x()];
        //                ((ChTriangleMeshShape*)(asset.get()))->GetMesh().m_vertices[1] = vert_pos[triangles[i].y()];
        //                ((ChTriangleMeshShape*)(asset.get()))->GetMesh().m_vertices[2] = ChVector<>(0);//vert_pos[triangles[i].z()];
        //              }
        //            }

        // Update collision information
        systemG->data_manager->shape_data.triangle_rigid[3 * i + 0] = real3(vert_pos[triangles[i].x()].x() - pos.x(), vert_pos[triangles[i].x()].y() - pos.y(), vert_pos[triangles[i].x()].z() - pos.z());
        systemG->data_manager->shape_data.triangle_rigid[3 * i + 1] = real3(vert_pos[triangles[i].y()].x() - pos.x(), vert_pos[triangles[i].y()].y() - pos.y(), vert_pos[triangles[i].y()].z() - pos.z());
        systemG->data_manager->shape_data.triangle_rigid[3 * i + 2] = real3(vert_pos[triangles[i].z()].x() - pos.x(), vert_pos[triangles[i].z()].y() - pos.y(), vert_pos[triangles[i].z()].z() - pos.z());
      }
      // END STEP 4

#ifndef CHRONO_OPENGL
      // now, just for debugging and some fun, draw some triangles
      // (only those that have a vertex that has a force applied):
      vert_forcesVisualization.clear();
      vert_indexesVisualization.clear();
      for(int i=0; i<vert_forces.size();i++) {
        if(vert_forces[i].Length()>1e-5) {
          vert_forcesVisualization.push_back(vert_forces[i]);
          vert_indexesVisualization.push_back(vert_indexes[i]);
        }
      }
      draw_affected_triangles(application, vert_pos, triangles, vert_indexesVisualization, vert_forcesVisualization, 0.01);


      // End of cosimulation block
      // -------------------------------------------------------------------------


      ChIrrTools::drawGrid(application.GetVideoDriver(), 0.1, 0.1, 20, 20, ChCoordsys<>(VNULL, CH_C_PI_2, VECT_X), video::SColor(50, 90, 90, 90), true);

      application.EndScene();
#endif
      timeIndex++;
      time+=time_step;
      std::cout << time << std::endl;
    }

    return 0;
  }

