// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_EmbeddedObstacle";

// Object geometry
enum class ObjectShape { SPHERE_PRIMITIVE, SPHERE_MESH, BOX_PRIMITIVE, CYLINDER_PRIMITIVE, BOX_FRAME };
ObjectShape object_shape = ObjectShape::BOX_FRAME;

// Output frequency
bool output = false;
double output_fps = 20;

// Object initial height
double initial_height = 0.25;

// Final simulation time
double t_end = 200.0;

// Enable/disable run-time visualization
bool render = true;
float render_fps = 300;

// Enable saving snapshots
bool snapshots = false;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = false;
bool show_boundary_bce = true;
bool show_particles_sph = true;

ChFsiVisualization::RenderMode render_mode = ChFsiVisualization::RenderMode::SOLID;
////ChFsiVisualization::RenderMode render_mode = ChFsiVisualization::RenderMode::WIREFRAME;

// -----------------------------------------------------------------------------

class MarkerPositionVisibilityCallback : public ChFsiVisualization::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}

    virtual bool get(unsigned int n) const override {
        auto p = pos[n];
        return p.x < 0 || p.y < 0;
    }
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double initial_spacing = 0.02;
    double step_size = 1e-4;
    bool verbose = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(sysMBS, initial_spacing);
    fsi.SetVerbose(verbose);
    ChFsiSystemSPH& sysFSI = fsi.GetSystemFSI();
    ChFluidSystemSPH& sysSPH = fsi.GetFluidSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Set integration step size
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    // Set soil propertiees
    ChFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.8;
    mat_props.mu_fric_2 = 0.8;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 1e3;  // default

    sysSPH.SetElasticSPH(mat_props);

    // Set SPH solution parameters
    ChFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    sph_params.boundary_type = BoundaryType::HOLMES;
    sysSPH.SetSPHParameters(sph_params);

    // Create body geometry
    double density = 5000;
    double mass = 0;
    ChMatrix33d inertia;
    utils::ChBodyGeometry geometry;
    geometry.materials.push_back(ChContactMaterialData());
    switch (object_shape) {
        case ObjectShape::SPHERE_PRIMITIVE: {
            double radius = 0.2;
            ChSphere sphere(radius);
            mass = density * sphere.GetVolume();
            inertia = mass * sphere.GetGyration();
            geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, radius, 0));
            break;
        }
        case ObjectShape::SPHERE_MESH: {
            double radius = 0.2;
            ChSphere sphere(radius);
            mass = density * sphere.GetVolume();
            inertia = mass * sphere.GetGyration();
            std::string mesh_filename = GetChronoDataFile("models/sphere.obj");
            geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, mesh_filename, VNULL, 0.01));
            break;
        }
        case ObjectShape::BOX_PRIMITIVE: {
            ChVector3d size(0.4, 0.11, 0.2);
            ChBox box(size);
            mass = density * box.GetVolume();
            inertia = density * box.GetGyration();
            geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(0.1, 0.1, 0), Q_ROTATE_Y_TO_Z, box, 0));
            break;
        }
        case ObjectShape::CYLINDER_PRIMITIVE: {
            double radius = 0.2;
            double length = 0.4;
            ChCylinder cylinder(radius, length);
            mass = density * cylinder.GetVolume();
            inertia = density * cylinder.GetGyration();
            geometry.coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(VNULL, QuatFromAngleX(CH_PI/4), cylinder, 0));
            break;
        }
        case ObjectShape::BOX_FRAME: {
            ChBox box1(ChVector3d(0.3, 0.1, 0.1));
            ChBox box2(ChVector3d(0.1, 0.1, 0.4));
            mass = density * box1.GetVolume();       // not exact
            inertia = density * box1.GetGyration();  // not exact
            geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, -0.15), QUNIT, box1, 0));
            geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, +0.15), QUNIT, box1, 0));
            geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(-0.2, 0, 0), QUNIT, box2, 0));
            geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(+0.2, 0, 0), QUNIT, box2, 0));
            break;
        }
    }

    // Create the rigid body
    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("ball");
    body->SetPos(ChVector3d(0, 0, initial_height));
    body->SetRot(QuatFromAngleZ(-CH_PI / 4));
    body->SetMass(mass);
    body->SetInertia(inertia);
    body->SetFixed(false);
    body->EnableCollision(false);
    sysMBS.AddBody(body);

    if (show_rigid)
        geometry.CreateVisualizationAssets(body, VisualizationType::COLLISION);

    // Add as an FSI body
    fsi.AddRigidBody(body, geometry, true);

    // Create SPH fluid particles and BCE boundary markers
    fsi.Construct(GetChronoDataFile("vehicle/terrain/height_maps/bump64.bmp"),  // height map image file
                  2, 1,                                                         // length (X) and width (Y)
                  {0, 0.3},                                                     // height range
                  0.3,                                                          // depth
                  true,                                                         // uniform depth
                  ChVector3d(0, 0, 0),                                          // patch center
                  BoxSide::Z_NEG                                                // bottom wall
    );

    fsi.Initialize();

    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphMethodTypeString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            cerr << "Error creating directory " << out_dir + "/particles" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            cerr << "Error creating directory " << out_dir + "/fsi" << endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
            cerr << "Error creating directory " << out_dir + "/vtk" << endl;
            return 1;
        }
    }

    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
            cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
            return 1;
        }
    }

    ////fsi.SaveInitialMarkers(out_dir);

    // Create a run-time visualizer
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif
#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        auto col_callback =
            chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.3f, 0.6f, 0.0f), -0.3, 0.3);
        ////auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 1.0);

        visFSI->SetTitle("Chrono::FSI cylinder drop");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(2, 1, 0.5), ChVector3d(0, 0, 0));
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetRenderMode(render_mode);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            cout << " -- Output frame " << out_frame << " at t = " << time << endl;
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            out_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;

            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    return 0;
}
