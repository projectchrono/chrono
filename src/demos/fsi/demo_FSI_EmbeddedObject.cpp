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
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].x < 0 || pos[n].y < 0; }
};
#endif

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double initial_spacing = 0.02;
    double step_size = 1e-4;
    bool verbose = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(verbose);
    auto sysFSI = fsi.GetFsiSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Set soil propertiees
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.8;
    mat_props.mu_fric_2 = 0.8;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 1e3;  // default

    fsi.SetElasticSPH(mat_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.d0_multiplier = 1;
    sph_params.free_surface_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::HOLMES;

    fsi.SetSPHParameters(sph_params);

    // Create body geometry
    double density = 5000;
    double mass = 0;
    ChMatrix33d inertia;
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(ChContactMaterialData());
    switch (object_shape) {
        case ObjectShape::SPHERE_PRIMITIVE: {
            double radius = 0.2;
            ChSphere sphere(radius);
            mass = density * sphere.GetVolume();
            inertia = mass * sphere.GetGyration();
            geometry->coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(VNULL, sphere, 0));
            break;
        }
        case ObjectShape::SPHERE_MESH: {
            double radius = 0.2;
            ChSphere sphere(radius);
            mass = density * sphere.GetVolume();
            inertia = mass * sphere.GetGyration();
            std::string mesh_filename = GetChronoDataFile("models/sphere.obj");
            geometry->coll_meshes.push_back(
                utils::ChBodyGeometry::TrimeshShape(VNULL, QUNIT, mesh_filename, VNULL, radius, 0.01, 0));
            break;
        }
        case ObjectShape::BOX_PRIMITIVE: {
            ChVector3d size(0.4, 0.11, 0.2);
            ChBox box(size);
            mass = density * box.GetVolume();
            inertia = density * box.GetGyration();
            geometry->coll_boxes.push_back(
                utils::ChBodyGeometry::BoxShape(ChVector3d(0.1, 0.1, 0), Q_ROTATE_Y_TO_Z, box, 0));
            break;
        }
        case ObjectShape::CYLINDER_PRIMITIVE: {
            double radius = 0.2;
            double length = 0.4;
            ChCylinder cylinder(radius, length);
            mass = density * cylinder.GetVolume();
            inertia = density * cylinder.GetGyration();
            geometry->coll_cylinders.push_back(
                utils::ChBodyGeometry::CylinderShape(VNULL, QuatFromAngleX(CH_PI / 4), cylinder, 0));
            break;
        }
        case ObjectShape::BOX_FRAME: {
            ChBox box1(ChVector3d(0.3, 0.1, 0.1));
            ChBox box2(ChVector3d(0.1, 0.1, 0.4));
            mass = density * box1.GetVolume();       // not exact
            inertia = density * box1.GetGyration();  // not exact
            geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, -0.15), QUNIT, box1, 0));
            geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, +0.15), QUNIT, box1, 0));
            geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(-0.2, 0, 0), QUNIT, box2, 0));
            geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(+0.2, 0, 0), QUNIT, box2, 0));
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
        geometry->CreateVisualizationAssets(body, VisualizationType::COLLISION);

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
    out_dir = out_dir + "/" + fsi.GetPhysicsProblemString() + "_" + fsi.GetSphIntegrationSchemeString();
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
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(-0.3, 0.3);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback, ChColormap::Type::COPPER);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Embedded Object");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(2, 1, 0.5), ChVector3d(0, 0, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

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
            fsi.SaveOutputData(time, out_dir + "/particles", out_dir + "/fsi");
            out_frame++;
        }

        // Render FSI system
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif

        // Call the FSI solver
        fsi.DoStepDynamics(step_size);

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    return 0;
}
