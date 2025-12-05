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
// Authors: Luning Bakke, Radu Serban
// =============================================================================
// Couette Flow simulation with coaxial cylinders.
// Outer cylinder spins at a constatn speed and the inner cylinder is fixed.
// Motor torque is recorded and saved to a file
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

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

// Output directories
std::string out_dir = GetChronoOutputPath() + "FSI_Couette_Flow";

// Angular speed of outer cylinder
double omega = 62.0 / 60;

// Save data as csv files to see the results off-line using Paraview
bool output = false;
double output_fps = 50;

// Enable/disable run-time visualization
bool render = true;
double render_fps = 500;

// Enable saving snapshots
bool snapshots = false;

// Visibility flags
bool show_rigid_bce = true;
bool show_boundary_bce = true;
bool show_particles_sph = true;

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].x < 0; }
};
#endif

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    double step_size = 5e-4;
    double time_end = 1;
    double initial_spacing = 0.15;  // mm
    double density = 0.001;         // g/mm3
    double viscosity = 0.001;       // g/(mm.s2)
    double g = 9810.0;              // mm/s2
    bool verbose = true;

    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    ////sysFSI.SetVerbose(verbose);

    // cylindrical container
    // container needs to be higher than fluid, otherwise water spills from the top
    double outer_cylinder_radius = 5.0;  // mm
    double inner_cylinder_radius = 3.8;  // mm
    double fluid_height = 2.0;
    double cylinder_height = 1.2 * fluid_height;

    double bxDim = 14.0;  // mm
    double byDim = 10.0;  // mm
    double bzDim = 14.0;  // mm

    // Set gravitational acceleration
    const ChVector3d gravity(0, -g, 0);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Set integration step size
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = density;
    fluid_props.viscosity = viscosity;
    sysSPH.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::IMPLICIT_SPH;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 1;                     // default: 1.0
    sph_params.shifting_xsph_eps = 0.1;              // default: 0.5
    sph_params.shifting_beta_implicit = 0.0;         // default: 1.0
    sph_params.min_distance_coefficient = 0.001;     // default: 0.01, use 0.001, otherwise fluid surface is asymmetric
    sph_params.use_density_based_projection = true;  // default: false
    sph_params.num_proximity_search_steps = 1;
    sysSPH.SetSPHParameters(sph_params);

    // Set linear solver parameters
    ChFsiFluidSystemSPH::LinSolverParameters linsolv_params;
    linsolv_params.type = SolverType::JACOBI;
    linsolv_params.atol = 0;
    linsolv_params.rtol = 0;
    linsolv_params.max_num_iters = 200;
    sysSPH.SetLinSolverParameters(linsolv_params);

    // Set the terrain container size
    sysSPH.SetContainerDim(ChVector3<>(bxDim, byDim, bzDim));
    ChVector3<> cMin(-bxDim / 2 * 1.2, -byDim * 1.2, -bzDim / 2 * 1.2);
    ChVector3<> cMax(bxDim / 2 * 1.2, byDim * 1.2, bzDim / 2 * 1.2);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_ALL_PERIODIC);

    bool use_polar_coords = true;

    auto points = sysSPH.CreatePointsCylinderAnnulus(inner_cylinder_radius + initial_spacing / 2,  //
                                                     outer_cylinder_radius - initial_spacing / 2,  //
                                                     fluid_height,                                 //
                                                     use_polar_coords);
    for (const auto& p : points) {
        double x = p.x();
        double y = p.z();
        double z = -p.y();
        double pressure = g * fluid_props.density * (fluid_height - y);
        sysSPH.AddSPHParticle({x, y, z}, fluid_props.density, pressure, fluid_props.viscosity);
    }

    // Add cylinder bottom plate, fixed
    ChVector2d bottom_plate_size(outer_cylinder_radius * 2.5, outer_cylinder_radius * 2.5);
    auto bottom_plate = chrono_types::make_shared<ChBody>();
    bottom_plate->SetPos(ChVector3d(0, -fluid_height / 2 - initial_spacing, 0));
    bottom_plate->SetFixed(true);
    sysMBS.AddBody(bottom_plate);

    auto bce_plate = sysSPH.CreatePointsPlate(bottom_plate_size);
    sysFSI.AddFsiBoundary(bce_plate, ChFrame<>(ChVector3d(0, -fluid_height / 2 - initial_spacing, 0), Q_ROTATE_Z_TO_Y));

    // Cylinder center
    ChVector3d cylinder_center(0, cylinder_height / 2 - fluid_height / 2, 0);

    // Create inner cylinder, fixed
    auto inner_cylinder = chrono_types::make_shared<ChBody>();
    inner_cylinder->SetFixed(true);
    inner_cylinder->EnableCollision(false);
    sysMBS.AddBody(inner_cylinder);

    auto bce_inner = sysSPH.CreatePointsCylinderAnnulus(inner_cylinder_radius - 3 * initial_spacing,  //
                                                        inner_cylinder_radius - initial_spacing / 2,  //
                                                        cylinder_height,                              //
                                                        use_polar_coords);
    const auto& fsi_inner_cylinder =
        sysFSI.AddFsiBody(inner_cylinder, bce_inner, ChFrame<>(cylinder_center, Q_ROTATE_Z_TO_Y), false);
    auto inner_cylinder_index = fsi_inner_cylinder->index;

    // Create outer cylinder, spinning
    double outer_cylinder_mass = 1.0;
    auto outer_cylinder = chrono_types::make_shared<ChBody>();
    sysMBS.AddBody(outer_cylinder);
    outer_cylinder->SetFixed(false);
    outer_cylinder->EnableCollision(false);
    outer_cylinder->SetMass(outer_cylinder_mass);
    outer_cylinder->SetInertia(ChMatrix33d(ChVector3d(1, 1, 1)));

    auto bce_outer = sysSPH.CreatePointsCylinderAnnulus(outer_cylinder_radius + initial_spacing / 2,  //
                                                        outer_cylinder_radius + 3 * initial_spacing,  //
                                                        cylinder_height,                              //
                                                        true);
    const auto& fsi_outer_cylinder =
        sysFSI.AddFsiBody(outer_cylinder, bce_outer, ChFrame<>(cylinder_center, Q_ROTATE_Z_TO_Y), false);
    auto outer_cylinder_index = fsi_outer_cylinder->index;

    // Add motor between outer cylinder and plate
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(omega));
    motor->Initialize(outer_cylinder, bottom_plate, ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y));
    sysMBS.AddLink(motor);

    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphIntegrationSchemeString();
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

    // Create a run-tme visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto v_max = omega * outer_cylinder_radius;
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, v_max);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Couette Flow");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->SetCameraVertical(CameraVerticalDir::Y);
        visVSG->AddCamera(ChVector3<>(1.1 * bxDim, 1.8 * byDim, 0), VNULL);
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Write the information into a txt file
    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    double timer_CFD = 0;
    double timer_MBD = 0;
    double timer_FSI = 0;
    double timer_step = 0;

    ChTimer timer;
    timer.start();
    while (time < time_end) {
        // Get the infomation of the spinning cylinder
        auto torque_motor = -motor->GetMotorTorque();
        auto angvel_outer = outer_cylinder->GetAngVelParent().y();
        auto torque_inner = sysFSI.GetFsiBodyTorque(inner_cylinder_index).y();
        auto torque_outer = sysFSI.GetFsiBodyTorque(outer_cylinder_index).y();
        ////auto torque_plate = sysFSI.GetFsiBodyTorque(bottom_plate_index).y();

        if (verbose) {
            cout << "time: " << time << endl;
            cout << "  cylinder angular velocity: " << angvel_outer << endl;
            cout << "  motor torque:              " << torque_motor << endl;
        }

        ofile << time << "\t" << angvel_outer << "\t";
        ofile << torque_motor << "\t" << torque_inner << "\t" << torque_outer << "\n";
        ////ofile << torque_motor << "\t" << torque_inner << "\t" << torque_outer << "\t" << torque_plate << "\n";

        if (output && time >= out_frame / output_fps) {
            cout << "-------- Output" << endl;
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            out_frame++;
        }

        // Render SPH particles
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
        sysFSI.DoStepDynamics(step_size);

        timer_CFD += sysFSI.GetTimerCFD();
        timer_MBD += sysFSI.GetTimerMBD();
        timer_FSI += sysFSI.GetTimerFSI();
        timer_step += sysFSI.GetTimerStep();
        if (verbose && sim_frame == 2000) {
            cout << "Cummulative timers at time: " << time << endl;
            cout << "   timer CFD:  " << timer_CFD << endl;
            cout << "   timer MBD:  " << timer_MBD << endl;
            cout << "   timer FSI:  " << timer_FSI << endl;
            cout << "   timer step: " << timer_step << endl;
        }

        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    ofile.close();

#ifdef CHRONO_POSTPROCESS
    std::string title = "Couette flow - Torques (" + sysSPH.GetSphIntegrationSchemeString() + ")";
    postprocess::ChGnuPlot gplot(out_dir + "/results.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("torque (g * mm^2 / s^2)");
    gplot.SetTitle(title);
    gplot.Plot(out_file, 1, 3, "motor", " every ::5 with lines lt -1 lw 2");
    gplot.Plot(out_file, 1, 4, "inner", " every ::5 with lines lt 1 lw 2");
    gplot.Plot(out_file, 1, 5, "outer", " every ::5 with lines lt 2 lw 2");
    ////gplot.Plot(out_file, 1, 6, "plate", " every ::5 with lines lt 3 lw 2");
#endif

    return 0;
}
