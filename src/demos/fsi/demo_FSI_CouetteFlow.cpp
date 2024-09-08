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
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotionImposed.h"

#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_fsi/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
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

class PositionVisibilityCallback : public ChParticleCloud::VisibilityCallback {
  public:
    PositionVisibilityCallback() {}

    virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override {
        auto p = cloud.GetParticlePos(n);
        return p.x() < 0;
    };
};

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
    ChSystemFsi sysFSI(&sysMBS);
    sysFSI.SetVerbose(verbose);

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

    // Set CFD fluid properties
    ChSystemFsi::FluidProperties fluid_props;
    fluid_props.density = density;
    fluid_props.viscosity = viscosity;
    sysFSI.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChSystemFsi::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::I2SPH;
    sph_params.num_bce_layers = 3;
    sph_params.kernel_h = initial_spacing;
    sph_params.initial_spacing = initial_spacing;
    sph_params.max_velocity = 1;                     // default: 1.0
    sph_params.xsph_coefficient = 0.1;               // default: 0.5
    sph_params.shifting_coefficient = 0.0;           // default: 0.1
    sph_params.min_distance_coefficient = 0.001;     // default: 0.01, use 0.001, otherwise fluid surface is asymmetric
    sph_params.use_density_based_projection = true;  // default: false
    sph_params.num_proximity_search_steps = 1;
    sysFSI.SetSPHParameters(sph_params);

    // Set linear solver parameters
    ChSystemFsi::LinSolverParameters linsolv_params;
    linsolv_params.type = SolverType::JACOBI;
    linsolv_params.atol = 0;
    linsolv_params.rtol = 0;
    linsolv_params.max_num_iters = 200;
    sysFSI.SetLinSolverParameters(linsolv_params);

    sysFSI.SetStepSize(step_size);

    // Set the terrain container size
    sysFSI.SetContainerDim(ChVector3<>(bxDim, byDim, bzDim));
    ChVector3<> cMin(-bxDim / 2 * 1.2, -byDim * 1.2, -bzDim / 2 * 1.2);
    ChVector3<> cMax(bxDim / 2 * 1.2, byDim * 1.2, bzDim / 2 * 1.2);
    sysFSI.SetBoundaries(cMin, cMax);

    bool use_polar_coords = true;

    std::vector<ChVector3d> points;
    sysFSI.CreateCylinderAnnulusPoints(inner_cylinder_radius + initial_spacing / 2,  //
                                       outer_cylinder_radius - initial_spacing / 2,  //
                                       fluid_height,                                 //
                                       use_polar_coords, initial_spacing,            //
                                       points);
    for (const auto& p : points) {
        double x = p.x();
        double y = p.z();
        double z = -p.y();
        double pressure = g * fluid_props.density * (fluid_height - y);
        sysFSI.AddSPHParticle({x, y, z}, fluid_props.density, pressure, fluid_props.viscosity);
    }

    // Add cylinder bottom plate
    ChVector2d bottom_plate_size(outer_cylinder_radius * 2.5, outer_cylinder_radius * 2.5);
    auto bottom_plate = chrono_types::make_shared<ChBody>();
    bottom_plate->SetPos(ChVector3d(0, -fluid_height / 2 - initial_spacing, 0));
    bottom_plate->SetFixed(true);
    sysMBS.AddBody(bottom_plate);

    sysFSI.AddWallBCE(bottom_plate, ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y), bottom_plate_size);

    ////sysFSI.AddWallBCE(bottom_plate, ChFrame<>(ChVector3d(0, -fluid_height, 0), Q_ROTATE_Z_TO_Y), ChVector2d(0.2,
    /// 0.2)); /sysFSI.AddBoxBCE(bottom_plate, ChFramed(ChVector3d(0, -2 * initial_spacing, 0), QNULL), /
    /// ChVector3d(outer_cylinder_radius * 2.5, 4 * initial_spacing, outer_cylinder_radius * 2.5), true); /auto
    /// bottom_plate_index = sysFSI.AddFsiBody(bottom_plate);

    // Cylinder center
    ChVector3d cylinder_center(0, cylinder_height / 2 - fluid_height / 2, 0);

    // Create inner cylinder, fixed
    auto inner_cylinder = chrono_types::make_shared<ChBody>();
    inner_cylinder->SetFixed(true);
    inner_cylinder->EnableCollision(false);
    sysMBS.AddBody(inner_cylinder);

    sysFSI.AddCylinderBCE(inner_cylinder,                               //
                          ChFrame<>(cylinder_center, Q_ROTATE_Z_TO_Y),  //
                          inner_cylinder_radius - initial_spacing / 2,  //
                          cylinder_height,                              //
                          true, false, true);
    auto inner_cylinder_index = sysFSI.AddFsiBody(inner_cylinder);

    // Create outer cylinder that spins
    double outer_cylinder_mass = 1.0;
    auto outer_cylinder = chrono_types::make_shared<ChBody>();
    sysMBS.AddBody(outer_cylinder);
    outer_cylinder->SetFixed(false);
    outer_cylinder->EnableCollision(false);
    outer_cylinder->SetMass(outer_cylinder_mass);
    outer_cylinder->SetInertia(ChMatrix33d(ChVector3d(1, 1, 1)));

    // Outer cylinder modeled as an annulus ring with a thickness of 3 * initial_spacing
    sysFSI.AddCylinderAnnulusBCE(outer_cylinder,                               //
                                 ChFrame<>(cylinder_center, Q_ROTATE_Z_TO_Y),  //
                                 outer_cylinder_radius + initial_spacing / 2,  //
                                 outer_cylinder_radius + 3 * initial_spacing,  //
                                 cylinder_height,                              //
                                 true);
    auto outer_cylinder_index = sysFSI.AddFsiBody(outer_cylinder);

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
    out_dir = out_dir + "/" + sysFSI.GetPhysicsProblemString() + "_" + sysFSI.GetSphMethodTypeString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
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
    if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
        cerr << "Error creating directory " << out_dir + "/snapshots" << endl;
        return 1;
    }

    // Create a run-tme visualizer
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

        auto v_max = omega * outer_cylinder_radius;
        auto col_callback = chrono_types::make_shared<VelocityColorCallback>(0, v_max);
        auto vis_callback = chrono_types::make_shared<PositionVisibilityCallback>();

        visFSI->SetTitle("Chrono::FSI BMC Simulation");
        visFSI->SetSize(1280, 720);
        visFSI->SetCameraVertical(CameraVerticalDir::Y);
        visFSI->AddCamera(ChVector3<>(1.1 * bxDim, 1.8 * byDim, 0), VNULL);
        visFSI->SetCameraMoveScale(100);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetBCEVisibilityCallback(vis_callback);
        visFSI->SetSPHVisibilityCallback(vis_callback);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Write the information into a txt file
    std::string out_file = out_dir + "/results.txt";
    std::ofstream ofile;
    ofile.open(out_file, std::ios::trunc);

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

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
            cout << "  time: " << time << endl;
            cout << "  cylinder angular velocity: " << angvel_outer << endl;
            cout << "  motor torque:              " << torque_motor << endl;
        }

        ofile << time << "\t" << angvel_outer << "\t";
        ofile << torque_motor << "\t" << torque_inner << "\t" << torque_outer << "\n";
        ////ofile << torque_motor << "\t" << torque_inner << "\t" << torque_outer << "\t" << torque_plate << "\n";

        if (output && time >= out_frame / output_fps) {
            cout << "-------- Output" << endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            out_frame++;
        }

        // Render SPH particles
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
        sysFSI.DoStepDynamics_FSI();
        time += step_size;
        sim_frame++;
    }
    timer.stop();
    cout << "\nSimulation time: " << timer() << " seconds\n" << endl;

    ofile.close();

#ifdef CHRONO_POSTPROCESS
    std::string title = "Couette flow - Torques (" + sysFSI.GetSphMethodTypeString() + ")";
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
