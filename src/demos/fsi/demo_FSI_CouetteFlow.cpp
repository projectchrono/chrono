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
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/physics/ChLinkMate.h"

#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono/core/ChTimer.h"

#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/functions/ChFunctionRotationAxis.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_fsi/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
// using namespace chrono::geometry;
// using namespace chrono::particlefactory;

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::OpenGL;
// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Couette_Flow";


// Enable/disable run-time visualization
bool render = true;
float render_fps = 100;

// Physical properties of the liquid
double density_steel = 0.0078;  // g/mm3
double density = 0.001;         // g/mm3

double bxDim = 14.0;  // mm
double byDim = 10.0;  // mm
double bzDim = 14.0;   // mm

// auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 50;

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

class PositionVisibilityCallback : public ChParticleCloud::VisibilityCallback {
  public:
    PositionVisibilityCallback() {}

    virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override {
        auto p = cloud.GetParticlePos(n);
        return p.x() < 0;
    };
};



int main(int argc, char* argv[]) {

    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChSystemFsi sysFSI(&sysMBS);
    sysFSI.SetVerbose(verbose);

    double step_size = 5e-4;
    double time_end = 10;
    double initial_spacing = 0.15;
    double kernel_h = initial_spacing;      // mm
    double density = 0.001;         // g/mm3
    double viscosity = 1e-3;                // for I2SPH

    // cylinder radius
    double outer_cylinder_radius = 5.0;  // mm
    double inner_cylinder_radius = 3.8;  // mm
    double fluid_height = 2.0;
    double cylinder_height = 2 * fluid_height;  // container needs to be higher than fluid, otherwise water spills from the top


    bool verbose = true;

    // Set gravitational acceleration
    const ChVector3d gravity(0, -9810, 0);
    sysFSI.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    // Set CFD fluid properties
    ChSystemFsi::FluidProperties fluid_props;
    fluid_props.density = density;
    fluid_props.viscosity = viscosity;  // for I2SPH

    sysFSI.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    ChSystemFsi::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::I2SPH;   
    sph_params.num_bce_layers = 3;
    sph_params.kernel_h = initial_spacing;
    sph_params.initial_spacing = initial_spacing;
    sph_params.max_velocity = 1;  // default is 1
    sph_params.xsph_coefficient = 0.1;   // default is 0.5
    sph_params.shifting_coefficient = 0.0;  // default is 0.1
    // TODO : add API for modifying Epsilon number, default is 0.01, I use 0.001, otherwise fluid surface is asymmetric, variable name, epsMinMarkersDis

    sysFSI.SetSPHParameters(sph_params);
    sysFSI.SetStepSize(step_size);

    // Set the terrain container size
    sysFSI.SetContainerDim(ChVector3<>(bxDim, byDim, bzDim));
    ChVector3<> cMin(-bxDim / 2 * 1.2, -byDim * 1.2, -bzDim / 2 * 1.2);
    ChVector3<> cMax( bxDim / 2 * 1.2,  byDim * 1.2,  bzDim / 2 * 1.2);
    sysFSI.SetBoundaries(cMin, cMax);

    chrono::utils::ChGridSampler<> sampler(initial_spacing);

    // Use sampler to create SPH particles that fill the annulus area between the inner and outer cylinders
    ChVector3<> boxCenter(0, 0, 0);
    ChVector3<> boxHalfDim(outer_cylinder_radius, fluid_height/2., outer_cylinder_radius);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    double gz = std::abs(sysFSI.GetGravitationalAcceleration().y());
    for (int i = 0; i < points.size(); i++) {
        double x = points[i].x();
        double z = points[i].z();
        double r = std::sqrt(std::pow(x, 2) + std::pow(z, 2));
        double p = gz * fluid_props.density * (fluid_height - points[i].y());  // hydrostatic pressure
        // Only add particles in the annulus area
        if (r > inner_cylinder_radius + initial_spacing && r < outer_cylinder_radius - initial_spacing) {
        sysFSI.AddSPHParticle(points[i], fluid_props.density, p, fluid_props.viscosity);
        }
    }


    // Add cylinder bottom 
    ChVector2d bottom_plate_size(outer_cylinder_radius * 2.5, outer_cylinder_radius * 2.5);  
    auto bottom_plate = chrono_types::make_shared<ChBody>();
    bottom_plate->SetPos(ChVector3d(0, -fluid_height / 2. - kernel_h, 0));
    bottom_plate->SetFixed(true);
    sysFSI.AddWallBCE(bottom_plate,
                      ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y),
                      bottom_plate_size);
    sysMBS.AddBody(bottom_plate);

    // Create inner cylinder, fixed
    auto inner_cylinder = chrono_types::make_shared<ChBody>();
    inner_cylinder->SetFixed(true);
    inner_cylinder->EnableCollision(false);
    sysFSI.AddCylinderBCE(inner_cylinder, ChFrame<>(ChVector3d(0, fluid_height/2., 0), Q_ROTATE_Z_TO_Y),
                          inner_cylinder_radius - initial_spacing / 2., fluid_height * 2., true, false, true);
    sysFSI.AddFsiBody(inner_cylinder);
    sysMBS.AddBody(inner_cylinder);

    // Create outer cylinder that spins
    double outer_cylinder_mass = 1.0;
    // Set the abs orientation, position and velocity
    auto outer_cylinder = chrono_types::make_shared<ChBody>();
    sysMBS.AddBody(outer_cylinder);
    outer_cylinder->SetFixed(false);
    outer_cylinder->EnableCollision(false);
    // assign mass & inertia
    outer_cylinder->SetMass(outer_cylinder_mass);
    outer_cylinder->SetInertia(ChMatrix33d(ChVector3d(1,1,1)));
    //Outer cylinder modeled as an annulus ring with a thickness of 3 * initial_spacing
    sysFSI.AddCylinderAnnulusBCE(outer_cylinder, ChFrame<>(ChVector3d(0, fluid_height/2., 0), Q_ROTATE_Z_TO_Y),
        outer_cylinder_radius + initial_spacing/2., outer_cylinder_radius + 3 * initial_spacing, fluid_height * 2., true);
    sysFSI.AddFsiBody(outer_cylinder);


    // Add motor between outer cylinder and plate
    motor->Initialize(outer_cylinder, bottom_plate, ChFrame<>(VNULL, Q_ROTATE_Z_TO_Y));  
    

    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(62. / 60.));
    sysMBS.AddLink(motor);

    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysFSI.GetPhysicsProblemString() + "_" + sysFSI.GetSphMethodTypeString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
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

        auto origin = sysMBS.GetBodies()[1]->GetPos();
        auto vis_callback = chrono_types::make_shared<PositionVisibilityCallback>();

        visFSI->SetTitle("Chrono::FSI BMC Simulation");
        visFSI->SetSize(1280, 720);
        visFSI->SetCameraVertical(CameraVerticalDir::Y);
        visFSI->AddCamera(origin + ChVector3<>(2 * bxDim, 2 * byDim, 2 * bzDim * 0), origin);
        visFSI->SetCameraMoveScale(100);  // mm
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetBCEVisibilityCallback(vis_callback);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<VelocityColorCallback>(0, 10));  // mm
        visFSI->SetSPHVisibilityCallback(vis_callback);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    ChVector3<> torque_m;
    ChVector3<> angvel;

    // Write the information into a txt file
    std::ofstream myFile;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
    }

    // Start the simulation//
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * step_size));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * step_size));

    double time = 0.0;
    int current_step = 0;

    ChTimer timer;
    timer.start();

    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    while (time < time_end) {
        // Get the infomation of the spinning cylinder
        angvel = outer_cylinder->GetAngVelParent();
        torque_m = motor->GetMotorTorque();

        if (verbose) {
            std::cout << "  time: " << time << std::endl;
            std::cout << "  cylinder angular velocity: " << angvel.y() << std::endl;
            std::cout << "  motor torque:              " << torque_m.y() << std::endl;
        }

        if (output) {
            myFile << time << "\t" << angvel.y() << "\t" << torque_m.y() << "\n";        }

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
        }

        // Render SPH particles
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += step_size;
        current_step++;
    }

    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    if (output) {
        myFile.close();
    }

    return 0;
}