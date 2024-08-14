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
// Author: Milad Rakhsha, Wei Hu
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

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

// -----------------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Cylinder_Drop";

// Output frequency
bool output = true;
double output_fps = 20;

// Dimension of the space domain
double bxDim = 1.0;
double byDim = 0.6;
double bzDim = 1.2;

// Size of the cylinder
double cyl_length = 0.2001;
double cyl_radius = 0.12;

// Final simulation time
double t_end = 2.0;

// Enable/disable run-time visualization
bool render = true;
float render_fps = 1000;

// Visibility flags
bool show_rigid = true;
bool show_rigid_bce = true;
bool show_boundary_bce = false;
bool show_particles_sph = true;

// -----------------------------------------------------------------------------

class PositionVisibilityCallback : public ChParticleCloud::VisibilityCallback {
  public:
    PositionVisibilityCallback() {}

    virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override {
        auto p = cloud.GetParticlePos(n);
        return p.x() < 0 || p.y() < 0;
    };
};

// -----------------------------------------------------------------------------

std::shared_ptr<ChBody> CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI);
void WriteCylinderVTK(const std::string& filename, double radius, double length, const ChFrame<>& frame, int res);

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_CylinderDrop_Explicit.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_CylinderDrop <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    // Set the periodic boundary condition (if not, set relative larger values)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim);
    ChVector3d cMax( bxDim / 2 * 1.2,  byDim / 2 * 1.2,  bzDim);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(0, 0, bzDim / 2);
    ChVector3d boxHalfDim = ChVector3d(bxDim / 2 - initSpace0, byDim / 2 - initSpace0, bzDim / 2 - initSpace0);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles to the FSI system
    double gz = std::abs(sysFSI.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double pre_ini = sysFSI.GetDensity() * gz * (-p.z() + bzDim);
        double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSPHParticle(p, rho_ini, pre_ini, sysFSI.GetViscosity(), ChVector3d(0));
    }

    // Create MBD and BCE particles for the solid domain
    auto cylinder = CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Set up integrator for the multi-body dynamics system
    sysMBS.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sysMBS.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxIters(1000);
    mystepper->SetAbsTolerances(1e-6);

    // Create oputput directories
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

        const auto& origin = ChVector3d(0, 0, 0.6 * bzDim);

        auto col_callback = chrono_types::make_shared<VelocityColorCallback>(0, 1.0);
        auto vis_callback = chrono_types::make_shared<PositionVisibilityCallback>();

        visFSI->SetTitle("Chrono::FSI cylinder drop");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(origin + ChVector3d(2 * bxDim, 2 * byDim, 0), origin);
        visFSI->SetCameraMoveScale(0.1f);
        visFSI->EnableFluidMarkers(show_particles_sph);
        visFSI->EnableBoundaryMarkers(show_boundary_bce);
        visFSI->EnableRigidBodyMarkers(show_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(vis_callback);
        visFSI->SetBCEVisibilityCallback(vis_callback);
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Record cylinder height
    ChFunctionInterp height_recorder;

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            static int counter = 0;
            std::string filename = out_dir + "/vtk/cylinder." + std::to_string(counter++) + ".vtk";
            WriteCylinderVTK(filename, cyl_radius, cyl_length, cylinder->GetFrameRefToAbs(), 100);
            out_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;
            render_frame++;
        }

        auto cylinder_height = sysMBS.GetBodies()[1]->GetPos().z();
        std::cout << "step: " << sim_frame << "\ttime: " << time << "\tRTF: " << sysFSI.GetRTF()
                  << "\tcyl z: " << cylinder_height << std::endl;
        height_recorder.AddPoint(time, cylinder_height);

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot(out_dir + "/height.gpl");
    gplot.SetGrid();
    std::string speed_title = "Cylinder height";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("height (m)");
    gplot.Plot(height_recorder, "", " with lines lt -1 lw 2 lc rgb'#3333BB' ");
#endif

    return 0;
}

// -----------------------------------------------------------------------------
// Create the solid objects in the MBD system and their counterparts in the FSI system

std::shared_ptr<ChBody> CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Set gravity to the rigid body system in chrono
    sysMBS.SetGravitationalAcceleration(sysFSI.GetGravitationalAcceleration());

    // Set common material Properties
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.2f);
    cmaterial->SetRestitution(0.05f);
    cmaterial->SetAdhesion(0);

    // Get particle spacing in the simulation
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(2, 2, -1),                           //
                                   false);
    box->EnableCollision(false);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxContainerBCE(box,                                            //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 2, -1));

    // Create a falling cylinder
    auto cylinder = chrono_types::make_shared<ChBody>();

    // Set the general properties of the cylinder
    double volume = ChCylinder::GetVolume(cyl_radius, cyl_length);
    double density = sysFSI.GetDensity() * 0.5;
    double mass = density * volume;
    ChVector3d cyl_pos = ChVector3d(0, 0, bzDim + cyl_radius + 2 * initSpace0);
    ChVector3d cyl_vel = ChVector3d(0.0, 0.0, 0.0);
    ChVector3d gyration = ChCylinder::GetGyration(cyl_radius, cyl_length / 2).diagonal();
    cylinder->SetPos(cyl_pos);
    cylinder->SetPosDt(cyl_vel);
    cylinder->SetMass(mass);
    cylinder->SetInertiaXX(mass * gyration);

    // Set visualization geometry
    cylinder->SetFixed(false);
    chrono::utils::AddCylinderGeometry(cylinder.get(), cmaterial, cyl_radius, cyl_length, VNULL,
                                       QuatFromAngleX(CH_PI_2), show_rigid);
    cylinder->GetCollisionModel()->SetSafeMargin(initSpace0);

    if (show_rigid)
        cylinder->GetVisualShape(0)->SetColor(ChColor(0.65f, 0.20f, 0.10f));

    // Add this body to chrono system
    sysMBS.AddBody(cylinder);

    // Add this body to the FSI system (only those have interaction with fluid)
    sysFSI.AddFsiBody(cylinder);

    // Add BCE particles attached on the cylinder into FSI system
    sysFSI.AddCylinderBCE(cylinder, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)), cyl_radius, cyl_length, true);

    return cylinder;
}

// -----------------------------------------------------------------------------
// Function to save cylinder to Paraview VTK files

void WriteCylinderVTK(const std::string& filename, double radius, double length, const ChFrame<>& frame, int res) {
    std::ofstream outf;
    outf.open(filename, std::ios::app);
    outf << "# vtk DataFile Version 1.0\nUnstructured Grid Example\nASCII\n\n" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID\nPOINTS " << 2 * res << " float\n";

    for (int i = 0; i < res; i++) {
        auto w = frame.TransformPointLocalToParent(
            ChVector3d(radius * cos(2 * i * 3.1415 / res), -1 * length / 2, radius * sin(2 * i * 3.1415 / res)));
        outf << w.x() << " " << w.y() << " " << w.z() << "\n";
    }

    for (int i = 0; i < res; i++) {
        auto w = frame.TransformPointLocalToParent(
            ChVector3d(radius * cos(2 * i * 3.1415 / res), +1 * length / 2, radius * sin(2 * i * 3.1415 / res)));
        outf << w.x() << " " << w.y() << " " << w.z() << "\n";
    }

    outf << "\n\nCELLS " << res + res << "\t" << 5 * (res + res) << "\n";

    for (int i = 0; i < res - 1; i++) {
        outf << "4 " << i << " " << i + 1 << " " << i + res + 1 << " " << i + res << "\n";
    }
    outf << "4 " << res - 1 << " " << 0 << " " << res << " " << 2 * res - 1 << "\n";

    for (int i = 0; i < res / 4; i++) {
        outf << "4 " << i << " " << i + 1 << " " << +res / 2 - i - 1 << " " << +res / 2 - i << "\n";
    }

    for (int i = 0; i < res / 4; i++) {
        outf << "4 " << i + res << " " << i + 1 + res << " " << +res / 2 - i - 1 + res << " " << +res / 2 - i + res
             << "\n";
    }

    outf << "4 " << +res / 2 << " " << 1 + res / 2 << " " << +res - 1 << " " << 0 << "\n";

    for (int i = 1; i < res / 4; i++) {
        outf << "4 " << i + res / 2 << " " << i + 1 + res / 2 << " " << +res / 2 - i - 1 + res / 2 << " "
             << +res / 2 - i + res / 2 << "\n";
    }

    outf << "4 " << 3 * res / 2 << " " << 1 + 3 * res / 2 << " " << +2 * res - 1 << " " << +res << "\n";

    for (int i = 1; i < res / 4; i++) {
        outf << "4 " << i + 3 * res / 2 << " " << i + 1 + 3 * res / 2 << " " << +2 * res - i - 1 << " " << +2 * res - i
             << "\n";
    }

    outf << "\nCELL_TYPES " << res + res << "\n";

    for (int iele = 0; iele < (res + res); iele++) {
        outf << "9\n";
    }
}
