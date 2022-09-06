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

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::fsi;

//------------------------------------------------------------------

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Dam_Break/";

// Output frequency
bool output = true;
double out_fps = 20;

// Dimension of the space domain
double bxDim = 6.0;
double byDim = 1.0;
double bzDim = 4.0;

// Dimension of the fluid domain
double fxDim = 2.0;
double fyDim = 1.0;
double fzDim = 2.0;

// Final simulation time
double t_end = 10.0;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // General setting of ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);
    ground->GetCollisionModel()->ClearModel();

    // Create the geometry of the boundaries
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Bottom and top wall - size and position
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 0 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 1 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);

    // Left and right wall - size and position
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 0 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);

    // Front and back wall - size and position
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 0 * initSpace0);

    // Add the walls into chrono system
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XY, pos_zp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XY, pos_zn, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xn, QUNIT, true);
    // You may uncomment the following lines to have side walls as well.
    // To show the use of Periodic boundary condition, these walls are not added
    // To this end, cMin and cMax were set up appropriately
    ////chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yp, QUNIT, true);
    ////chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yn, QUNIT, true);
    ground->GetCollisionModel()->BuildModel();
    sysMBS.AddBody(ground);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxBCE(ground, pos_zn, QUNIT, size_XY, 12);
    sysFSI.AddBoxBCE(ground, pos_zp, QUNIT, size_XY, 12);
    sysFSI.AddBoxBCE(ground, pos_xp, QUNIT, size_YZ, 23);
    sysFSI.AddBoxBCE(ground, pos_xn, QUNIT, size_YZ, 23);
    // If you uncommented the above lines that add the side walls, you should uncomment the following two lines as
    // well. This is necessary in order to populate the walls with BCE markers for the fluid simulation
    ////sysFSI.AddBoxBce(ground, pos_yp, QUNIT, size_XZ, 13);
    ////sysFSI.AddBoxBce(ground, pos_yn, QUNIT, size_XZ, 13);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(sysMBS);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_DamBreak_Explicit.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_DamBreak <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    // Set up the periodic boundary condition (only in Y direction)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin = ChVector<>(-bxDim / 2 - 10.0 * initSpace0, -byDim / 2 - 1.0 * initSpace0 / 2.0, -2.0 * bzDim);
    ChVector<> cMax = ChVector<>(bxDim / 2 + 10.0 * initSpace0, byDim / 2 + 1.0 * initSpace0 / 2.0, 2.0 * bzDim);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create Fluid region and discretize with SPH particles
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);

    // Use a chrono sampler to create a bucket of points
    chrono::utils::GridSampler<> sampler(initSpace0);
    chrono::utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    double gz = std::abs(sysFSI.Get_G_acc().z());
    for (int i = 0; i < numPart; i++) {
        // Calculate the pressure of a steady state (p = rho*g*h)
        auto pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + fzDim);
        auto rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSPHParticle(points[i], rho_ini, pre_ini, sysFSI.GetViscosity(), sysFSI.GetKernelLength());
    }

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Create a run-tme visualizer
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Chrono::FSI dam break");
        fsi_vis.SetCameraPosition(ChVector<>(0, -3 * byDim, bzDim), ChVector<>(0, 0, 0));
        fsi_vis.SetCameraMoveScale(1.0f);
        fsi_vis.EnableBoundaryMarkers(false);
        fsi_vis.Initialize();
    }

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));

    double time = 0;
    int current_step = 0;

    ChTimer<> timer;
    timer.start();
    while (time < t_end) {
        std::cout << "step: " << current_step << "  time: " << time << std::endl;

        // Save data of the simulation
        if (output && current_step % output_steps == 0) {
            std::cout << "------- OUTPUT" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
        }

        // Render SPH particles
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();

        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
