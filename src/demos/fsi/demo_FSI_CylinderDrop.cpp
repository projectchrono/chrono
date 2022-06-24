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
#include "chrono/assets/ChBoxShape.h"
#include "chrono/core/ChTransform.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::fsi;

// -----------------------------------------------------------------

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_CYLINDER_DROP/";

// Output frequency
bool output = true;
double out_fps = 20;

// Dimension of the space domain
double bxDim, byDim, bzDim;

// Size of the cylinder
double cyl_length = 0.2001;
double cyl_radius = 0.12;

// Final simulation time
double t_end = 2.0;

//------------------------------------------------------------------
// Function to add walls into Chrono system
//------------------------------------------------------------------
void AddWall(std::shared_ptr<ChBody> body,
             const ChVector<>& dim,
             std::shared_ptr<ChMaterialSurface> mat,
             const ChVector<>& loc) {
    body->GetCollisionModel()->AddBox(mat, dim.x(), dim.y(), dim.z(), loc);
    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = dim;
}

//------------------------------------------------------------------
// Function to save cylinder to Paraview VTK files
//------------------------------------------------------------------
void WriteCylinderVTK(std::shared_ptr<ChBody> Body, double radius, double length, int res, char SaveAsBuffer[256]) {
    std::ofstream output;
    output.open(SaveAsBuffer, std::ios::app);
    output << "# vtk DataFile Version 1.0\nUnstructured Grid Example\nASCII\n\n" << std::endl;
    output << "DATASET UNSTRUCTURED_GRID\nPOINTS " << 2 * res << " float\n";

    ChVector<> center = Body->GetPos();

    ChMatrix33<> Rotation = Body->GetRot();
    ChVector<double> vertex;
    for (int i = 0; i < res; i++) {
        ChVector<double> thisNode;
        thisNode.x() = radius * cos(2 * i * 3.1415 / res);
        thisNode.y() = -1 * length / 2;
        thisNode.z() = radius * sin(2 * i * 3.1415 / res);
        vertex = Rotation * thisNode + center;  // rotate/scale, if needed
        output << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
    }

    for (int i = 0; i < res; i++) {
        ChVector<double> thisNode;
        thisNode.x() = radius * cos(2 * i * 3.1415 / res);
        thisNode.y() = +1 * length / 2;
        thisNode.z() = radius * sin(2 * i * 3.1415 / res);
        vertex = Rotation * thisNode + center;  // rotate/scale, if needed
        output << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
    }

    output << "\n\nCELLS " << (unsigned int)res + res << "\t" << (unsigned int)5 * (res + res) << "\n";

    for (int i = 0; i < res - 1; i++) {
        output << "4 " << i << " " << i + 1 << " " << i + res + 1 << " " << i + res << "\n";
    }
    output << "4 " << res - 1 << " " << 0 << " " << res << " " << 2 * res - 1 << "\n";

    for (int i = 0; i < res / 4; i++) {
        output << "4 " << i << " " << i + 1 << " " << +res / 2 - i - 1 << " " << +res / 2 - i << "\n";
    }

    for (int i = 0; i < res / 4; i++) {
        output << "4 " << i + res << " " << i + 1 + res << " " << +res / 2 - i - 1 + res << " " << +res / 2 - i + res
               << "\n";
    }

    output << "4 " << +res / 2 << " " << 1 + res / 2 << " " << +res - 1 << " " << 0 << "\n";

    for (int i = 1; i < res / 4; i++) {
        output << "4 " << i + res / 2 << " " << i + 1 + res / 2 << " " << +res / 2 - i - 1 + res / 2 << " "
               << +res / 2 - i + res / 2 << "\n";
    }

    output << "4 " << 3 * res / 2 << " " << 1 + 3 * res / 2 << " " << +2 * res - 1 << " " << +res << "\n";

    for (int i = 1; i < res / 4; i++) {
        output << "4 " << i + 3 * res / 2 << " " << i + 1 + 3 * res / 2 << " " << +2 * res - i - 1 << " "
               << +2 * res - i << "\n";
    }

    output << "\nCELL_TYPES " << res + res << "\n";

    for (int iele = 0; iele < (res + res); iele++) {
        output << "9\n";
    }
}

//------------------------------------------------------------------
// Function to save the paraview files
//------------------------------------------------------------------
void SaveParaViewFiles(ChSystemFsi& sysFSI,
                       ChSystemSMC& sysMBS,
                       int this_frame,
                       double mTime,
                       std::shared_ptr<ChBody> Cylinder) {
    /// Simulation time between two output frames
    double frame_time = 1.0 / out_fps;

    /// Output data to files
    if (output && std::abs(mTime - (this_frame)*frame_time) < 1e-5) {
        /// save particles to cvs files
        sysFSI.PrintParticleToFile(out_dir + "/particles");

        /// save rigid bodies to vtk files
        char SaveAsRigidObjVTK[256];
        static int RigidCounter = 0;
        snprintf(SaveAsRigidObjVTK, sizeof(char) * 256, (out_dir + "vtk/Cylinder.%d.vtk").c_str(), RigidCounter);
        WriteCylinderVTK(Cylinder, cyl_radius, cyl_length, 100, SaveAsRigidObjVTK);
        RigidCounter++;
        std::cout << "\n--------------------------------\n" << std::endl;
        std::cout << "------------ Output Frame:   " << this_frame << std::endl;
        std::cout << "------------ Sim Time:       " << mTime << " (s)\n" << std::endl;
        std::cout << "--------------------------------\n" << std::endl;
    }
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS,
                      ChSystemFsi& sysFSI) {
    // Set gravity to the rigid body system in chrono
    sysMBS.Set_G_acc(sysFSI.Get_G_acc());

    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(1e8);
    mysurfmaterial->SetFriction(0.2f);
    mysurfmaterial->SetRestitution(0.05f);
    mysurfmaterial->SetAdhesion(0);

    // Get particle spacing in the simulation
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Bottom and top wall - size and position
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, 2 * bzDim + 1 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);

    // Left and right wall - size and position
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim + 0 * initSpace0);

    // Front and back wall - size and position
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim + 0 * initSpace0);

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector<>(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetIdentifier(-1);
    box->SetBodyFixed(true);
    box->GetCollisionModel()->ClearModel();
    box->GetCollisionModel()->SetSafeMargin(initSpace0 / 2);

    // Add the walls into chrono system
    AddWall(box, size_XY, mysurfmaterial, pos_zp);
    AddWall(box, size_XY, mysurfmaterial, pos_zn);
    AddWall(box, size_YZ, mysurfmaterial, pos_xp);
    AddWall(box, size_YZ, mysurfmaterial, pos_xn);
    AddWall(box, size_XZ, mysurfmaterial, pos_yp);
    AddWall(box, size_XZ, mysurfmaterial, pos_yn);
    box->GetCollisionModel()->BuildModel();
    box->SetCollide(true);
    sysMBS.AddBody(box);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBceBox(box, pos_zp, QUNIT, size_XY, 12);
    sysFSI.AddBceBox(box, pos_zn, QUNIT, size_XY, 12);
    sysFSI.AddBceBox(box, pos_xp, QUNIT, size_YZ, 23);
    sysFSI.AddBceBox(box, pos_xn, QUNIT, size_YZ, 23);
    sysFSI.AddBceBox(box, pos_yp, QUNIT, size_XZ, 13);
    sysFSI.AddBceBox(box, pos_yn, QUNIT, size_XZ, 13);

    // Create a falling cylinder
    auto cylinder = chrono_types::make_shared<ChBody>();

    // Set the general properties of the cylinder
    double volume = chrono::utils::CalcCylinderVolume(cyl_radius, cyl_length / 2);
    double density = sysFSI.GetDensity() * 2.0;
    double mass = density * volume;
    ChVector<> cyl_pos = ChVector<>(0, 0, bzDim + cyl_radius + 2 * initSpace0);
    ChVector<> cyl_vel = ChVector<>(0.0, 0.0, 0.0);
    ChQuaternion<> cyl_rot = QUNIT;
    ChVector<> gyration = chrono::utils::CalcCylinderGyration(cyl_radius, cyl_length / 2).diagonal();
    cylinder->SetPos(cyl_pos);
    cylinder->SetPos_dt(cyl_vel);
    cylinder->SetMass(mass);
    cylinder->SetInertiaXX(mass * gyration);

    // Set the collision type of the cylinder
    cylinder->SetCollide(true);
    cylinder->SetBodyFixed(false);
    cylinder->GetCollisionModel()->ClearModel();
    cylinder->GetCollisionModel()->SetSafeMargin(initSpace0);
    chrono::utils::AddCylinderGeometry(cylinder.get(), mysurfmaterial, cyl_radius, cyl_length, ChVector<>(0.0, 0.0, 0.0),
                               cyl_rot);
    cylinder->GetCollisionModel()->BuildModel();

    // Add this body to chrono system
    sysMBS.AddBody(cylinder);

    // Add this body to the FSI system (only those have inetraction with fluid)
    sysFSI.AddFsiBody(cylinder);

    // Add BCE particles attached on the cylinder into FSI system
    sysFSI.AddBceCylinder(cylinder, ChVector<>(0), ChQuaternion<>(1, 0, 0, 0), 
        cyl_radius, cyl_length + initSpace0, sysFSI.GetKernelLength(), false);
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
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(sysMBS);

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

    // Reset the domain size 
    ChVector<> bDim = sysFSI.GetContainerDim();
    bxDim = bDim.x();
    byDim = bDim.y();
    bzDim = bDim.z();

    // Set the periodic boundary condition (if not, set relative larger values)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin(-bxDim / 2 * 10, -byDim / 2 * 10, -bzDim * 10);
    ChVector<> cMax( bxDim / 2 * 10,  byDim / 2 * 10,  bzDim * 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Setup the output directory for FSI data
    sysFSI.SetOutputDirectory(out_dir);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);
    
    // Use a chrono sampler to create a bucket of granular material
    ChVector<> boxCenter(0, 0, bzDim / 2);
    ChVector<> boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    size_t numPart = (int)points.size();
    double gz = std::abs(sysFSI.Get_G_acc().z());
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
        double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSphMarker(points[i], rho_ini, pre_ini, sysFSI.GetViscosity(), sysFSI.GetKernelLength(), -1, ChVector<>(0));
    }
    sysFSI.AddRefArray(0, (int)numPart, -1, -1);

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Set up integrator for the multi-body dynamics system
    sysMBS.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sysMBS.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(1000);
    mystepper->SetAbsTolerances(1e-6);
    mystepper->SetMode(ChTimestepperHHT::ACCELERATION);
    mystepper->SetScaling(true);

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    double time = 0;
    int stepEnd = int(t_end / dT);
    double TIMING_sta = clock();
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / out_fps;
        int this_frame = (int)floor((time + 1e-6) / frame_time);

        /// Get the position of the container and cylinder
        auto box = sysMBS.Get_bodylist()[0];
        auto cyl = sysMBS.Get_bodylist()[1];

        printf("box=%f,%f,%f\n", box->GetPos().x(), box->GetPos().y(), box->GetPos().z());
        printf("cyl=%f,%f,%f\n", cyl->GetPos().x(), cyl->GetPos().y(), cyl->GetPos().z());

        /// Get the cylinder from the FSI system and Save data of the simulation
        std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = sysFSI.GetFsiBodies();
        auto Cylinder = FSI_Bodies[0];
        SaveParaViewFiles(sysFSI, sysMBS, this_frame, time, Cylinder);

        /// Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
    }

    // Total computational cost
    double TIMING_end = (clock() - TIMING_sta) / (double)CLOCKS_PER_SEC;
    printf("\nSimulation Finished in %f (s)\n", TIMING_end);

    return 0;
}
