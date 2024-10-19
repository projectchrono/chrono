// ==================================================================================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci, Maurizio Zama, Iseo Serrature S.p.a, projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ==================================================================================================================================================
// Authors: Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci
// ==================================================================================================================================================

// =========================
// ======== HEADERS ========
// =========================
    // ======== Multi-purposes headers ========
#if defined(_DEBUG)
#undef _DEBUG
#include <Python.h>
#define _DEBUG
#else
// #include <Python.h>
#endif
// #include <pybind11/pybind11.h>
// #include <pybind11/embed.h>
// #include <pybind11/numpy.h>
// #include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <filesystem>
#include <exception>
#define _USE_MATH_DEFINES 
#include <cmath>
// #include "externals/json.hpp"
#include <future>
#include <sstream>
// #include <chrono> // Header, which provides a high-resolution clock
// #include <nlohmann/json.hpp>
#include <thread>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChRandom.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"
#include <chrono/physics/ChSystem.h>

    // ======== ChElectronics headers ========
// #include "chrono_powerelectronics/ChElectronics.h"
// #include "chrono_powerelectronics/ChElectronicsManipulator.h"
// #include "chrono_powerelectronics/ChElectronicsSources.h"
#include "chrono_powerelectronics/ChElectronicsCosimulation.h"
#include "chrono_powerelectronics/circuits/ChElectronicMotor.h"

// ============================
// ======== NAMESPACES ========
// ============================
using namespace ::chrono;
using namespace ::chrono::irrlicht;
using namespace ::chrono::powerelectronics;
// namespace py = pybind11;
// namespace fs = std::filesystem;
// using json = nlohmann::json;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// =====================================
// ======== FUNCTIONS & CLASSES ========
// =====================================

// ======== Method: allows to put in pause the execution for sec ========
void _sleep(int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds)); 
}

// ======== Method: allows to read a .json file ========
// bool readJsonFile(const std::string& filename, json& j) {
//     std::ifstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Error opening JSON file." << std::endl;
//         return false;
//     }
//     file >> j;
//     return true;
// }

// ======== Method: allows to write into a .json file ========
// auto writeJsonFile(const std::string& filename, json data)
// {
//     // Specify the file name
//     std::string fileName = "Sim_Res/" + filename + ".json";
//     // Open a file stream for writing
//     std::ofstream file(fileName);
//     // Check if the file stream is open
//     if (!file.is_open()) {
//         std::cerr << "Error opening file for writing!" << std::endl;
//         return 1;
//     }
//     // Write the JSON object to the file
//     file << data.dump(4); // Pretty print with indentation of 4 spaces
//     // Close the file stream
//     file.close();
//     std::cout << "\n!!!!! Results exported to: " << fileName << " !!!!!\n";
// }

// ======== Class: allows to compute the derivative in-between a single simulation time-step through the numerical differentiation method ========
class NumericalDifferentiation {
public:
    double Differentiate(double& dt, double& f_new)
    {
        f_new1 = f_new;
        dt1 = dt;
        if (flag == 0)
        {
            Derivative_res = 0.0;
            f_old1 = f_new1;
            flag = 1;
        }
        else
        {
            Derivative_res = (f_new1 - f_old1) / dt1;
            f_old1 = f_new1;
        }
        return Derivative_res;
    }
private:
    double Derivative_res = 0.0;
    double f_old1 = 0.0;
    double f_new1;
    double dt1;
    int flag = 0;
};

// ======== Class: allows to compute the integral in-between a single simulation time-step through the cumulative trapezoidal method ========
class CumTrapezIntegration {
public:
    double Integrate(double& dt, double& f_new)
    {
        f_new1 = f_new;
        dt1 = dt;
        Integral_res += dt1 * ((f_old1 + f_new1) / 2);
        //std::cout << "\n!!!!! f_new1: " << f_new1 << " !!!!!\n";            // DEBUG: Scope some needed results
        //std::cout << "\n!!!!! f_old1: " << f_old1 << " !!!!!\n";            // DEBUG: Scope some needed results
        f_old1 = f_new1;
        return Integral_res;
    }
private:
    double Integral_res = 0.0;
    double f_old1 = 0.0;
    double f_new1;
    double dt1;
};

// ======== Method: calculate the effective Euler angular position of a body from the angular velocity along x-y-z- axis ========
std::vector<double> GetEulerAngPos(std::shared_ptr<::chrono::ChBody> body, double& t_step_mechanic)
{
    // Get the effective angular velocity along x-y-z axis
    ChVector3d body_Euler_Vel = body->GetAngVelLocal(); // Get the angular velocity 
    double Rotor_Euler_dt_Yaw = body_Euler_Vel[0];
    double Rotor_Euler_dt_Pitch = body_Euler_Vel[1];
    double Rotor_Euler_dt_Roll = body_Euler_Vel[2];

    // Create the object only once through a static variable (the static variable allows to initialize it only once during the execution of the entire code)
    static CumTrapezIntegration body_Euler_Yaw_Integrator;
    static CumTrapezIntegration body_Euler_Pitch_Integrator;
    static CumTrapezIntegration body_Euler_Roll_Integrator;

    // Compute the effective angular position along x-y-z axis
    double body_Euler_Yaw = body_Euler_Yaw_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[0]);
    double body_Euler_Pitch = body_Euler_Pitch_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[1]);
    double body_Euler_Roll = body_Euler_Roll_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[2]);

    // Populate the result vector
    std::vector<double> Results = { body_Euler_Yaw , body_Euler_Pitch, body_Euler_Roll };

    return Results;
}
/*
    // ======== Class: Replica of the Simulink block: 1-D Lookup Table ========
class LookupTable1D {
public:
    LookupTable1D(const std::vector<double>& breakpoints, const std::vector<double>& table_data)
        : breakpoints(breakpoints), table_data(table_data) {}

    double interpolate(double x) {
        // Find the index of the left breakpoint
        size_t left_index = 0;
        for (size_t i = 1; i < breakpoints.size(); ++i) {
            if (breakpoints[i] <= x) {
                left_index = i;
            }
            else {
                break;
            }
        }
        // If x is beyond the last breakpoint, use the last data point
        if (left_index == breakpoints.size() - 1) {
            return table_data.back();
        }
        // Linear interpolation
        double x_left = breakpoints[left_index];
        double x_right = breakpoints[left_index + 1];
        double y_left = table_data[left_index];
        double y_right = table_data[left_index + 1];
        double y_interp = y_left + (y_right - y_left) * (x - x_left) / (x_right - x_left);
        return y_interp;
    }
private:
    std::vector<double> breakpoints;
    std::vector<double> table_data;
};
*/
// ===========================
// ======== MAIN LOOP ========
// ===========================
int main(int argc, char* argv[]) {
    // ========================================
    // ======== PRELIMINARY OPERATIONS ========
    // ========================================

        // ============================================
        // ======== SET THE PYTHON INTERPRETER ========
        // ============================================
        // ======== Python global variables ========  
    // py::scoped_interpreter guard{};         // Initialize Python interpreter  

    // =%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=
    // =%=%=%=% MECHANICAL DOMAIN =%=%=%=%
    // =%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=

    // ==============================================
    // ======== MECHANICAL DOMAIN PARAMETERS ========
    // ==============================================

        // ============================================
        // ======== CREATE A NSC CHRONO SYSTEM ========
        // ============================================
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys; // Create a Chrono physical system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // NSC = Non Smooth Contact that is suitable for impact phenomena
    // SMC = SMooth Contacy that i suitable for continuous phenomena
    // Mechanical Unit system: [kg]-[mm]-[s] -> The use of [mm] is require cause extrimelly small collisiion parameters crash the simulation
    // Electric Unit system: [kg]-[m]-[s]

        // ==============================================
        // ======== SET THE GRAVITY ACCELERATION ========
        // ==============================================
    ChVector3d gravity_acc = sys.GetGravitationalAcceleration(); 
    std::cout << "The gravity acceleration  vector is: " << gravity_acc << "\n\n";
    double gravity = 9.81e3; //[mm/s^2]
    sys.SetGravitationalAcceleration(ChVector3d(gravity, 0, 0));
    ChVector3d gravity_acc_new = sys.GetGravitationalAcceleration(); 
    std::cout << "The new gravity acceleration  vector is: " << gravity_acc_new << "\n\n";
  

        // =======================================
        // ======== RIGID BODIES CREATION ========
        // =======================================

        // ==============================================
        // ======== RIGID BODY DEFINITION: Floor ========
        // ==============================================
    auto floor_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    floor_mat->SetFriction(1.0); 
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(2, 300, 300, 1000, true, true, floor_mat);
    floor->SetPos(ChVector3d(100.0, 0.0, 0.0));
    floor->SetFixed(true);
    floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(floor);

    // =======================================================================
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> FrameGlobal ========
    // =======================================================================
    // ======== File name ========
    std::string FrameGlobal_file_name = "Test_Model/FrameGlobal_OBJ.obj";
    // ======== Meshes ========
    // ======== Visualization Mesh ========
    auto FrameGlobal_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(FrameGlobal_file_name));
    auto FrameGlobal_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    FrameGlobal_mesh->SetMesh(FrameGlobal_trimesh);
    FrameGlobal_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto FrameGlobal_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(FrameGlobal_file_name));
    auto FrameGlobal_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    FrameGlobal_coll_mesh->SetMesh(FrameGlobal_coll_trimesh);
    FrameGlobal_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_FrameGlobal = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(FrameGlobal_file_name));
    // ======== Compute mass inertia from mesh ========
    double FrameGlobal_volume; // [mm^3]  
    ChVector3d FrameGlobal_cog; // [mm]   
    ChMatrix33<> FrameGlobal_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: FrameGlobal_mesh->ComputeMassProperties) 
    FrameGlobal_trimesh->ComputeMassProperties(true, FrameGlobal_volume, FrameGlobal_cog, FrameGlobal_geometric_inertia); // It returns: FrameGlobal_volume:[mm^3], FrameGlobal_cog:[mm], FrameGlobal_inertia:[mm^5] that is the geometric inertia tensor 
    double FrameGlobal_density = 7500.00 / (1e9); // [kg/mm^3]
    double FrameGlobal_mass = FrameGlobal_density * FrameGlobal_volume; // [kg]
    ChMatrix33<> FrameGlobal_inertia = FrameGlobal_density * FrameGlobal_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! FrameGlobal -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The FrameGlobal mass is: " << FrameGlobal_mass << " [kg]" << "\n\n";
    std::cout << "The FrameGlobal cog is: " << FrameGlobal_cog << " [mm]" << "\n\n";
    std::cout << "The FrameGlobal inertia tensor is:\n" << FrameGlobal_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto FrameGlobal_body = chrono_types::make_shared<ChBody>();
    sys.Add(FrameGlobal_body);
    FrameGlobal_body->SetFixed(true);
    FrameGlobal_body->SetMass(FrameGlobal_mass);
    FrameGlobal_body->SetInertiaXX(ChVector3d(FrameGlobal_inertia(0, 0), FrameGlobal_inertia(1, 1), FrameGlobal_inertia(2, 2)));
    FrameGlobal_body->SetPos(FrameGlobal_cog);
    // ======== Visulaization ========
    //FrameGlobal_mesh->SetMutable(false);
    //FrameGlobal_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //FrameGlobal_mesh->SetOpacity(0.5f);
    //FrameGlobal_mesh->SetBackfaceCull(true);
    FrameGlobal_body->AddVisualShape(FrameGlobal_mesh, ChFrame<>(-FrameGlobal_cog, ChMatrix33<>(1)));
    FrameGlobal_body->AddVisualShape(FrameGlobal_coll_mesh, ChFrame<>(-FrameGlobal_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto FrameGlobal_coll_model = chrono_types::make_shared<ChCollisionModel>();
    FrameGlobal_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    FrameGlobal_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_FrameGlobal->Transform(-FrameGlobal_cog, ChMatrix33<>(1));
    auto FrameGlobal_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    FrameGlobal_mat->SetFriction(0.30);
    FrameGlobal_mat->SetRestitution(0.001); //In the range[0, 1].
    auto FrameGlobal_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(FrameGlobal_mat, trimesh_FrameGlobal, false, false, 0.001);
    FrameGlobal_coll_model->AddShape(FrameGlobal_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    FrameGlobal_body->AddCollisionModel(FrameGlobal_coll_model);
    FrameGlobal_body->EnableCollision(false);

    // =================================================================  
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Crank ========
    // =================================================================
    // ======== File name ========
    std::string Crank_file_name = "Test_Model/Crank_OBJ.obj";
    // ======== Meshes ========
        // ======== Visualization Mesh ========
    auto Crank_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Crank_file_name));
    auto Crank_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Crank_mesh->SetMesh(Crank_trimesh);
    Crank_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto Crank_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Crank_file_name));
    auto Crank_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Crank_coll_mesh->SetMesh(Crank_coll_trimesh);
    Crank_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_Crank = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Crank_file_name));
    // ======== Compute mass inertia from mesh ========
    double Crank_volume; // [mm^3]  
    ChVector3d Crank_cog; // [mm]   
    ChMatrix33<> Crank_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Crank_mesh->ComputeMassProperties) 
    Crank_trimesh->ComputeMassProperties(true, Crank_volume, Crank_cog, Crank_geometric_inertia); // It returns: Crank_volume:[mm^3], Crank_cog:[mm], Crank_inertia:[mm^5] that is the geometric inertia tensor 
    double Crank_density = 2700.00 / (1e9); // [kg/mm^3]
    double Crank_mass = Crank_density * Crank_volume; // [kg]
    ChMatrix33<> Crank_inertia = Crank_density * Crank_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Crank -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Crank mass is: " << Crank_mass << " [kg]" << "\n\n";
    std::cout << "The Crank cog is: " << Crank_cog << " [mm]" << "\n\n";
    std::cout << "The Crank inertia tensor is:\n" << Crank_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto Crank_body = chrono_types::make_shared<ChBody>();
    sys.Add(Crank_body);
    Crank_body->SetFixed(false);
    Crank_body->SetMass(Crank_mass);
    Crank_body->SetInertiaXX(ChVector3d(Crank_inertia(0, 0), Crank_inertia(1, 1), Crank_inertia(2, 2)));
    Crank_body->SetPos(Crank_cog);
    // ======== Visulaization ========
    //Crank_mesh->SetMutable(false);
    //Crank_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //Crank_mesh->SetOpacity(0.5f);
    //Crank_mesh->SetBackfaceCull(true);
    Crank_body->AddVisualShape(Crank_mesh, ChFrame<>(-Crank_cog, ChMatrix33<>(1)));
    Crank_body->AddVisualShape(Crank_coll_mesh, ChFrame<>(-Crank_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto Crank_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Crank_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Crank_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Crank->Transform(-Crank_cog, ChMatrix33<>(1));
    auto Crank_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Crank_mat->SetFriction(0.30);
    Crank_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Crank_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Crank_mat, trimesh_Crank, false, false, 0.001);
    Crank_coll_model->AddShape(Crank_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Crank_body->AddCollisionModel(Crank_coll_model);
    Crank_body->EnableCollision(false);

    // =================================================================  
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Piston =======
    // =================================================================
    // ======== File name ========
    std::string Piston_file_name = "Test_Model/Piston_OBJ.obj";
    // ======== Meshes ========
        // ======== Visualization Mesh ========
    auto Piston_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Piston_file_name));
    auto Piston_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Piston_mesh->SetMesh(Piston_trimesh);
    Piston_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto Piston_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Piston_file_name));
    auto Piston_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Piston_coll_mesh->SetMesh(Piston_coll_trimesh);
    Piston_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_Piston = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Piston_file_name));
    // ======== Compute mass inertia from mesh ========
    double Piston_volume; // [mm^3]  
    ChVector3d Piston_cog; // [mm]   
    ChMatrix33<> Piston_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Piston_mesh->ComputeMassProperties) 
    Piston_trimesh->ComputeMassProperties(true, Piston_volume, Piston_cog, Piston_geometric_inertia); // It returns: Piston_volume:[mm^3], Piston_cog:[mm], Piston_inertia:[mm^5] that is the geometric inertia tensor 
    double Piston_density = 2700.00 / (1e9); // [kg/mm^3]
    double Piston_mass = Piston_density * Piston_volume; // [kg]
    ChMatrix33<> Piston_inertia = Piston_density * Piston_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Piston -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Piston mass is: " << Piston_mass << " [kg]" << "\n\n";
    std::cout << "The Piston cog is: " << Piston_cog << " [mm]" << "\n\n";
    std::cout << "The Piston inertia tensor is:\n" << Piston_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto Piston_body = chrono_types::make_shared<ChBody>();
    sys.Add(Piston_body);
    Piston_body->SetFixed(false);
    Piston_body->SetMass(Piston_mass);
    Piston_body->SetInertiaXX(ChVector3d(Piston_inertia(0, 0), Piston_inertia(1, 1), Piston_inertia(2, 2)));
    Piston_body->SetPos(Piston_cog);
    // ======== Visulaization ========
    //Piston_mesh->SetMutable(false);
    //Piston_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //Piston_mesh->SetOpacity(0.5f);
    //Piston_mesh->SetBackfaceCull(true);
    Piston_body->AddVisualShape(Piston_mesh, ChFrame<>(-Piston_cog, ChMatrix33<>(1)));
    Piston_body->AddVisualShape(Piston_coll_mesh, ChFrame<>(-Piston_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto Piston_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Piston_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Piston_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Piston->Transform(-Piston_cog, ChMatrix33<>(1));
    auto Piston_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Piston_mat->SetFriction(0.30);
    Piston_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Piston_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Piston_mat, trimesh_Piston, false, false, 0.001);
    Piston_coll_model->AddShape(Piston_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Piston_body->AddCollisionModel(Piston_coll_model);
    Piston_body->EnableCollision(false);

    // =================================================================  
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Rod ==========
    // =================================================================
    // ======== File name ========
    std::string Rod_file_name = "Test_Model/Rod_OBJ.obj";
    // ======== Meshes ========
        // ======== Visualization Mesh ========
    auto Rod_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rod_file_name));
    auto Rod_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Rod_mesh->SetMesh(Rod_trimesh);
    Rod_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto Rod_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rod_file_name));
    auto Rod_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Rod_coll_mesh->SetMesh(Rod_coll_trimesh);
    Rod_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_Rod = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rod_file_name));
    // ======== Compute mass inertia from mesh ========
    double Rod_volume; // [mm^3]  
    ChVector3d Rod_cog; // [mm]   
    ChMatrix33<> Rod_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Rod_mesh->ComputeMassProperties) 
    Rod_trimesh->ComputeMassProperties(true, Rod_volume, Rod_cog, Rod_geometric_inertia); // It returns: Rod_volume:[mm^3], Rod_cog:[mm], Rod_inertia:[mm^5] that is the geometric inertia tensor 
    double Rod_density = 7500.00 / (1e9); // [kg/mm^3]
    double Rod_mass = Rod_density * Rod_volume; // [kg]
    ChMatrix33<> Rod_inertia = Rod_density * Rod_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Rod -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Rod mass is: " << Rod_mass << " [kg]" << "\n\n";
    std::cout << "The Rod cog is: " << Rod_cog << " [mm]" << "\n\n";
    std::cout << "The Rod inertia tensor is:\n" << Rod_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto Rod_body = chrono_types::make_shared<ChBody>();
    sys.Add(Rod_body);
    Rod_body->SetFixed(false);
    Rod_body->SetMass(Rod_mass);
    Rod_body->SetInertiaXX(ChVector3d(Rod_inertia(0, 0), Rod_inertia(1, 1), Rod_inertia(2, 2)));
    Rod_body->SetPos(Rod_cog);
    // ======== Visulaization ========
    //Rod_mesh->SetMutable(false);
    //Rod_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //Rod_mesh->SetOpacity(0.5f);
    //Rod_mesh->SetBackfaceCull(true);
    Rod_body->AddVisualShape(Rod_mesh, ChFrame<>(-Rod_cog, ChMatrix33<>(1)));
    Rod_body->AddVisualShape(Rod_coll_mesh, ChFrame<>(-Rod_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto Rod_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Rod_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Rod_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Rod->Transform(-Rod_cog, ChMatrix33<>(1));
    auto Rod_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Rod_mat->SetFriction(0.30);
    Rod_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Rod_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Rod_mat, trimesh_Rod, false, false, 0.001);
    Rod_coll_model->AddShape(Rod_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Rod_body->AddCollisionModel(Rod_coll_model);
    Rod_body->EnableCollision(false);

    // =================================================================  
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Rotor ========
    // =================================================================
    // ======== File name ========
    std::string Rotor_file_name = "Test_Model/Rotor_OBJ.obj";
    // ======== Meshes ========
        // ======== Visualization Mesh ========
    auto Rotor_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rotor_file_name));
    auto Rotor_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Rotor_mesh->SetMesh(Rotor_trimesh);
    Rotor_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto Rotor_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rotor_file_name));
    auto Rotor_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Rotor_coll_mesh->SetMesh(Rotor_coll_trimesh);
    Rotor_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_Rotor = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Rotor_file_name));
    // ======== Compute mass inertia from mesh ========
    double Rotor_volume; // [mm^3]  
    ChVector3d Rotor_cog; // [mm]   
    ChMatrix33<> Rotor_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Rotor_mesh->ComputeMassProperties) 
    Rotor_trimesh->ComputeMassProperties(true, Rotor_volume, Rotor_cog, Rotor_geometric_inertia); // It returns: Rotor_volume:[mm^3], Rotor_cog:[mm], Rotor_inertia:[mm^5] that is the geometric inertia tensor 
    double Rotor_density = 5820.00 / (1e9); // [kg/mm^3]
    double Rotor_mass = Rotor_density * Rotor_volume; // [kg]
    ChMatrix33<> Rotor_inertia = Rotor_density * Rotor_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Rotor -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Rotor mass is: " << Rotor_mass << " [kg]" << "\n\n";
    std::cout << "The Rotor cog is: " << Rotor_cog << " [mm]" << "\n\n";
    std::cout << "The Rotor inertia tensor is:\n" << Rotor_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto Rotor_body = chrono_types::make_shared<ChBody>();
    sys.Add(Rotor_body);
    Rotor_body->SetFixed(false);
    Rotor_body->SetMass(Rotor_mass);
    Rotor_body->SetInertiaXX(ChVector3d(Rotor_inertia(0, 0), Rotor_inertia(1, 1), Rotor_inertia(2, 2)));
    Rotor_body->SetPos(Rotor_cog);
    // ======== Visulaization ========
    //Rotor_mesh->SetMutable(false);
    //Rotor_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //Rotor_mesh->SetOpacity(0.5f);
    //Rotor_mesh->SetBackfaceCull(true);
    Rotor_body->AddVisualShape(Rotor_mesh, ChFrame<>(-Rotor_cog, ChMatrix33<>(1)));
    Rotor_body->AddVisualShape(Rotor_coll_mesh, ChFrame<>(-Rotor_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto Rotor_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Rotor_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Rotor_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Rotor->Transform(-Rotor_cog, ChMatrix33<>(1));
    auto Rotor_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Rotor_mat->SetFriction(0.30);
    Rotor_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Rotor_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Rotor_mat, trimesh_Rotor, false, false, 0.001);
    Rotor_coll_model->AddShape(Rotor_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Rotor_body->AddCollisionModel(Rotor_coll_model);
    Rotor_body->EnableCollision(false);

    // =================================================================  
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Screw1 =======
    // =================================================================
    // ======== File name ========
    std::string Screw1_file_name = "Test_Model/Screw1_OBJ.obj";
    // ======== Meshes ========
        // ======== Visualization Mesh ========
    auto Screw1_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Screw1_file_name));
    auto Screw1_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Screw1_mesh->SetMesh(Screw1_trimesh);
    Screw1_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto Screw1_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Screw1_file_name));
    auto Screw1_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Screw1_coll_mesh->SetMesh(Screw1_coll_trimesh);
    Screw1_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_Screw1 = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Screw1_file_name));
    // ======== Compute mass inertia from mesh ========
    double Screw1_volume; // [mm^3]  
    ChVector3d Screw1_cog; // [mm]   
    ChMatrix33<> Screw1_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Screw1_mesh->ComputeMassProperties) 
    Screw1_trimesh->ComputeMassProperties(true, Screw1_volume, Screw1_cog, Screw1_geometric_inertia); // It returns: Screw1_volume:[mm^3], Screw1_cog:[mm], Screw1_inertia:[mm^5] that is the geometric inertia tensor 
    double Screw1_density = 7500.00 / (1e9); // [kg/mm^3]
    double Screw1_mass = Screw1_density * Screw1_volume; // [kg]
    ChMatrix33<> Screw1_inertia = Screw1_density * Screw1_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Screw1 -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Screw1 mass is: " << Screw1_mass << " [kg]" << "\n\n";
    std::cout << "The Screw1 cog is: " << Screw1_cog << " [mm]" << "\n\n";
    std::cout << "The Screw1 inertia tensor is:\n" << Screw1_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto Screw1_body = chrono_types::make_shared<ChBody>();
    sys.Add(Screw1_body);
    Screw1_body->SetFixed(false);
    Screw1_body->SetMass(Screw1_mass);
    Screw1_body->SetInertiaXX(ChVector3d(Screw1_inertia(0, 0), Screw1_inertia(1, 1), Screw1_inertia(2, 2)));
    Screw1_body->SetPos(Screw1_cog);
    // ======== Visulaization ========
    //Screw1_mesh->SetMutable(false);
    //Screw1_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //Screw1_mesh->SetOpacity(0.5f);
    //Screw1_mesh->SetBackfaceCull(true);
    Screw1_body->AddVisualShape(Screw1_mesh, ChFrame<>(-Screw1_cog, ChMatrix33<>(1)));
    Screw1_body->AddVisualShape(Screw1_coll_mesh, ChFrame<>(-Screw1_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto Screw1_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Screw1_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Screw1_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Screw1->Transform(-Screw1_cog, ChMatrix33<>(1));
    auto Screw1_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Screw1_mat->SetFriction(0.30);
    Screw1_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Screw1_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Screw1_mat, trimesh_Screw1, false, false, 0.001);
    Screw1_coll_model->AddShape(Screw1_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Screw1_body->AddCollisionModel(Screw1_coll_model);
    Screw1_body->EnableCollision(false);

    // =================================================================  
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Screw2 =======
    // =================================================================
    // ======== File name ========
    std::string Screw2_file_name = "Test_Model/Screw2_OBJ.obj";
    // ======== Meshes ========
        // ======== Visualization Mesh ========
    auto Screw2_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Screw2_file_name));
    auto Screw2_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Screw2_mesh->SetMesh(Screw2_trimesh);
    Screw2_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto Screw2_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Screw2_file_name));
    auto Screw2_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Screw2_coll_mesh->SetMesh(Screw2_coll_trimesh);
    Screw2_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_Screw2 = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Screw2_file_name));
    // ======== Compute mass inertia from mesh ========
    double Screw2_volume; // [mm^3]  
    ChVector3d Screw2_cog; // [mm]   
    ChMatrix33<> Screw2_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Screw2_mesh->ComputeMassProperties) 
    Screw2_trimesh->ComputeMassProperties(true, Screw2_volume, Screw2_cog, Screw2_geometric_inertia); // It returns: Screw2_volume:[mm^3], Screw2_cog:[mm], Screw2_inertia:[mm^5] that is the geometric inertia tensor 
    double Screw2_density = 7500.00 / (1e9); // [kg/mm^3]
    double Screw2_mass = Screw2_density * Screw2_volume; // [kg]
    ChMatrix33<> Screw2_inertia = Screw2_density * Screw2_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Screw2 -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Screw2 mass is: " << Screw2_mass << " [kg]" << "\n\n";
    std::cout << "The Screw2 cog is: " << Screw2_cog << " [mm]" << "\n\n";
    std::cout << "The Screw2 inertia tensor is:\n" << Screw2_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto Screw2_body = chrono_types::make_shared<ChBody>();
    sys.Add(Screw2_body);
    Screw2_body->SetFixed(false);
    Screw2_body->SetMass(Screw2_mass);
    Screw2_body->SetInertiaXX(ChVector3d(Screw2_inertia(0, 0), Screw2_inertia(1, 1), Screw2_inertia(2, 2)));
    Screw2_body->SetPos(Screw2_cog);
    // ======== Visulaization ========
    //Screw2_mesh->SetMutable(false);
    //Screw2_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //Screw2_mesh->SetOpacity(0.5f);
    //Screw2_mesh->SetBackfaceCull(true);
    Screw2_body->AddVisualShape(Screw2_mesh, ChFrame<>(-Screw2_cog, ChMatrix33<>(1)));
    Screw2_body->AddVisualShape(Screw2_coll_mesh, ChFrame<>(-Screw2_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto Screw2_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Screw2_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Screw2_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Screw2->Transform(-Screw2_cog, ChMatrix33<>(1));
    auto Screw2_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Screw2_mat->SetFriction(0.30);
    Screw2_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Screw2_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Screw2_mat, trimesh_Screw2, false, false, 0.001);
    Screw2_coll_model->AddShape(Screw2_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Screw2_body->AddCollisionModel(Screw2_coll_model);
    Screw2_body->EnableCollision(false);

    // =================================================================  
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> ScrewNut =====
    // =================================================================
    // ======== File name ========
    std::string ScrewNut_file_name = "Test_Model/ScrewNut_OBJ.obj";
    // ======== Meshes ========
        // ======== Visualization Mesh ========
    auto ScrewNut_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(ScrewNut_file_name));
    auto ScrewNut_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    ScrewNut_mesh->SetMesh(ScrewNut_trimesh);
    ScrewNut_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto ScrewNut_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(ScrewNut_file_name));
    auto ScrewNut_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    ScrewNut_coll_mesh->SetMesh(ScrewNut_coll_trimesh);
    ScrewNut_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_ScrewNut = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(ScrewNut_file_name));
    // ======== Compute mass inertia from mesh ========
    double ScrewNut_volume; // [mm^3]  
    ChVector3d ScrewNut_cog; // [mm]   
    ChMatrix33<> ScrewNut_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: ScrewNut_mesh->ComputeMassProperties) 
    ScrewNut_trimesh->ComputeMassProperties(true, ScrewNut_volume, ScrewNut_cog, ScrewNut_geometric_inertia); // It returns: ScrewNut_volume:[mm^3], ScrewNut_cog:[mm], ScrewNut_inertia:[mm^5] that is the geometric inertia tensor 
    double ScrewNut_density = 7500.00 / (1e9); // [kg/mm^3]
    double ScrewNut_mass = ScrewNut_density * ScrewNut_volume; // [kg]
    ChMatrix33<> ScrewNut_inertia = ScrewNut_density * ScrewNut_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! ScrewNut -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The ScrewNut mass is: " << ScrewNut_mass << " [kg]" << "\n\n";
    std::cout << "The ScrewNut cog is: " << ScrewNut_cog << " [mm]" << "\n\n";
    std::cout << "The ScrewNut inertia tensor is:\n" << ScrewNut_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto ScrewNut_body = chrono_types::make_shared<ChBody>();
    sys.Add(ScrewNut_body);
    ScrewNut_body->SetFixed(false);
    ScrewNut_body->SetMass(ScrewNut_mass);
    ScrewNut_body->SetInertiaXX(ChVector3d(ScrewNut_inertia(0, 0), ScrewNut_inertia(1, 1), ScrewNut_inertia(2, 2)));
    ScrewNut_body->SetPos(ScrewNut_cog);
    // ======== Visulaization ========
    //ScrewNut_mesh->SetMutable(false);
    //ScrewNut_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //ScrewNut_mesh->SetOpacity(0.5f);
    //ScrewNut_mesh->SetBackfaceCull(true);
    ScrewNut_body->AddVisualShape(ScrewNut_mesh, ChFrame<>(-ScrewNut_cog, ChMatrix33<>(1)));
    ScrewNut_body->AddVisualShape(ScrewNut_coll_mesh, ChFrame<>(-ScrewNut_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto ScrewNut_coll_model = chrono_types::make_shared<ChCollisionModel>();
    ScrewNut_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    ScrewNut_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_ScrewNut->Transform(-ScrewNut_cog, ChMatrix33<>(1));
    auto ScrewNut_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    ScrewNut_mat->SetFriction(0.30);
    ScrewNut_mat->SetRestitution(0.001); //In the range[0, 1].
    auto ScrewNut_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(ScrewNut_mat, trimesh_ScrewNut, false, false, 0.001);
    ScrewNut_coll_model->AddShape(ScrewNut_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    ScrewNut_body->AddCollisionModel(ScrewNut_coll_model);
    ScrewNut_body->EnableCollision(false);

    // =================================================================  
    // ======== RIGID BODY DEFINITION: WaveFront Shape -> Stator =======
    // =================================================================
    // ======== File name ========
    std::string Stator_file_name = "Test_Model/Stator_OBJ.obj";
    // ======== Meshes ========
        // ======== Visualization Mesh ========
    auto Stator_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_file_name));
    auto Stator_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Stator_mesh->SetMesh(Stator_trimesh);
    Stator_mesh->SetVisible(true);
    // ======== Visualization Collision Mesh ========
    auto Stator_coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_file_name));
    auto Stator_coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    Stator_coll_mesh->SetMesh(Stator_coll_trimesh);
    Stator_coll_mesh->SetVisible(false);
    // ======== Triangle Mesh for collision the model ========
    auto trimesh_Stator = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(Stator_file_name));
    // ======== Compute mass inertia from mesh ========
    double Stator_volume; // [mm^3]  
    ChVector3d Stator_cog; // [mm]   
    ChMatrix33<> Stator_geometric_inertia; // [mm^5] it is the geometric inertia tensor (see when you call: Stator_mesh->ComputeMassProperties) 
    Stator_trimesh->ComputeMassProperties(true, Stator_volume, Stator_cog, Stator_geometric_inertia); // It returns: Stator_volume:[mm^3], Stator_cog:[mm], Stator_inertia:[mm^5] that is the geometric inertia tensor 
    double Stator_density = 7920.00 / (1e9); // [kg/mm^3]
    double Stator_mass = Stator_density * Stator_volume; // [kg]
    ChMatrix33<> Stator_inertia = Stator_density * Stator_geometric_inertia; // [kg*mm^2]
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "!!!!!!! Stator -> Inertia properies: !!!!!!!" << "\n"; // Display the Inertia properties of the body
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
    std::cout << "The Stator mass is: " << Stator_mass << " [kg]" << "\n\n";
    std::cout << "The Stator cog is: " << Stator_cog << " [mm]" << "\n\n";
    std::cout << "The Stator inertia tensor is:\n" << Stator_inertia << " [kg*mm^2]" << "\n\n";
    // ======== Define the rigid body ========
    auto Stator_body = chrono_types::make_shared<ChBody>();
    sys.Add(Stator_body);
    Stator_body->SetFixed(false);
    Stator_body->SetMass(Stator_mass);
    Stator_body->SetInertiaXX(ChVector3d(Stator_inertia(0, 0), Stator_inertia(1, 1), Stator_inertia(2, 2)));
    Stator_body->SetPos(Stator_cog);
    // ======== Visulaization ========
    //Stator_mesh->SetMutable(false);
    //Stator_mesh->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    //Stator_mesh->SetOpacity(0.5f);
    //Stator_mesh->SetBackfaceCull(true);
    Stator_body->AddVisualShape(Stator_mesh, ChFrame<>(-Stator_cog, ChMatrix33<>(1)));
    Stator_body->AddVisualShape(Stator_coll_mesh, ChFrame<>(-Stator_cog, ChMatrix33<>(1)));
    // ======== Collision ========
    auto Stator_coll_model = chrono_types::make_shared<ChCollisionModel>();
    Stator_coll_model->SetSafeMargin(0.1f);  // inward safe margin
    Stator_coll_model->SetEnvelope(0.001f);    // distance of the outward "collision envelope"
    trimesh_Stator->Transform(-Stator_cog, ChMatrix33<>(1));
    auto Stator_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    Stator_mat->SetFriction(0.30);
    Stator_mat->SetRestitution(0.001); //In the range[0, 1].
    auto Stator_coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(Stator_mat, trimesh_Stator, false, false, 0.001);
    Stator_coll_model->AddShape(Stator_coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    Stator_body->AddCollisionModel(Stator_coll_model);
    Stator_body->EnableCollision(false);


    // ==========================================
    // ======== KINEMATIC LINKS CREATION ========
    // ==========================================
    // ======================================================================
    // ======== LINK DEFINITION -> FIXED JOINT: Stator - FrameGlobal ========
    // ======================================================================
    ChVector3d Stator_FrameGlobal_Link_Position(Stator_body->GetPos());   // [mm] set the position in the 3D space of the link respect to the absolute frame
    //Stator_FrameGlobal_Link_Position[2] = Stator_FrameGlobal_Link_Position[2] + 7.0;  
    ChQuaternion<> Stator_FrameGlobal_Link_Orientation; 
    Stator_FrameGlobal_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> Stator_FrameGlobal_Link_Frame(Stator_FrameGlobal_Link_Position, Stator_FrameGlobal_Link_Orientation);
    auto Stator_FrameGlobal_Link_Fixed = chrono_types::make_shared<ChLinkLockLock>();
    Stator_FrameGlobal_Link_Fixed->Initialize(Stator_body,                      // Body 1  
        FrameGlobal_body,                     // Body 2  
        Stator_FrameGlobal_Link_Frame);        // Location and orientation of the frame   
    sys.AddLink(Stator_FrameGlobal_Link_Fixed);

    // ===================================================================
    // ======== LINK DEFINITION -> REVOLUTE JOINT: Rotor - Stator ========
    // ===================================================================
    ChVector3d Rotor_Stator_Link_Position(Rotor_body->GetPos());            // [mm] set the position in the 3D space of the link respect to the absolute frame
    //Rotor_Stator_Link_Position[2] = Rotor_Stator_Link_Position[2] + 7.0;
    ChQuaternion<> Rotor_Stator_Link_Orientation;
    Rotor_Stator_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> Rotor_Stator_Link_Frame(Rotor_Stator_Link_Position, Rotor_Stator_Link_Orientation);
    auto Rotor_Stator_Link_Revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    Rotor_Stator_Link_Revolute->Initialize(Rotor_body,                      // Body 1  
        Stator_body,                     // Body 2  
        Rotor_Stator_Link_Frame);        // Location and orientation of the frame  
    sys.AddLink(Rotor_Stator_Link_Revolute);

    // ======================================================================
    // ======== LINK DEFINITION -> FIXED JOINT: Rotor - Crank ========
    // ======================================================================
    ChVector3d Rotor_Crank_Link_Position(Rotor_body->GetPos());   // [mm] set the position in the 3D space of the link respect to the absolute frame
    //Rotor_Crank_Link_Position[2] = Rotor_Crank_Link_Position[2] + 7.0;  
    ChQuaternion<> Rotor_Crank_Link_Orientation;
    Rotor_Crank_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> Rotor_Crank_Link_Frame(Rotor_Crank_Link_Position, Rotor_Crank_Link_Orientation);
    auto Rotor_Crank_Link_Fixed = chrono_types::make_shared<ChLinkLockLock>();
    Rotor_Crank_Link_Fixed->Initialize(Rotor_body,                      // Body 1  
        Crank_body,                     // Body 2  
        Rotor_Crank_Link_Frame);        // Location and orientation of the frame   
    sys.AddLink(Rotor_Crank_Link_Fixed);

    // ======================================================================
    // ======== LINK DEFINITION -> FIXED JOINT: Screw1 - Crank ========
    // ======================================================================
    ChVector3d Screw1_Crank_Link_Position(Screw1_body->GetPos());   // [mm] set the position in the 3D space of the link respect to the absolute frame
    //Screw1_Crank_Link_Position[2] = Screw1_Crank_Link_Position[2] + 7.0;  
    ChQuaternion<> Screw1_Crank_Link_Orientation;
    Screw1_Crank_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> Screw1_Crank_Link_Frame(Screw1_Crank_Link_Position, Screw1_Crank_Link_Orientation);
    auto Screw1_Crank_Link_Fixed = chrono_types::make_shared<ChLinkLockLock>();
    Screw1_Crank_Link_Fixed->Initialize(Screw1_body,                      // Body 1  
        Crank_body,                     // Body 2  
        Screw1_Crank_Link_Frame);        // Location and orientation of the frame   
    sys.AddLink(Screw1_Crank_Link_Fixed);

    // ======================================================================
    // ======== LINK DEFINITION -> FIXED JOINT: Screw2 - Crank ========
    // ======================================================================
    ChVector3d Screw2_Crank_Link_Position(Screw2_body->GetPos());   // [mm] set the position in the 3D space of the link respect to the absolute frame
    //Screw2_Crank_Link_Position[2] = Screw2_Crank_Link_Position[2] + 7.0;  
    ChQuaternion<> Screw2_Crank_Link_Orientation;
    Screw2_Crank_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> Screw2_Crank_Link_Frame(Screw2_Crank_Link_Position, Screw2_Crank_Link_Orientation);
    auto Screw2_Crank_Link_Fixed = chrono_types::make_shared<ChLinkLockLock>();
    Screw2_Crank_Link_Fixed->Initialize(Screw2_body,                      // Body 1  
        Crank_body,                     // Body 2  
        Screw2_Crank_Link_Frame);        // Location and orientation of the frame   
    sys.AddLink(Screw2_Crank_Link_Fixed);

    // ===================================================================
    // ======== LINK DEFINITION -> REVOLUTE JOINT: Screw2 - Rod ========
    // ===================================================================
    ChVector3d Screw2_Rod_Link_Position(Screw2_body->GetPos());            // [mm] set the position in the 3D space of the link respect to the absolute frame
    //Screw2_Rod_Link_Position[2] = Screw2_Rod_Link_Position[2] + 7.0;
    ChQuaternion<> Screw2_Rod_Link_Orientation;
    Screw2_Rod_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> Screw2_Rod_Link_Frame(Screw2_Rod_Link_Position, Screw2_Rod_Link_Orientation);
    auto Screw2_Rod_Link_Revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    Screw2_Rod_Link_Revolute->Initialize(Screw2_body,                      // Body 1  
        Rod_body,                     // Body 2  
        Screw2_Rod_Link_Frame);        // Location and orientation of the frame  
    sys.AddLink(Screw2_Rod_Link_Revolute);

    // ===================================================================
    // ======== LINK DEFINITION -> REVOLUTE JOINT: ScrewNut - Rod ========
    // ===================================================================
    ChVector3d ScrewNut_Rod_Link_Position(ScrewNut_body->GetPos());            // [mm] set the position in the 3D space of the link respect to the absolute frame
    //ScrewNut_Rod_Link_Position[2] = ScrewNut_Rod_Link_Position[2] + 7.0;
    ChQuaternion<> ScrewNut_Rod_Link_Orientation;
    ScrewNut_Rod_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> ScrewNut_Rod_Link_Frame(ScrewNut_Rod_Link_Position, ScrewNut_Rod_Link_Orientation);
    auto ScrewNut_Rod_Link_Revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    ScrewNut_Rod_Link_Revolute->Initialize(ScrewNut_body,                      // Body 1  
        Rod_body,                     // Body 2  
        ScrewNut_Rod_Link_Frame);        // Location and orientation of the frame  
    sys.AddLink(ScrewNut_Rod_Link_Revolute);

    // ===================================================================
    // ======== LINK DEFINITION -> REVOLUTE JOINT: ScrewNut - Piston ========
    // ===================================================================
    ChVector3d ScrewNut_Piston_Link_Position(ScrewNut_body->GetPos());            // [mm] set the position in the 3D space of the link respect to the absolute frame
    //ScrewNut_Piston_Link_Position[2] = ScrewNut_Piston_Link_Position[2] + 7.0;
    ChQuaternion<> ScrewNut_Piston_Link_Orientation;
    ScrewNut_Piston_Link_Orientation.SetFromAngleAxis(0.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));       // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> ScrewNut_Piston_Link_Frame(ScrewNut_Piston_Link_Position, ScrewNut_Piston_Link_Orientation);
    auto ScrewNut_Piston_Link_Revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    ScrewNut_Piston_Link_Revolute->Initialize(ScrewNut_body,                      // Body 1  
        Piston_body,                     // Body 2  
        ScrewNut_Piston_Link_Frame);        // Location and orientation of the frame  
    sys.AddLink(ScrewNut_Piston_Link_Revolute);

    // ======================================================================
    // ======== LINK DEFINITION -> PRISMATIC JOINT: FrameGlobal - Piston ========
    // ======================================================================
    ChVector3d FrameGlobal_Piston_Link_Position(Piston_body->GetPos());                                                        // [mm] set the position in the 3D space of the link respect to the absolute frame
    ChQuaternion<> FrameGlobal_Piston_Link_Orientation;
    FrameGlobal_Piston_Link_Orientation.SetFromAngleAxis(90.0 * (M_PI / 180.0), ChVector3d(0, 1, 0));        // !!! IMPORTANT !!! the Revolute is always arround Z-axis -> Set correctly the orientation 
    ChFrame<> FrameGlobal_Piston_Link_Frame(FrameGlobal_Piston_Link_Position, FrameGlobal_Piston_Link_Orientation);
    auto FrameGlobal_Piston_Link_Prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    FrameGlobal_Piston_Link_Prismatic->Initialize(FrameGlobal_body,                      // Body 1   
        Piston_body,                     // Body 2   
        FrameGlobal_Piston_Link_Frame);        // Location and orientation of the frame   
    sys.AddLink(FrameGlobal_Piston_Link_Prismatic);

    // =============================================================================
    // ======== F / T DEFINITION -> TORSIONAL SPRING/DAMPER: Rotor - Stator ========
    // =============================================================================
    // ======== Torsional spring coefficient ========
    double k_eq_Rotor_Stator_spr = 0.0; // [(N * m) / rad]
    k_eq_Rotor_Stator_spr = k_eq_Rotor_Stator_spr * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s]) 

    // ======== Torsional damping coefficient ========
    double r_ShaftBushing_experimental = 0.407e-4; //[(N*m*s)/rad]
    double r_eq_Rotor_Stator_spr = r_ShaftBushing_experimental * 1e3 * 1e3; // Conversion to ([kg]-[mm]-[s])  

    // ======== Torsional spring/damper implementation ========
    auto Rotor_Stator_Torsional_Spring = chrono_types::make_shared<ChLinkRSDA>();
    ChVector3d Rotor_Stator_Torsional_Spring_Position(Stator_body->GetPos());  //[mm] set the position in the 3D space of the link respect to the absolute frame
    Rotor_Stator_Torsional_Spring_Position[2] += 6.0;  //[mm] Rise the position of the spring along y-axis in order to see it better in the animation
    ChQuaternion<> Rotor_Stator_Torsional_Spring_Orientation;
    Rotor_Stator_Torsional_Spring_Orientation.SetFromAngleAxis(0.0 * M_PI / 180.0, ChVector3d(1, 0, 0)); // !!! IMPORTANT !!! the Torsional Spring is oriented always arround Z-axis -> Set correctly the orientation 
    ChFrame<> Rotor_Stator_Torsional_Spring_Frame(Rotor_Stator_Torsional_Spring_Position, Rotor_Stator_Torsional_Spring_Orientation);
    Rotor_Stator_Torsional_Spring->Initialize(Rotor_body,                                   // Body 1 
        Stator_body,                                  // Body 2 
        false,                                        // the two following frames are in absolute, not relative, coords.
        Rotor_Stator_Torsional_Spring_Frame,          // Location and orientation of the Body 1 frame 
        Rotor_Stator_Torsional_Spring_Frame);         // Location and orientation of the Body 1 frame
    Rotor_Stator_Torsional_Spring->SetRestAngle(0.0 * (M_PI / 180.0)); //[rad] Starting angular position
    Rotor_Stator_Torsional_Spring->SetSpringCoefficient(k_eq_Rotor_Stator_spr); // [(kg mm mm)/(s^2 rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m/rad]
    Rotor_Stator_Torsional_Spring->SetDampingCoefficient(r_eq_Rotor_Stator_spr); // [(kg mm mm s)/(s^2 mm rad)] that should be the SI conversion ([kg]-[mm]-[s]) of [N m s/rad]
    sys.AddLink(Rotor_Stator_Torsional_Spring);
    Rotor_Stator_Torsional_Spring->AddVisualShape(chrono_types::make_shared<ChVisualShapeRotSpring>(10, 15)); // var1 = radius of the spring, var2 = graphical resolution of the spring


    auto Piston_Frame_Spring = chrono_types::make_shared<ChLinkTSDA>();
    ChVector3d Piston_Frame_Spring_Position(Piston_body->GetPos());  //[mm] set the position in the 3D space of the link respect to the absolute frame
    ChVector3d Piston_Frame_Spring_Position_1 = Piston_Frame_Spring_Position;
    Piston_Frame_Spring_Position_1[0] -= 6.0;  //[mm] Rise the position of the spring along y-axis in order to see it better in the animation
    Piston_Frame_Spring->Initialize(Piston_body, FrameGlobal_body, true, Piston_Frame_Spring_Position, Piston_Frame_Spring_Position_1);
    Piston_Frame_Spring->SetRestLength(0.0);
    Piston_Frame_Spring->SetSpringCoefficient(0.0);
    Piston_Frame_Spring->SetDampingCoefficient(0.0005);
    sys.AddLink(Piston_Frame_Spring);

    // =%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%
    // =%=%=%=% MULTI-PHYSICS SIMULATION =%=%=%=%
    // =%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%

    // ===============================================
    // ======== IRRLICHT VISUALIZATION SYSTEM ========
    // ===============================================
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1200, 800);
    vis->SetWindowTitle("Modeling a simplified trackjed vehicle");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 100, 300));
    vis->AddLight(ChVector3d(30.f, 100.f, -30.f), 300, ChColor(0.7f, 0.7f, 0.7f));
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);

    // =================================
    // ======== SOLVER SETTINGS ========
    // =================================
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    //sys.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(100000.0);
    sys.SetMaxPenetrationRecoverySpeed(1000.1);
    sys.SetMinBounceSpeed(0.001);
    ChRealtimeStepTimer realtime_timer;

    // =============================================================
    // ======== SET THE MULTI-PHYSICS SYMULATION PARAMETERS ========
    // =============================================================
    // ======== Mechanical domain ========
    double f_ToSample_mechanic = 0.5e4; // [Hz]
    double t_step_mechanic = 1 / f_ToSample_mechanic; // [s]
    // ======== Electronic domain ======== 
    double f_ToSample_electronic = 0.5e4; // [Hz]                              Frequency at which the electronic domain is called respect to the global time line
    double T_ToSample_electronic = 1 / f_ToSample_electronic;               // Period at which the electronic domain is called respect to the global time line
    double T_sampling_electronic = t_step_mechanic;                         // Time window of the electronic (SPICE) simulation
    double t_step_electronic = 1.0e-6; // [s]                                  Discretization of the electronic time window

    // ==================================================
    // ======== INITIALIZE THE MOTOR ========
    // ==================================================

    ChElectronicMotor motor(Rotor_body, t_step_electronic);
    double kt_motor = 0.1105*1e6;//*1e3*1e3;  // Motor torque constant [Nm/A]
    double ke_motor = -0.0953*1.0;  // Motor back EMF constant [V/(rad/s)]

    motor.InitParams(kt_motor,ke_motor);
    motor.Initialize();


    // ==================================================
    // ======== MULTI-PHYSICS CO-SYMULATION LOOP ========
    // ==================================================

    // ======== SET -> the Multi-physics timeline ========
    double t_simulation_STOP = 400.0e-3; //[s]
    double t_sim_mechanics = 0.0; //[s] 
    double t_sim_electronics = 0.0; //[s]
    double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line
    int brake_flag = 1; // Set a brake flag in the case you want to stop the simulation before: t_simulation_STOP 

    // ======== INITIALIZE -> some needed variables ========
    double IVprobe1 = 0.0; //[A] Current circulating in the Motor
    double T_PWM = 4000.0e-6; //[s] PWM Period
    double Duty_PWM = 85.0 / 100; //[s] PWM Duty
    double t_PWM_counter = 0.0; //[s] PWM Period

    while (t_sim_mechanics < t_simulation_STOP && brake_flag == 1) {
        // ======== RUN -> the Irrlicht visualizer ========
        vis->Run();
        //tools::drawGrid(vis.get(), 2, 2, 30, 30, ChCoordsys<>(ChVector3d(0, 0.01, 0), QuatFromAngleX(CH_PI_2)),ChColor(0.3f, 0.3f, 0.3f), true);
        if (vis->Run()) { brake_flag = 1; } // Check if the User wanted to stop de simulation before: t_simulation_STOP
        else { brake_flag = 0; }
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // ======== SOLVE & UPDATE -> the Electronic domain ========
        if (t_sim_mechanics > 1.0e-4)
        {
            if (t_PWM_counter < T_PWM * Duty_PWM)
            {
                motor.SetPWM(5.2); //[V]
                t_PWM_counter += t_step_mechanic;
            }
            else
            {
                motor.SetPWM(0.0); //[V]
                t_PWM_counter += t_step_mechanic;
            }
            if (t_PWM_counter >= T_PWM)
            {
                t_PWM_counter = 0.0;
            }
        }

        if (t_sampling_electronic_counter >= T_ToSample_electronic)
        {
            // ======== COSIMULATE -> the SPICE circuit ========
            motor.Advance(t_step_mechanic);
            auto res = motor.GetResult();
            std::cout << "IVprobe1 " << res["vprobe1"].back() << std::endl;
            assert(!res.empty());
            t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
        }

        _sleep(300.0e-3); // Wait until Python circuit solution is completed

        // ======== RUN -> the Mechanic solver ========
        sys.DoStepDynamics(t_step_mechanic);
        realtime_timer.Spin(t_step_mechanic);

        // ======== UPDATE -> the Multi-physics timeline ======== 
        t_sampling_electronic_counter += t_step_mechanic;
        t_sim_electronics += t_step_mechanic;
        t_sim_mechanics += t_step_mechanic;
    }

    // ============================================================
    // ======== CLOSE THE MULTI-PHYSICS CO-SIMULATION LOOP ========
    // ============================================================
    // system("pause>0"); // Pause the execution of the code to see the results onto the cmd terminal
    return 0;
}
