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
#include <nlohmann/json.hpp>
#include <thread>
#include <fstream>

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
#include "chrono_powerelectronics/circuits/ChElectronicCircuit.h"
#include "chrono_powerelectronics/circuits/ChElectronicGeneric.h"

// ============================
// ======== NAMESPACES ========
// ============================
using namespace ::chrono;
using namespace ::chrono::irrlicht;
using namespace ::chrono::powerelectronics;
// namespace py = pybind11;
// namespace fs = std::filesystem;
using json = nlohmann::json;

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

    //std::string Netlist_location = "C:/workspace-WISC/Validation/RC_Circuit/Project_build/data/Circuit/MotorControl/Circuit_Netlist1.cir";
    
    std::string Netlist_location = "../data/powerelectronics/Circuit/RCvalidation/Circuit_Netlist1.cir";    
    
    ChElectronicGeneric Generic_Circuit(Netlist_location, t_step_electronic); 
    Generic_Circuit.Initialize(t_step_mechanic);

    std::map<std::string, double> PWLIn = {
        {"V1VAR", 5.0}
    };
    std::map<std::string, double> FlowIn = {
        {"R1", 120.0},
        {"C1", 300.0e-6}
    };

    std::map<std::string, std::vector<double>> OutputMap;
    OutputMap["n2"] = {};
    OutputMap["v1var"] = {};
    OutputMap["t_electronics"] = {};


    Generic_Circuit.InputDefinition(PWLIn, FlowIn); 

    

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

        if (t_sampling_electronic_counter >= T_ToSample_electronic)
        {
            /*auto netlist_content = Generic_Circuit.GetNetlist();
            std::cout << "Netlist file: " << std::endl;
            for (const auto& line : netlist_content) {
                std::cout << line << std::endl;
            }*/
            Generic_Circuit.Advance(t_step_mechanic);

            //system("pause>0");
            /*
            auto netlist_content = Generic_Circuit.GetNetlist();
            std::cout << "Netlist file: " << std::endl;
            for (const auto& line : netlist_content) {
                std::cout << line << std::endl;
            }*/
            auto res1 = Generic_Circuit.GetResult();
            // Plot the results
            /*std::cout << "Results:\n" << std::endl;
            for (const auto& [key, values] : res1) { 
                std::cout << key << ": ";
                if (key=="n2") {
                    for (double value : values) {
                    std::cout << value << " ";
                    }
                }
                std::cout << std::endl;}*/

            OutputMap["n2"].push_back(res1["n2"].back());
            OutputMap["v1var"].push_back(res1["v1var"].back());
            OutputMap["t_electronics"].push_back(t_sim_mechanics);
            //auto aaa=res1["n1"];
            /*
            auto aaa=OutputMap["t_electronics"];
            for (double value : aaa) {
                std::cout << value << " ";
                }
            std::cout << std::endl;*/
            std::cout << t_sim_mechanics << std::endl;



            t_sampling_electronic_counter = 0;      // The variable is nulled to re-start with the counter for the next call of the electronic domain
        }

        //_sleep(300.0e-3); // Wait until Python circuit solution is completed

        // ======== RUN -> the Mechanic solver ========
        sys.DoStepDynamics(t_step_mechanic);
        realtime_timer.Spin(t_step_mechanic);

        // ======== UPDATE -> the Multi-physics timeline ======== 
        t_sampling_electronic_counter += t_step_mechanic;
        t_sim_electronics += t_step_mechanic;
        t_sim_mechanics += t_step_mechanic;
    }

    // =====================================================
    // ======== EXPORT THE RESULTS INTO A JSON FILE ========
    // =====================================================
    json j; // Create a json object to contain the output data

    // Populate the JSON object with data
    for (const auto& item : OutputMap) { 
        j[item.first] = item.second;
    }

    // Export the output data in a .json file
    std::ofstream out_file("output.json");
    out_file << j.dump(4); // "4" is the indentation parameter, you can change it to have a more or less readable structure
    out_file.close();

    std::cout << "Data exported to 'output.json'" << std::endl;

    // ============================================================
    // ======== CLOSE THE MULTI-PHYSICS CO-SIMULATION LOOP ========
    // ============================================================
    // system("pause>0"); // Pause the execution of the code to see the results onto the cmd terminal

    return 0;
}
