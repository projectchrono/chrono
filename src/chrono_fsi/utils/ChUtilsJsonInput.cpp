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
// Authors: Milad Rakhsha
// =============================================================================

#include "chrono_fsi/ChDeviceUtils.cuh"
#include "chrono_fsi/ChFsiLinearSolver.h"

#include "chrono_fsi/utils/ChUtilsJsonInput.h"
#include "chrono_fsi/utils/ChUtilsPrintStruct.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

using std::cout;
using std::endl;

namespace chrono {
namespace fsi {
namespace utils {

Real3 LoadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return mR3(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

// Returns true on successful parameter load.
// Returns false and prints error on invalid argument.
bool ParseJSON(const char* json_file, SimParams* paramsH, Real3 Domain) {
    cout << "Reading parameters: " << json_file << endl;
    FILE* fp = fopen(json_file, "r");
    if (!fp) {
        cout << "Invalid JSON file!" << endl;
        return false;
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;

    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        std::cerr << "Invalid JSON file!!" << endl;
        return false;
    }

    cout << "--- Parsing JSON ---" << endl;
    if (doc.HasMember("Output Folder"))
        strcpy(paramsH->out_name, doc["Output Folder"].GetString());
    else
        strcpy(paramsH->out_name, "Undefined");

    if (doc.HasMember("Physical Properties of Fluid")) {
        if (doc["Physical Properties of Fluid"].HasMember("Density"))
            paramsH->rho0 = doc["Physical Properties of Fluid"]["Density"].GetDouble();
        else
            paramsH->rho0 = 1.0;

        if (doc["Physical Properties of Fluid"].HasMember("Solid Density"))
            paramsH->rho_solid = doc["Physical Properties of Fluid"]["Solid Density"].GetDouble();
        else
            paramsH->rho_solid = paramsH->rho0;

        if (doc["Physical Properties of Fluid"].HasMember("Viscosity"))
            paramsH->mu0 = doc["Physical Properties of Fluid"]["Viscosity"].GetDouble();
        else
            paramsH->mu0 = 0.001;

        if (doc["Physical Properties of Fluid"].HasMember("Body Force"))
            paramsH->bodyForce3 = LoadVectorJSON(doc["Physical Properties of Fluid"]["Body Force"]);
        else
            paramsH->bodyForce3 = mR3(0.0, 0, 0);

        if (doc["Physical Properties of Fluid"].HasMember("Gravity"))
            paramsH->gravity = LoadVectorJSON(doc["Physical Properties of Fluid"]["Gravity"]);
        else
            paramsH->gravity = mR3(0.0, 0, 0);

        if (doc["Physical Properties of Fluid"].HasMember("Surface Tension Kappa"))
            paramsH->kappa = doc["Physical Properties of Fluid"]["Surface Tension Kappa"].GetDouble();
        else
            paramsH->kappa = 0.0;

        if (doc["Physical Properties of Fluid"].HasMember("Characteristic Length"))
            paramsH->L_Characteristic = doc["Physical Properties of Fluid"]["Characteristic Length"].GetDouble();
        else
            paramsH->L_Characteristic = 1.0;
    }

    if (doc.HasMember("SPH Parameters")) {
        if (doc["SPH Parameters"].HasMember("Kernel h"))
            paramsH->HSML = doc["SPH Parameters"]["Kernel h"].GetDouble();
        else
            paramsH->HSML = 0.02;

        std::cout << "paramsH->HSML: " << paramsH->HSML << std::endl;

        if (doc["SPH Parameters"].HasMember("Initial Spacing"))
            paramsH->MULT_INITSPACE = doc["SPH Parameters"]["Initial Spacing"].GetDouble() / paramsH->HSML;
        else
            paramsH->MULT_INITSPACE = 1.0;

        if (doc["SPH Parameters"].HasMember("Initial Spacing Solid"))
            paramsH->MULT_INITSPACE_Shells = doc["SPH Parameters"]["Initial Spacing Solid"].GetDouble() / paramsH->HSML;
        else
            paramsH->MULT_INITSPACE_Shells = 1.0;

        if (doc["SPH Parameters"].HasMember("Epsilon"))
            paramsH->epsMinMarkersDis = doc["SPH Parameters"]["Epsilon"].GetDouble();
        else
            paramsH->epsMinMarkersDis = 0.01;

        if (doc["SPH Parameters"].HasMember("Maximum Velocity"))
            paramsH->v_Max = doc["SPH Parameters"]["Maximum Velocity"].GetDouble();
        else
            paramsH->v_Max = 1.0;

        if (doc["SPH Parameters"].HasMember("XSPH Coefficient"))
            paramsH->EPS_XSPH = doc["SPH Parameters"]["XSPH Coefficient"].GetDouble();
        else
            paramsH->EPS_XSPH = 0.12;

        if (doc["SPH Parameters"].HasMember("Shifting Coefficient"))
            paramsH->beta_shifting = doc["SPH Parameters"]["Shifting Coefficient"].GetDouble();
        else
            paramsH->beta_shifting = 0.0;

        if (doc["SPH Parameters"].HasMember("Density Reinitialization"))
            paramsH->densityReinit = doc["SPH Parameters"]["Density Reinitialization"].GetInt();
        else
            paramsH->densityReinit = 2147483647;

        if (doc["SPH Parameters"].HasMember("Conservative Discretization"))
            paramsH->Conservative_Form = doc["SPH Parameters"]["Conservative Discretization"].GetBool();
        else
            paramsH->Conservative_Form = true;
    }

    if (doc.HasMember("Time Stepping")) {
        if (doc["Time Stepping"].HasMember("Adaptive Time stepping"))
            paramsH->Adaptive_time_stepping = doc["Time Stepping"]["Adaptive Time stepping"].GetBool();
        else
            paramsH->Adaptive_time_stepping = false;

        if (doc["Time Stepping"].HasMember("CFL number"))
            paramsH->Co_number = doc["Time Stepping"]["CFL number"].GetDouble();
        else
            paramsH->Co_number = 0.1;

        if (doc["Time Stepping"].HasMember("Fluid time step"))
            paramsH->dT = doc["Time Stepping"]["Fluid time step"].GetDouble();
        else
            paramsH->dT = 0.01;

        if (doc["Time Stepping"].HasMember("Solid time step"))
            paramsH->dT_Flex = doc["Time Stepping"]["Solid time step"].GetDouble();
        else
            paramsH->dT_Flex = paramsH->dT;

        if (doc["Time Stepping"].HasMember("Maximum time step"))
            paramsH->dT_Max = doc["Time Stepping"]["Maximum time step"].GetDouble();
        else
            paramsH->dT_Max = 1.0;

        if (doc["Time Stepping"].HasMember("Write frame per second"))
            paramsH->out_fps = doc["Time Stepping"]["Write frame per second"].GetInt();
        else
            paramsH->out_fps = 20;

        if (doc["Time Stepping"].HasMember("End time"))
            paramsH->tFinal = doc["Time Stepping"]["End time"].GetDouble();
        else
            paramsH->tFinal = 2000;
    }

    if (doc.HasMember("Pressure Equation")) {
        if (doc["Pressure Equation"].HasMember("Linear solver")) {
            paramsH->PPE_Solution_type = FORM_SPARSE_MATRIX;
            std::string solver = doc["Pressure Equation"]["Linear solver"].GetString();
            if (solver == "Jacobi") {
                paramsH->USE_LinearSolver = false;
            } else {
                paramsH->USE_LinearSolver = true;
                if (solver == "BICGSTAB")
                    paramsH->LinearSolver = ChFsiLinearSolver::SolverType::BICGSTAB;
                if (solver == "GMRES")
                    paramsH->LinearSolver = ChFsiLinearSolver::SolverType::GMRES;
            }
        } else {
            paramsH->PPE_Solution_type = MATRIX_FREE;
        }

        if (doc["Pressure Equation"].HasMember("Poisson source term")) {
            std::string source = doc["Pressure Equation"]["Poisson source term"].GetString();
            if (source == "Density-Based")
                paramsH->DensityBaseProjetion = true;
            else
                paramsH->DensityBaseProjetion = false;
        }

        if (doc["Pressure Equation"].HasMember("Projection method")) {
            std::string source = doc["Pressure Equation"]["Projection method"].GetString();
            if (source == "Incremental")
                paramsH->USE_NonIncrementalProjection = false;
            else
                paramsH->USE_NonIncrementalProjection = true;
        }

        if (doc["Pressure Equation"].HasMember("Clamp Pressure"))
            paramsH->ClampPressure = doc["Pressure Equation"]["Clamp Pressure"].GetBool();
        else
            paramsH->ClampPressure = true;

        if (doc["Pressure Equation"].HasMember("Max Pressure"))
            paramsH->Max_Pressure = doc["Pressure Equation"]["Max Pressure"].GetDouble();
        else
            paramsH->Max_Pressure = 1e20;

        if (doc["Pressure Equation"].HasMember("Under-relaxation"))
            paramsH->PPE_relaxation = doc["Pressure Equation"]["Under-relaxation"].GetDouble();
        else
            paramsH->PPE_relaxation = 1.0;

        if (doc["Pressure Equation"].HasMember("Absolute residual"))
            paramsH->LinearSolver_Abs_Tol = doc["Pressure Equation"]["Absolute residual"].GetDouble();
        else
            paramsH->LinearSolver_Abs_Tol = 0.0;

        if (doc["Pressure Equation"].HasMember("Relative residual"))
            paramsH->LinearSolver_Rel_Tol = doc["Pressure Equation"]["Relative residual"].GetDouble();
        else
            paramsH->LinearSolver_Rel_Tol = 0.0;

        if (doc["Pressure Equation"].HasMember("Maximum Iterations"))
            paramsH->LinearSolver_Max_Iter = doc["Pressure Equation"]["Maximum Iterations"].GetInt();
        else
            paramsH->LinearSolver_Max_Iter = 1000;

        if (doc["Pressure Equation"].HasMember("Verbose monitoring"))
            paramsH->Verbose_monitoring = doc["Pressure Equation"]["Verbose monitoring"].GetBool();
        else
            paramsH->Verbose_monitoring = false;

        if (doc["Pressure Equation"].HasMember("Constraint Pressure")) {
            paramsH->Pressure_Constraint = doc["Pressure Equation"]["Constraint Pressure"].GetBool();
            if (doc["Pressure Equation"].HasMember("Average Pressure"))
                paramsH->BASEPRES = doc["Pressure Equation"]["Average Pressure"].GetDouble();
            else
                paramsH->BASEPRES = 0.0;
        } else
            paramsH->Pressure_Constraint = false;

        if (doc["Pressure Equation"].HasMember("Boundary Conditions")) {
            std::string BC = doc["Pressure Equation"]["Boundary Conditions"].GetString();
            if (BC == "Generalized Wall BC")
                paramsH->bceType = ADAMI;
            else
                paramsH->bceType = mORIGINAL;
        } else
            paramsH->bceType = ADAMI;
    }

    paramsH->cMin = mR3(-Domain.x * 2, -Domain.y * 2, -2 * Domain.z) - 3 * mR3(paramsH->HSML);
    paramsH->cMax = mR3(Domain.x * 2, Domain.y * 2, 2 * Domain.z) + 3 * mR3(paramsH->HSML);

    std::cout << "parameters of the simulation" << std::endl;
    std::cout << "paramsH->rho0: " << paramsH->rho0 << std::endl;
    std::cout << "paramsH->mu0: " << paramsH->mu0 << std::endl;
    std::cout << "paramsH->bodyForce3: ";
    utils::printStruct(paramsH->bodyForce3);
    std::cout << "paramsH->gravity: ";
    utils::printStruct(paramsH->gravity);

    std::cout << "paramsH->HSML: " << paramsH->HSML << std::endl;
    std::cout << "paramsH->MULT_INITSPACE: " << paramsH->MULT_INITSPACE << std::endl;
    std::cout << "paramsH->NUM_BOUNDARY_LAYERS: " << paramsH->NUM_BOUNDARY_LAYERS << std::endl;
    std::cout << "paramsH->epsMinMarkersDis: " << paramsH->epsMinMarkersDis << std::endl;
    std::cout << "paramsH->v_Max: " << paramsH->v_Max << std::endl;
    std::cout << "paramsH->EPS_XSPH: " << paramsH->EPS_XSPH << std::endl;
    std::cout << "paramsH->beta_shifting: " << paramsH->beta_shifting << std::endl;
    std::cout << "paramsH->densityReinit: " << paramsH->densityReinit << std::endl;

    std::cout << "paramsH->Adaptive_time_stepping: " << paramsH->Adaptive_time_stepping << std::endl;
    std::cout << "paramsH->Co_number: " << paramsH->Co_number << std::endl;
    std::cout << "paramsH->dT: " << paramsH->dT << std::endl;
    std::cout << "paramsH->dT_Max: " << paramsH->dT_Max << std::endl;
    std::cout << "paramsH->dT_Flex: " << paramsH->dT_Flex << std::endl;
    std::cout << "paramsH->cMin: ";
    utils::printStruct(paramsH->cMin);
    std::cout << "paramsH->cMax: ";
    utils::printStruct(paramsH->cMax);

    std::cout << "paramsH->bceType: " << paramsH->bceType << std::endl;
    std::cout << "paramsH->USE_NonIncrementalProjection : " << paramsH->USE_NonIncrementalProjection << std::endl;
    //    std::cout << "paramsH->LinearSolver: " << paramsH->LinearSolver << std::endl;
    std::cout << "paramsH->PPE_relaxation: " << paramsH->PPE_relaxation << std::endl;
    std::cout << "paramsH->Conservative_Form: " << paramsH->Conservative_Form << std::endl;
    std::cout << "paramsH->Pressure_Constraint: " << paramsH->Pressure_Constraint << std::endl;

    std::cout << "paramsH->deltaPress: ";
    utils::printStruct(paramsH->deltaPress);
    std::cout << "paramsH->binSize0: " << paramsH->binSize0 << std::endl;
    std::cout << "paramsH->boxDims: ";
    utils::printStruct(paramsH->boxDims);
    std::cout << "paramsH->gridSize: ";
    utils::printStruct(paramsH->gridSize);
    std::cout << "********************" << std::endl;
    return true;
}
}  // namespace utils
}  // namespace fsi
}  // namespace chrono
