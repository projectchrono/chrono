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
// Author:  Milad Rakhsha, Wei Hu
// =============================================================================
//
// Utility function to read input from JSON files
// =============================================================================/

#if defined(_WIN32)

#else
    #include <dirent.h>

#endif

#include <fstream>

#include "ChUtilsJSON.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/math/ChFsiLinearSolver.h"
#include "chrono_fsi/utils/ChUtilsPrintStruct.h"
// Chrono general utils
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace std;

namespace chrono {
namespace fsi {
namespace utils {

Real3 LoadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return mR3(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}


void InvalidArg(std::string arg) {
    cout << "Invalid arg: " << arg << endl;
}

bool ParseJSON(const std::string& json_file, std::shared_ptr<SimParams> paramsH, bool verbose) {
    if (verbose)
        cout << "Reading parameters from: " << json_file << endl;

    FILE* fp = fopen(json_file.c_str(), "r");
    if (!fp) {
        cerr << "Invalid JSON file!" << endl;
        return false;
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;

    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        cerr << "Invalid JSON file!!" << endl;
        return false;
    }

    if (doc.HasMember("Data Output Length"))
        paramsH->output_length = doc["Data Output Length"].GetInt();

    if (doc.HasMember("Output FSI"))
        paramsH->output_fsi = doc["Output FSI"].GetBool();

    if (doc.HasMember("Physical Properties of Fluid")) {
        if (doc["Physical Properties of Fluid"].HasMember("Density"))
            paramsH->rho0 = doc["Physical Properties of Fluid"]["Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Solid Density"))
            paramsH->rho_solid = doc["Physical Properties of Fluid"]["Solid Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Viscosity"))
            paramsH->mu0 = doc["Physical Properties of Fluid"]["Viscosity"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Body Force"))
            paramsH->bodyForce3 = LoadVectorJSON(doc["Physical Properties of Fluid"]["Body Force"]);

        if (doc["Physical Properties of Fluid"].HasMember("Gravity"))
            paramsH->gravity = LoadVectorJSON(doc["Physical Properties of Fluid"]["Gravity"]);

        if (doc["Physical Properties of Fluid"].HasMember("Surface Tension Kappa"))
            paramsH->kappa = doc["Physical Properties of Fluid"]["Surface Tension Kappa"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Characteristic Length"))
            paramsH->L_Characteristic = doc["Physical Properties of Fluid"]["Characteristic Length"].GetDouble();
    }

    if (doc.HasMember("SPH Parameters")) {
        if (doc["SPH Parameters"].HasMember("Method")) {
            std::string SPH = doc["SPH Parameters"]["Method"].GetString();
            if (verbose)
                cout << "Modeling method is: " << SPH << endl;
            if (SPH == "I2SPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::I2SPH;
            else if (SPH == "IISPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::IISPH;
            else if (SPH == "WCSPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::WCSPH;
            else {
                cerr << "Incorrect SPH method in the JSON file: " << SPH << endl;
                cerr << "Falling back to WCSPH " << endl;
                paramsH->fluid_dynamic_type = fluid_dynamics::WCSPH;
            }
        }

        if (doc["SPH Parameters"].HasMember("Kernel h"))
            paramsH->HSML = doc["SPH Parameters"]["Kernel h"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Initial Spacing"))
            paramsH->INITSPACE = doc["SPH Parameters"]["Initial Spacing"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Initial Spacing Solid"))
            paramsH->MULT_INITSPACE_Shells = doc["SPH Parameters"]["Initial Spacing Solid"].GetDouble() / paramsH->HSML;

        if (doc["SPH Parameters"].HasMember("Epsilon"))
            paramsH->epsMinMarkersDis = doc["SPH Parameters"]["Epsilon"].GetDouble();
        else
            paramsH->epsMinMarkersDis = 0.01;

        if (doc["SPH Parameters"].HasMember("Maximum Velocity"))
            paramsH->v_Max = doc["SPH Parameters"]["Maximum Velocity"].GetDouble();

        if (doc["SPH Parameters"].HasMember("XSPH Coefficient"))
            paramsH->EPS_XSPH = doc["SPH Parameters"]["XSPH Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Viscous damping"))
            paramsH->Vis_Dam = doc["SPH Parameters"]["Viscous damping"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Shifting Coefficient"))
            paramsH->beta_shifting = doc["SPH Parameters"]["Shifting Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Density Reinitialization"))
            paramsH->densityReinit = doc["SPH Parameters"]["Density Reinitialization"].GetInt();

        if (doc["SPH Parameters"].HasMember("Conservative Discretization"))
            paramsH->Conservative_Form = doc["SPH Parameters"]["Conservative Discretization"].GetBool();

        if (doc["SPH Parameters"].HasMember("Gradient Discretization Type"))
            paramsH->gradient_type = doc["SPH Parameters"]["Gradient Discretization Type"].GetInt();

        if (doc["SPH Parameters"].HasMember("Laplacian Discretization Type"))
            paramsH->laplacian_type = doc["SPH Parameters"]["Laplacian Discretization Type"].GetInt();

        if (doc["SPH Parameters"].HasMember("Consistent Discretization for Laplacian"))
            paramsH->USE_Consistent_L = doc["SPH Parameters"]["Consistent Discretization for Laplacian"].GetInt();

        if (doc["SPH Parameters"].HasMember("Consistent Discretization for Gradient"))
            paramsH->USE_Consistent_G = doc["SPH Parameters"]["Consistent Discretization for Gradient"].GetInt();
    }

    if (doc.HasMember("Time Stepping")) {
        if (doc["Time Stepping"].HasMember("Adaptive Time stepping"))
            paramsH->Adaptive_time_stepping = doc["Time Stepping"]["Adaptive Time stepping"].GetBool();

        if (doc["Time Stepping"].HasMember("CFL number"))
            paramsH->Co_number = doc["Time Stepping"]["CFL number"].GetDouble();

        if (doc["Time Stepping"].HasMember("Beta"))
            paramsH->Beta = doc["Time Stepping"]["Beta"].GetDouble();

        if (doc["Time Stepping"].HasMember("Fluid time step"))
            paramsH->dT = doc["Time Stepping"]["Fluid time step"].GetDouble();

        if (doc["Time Stepping"].HasMember("Solid time step"))
            paramsH->dT_Flex = doc["Time Stepping"]["Solid time step"].GetDouble();
        else
            paramsH->dT_Flex = paramsH->dT;

        if (doc["Time Stepping"].HasMember("Maximum time step"))
            paramsH->dT_Max = doc["Time Stepping"]["Maximum time step"].GetDouble();
    }

    if (doc.HasMember("Pressure Equation")) {
        if (doc["Pressure Equation"].HasMember("Linear solver")) {
            paramsH->PPE_Solution_type = PPE_SolutionType::FORM_SPARSE_MATRIX;
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

        if (doc["Pressure Equation"].HasMember("Alpha Source Term"))
            paramsH->Alpha = doc["Pressure Equation"]["Alpha Source Term"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Under-relaxation"))
            paramsH->PPE_relaxation = doc["Pressure Equation"]["Under-relaxation"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Absolute residual"))
            paramsH->LinearSolver_Abs_Tol = doc["Pressure Equation"]["Absolute residual"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Relative residual"))
            paramsH->LinearSolver_Rel_Tol = doc["Pressure Equation"]["Relative residual"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Maximum Iterations"))
            paramsH->LinearSolver_Max_Iter = doc["Pressure Equation"]["Maximum Iterations"].GetInt();

        if (doc["Pressure Equation"].HasMember("Verbose monitoring"))
            paramsH->Verbose_monitoring = doc["Pressure Equation"]["Verbose monitoring"].GetBool();

        if (doc["Pressure Equation"].HasMember("Constraint Pressure")) {
            paramsH->Pressure_Constraint = doc["Pressure Equation"]["Constraint Pressure"].GetBool();
            if (doc["Pressure Equation"].HasMember("Average Pressure"))
                paramsH->BASEPRES = doc["Pressure Equation"]["Average Pressure"].GetDouble();
        }

        if (doc["Pressure Equation"].HasMember("Clamp Pressure"))
            paramsH->ClampPressure = doc["Pressure Equation"]["Clamp Pressure"].GetBool();

        if (doc["Pressure Equation"].HasMember("Boundary Conditions")) {
            std::string BC = doc["Pressure Equation"]["Boundary Conditions"].GetString();
            if (BC == "Generalized Wall BC")
                paramsH->bceType = BceVersion::ADAMI;
            else
                paramsH->bceType = BceVersion::ORIGINAL;
        }
    }

    // this part is for modeling granular material dynamics using elastic SPH
    if (doc.HasMember("Elastic SPH")) {
        paramsH->elastic_SPH = true;

        if (doc["Elastic SPH"].HasMember("Poisson ratio"))
            paramsH->Nu_poisson = doc["Elastic SPH"]["Poisson ratio"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Young modulus"))
            paramsH->E_young = doc["Elastic SPH"]["Young modulus"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial stress"))
            paramsH->Ar_stress = doc["Elastic SPH"]["Artificial stress"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity alpha"))
            paramsH->Ar_vis_alpha = doc["Elastic SPH"]["Artificial viscosity alpha"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity beta"))
            paramsH->Ar_vis_beta = doc["Elastic SPH"]["Artificial viscosity beta"].GetDouble();

        if (doc["Elastic SPH"].HasMember("I0"))
            paramsH->mu_I0 = doc["Elastic SPH"]["I0"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_s"))
            paramsH->mu_fric_s = doc["Elastic SPH"]["mu_s"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_2"))
            paramsH->mu_fric_2 = doc["Elastic SPH"]["mu_2"].GetDouble();

        if (doc["Elastic SPH"].HasMember("particle diameter"))
            paramsH->ave_diam = doc["Elastic SPH"]["particle diameter"].GetDouble();

        if (doc["Elastic SPH"].HasMember("frictional angle"))
            paramsH->Fri_angle = doc["Elastic SPH"]["frictional angle"].GetDouble();

        if (doc["Elastic SPH"].HasMember("dilate angle"))
            paramsH->Dil_angle = doc["Elastic SPH"]["dilate angle"].GetDouble();

        if (doc["Elastic SPH"].HasMember("cohesion coefficient"))
            paramsH->Coh_coeff = doc["Elastic SPH"]["cohesion coefficient"].GetDouble();

        if (doc["Elastic SPH"].HasMember("kernel threshold"))
            paramsH->C_Wi = doc["Elastic SPH"]["kernel threshold"].GetDouble();
    }

    // Geometry Information
    if (doc.HasMember("Geometry Inf")) {
        if (doc["Geometry Inf"].HasMember("BoxDimensionX"))
            paramsH->boxDimX = doc["Geometry Inf"]["BoxDimensionX"].GetDouble();

        if (doc["Geometry Inf"].HasMember("BoxDimensionY"))
            paramsH->boxDimY = doc["Geometry Inf"]["BoxDimensionY"].GetDouble();

        if (doc["Geometry Inf"].HasMember("BoxDimensionZ"))
            paramsH->boxDimZ = doc["Geometry Inf"]["BoxDimensionZ"].GetDouble();
    }

    if (doc.HasMember("Body Active Domain"))
        paramsH->bodyActiveDomain = LoadVectorJSON(doc["Body Active Domain"]);

    if (doc.HasMember("Settling Time"))
        paramsH->settlingTime = doc["Settling Time"].GetDouble();

    //===============================================================
    // Material Models
    //===============================================================
    if (doc.HasMember("Material Model")) {
        paramsH->non_newtonian = doc["Material Model"]["Non-Newtonian"].GetBool();
        //===============================================================
        // For a simple non-newtonian flow
        //==============================================================
        if (paramsH->non_newtonian) {
            paramsH->mu_max = doc["Material Model"]["max Viscosity"].GetDouble();

            if (paramsH->non_newtonian) {
                if (doc["Material Model"].HasMember("Herschel–Bulkley")) {
                    paramsH->HB_k = doc["Material Model"]["Herschel–Bulkley"]["k"].GetDouble();
                    paramsH->HB_n = doc["Material Model"]["Herschel–Bulkley"]["n"].GetInt();
                    paramsH->HB_tau0 = doc["Material Model"]["Herschel–Bulkley"]["tau_0"].GetDouble();
                    if (doc["Material Model"]["Herschel–Bulkley"].HasMember("sr0"))
                        paramsH->HB_sr0 = doc["Material Model"]["Herschel–Bulkley"]["sr0"].GetDouble();
                    else
                        paramsH->HB_sr0 = 0.0;
                } else {
                    if (verbose)
                        cout << "Constants of Herschel–Bulkley not found. Using default Newtonian values." << endl;
                    paramsH->HB_k = paramsH->mu0;
                    paramsH->HB_n = 1;
                    paramsH->HB_tau0 = 0;
                    paramsH->HB_sr0 = 0.0;
                }
            }
        }
    } else {
        paramsH->non_newtonian = false;
    }

    // Calculate dependent parameters
    paramsH->INVHSML = 1 / paramsH->HSML;
    paramsH->INV_INIT = 1 / paramsH->INITSPACE;
    paramsH->volume0 = cube(paramsH->INITSPACE);
    paramsH->MULT_INITSPACE = paramsH->INITSPACE / paramsH->HSML;
    paramsH->markerMass = paramsH->volume0 * paramsH->rho0;
    paramsH->INV_dT = 1 / paramsH->dT;
    paramsH->invrho0 = 1 / paramsH->rho0;

    if (paramsH->elastic_SPH) {
        paramsH->G_shear = paramsH->E_young / (2.0 * (1.0 + paramsH->Nu_poisson));
        paramsH->INV_G_shear = 1.0 / paramsH->G_shear;
        paramsH->K_bulk = paramsH->E_young / (3.0 * (1.0 - 2.0 * paramsH->Nu_poisson));
        paramsH->Cs = sqrt(paramsH->K_bulk / paramsH->rho0);

        Real sfri = std::sin(paramsH->Fri_angle);
        Real cfri = std::cos(paramsH->Fri_angle);
        Real sdil = std::sin(paramsH->Dil_angle);
        paramsH->Q_FA = 6 * sfri / (sqrt(3) * (3 + sfri));
        paramsH->Q_DA = 6 * sdil / (sqrt(3) * (3 + sdil));
        paramsH->K_FA = 6 * paramsH->Coh_coeff * cfri / (sqrt(3) * (3 + sfri));
    } else {
        paramsH->Cs = 10 * paramsH->v_Max;
    }

    return true;
}

}  // namespace utils
}  // namespace fsi
}  // namespace chrono
