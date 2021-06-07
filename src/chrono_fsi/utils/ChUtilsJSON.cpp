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
#include "chrono_fsi/physics/ChParams.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"
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

Real massCalculator(int& num_nei, Real Kernel_h, Real InitialSpacing, Real rho0) {
    int IDX = 10;
    Real sum_wij = 0;
    int count = 0;
    for (int i = -IDX; i <= IDX; i++)
        for (int j = -IDX; j <= IDX; j++)
            for (int k = -IDX; k <= IDX; k++) {
                Real3 pos = mR3(i, j, k) * InitialSpacing;
                Real W = W3h_Spline(length(pos), Kernel_h);
                if (W > 0) {              
                    count++;
                }
                sum_wij += W;
            }
    printf("Kernel_h=%f, InitialSpacing=%f, Number of Neighbors=%d, Mass_i= %f, Sum_wi= %f\n", Kernel_h, InitialSpacing,
           count, rho0 / sum_wij, sum_wij);
    num_nei = count;
    return rho0 / sum_wij;
}

void InvalidArg(std::string arg) {
    std::cout << "Invalid arg: " << arg << std::endl;
}

bool ParseJSON(const std::string& json_file, std::shared_ptr<SimParams> paramsH, Real3 Domain) {
    std::cout << "Reading parameters: " << json_file << std::endl;
    FILE* fp = fopen(json_file.c_str(), "r");
    if (!fp) {
        std::cout << "Invalid JSON file!" << std::endl;
        return false;
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;

    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        std::cerr << "Invalid JSON file!!" << std::endl;
        return false;
    }

    std::cout << "--- Parsing JSON ---" << std::endl;
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
        if (doc["SPH Parameters"].HasMember("Method")) {
            std::string SPH = doc["SPH Parameters"]["Method"].GetString();
            std::cout << SPH << std::endl;
            if (SPH == "I2SPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::I2SPH;
            else if (SPH == "IISPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::IISPH;
            else if (SPH == "WCSPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::WCSPH;
            else {
                std::cerr << "Incorrect SPH method in the JSON file: " << SPH << std::endl;
                std::cerr << "Falling back to I2SPH " << std::endl;
                paramsH->fluid_dynamic_type = fluid_dynamics::I2SPH;
            }
        } else
            paramsH->fluid_dynamic_type = fluid_dynamics::I2SPH;

        if (doc["SPH Parameters"].HasMember("Kernel h")){
            paramsH->HSML = doc["SPH Parameters"]["Kernel h"].GetDouble();
            paramsH->INVHSML = 1.0 / paramsH->HSML;}
        else{
            paramsH->HSML = 0.02;
            paramsH->INVHSML = 1.0 / paramsH->HSML;
        }

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

        if (doc["SPH Parameters"].HasMember("Maximum Velocity")){
            paramsH->v_Max = doc["SPH Parameters"]["Maximum Velocity"].GetDouble();
            paramsH->Cs = 10.0*paramsH->v_Max;}
        else{
            paramsH->v_Max = 1.0;
            paramsH->Cs = 10.0*paramsH->v_Max;}

        if (doc["SPH Parameters"].HasMember("XSPH Coefficient"))
            paramsH->EPS_XSPH = doc["SPH Parameters"]["XSPH Coefficient"].GetDouble();
        else
            paramsH->EPS_XSPH = 0.11;

        if (doc["SPH Parameters"].HasMember("Viscous damping"))
            paramsH->Vis_Dam = doc["SPH Parameters"]["Viscous damping"].GetDouble();

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

        if (doc["SPH Parameters"].HasMember("Gradient Discretization Type"))
            paramsH->gradient_type = doc["SPH Parameters"]["Gradient Discretization Type"].GetInt();
        else
            paramsH->gradient_type = 0;

        if (doc["SPH Parameters"].HasMember("Laplacian Discretization Type"))
            paramsH->laplacian_type = doc["SPH Parameters"]["Laplacian Discretization Type"].GetInt();
        else
            paramsH->laplacian_type = 0;
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

        if (doc["Time Stepping"].HasMember("Beta"))
            paramsH->Beta = doc["Time Stepping"]["Beta"].GetDouble();
        else
            paramsH->Beta = 0.5;

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
            paramsH->out_fps = doc["Time Stepping"]["Write frame per second"].GetDouble();
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

        if (doc["Pressure Equation"].HasMember("Alpha Source Term"))
            paramsH->Alpha = doc["Pressure Equation"]["Alpha Source Term"].GetDouble();
        else
            paramsH->Alpha = paramsH->HSML;

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
                paramsH->BASEPRES = false;
        } else
            paramsH->Pressure_Constraint = false;

        if (doc["Pressure Equation"].HasMember("Clamp Pressure")) {
            paramsH->ClampPressure = doc["Pressure Equation"]["Clamp Pressure"].GetBool();
        } else
            paramsH->ClampPressure = false;

        if (doc["Pressure Equation"].HasMember("Boundary Conditions")) {
            std::string BC = doc["Pressure Equation"]["Boundary Conditions"].GetString();
            if (BC == "Generalized Wall BC")
                paramsH->bceType = ADAMI;
            else
                paramsH->bceType = mORIGINAL;
        } else
            paramsH->bceType = ADAMI;
    }

    // this part is for modeling granular material dynamics using elastic SPH
    if (doc.HasMember("Elastic SPH")) {
        paramsH->elastic_SPH = true;
        if (doc["Elastic SPH"].HasMember("Poisson ratio")) {
            paramsH->Nu_poisson = doc["Elastic SPH"]["Poisson ratio"].GetDouble();
        }
        if (doc["Elastic SPH"].HasMember("Young modulus")) {
            paramsH->E_young = doc["Elastic SPH"]["Young modulus"].GetDouble();              // Young's modulus
            paramsH->G_shear = paramsH->E_young / (2.0 * (1.0 + paramsH->Nu_poisson));       // shear modulus
            paramsH->K_bulk = paramsH->E_young / (3.0 * (1.0 - 2.0 * paramsH->Nu_poisson));  // bulk modulus
            paramsH->Cs = sqrt(paramsH->K_bulk / paramsH->rho0);
        }
        if (doc["Elastic SPH"].HasMember("Artificial stress")) {
            paramsH->Ar_stress = doc["Elastic SPH"]["Artificial stress"].GetDouble();
        }
        if (doc["Elastic SPH"].HasMember("Artificial viscosity alpha")) {
            paramsH->Ar_vis_alpha = doc["Elastic SPH"]["Artificial viscosity alpha"].GetDouble();
        }
        if (doc["Elastic SPH"].HasMember("Artificial viscosity beta")) {
            paramsH->Ar_vis_beta = doc["Elastic SPH"]["Artificial viscosity beta"].GetDouble();
        }
        if (doc["Elastic SPH"].HasMember("I0")) {
            paramsH->mu_I0 = doc["Elastic SPH"]["I0"].GetDouble();
        }
        if (doc["Elastic SPH"].HasMember("mu_s")) {
            paramsH->mu_fric_s = doc["Elastic SPH"]["mu_s"].GetDouble();
        }
        if (doc["Elastic SPH"].HasMember("mu_2")) {
            paramsH->mu_fric_2 = doc["Elastic SPH"]["mu_2"].GetDouble();
        }
        if (doc["Elastic SPH"].HasMember("particle diameter")) {
            paramsH->ave_diam = doc["Elastic SPH"]["particle diameter"].GetDouble();  // average particle diameter
        }
        if (doc["Elastic SPH"].HasMember("frictional angle")) {
            paramsH->Fri_angle = doc["Elastic SPH"]["frictional angle"].GetDouble();  // frictional angle of granular material
        }
        if (doc["Elastic SPH"].HasMember("dilate angle")) {
            paramsH->Dil_angle = doc["Elastic SPH"]["dilate angle"].GetDouble();  // dilate angle of granular material
        }
        if (doc["Elastic SPH"].HasMember("cohesion coefficient")) {
            paramsH->Coh_coeff = doc["Elastic SPH"]["cohesion coefficient"].GetDouble();  // cohesion coefficient
            paramsH->Q_FA = 6 * sin(paramsH->Fri_angle) / (sqrt(3) * (3 + sin(paramsH->Fri_angle)));  // material constants calculate from frictional angle
            paramsH->Q_DA = 6 * sin(paramsH->Dil_angle) / (sqrt(3) * (3 + sin(paramsH->Dil_angle)));  // material constants calculate from dilate angle
            paramsH->K_FA = 6 * paramsH->Coh_coeff * cos(paramsH->Fri_angle) / (sqrt(3) * (3 + sin(paramsH->Fri_angle)));  // material constants calculate from frictional angle and cohesion coefficient
        }
        if (doc["Elastic SPH"].HasMember("kernel threshold")) {
            paramsH->C_Wi = doc["Elastic SPH"]["kernel threshold"].GetDouble(); 
        } else{
            paramsH->C_Wi = 0.8;
        }
    } else {
        paramsH->elastic_SPH = false;
    }

    // Geometry Information
    if (doc.HasMember("Geometry Inf")) {
        if (doc["Geometry Inf"].HasMember("BoxDimensionX")) {
            paramsH->boxDimX = doc["Geometry Inf"]["BoxDimensionX"].GetDouble();
        }
        if (doc["Geometry Inf"].HasMember("BoxDimensionY")) {
            paramsH->boxDimY = doc["Geometry Inf"]["BoxDimensionY"].GetDouble();
        }
        if (doc["Geometry Inf"].HasMember("BoxDimensionZ")) {
            paramsH->boxDimZ = doc["Geometry Inf"]["BoxDimensionZ"].GetDouble();
        }
        if (doc["Geometry Inf"].HasMember("FluidDimensionX")) {
            paramsH->fluidDimX = doc["Geometry Inf"]["FluidDimensionX"].GetDouble();
        }
        if (doc["Geometry Inf"].HasMember("FluidDimensionY")) {
            paramsH->fluidDimY = doc["Geometry Inf"]["FluidDimensionY"].GetDouble();
        }
        if (doc["Geometry Inf"].HasMember("FluidDimensionZ")) {
            paramsH->fluidDimZ = doc["Geometry Inf"]["FluidDimensionZ"].GetDouble();
        }
    }

    // Body Information
    if (doc.HasMember("Body Inf")) {
        if (doc["Body Inf"].HasMember("BodyDimensionX")) {
            paramsH->bodyDimX = doc["Body Inf"]["BodyDimensionX"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyDimensionY")) {
            paramsH->bodyDimY = doc["Body Inf"]["BodyDimensionY"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyDimensionZ")) {
            paramsH->bodyDimZ = doc["Body Inf"]["BodyDimensionZ"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyRadius")) {
            paramsH->bodyRad = doc["Body Inf"]["BodyRadius"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyLength")) {
            paramsH->bodyLength = doc["Body Inf"]["BodyLength"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyIniPosX")) {
            paramsH->bodyIniPosX = doc["Body Inf"]["BodyIniPosX"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyIniPosY")) {
            paramsH->bodyIniPosY = doc["Body Inf"]["BodyIniPosY"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyIniPosZ")) {
            paramsH->bodyIniPosZ = doc["Body Inf"]["BodyIniPosZ"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyIniVelX")) {
            paramsH->bodyIniVelX = doc["Body Inf"]["BodyIniVelX"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyIniVelY")) {
            paramsH->bodyIniVelY = doc["Body Inf"]["BodyIniVelY"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyIniVelZ")) {
            paramsH->bodyIniVelZ = doc["Body Inf"]["BodyIniVelZ"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyIniAngVel")) {
            paramsH->bodyIniAngVel = doc["Body Inf"]["BodyIniAngVel"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyMass")) {
            paramsH->bodyMass = doc["Body Inf"]["BodyMass"].GetDouble();
        }
        if (doc["Body Inf"].HasMember("BodyDensity")) {
            paramsH->bodyDensity = doc["Body Inf"]["BodyDensity"].GetDouble();
        }
    }

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

            if (paramsH->non_newtonian && !paramsH->granular_material) {
                if (doc["Material Model"].HasMember("Herschel–Bulkley")) {
                    paramsH->HB_k = doc["Material Model"]["Herschel–Bulkley"]["k"].GetDouble();
                    paramsH->HB_n = doc["Material Model"]["Herschel–Bulkley"]["n"].GetInt();
                    paramsH->HB_tau0 = doc["Material Model"]["Herschel–Bulkley"]["tau_0"].GetDouble();
                    if (doc["Material Model"]["Herschel–Bulkley"].HasMember("sr0"))
                        paramsH->HB_sr0 = doc["Material Model"]["Herschel–Bulkley"]["sr0"].GetDouble();
                    else
                        paramsH->HB_sr0 = 0.0;
                } else {
                    std::cout << "Constants of Herschel–Bulkley are not found. Using the default Newtonian values" << std::endl;
                    paramsH->HB_k = paramsH->mu0;
                    paramsH->HB_n = 1;
                    paramsH->HB_tau0 = 0;
                    paramsH->HB_sr0 = 0.0;
                }
            }
        }
        //===============================================================
        // Granular Material Model
        //===============================================================
        if (doc["Material Model"].HasMember("granular material")) {
            paramsH->granular_material = doc["Material Model"]["granular material"].GetBool();
            // Apply Non-Newtonian model to get the granular material model
            if (paramsH->granular_material) {
                if (doc["Material Model"]["granular model"].HasMember("Shear Modulus"))
                    paramsH->Shear_Mod = doc["Material Model"]["granular model"]["Shear Modulus"].GetDouble();
                else
                    paramsH->Shear_Mod = 1e6;

                paramsH->ave_diam = doc["Material Model"]["granular model"]["particle diameter"].GetDouble();
                if (doc["Material Model"]["granular model"].HasMember("cohesion"))
                    paramsH->cohesion = doc["Material Model"]["granular model"]["cohesion"].GetDouble();

                //===============================================================
                // rheology
                //===============================================================
                if (doc["Material Model"].HasMember("granular model")) {
                    std::string source = doc["Material Model"]["granular model"]["rheology model"].GetString();
                    if (source == "Inertia rheology")
                        paramsH->rheology_model = rheology::Inertia_rheology;
                    else
                        paramsH->rheology_model = rheology::nonlocal_fluidity;
                } else {
                    throw std::runtime_error("No rheology model is indicated.\n");
                }
                //===============================================================
                /// mu(I) functionality
                //===============================================================
                if (doc["Material Model"]["granular model"].HasMember("mu(I)")) {
                    std::string type = doc["Material Model"]["granular model"]["mu(I)"]["type"].GetString();
                    if (type == "constant") {
                        paramsH->mu_of_I = friction_law::constant;
                        paramsH->mu_fric_s =
                            doc["Material Model"]["granular model"]["mu(I)"]["constant"]["mu_s"].GetDouble();
                    } else if (type == "linear") {
                        paramsH->mu_of_I = friction_law::linear;
                        paramsH->mu_I_b = doc["Material Model"]["granular model"]["mu(I)"]["linear"]["b"].GetDouble();
                        paramsH->mu_fric_s =
                            doc["Material Model"]["granular model"]["mu(I)"]["linear"]["mu_s"].GetDouble();
                    } else if (type == "nonlinear") {
                        paramsH->mu_of_I = friction_law::nonlinear;
                        paramsH->mu_I0 =
                            doc["Material Model"]["granular model"]["mu(I)"]["nonlinear"]["I0"].GetDouble();
                        paramsH->mu_fric_s =
                            doc["Material Model"]["granular model"]["mu(I)"]["nonlinear"]["mu_s"].GetDouble();
                        paramsH->mu_fric_2 =
                            doc["Material Model"]["granular model"]["mu(I)"]["nonlinear"]["mu_2"].GetDouble();
                    } else {
                        throw std::runtime_error("constant, linear or nonlinear are the only options for now.\n");
                    }
                } else {
                    throw std::runtime_error(
                        "mu(I) functionality is not set properly although you are modeling granular material.\n");
                }
                //===============================================================
            }

        } else {
            paramsH->granular_material = false;
        }

    } else {
        paramsH->non_newtonian = false;
    }
    
    int NN = 0;
    paramsH->markerMass = massCalculator(NN, paramsH->HSML, paramsH->MULT_INITSPACE * paramsH->HSML, paramsH->rho0);
    paramsH->markerMass = cube(paramsH->MULT_INITSPACE * paramsH->HSML) * paramsH->rho0;
    paramsH->volume0 = paramsH->markerMass / paramsH->rho0;
    paramsH->invrho0 = 1.0 / paramsH->rho0;
    paramsH->num_neighbors = NN;
    

    paramsH->Max_Pressure = 1e20;
    paramsH->cMin = mR3(-Domain.x * 2, -Domain.y * 2, -2 * Domain.z) - 10 * mR3(paramsH->HSML);
    paramsH->cMax = mR3(Domain.x * 2, Domain.y * 2, 2 * Domain.z) + 10 * mR3(paramsH->HSML);

    std::cout << "parameters of the simulation" << std::endl;

    std::cout << "paramsH->num_neighbors: " << paramsH->num_neighbors << std::endl;
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
    std::cout << "paramsH->markerMass: " << paramsH->markerMass << std::endl;
    std::cout << "paramsH->gradient_type: " << paramsH->gradient_type << std::endl;

    std::cout << "paramsH->v_Max: " << paramsH->v_Max << std::endl;
    std::cout << "paramsH->EPS_XSPH: " << paramsH->EPS_XSPH << std::endl;
    std::cout << "paramsH->beta_shifting: " << paramsH->beta_shifting << std::endl;
    std::cout << "paramsH->densityReinit: " << paramsH->densityReinit << std::endl;

    std::cout << "paramsH->Adaptive_time_stepping: " << paramsH->Adaptive_time_stepping << std::endl;
    std::cout << "paramsH->Co_number: " << paramsH->Co_number << std::endl;
    std::cout << "paramsH->dT: " << paramsH->dT << std::endl;
    std::cout << "paramsH->dT_Max: " << paramsH->dT_Max << std::endl;
    std::cout << "paramsH->dT_Flex: " << paramsH->dT_Flex << std::endl;

    std::cout << "paramsH->non_newtonian: " << paramsH->non_newtonian << std::endl;
    std::cout << "paramsH->granular_material: " << paramsH->granular_material << std::endl;
    std::cout << "paramsH->mu_of_I : " << paramsH->mu_of_I << std::endl;
    std::cout << "paramsH->rheology_model: " << paramsH->rheology_model << std::endl;
    std::cout << "paramsH->ave_diam: " << paramsH->ave_diam << std::endl;
    std::cout << "paramsH->mu_max: " << paramsH->mu_max << std::endl;
    std::cout << "paramsH->mu_fric_s: " << paramsH->mu_fric_s << std::endl;
    std::cout << "paramsH->mu_fric_2: " << paramsH->mu_fric_2 << std::endl;
    std::cout << "paramsH->mu_I0: " << paramsH->mu_I0 << std::endl;
    std::cout << "paramsH->mu_I_b: " << paramsH->mu_I_b << std::endl;
    std::cout << "paramsH->HB_k: " << paramsH->HB_k << std::endl;
    std::cout << "paramsH->HB_n: " << paramsH->HB_n << std::endl;
    std::cout << "paramsH->HB_tau0: " << paramsH->HB_tau0 << std::endl;

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
}  // namespace utils
void PrepareOutputDir(std::shared_ptr<fsi::SimParams> paramsH,
                      std::string& demo_dir,
                      std::string out_dir,
                      std::string jsonFile) {
    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    std::cout << asctime(timeinfo);
    char buffer[80];

    strftime(buffer, 80, "%F_%T", timeinfo);

    out_dir = filesystem::path(out_dir).str();

    if (strcmp(paramsH->out_name, "Undefined") == 0)
        demo_dir = filesystem::path(filesystem::path(out_dir) / filesystem::path(buffer)).str();

    else
        demo_dir = filesystem::path(filesystem::path(out_dir) / filesystem::path(paramsH->out_name)).str();

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(demo_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return;
    }

    strcpy(paramsH->demo_dir, filesystem::path(demo_dir).str().c_str());

#if defined(_WIN32)
    HANDLE hFind;
    WIN32_FIND_DATA data;

    hFind = FindFirstFile((demo_dir + "\\*").c_str(), &data);
    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            filesystem::path newfile(filesystem::path(demo_dir) / filesystem::path(data.cFileName));
            newfile.remove_file();
        } while (FindNextFile(hFind, &data));
        FindClose(hFind);
    }
#else
    struct dirent* entry;
    DIR* dir = opendir(demo_dir.c_str());
    while ((entry = readdir(dir)) != NULL) {
        filesystem::path(demo_dir + entry->d_name).remove_file();
    }
#endif

    std::string js = (filesystem::path(demo_dir) / filesystem::path("input.json")).str();
    std::ifstream srce(jsonFile);
    std::ofstream dest(js);
    dest << srce.rdbuf();

    std::cout << "out_dir directory= " << out_dir << std::endl;
    std::cout << "Demo Directory= " << paramsH->demo_dir << std::endl;
    std::cout << "input json file: " << jsonFile << "\n"
              << "backup json file: " << js << std::endl;
}
}  // namespace utils
}  // namespace fsi
}  // namespace chrono
