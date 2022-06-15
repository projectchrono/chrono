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
Real W3h_Spline(Real d, Real h) {
    Real invh = 1.0 / h;
    Real q = fabs(d) * invh;
    if (q < 1) {
        return (0.25f * (INVPI * invh * invh * invh) * (cube(2 - q) - 4 * cube(1 - q)));
    }
    if (q < 2) {
        return (0.25f * (INVPI * invh * invh * invh) * cube(2 - q));
    }
    return 0;
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
    num_nei = count;
    return rho0 / sum_wij;
}

void InvalidArg(std::string arg) {
    cout << "Invalid arg: " << arg << endl;
}

bool ParseJSON(const std::string& json_file, std::shared_ptr<SimParams> paramsH, Real3 Domain) {
    if (paramsH->verbose)
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

    if (doc.HasMember("Output Folder"))
        strcpy(paramsH->out_name, doc["Output Folder"].GetString());

    if (doc.HasMember("Data Output Length"))
        paramsH->output_length = doc["Data Output Length"].GetInt();

    if (doc.HasMember("Output FSI"))
        paramsH->output_fsi = doc["Output FSI"].GetBool();

    if (doc.HasMember("Physical Properties of Fluid")) {
        if (doc["Physical Properties of Fluid"].HasMember("Density")) {
            paramsH->rho0 = doc["Physical Properties of Fluid"]["Density"].GetDouble();
        }

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
            if (paramsH->verbose)
                cout << "Modeling method is: " << SPH << endl;
            if (SPH == "I2SPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::I2SPH;
            else if (SPH == "IISPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::IISPH;
            else if (SPH == "WCSPH")
                paramsH->fluid_dynamic_type = fluid_dynamics::WCSPH;
            else {
                cerr << "Incorrect SPH method in the JSON file: " << SPH << endl;
                cerr << "Falling back to I2SPH " << endl;
                paramsH->fluid_dynamic_type = fluid_dynamics::I2SPH;
            }
        }

        if (doc["SPH Parameters"].HasMember("Kernel h")) {
            paramsH->HSML = doc["SPH Parameters"]["Kernel h"].GetDouble();
        }

        if (doc["SPH Parameters"].HasMember("Initial Spacing")) {
            paramsH->INITSPACE = doc["SPH Parameters"]["Initial Spacing"].GetDouble();
        }

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

        if (doc["Time Stepping"].HasMember("Fluid time step")) {
            paramsH->dT = doc["Time Stepping"]["Fluid time step"].GetDouble();
        }

        if (doc["Time Stepping"].HasMember("Solid time step"))
            paramsH->dT_Flex = doc["Time Stepping"]["Solid time step"].GetDouble();
        else
            paramsH->dT_Flex = paramsH->dT;

        if (doc["Time Stepping"].HasMember("Maximum time step"))
            paramsH->dT_Max = doc["Time Stepping"]["Maximum time step"].GetDouble();

        if (doc["Time Stepping"].HasMember("Write frame per second"))
            paramsH->out_fps = doc["Time Stepping"]["Write frame per second"].GetDouble();

        if (doc["Time Stepping"].HasMember("End time"))
            paramsH->tFinal = doc["Time Stepping"]["End time"].GetDouble();
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

        if (doc["Pressure Equation"].HasMember("Clamp Pressure")) {
            paramsH->ClampPressure = doc["Pressure Equation"]["Clamp Pressure"].GetBool();
        }

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
        if (doc["Elastic SPH"].HasMember("Poisson ratio")) {
            paramsH->Nu_poisson = doc["Elastic SPH"]["Poisson ratio"].GetDouble();
        }
        if (doc["Elastic SPH"].HasMember("Young modulus")) {
            paramsH->E_young = doc["Elastic SPH"]["Young modulus"].GetDouble();         // Young's modulus
            paramsH->G_shear = paramsH->E_young / (2.0 * (1.0 + paramsH->Nu_poisson));  // shear modulus
            paramsH->INV_G_shear = 1.0 / paramsH->G_shear;
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
            paramsH->Fri_angle =
                doc["Elastic SPH"]["frictional angle"].GetDouble();  // frictional angle of granular material
        }
        if (doc["Elastic SPH"].HasMember("dilate angle")) {
            paramsH->Dil_angle = doc["Elastic SPH"]["dilate angle"].GetDouble();  // dilate angle of granular material
        }
        if (doc["Elastic SPH"].HasMember("cohesion coefficient")) {
            paramsH->Coh_coeff = doc["Elastic SPH"]["cohesion coefficient"].GetDouble();  // cohesion coefficient
            paramsH->Q_FA =
                6 * sin(paramsH->Fri_angle) /
                (sqrt(3) * (3 + sin(paramsH->Fri_angle)));  // material constants calculate from frictional angle
            paramsH->Q_DA =
                6 * sin(paramsH->Dil_angle) /
                (sqrt(3) * (3 + sin(paramsH->Dil_angle)));  // material constants calculate from dilate angle
            paramsH->K_FA = 6 * paramsH->Coh_coeff * cos(paramsH->Fri_angle) /
                            (sqrt(3) * (3 + sin(paramsH->Fri_angle)));  // material constants calculate from frictional
                                                                        // angle and cohesion coefficient
        }
        if (doc["Elastic SPH"].HasMember("kernel threshold"))
            paramsH->C_Wi = doc["Elastic SPH"]["kernel threshold"].GetDouble();
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
                    if (paramsH->verbose)
                        cout << "Constants of Herschel–Bulkley not found. Using default Newtonian values." << endl;
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
                // mu(I) functionality
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

    // Calculate dependent parameters
    paramsH->INVHSML = 1 / paramsH->HSML;
    paramsH->INV_INIT = 1 / paramsH->INITSPACE;
    paramsH->volume0 = cube(paramsH->INITSPACE);
    paramsH->MULT_INITSPACE = paramsH->INITSPACE / paramsH->HSML;
    paramsH->markerMass = paramsH->volume0 * paramsH->rho0;

    paramsH->INV_dT = 1 / paramsH->dT;

    paramsH->invrho0 = 1 / paramsH->rho0;
    paramsH->markerMass = paramsH->volume0 * paramsH->rho0;

    // Print information
    if (paramsH->verbose) {
        cout << "Simulation parameters" << endl;

        cout << "paramsH->num_neighbors: " << paramsH->num_neighbors << endl;
        cout << "paramsH->rho0: " << paramsH->rho0 << endl;
        cout << "paramsH->mu0: " << paramsH->mu0 << endl;
        cout << "paramsH->bodyForce3: ";
        utils::printStruct(paramsH->bodyForce3);
        cout << "paramsH->gravity: ";
        utils::printStruct(paramsH->gravity);

        cout << "paramsH->HSML: " << paramsH->HSML << endl;
        cout << "paramsH->INITSPACE: " << paramsH->INITSPACE << endl;
        cout << "paramsH->MULT_INITSPACE: " << paramsH->MULT_INITSPACE << endl;
        cout << "paramsH->NUM_BOUNDARY_LAYERS: " << paramsH->NUM_BOUNDARY_LAYERS << endl;
        cout << "paramsH->epsMinMarkersDis: " << paramsH->epsMinMarkersDis << endl;
        cout << "paramsH->markerMass: " << paramsH->markerMass << endl;
        cout << "paramsH->gradient_type: " << paramsH->gradient_type << endl;

        cout << "paramsH->v_Max: " << paramsH->v_Max << endl;
        cout << "paramsH->EPS_XSPH: " << paramsH->EPS_XSPH << endl;
        cout << "paramsH->beta_shifting: " << paramsH->beta_shifting << endl;
        cout << "paramsH->densityReinit: " << paramsH->densityReinit << endl;

        cout << "paramsH->Adaptive_time_stepping: " << paramsH->Adaptive_time_stepping << endl;
        cout << "paramsH->Co_number: " << paramsH->Co_number << endl;
        cout << "paramsH->dT: " << paramsH->dT << endl;
        cout << "paramsH->dT_Max: " << paramsH->dT_Max << endl;
        cout << "paramsH->dT_Flex: " << paramsH->dT_Flex << endl;

        cout << "paramsH->non_newtonian: " << paramsH->non_newtonian << endl;
        cout << "paramsH->granular_material: " << paramsH->granular_material << endl;
        cout << "paramsH->mu_of_I : " << (int)paramsH->mu_of_I << endl;
        cout << "paramsH->rheology_model: " << (int)paramsH->rheology_model << endl;
        cout << "paramsH->ave_diam: " << paramsH->ave_diam << endl;
        cout << "paramsH->mu_max: " << paramsH->mu_max << endl;
        cout << "paramsH->mu_fric_s: " << paramsH->mu_fric_s << endl;
        cout << "paramsH->mu_fric_2: " << paramsH->mu_fric_2 << endl;
        cout << "paramsH->mu_I0: " << paramsH->mu_I0 << endl;
        cout << "paramsH->mu_I_b: " << paramsH->mu_I_b << endl;
        cout << "paramsH->HB_k: " << paramsH->HB_k << endl;
        cout << "paramsH->HB_n: " << paramsH->HB_n << endl;
        cout << "paramsH->HB_tau0: " << paramsH->HB_tau0 << endl;

        cout << "paramsH->cMin: ";
        utils::printStruct(paramsH->cMin);
        cout << "paramsH->cMax: ";
        utils::printStruct(paramsH->cMax);

        cout << "paramsH->bceType: " << (int)paramsH->bceType << endl;
        cout << "paramsH->USE_NonIncrementalProjection : " << paramsH->USE_NonIncrementalProjection << endl;
        ////cout << "paramsH->LinearSolver: " << paramsH->LinearSolver << endl;
        cout << "paramsH->PPE_relaxation: " << paramsH->PPE_relaxation << endl;
        cout << "paramsH->Conservative_Form: " << paramsH->Conservative_Form << endl;
        cout << "paramsH->Pressure_Constraint: " << paramsH->Pressure_Constraint << endl;

        cout << "paramsH->deltaPress: ";
        utils::printStruct(paramsH->deltaPress);
        cout << "paramsH->binSize0: " << paramsH->binSize0 << endl;
        cout << "paramsH->boxDims: ";
        utils::printStruct(paramsH->boxDims);
        cout << "paramsH->gridSize: ";
        utils::printStruct(paramsH->gridSize);
    }

    return true;
}

void PrepareOutputDir(std::shared_ptr<fsi::SimParams> paramsH,
                      std::string& demo_dir,
                      std::string out_dir,
                      std::string jsonFile) {
    out_dir = filesystem::path(out_dir).str();

    if (strcmp(paramsH->out_name, "Undefined") == 0) {
        time_t rawtime;
        struct tm* timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        cout << asctime(timeinfo);
        char buffer[80];
        strftime(buffer, 80, "%F_%T", timeinfo);
        demo_dir = filesystem::path(filesystem::path(out_dir) / filesystem::path(buffer)).str();
    } else {
        demo_dir = filesystem::path(filesystem::path(out_dir) / filesystem::path(paramsH->out_name)).str();
    }

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(demo_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
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

    std::string js = (filesystem::path(demo_dir) / filesystem::path("Backup.json")).str();
    std::ifstream srce(jsonFile);
    std::ofstream dest(js);
    dest << srce.rdbuf();

    if (paramsH->verbose) {
        cout << "Output Directory: " << out_dir << endl;
        cout << "Demo Directory: " << paramsH->demo_dir << endl;
        cout << "Input JSON File: " << jsonFile << endl;
        cout << "Backup JSON File: " << js << endl;
    }
}

}  // namespace utils
}  // namespace fsi
}  // namespace chrono
