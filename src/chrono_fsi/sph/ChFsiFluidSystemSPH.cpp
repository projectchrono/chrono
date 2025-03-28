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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Implementation of an FSI-aware SPH fluid solver.
//
// =============================================================================

//// TODO:
////   - use ChFsiParamsSPH::C_Wi (kernel threshold) for both CFD and CRM (currently, only CRM)
 
// // #define DEBUG_LOG

#include <cmath>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#include "chrono_fsi/sph/physics/SphGeneral.cuh"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"
#include "chrono_fsi/sph/physics/FluidDynamics.cuh"
#include "chrono_fsi/sph/physics/BceManager.cuh"

#include "chrono_fsi/sph/math/CustomMath.cuh"

#include "chrono_fsi/sph/utils/UtilsTypeConvert.cuh"
#include "chrono_fsi/sph/utils/UtilsPrintSph.cuh"
#include "chrono_fsi/sph/utils/UtilsDevice.cuh"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {
namespace sph {

ChFsiFluidSystemSPH::ChFsiFluidSystemSPH()
    : ChFsiFluidSystem(),
      m_pattern1D(BcePatternMesh1D::FULL),
      m_pattern2D(BcePatternMesh2D::CENTERED),
      m_remove_center1D(false),
      m_remove_center2D(false),
      m_num_rigid_bodies(0),
      m_num_flex1D_nodes(0),
      m_num_flex2D_nodes(0),
      m_num_flex1D_elements(0),
      m_num_flex2D_elements(0),
      m_output_level(OutputLevel::STATE_PRESSURE),
      m_first_step(true) {
    m_paramsH = chrono_types::make_shared<ChFsiParamsSPH>();
    InitParams();

    m_data_mgr = chrono_types::make_unique<FsiDataManager>(m_paramsH);
}

ChFsiFluidSystemSPH::~ChFsiFluidSystemSPH() {}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::InitParams() {
    //// RADU TODO
    //// Provide default values for *all* parameters!

    // Fluid properties
    m_paramsH->rho0 = Real(1000.0);
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;
    m_paramsH->mu0 = Real(0.001);
    m_paramsH->bodyForce3 = mR3(0, 0, 0);
    m_paramsH->gravity = mR3(0, 0, 0);
    m_paramsH->L_Characteristic = Real(1.0);

    // SPH parameters
    m_paramsH->sph_method = SPHMethod::WCSPH;
    m_paramsH->eos_type = EosType::ISOTHERMAL;
    m_paramsH->viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
    m_paramsH->boundary_type = BoundaryType::ADAMI;
    m_paramsH->kernel_type = KernelType::CUBIC_SPLINE;
    m_paramsH->shifting_method = ShiftingMethod::XSPH;
    m_paramsH->periodic_sides = static_cast<int>(PeriodicSide::NONE);

    m_paramsH->d0 = Real(0.01);
    m_paramsH->ood0 = 1 / m_paramsH->d0;
    m_paramsH->d0_multiplier = Real(1.2);
    m_paramsH->h = m_paramsH->d0_multiplier * m_paramsH->d0;
    m_paramsH->ooh = 1 / m_paramsH->h;
    m_paramsH->h_multiplier = 2;

    m_paramsH->volume0 = cube(m_paramsH->d0);
    m_paramsH->v_Max = Real(1.0);
    m_paramsH->shifting_xsph_eps = Real(0.5);
    m_paramsH->shifting_ppst_push = Real(3.0);
    m_paramsH->shifting_ppst_pull = Real(1.0);
    m_paramsH->shifting_beta_implicit = Real(1.0);
    m_paramsH->shifting_diffusion_A = Real(1.0);
    m_paramsH->shifting_diffusion_AFSM = Real(3.0);
    m_paramsH->shifting_diffusion_AFST = Real(2);
    m_paramsH->densityReinit = 2147483647;
    m_paramsH->Conservative_Form = true;
    m_paramsH->gradient_type = 0;
    m_paramsH->laplacian_type = 0;
    m_paramsH->USE_Consistent_L = false;
    m_paramsH->USE_Consistent_G = false;

    m_paramsH->density_delta = Real(0.1);
    m_paramsH->USE_Delta_SPH = false;

    m_paramsH->epsMinMarkersDis = Real(0.01);

    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;

    m_paramsH->num_bce_layers = 3;

    m_paramsH->dT = Real(-1);

    // Pressure equation
    m_paramsH->DensityBaseProjection = false;
    m_paramsH->Alpha = m_paramsH->h;
    m_paramsH->PPE_relaxation = Real(1.0);
    m_paramsH->LinearSolver = SolverType::JACOBI;
    m_paramsH->LinearSolver_Abs_Tol = Real(0.0);
    m_paramsH->LinearSolver_Rel_Tol = Real(0.0);
    m_paramsH->LinearSolver_Max_Iter = 1000;
    m_paramsH->Verbose_monitoring = false;
    m_paramsH->Pressure_Constraint = false;
    m_paramsH->base_pressure = Real(0.0);
    m_paramsH->ClampPressure = false;

    // Elastic SPH
    m_paramsH->C_Wi = Real(0.8);

    //
    m_paramsH->bodyActiveDomain = mR3(1e10, 1e10, 1e10);
    m_paramsH->use_active_domain = false;
    m_paramsH->settlingTime = Real(0);

    //
    m_paramsH->Max_Pressure = Real(1e20);

    //// RADU TODO
    //// material model

    // Elastic SPH
    ElasticMaterialProperties mat_props;
    SetElasticSPH(mat_props);
    m_paramsH->elastic_SPH = false;        // default: fluid dynamics
    m_paramsH->Ar_vis_alpha = Real(0.02);  // Does this mess with one for CRM?

    m_paramsH->Cs = 10 * m_paramsH->v_Max;

    m_paramsH->use_default_limits = true;
    m_paramsH->use_init_pressure = false;

    m_paramsH->num_proximity_search_steps = 4;
}

//--------------------------------------------------------------------------------------------------------------------------------

Real3 LoadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return mR3(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

void ChFsiFluidSystemSPH::ReadParametersFromFile(const std::string& json_file) {
    if (m_verbose)
        cout << "Reading parameters from: " << json_file << endl;

    FILE* fp = fopen(json_file.c_str(), "r");
    if (!fp) {
        cerr << "Invalid JSON file!" << endl;
        return;
    }

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;

    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        cerr << "Invalid JSON file!!" << endl;
        return;
    }

    if (doc.HasMember("Physical Properties of Fluid")) {
        if (doc["Physical Properties of Fluid"].HasMember("Density"))
            m_paramsH->rho0 = doc["Physical Properties of Fluid"]["Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Viscosity"))
            m_paramsH->mu0 = doc["Physical Properties of Fluid"]["Viscosity"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Body Force"))
            m_paramsH->bodyForce3 = LoadVectorJSON(doc["Physical Properties of Fluid"]["Body Force"]);

        if (doc["Physical Properties of Fluid"].HasMember("Gravity"))
            m_paramsH->gravity = LoadVectorJSON(doc["Physical Properties of Fluid"]["Gravity"]);

        if (doc["Physical Properties of Fluid"].HasMember("Characteristic Length"))
            m_paramsH->L_Characteristic = doc["Physical Properties of Fluid"]["Characteristic Length"].GetDouble();
    }

    if (doc.HasMember("SPH Parameters")) {
        if (doc["SPH Parameters"].HasMember("Method")) {
            std::string SPH = doc["SPH Parameters"]["Method"].GetString();
            if (m_verbose)
                cout << "Modeling method is: " << SPH << endl;
            if (SPH == "I2SPH") {
                m_paramsH->sph_method = SPHMethod::I2SPH;
                if (doc["SPH Parameters"].HasMember("Shifting Coefficient"))
                    m_paramsH->shifting_beta_implicit = doc["SPH Parameters"]["Shifting Coefficient"].GetDouble();
            } else if (SPH == "WCSPH")
                m_paramsH->sph_method = SPHMethod::WCSPH;
            else {
                cerr << "Incorrect SPH method in the JSON file: " << SPH << endl;
                cerr << "Falling back to WCSPH " << endl;
                m_paramsH->sph_method = SPHMethod::WCSPH;
            }
        }

        if (doc["SPH Parameters"].HasMember("Initial Spacing"))
            m_paramsH->d0 = doc["SPH Parameters"]["Initial Spacing"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Kernel Multiplier"))
            m_paramsH->d0_multiplier = doc["SPH Parameters"]["Kernel Multiplier"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Epsilon"))
            m_paramsH->epsMinMarkersDis = doc["SPH Parameters"]["Epsilon"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Maximum Velocity"))
            m_paramsH->v_Max = doc["SPH Parameters"]["Maximum Velocity"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Shifting Method")) {
            std::string method = doc["SPH Parameters"]["Shifting Method"].GetString();
            if (method == "XSPH")
                m_paramsH->shifting_method = ShiftingMethod::XSPH;
            else if (method == "PPST_XSPH")
                m_paramsH->shifting_method = ShiftingMethod::PPST_XSPH;
            else if (method == "PPST")
                m_paramsH->shifting_method = ShiftingMethod::PPST;
            else if (method == "DIFFUSION")
                m_paramsH->shifting_method = ShiftingMethod::DIFFUSION;
            else if (method == "DIFFUSION_XSPH")
                m_paramsH->shifting_method = ShiftingMethod::DIFFUSION_XSPH;
            else {
                m_paramsH->shifting_method = ShiftingMethod::NONE;
            }
        }

        if (doc["SPH Parameters"].HasMember("XSPH Coefficient"))
            m_paramsH->shifting_xsph_eps = doc["SPH Parameters"]["XSPH Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("PPST Push Coefficient"))
            m_paramsH->shifting_ppst_push = doc["SPH Parameters"]["PPST Push Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("PPST Pull Coefficient"))
            m_paramsH->shifting_ppst_pull = doc["SPH Parameters"]["PPST Pull Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Diffusion A Coefficient"))
            m_paramsH->shifting_diffusion_A = doc["SPH Parameters"]["Diffusion A Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Diffusion AFSM"))
            m_paramsH->shifting_diffusion_AFSM = doc["SPH Parameters"]["Diffusion AFSM"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Diffusion AFST"))
            m_paramsH->shifting_diffusion_AFST = doc["SPH Parameters"]["Diffusion AFST"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Kernel Type")) {
            std::string type = doc["SPH Parameters"]["Kernel Type"].GetString();
            if (type == "Quadratic")
                m_paramsH->kernel_type = KernelType::QUADRATIC;
            else if (type == "Cubic")
                m_paramsH->kernel_type = KernelType::CUBIC_SPLINE;
            else if (type == "Quintic")
                m_paramsH->kernel_type = KernelType::QUINTIC_SPLINE;
            else if (type == "Wendland")
                m_paramsH->kernel_type = KernelType::WENDLAND;
            else {
                cerr << "Incorrect kernel type in the JSON file: " << type << endl;
                cerr << "Falling back to cubic spline." << endl;
                m_paramsH->kernel_type = KernelType::CUBIC_SPLINE;
            }
        }

        if (doc["SPH Parameters"].HasMember("Boundary Treatment Type")) {
            std::string type = doc["SPH Parameters"]["Boundary Treatment Type"].GetString();
            if (type == "Adami")
                m_paramsH->boundary_type = BoundaryType::ADAMI;
            else if (type == "Holmes")
                m_paramsH->boundary_type = BoundaryType::HOLMES;
            else {
                cerr << "Incorrect boundary treatment type in the JSON file: " << type << endl;
                cerr << "Falling back to Adami " << endl;
                m_paramsH->boundary_type = BoundaryType::ADAMI;
            }
        }

        if (doc["SPH Parameters"].HasMember("Viscosity Treatment Type")) {
            std::string type = doc["SPH Parameters"]["Viscosity Treatment Type"].GetString();
            if (m_verbose)
                cout << "viscosity treatment is : " << type << endl;
            if (type == "Laminar")
                m_paramsH->viscosity_type = ViscosityType::LAMINAR;
            else if (type == "Artificial Unilateral") {
                m_paramsH->viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
            } else if (type == "Artificial Bilateral") {
                m_paramsH->viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
            } else {
                cerr << "Incorrect viscosity type in the JSON file: " << type << endl;
                cerr << "Falling back to Artificial Unilateral Viscosity" << endl;
                m_paramsH->viscosity_type = ViscosityType::ARTIFICIAL_UNILATERAL;
            }
        }

        if (doc["SPH Parameters"].HasMember("Artificial viscosity alpha"))
            m_paramsH->Ar_vis_alpha = doc["SPH Parameters"]["Artificial viscosity alpha"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Use Delta SPH"))
            m_paramsH->USE_Delta_SPH = doc["SPH Parameters"]["Use Delta SPH"].GetBool();

        if (doc["SPH Parameters"].HasMember("density diffusion delta"))
            m_paramsH->density_delta = doc["SPH Parameters"]["density diffusion delta"].GetDouble();

        if (doc["SPH Parameters"].HasMember("EOS Type")) {
            std::string type = doc["SPH Parameters"]["EOS Type"].GetString();
            if (m_verbose)
                cout << "Eos type is : " << type << endl;
            if (type == "Tait")
                m_paramsH->eos_type = EosType::TAIT;
            else if (type == "Isothermal")
                m_paramsH->eos_type = EosType::ISOTHERMAL;
            else {
                cerr << "Incorrect eos type in the JSON file: " << type << endl;
                cerr << "Falling back to Tait Equation of State " << endl;
                m_paramsH->eos_type = EosType::ISOTHERMAL;
            }
        }

        if (doc["SPH Parameters"].HasMember("Density Reinitialization"))
            m_paramsH->densityReinit = doc["SPH Parameters"]["Density Reinitialization"].GetInt();

        if (doc["SPH Parameters"].HasMember("Conservative Discretization"))
            m_paramsH->Conservative_Form = doc["SPH Parameters"]["Conservative Discretization"].GetBool();

        if (doc["SPH Parameters"].HasMember("Gradient Discretization Type"))
            m_paramsH->gradient_type = doc["SPH Parameters"]["Gradient Discretization Type"].GetInt();

        if (doc["SPH Parameters"].HasMember("Laplacian Discretization Type"))
            m_paramsH->laplacian_type = doc["SPH Parameters"]["Laplacian Discretization Type"].GetInt();

        if (doc["SPH Parameters"].HasMember("Consistent Discretization for Laplacian"))
            m_paramsH->USE_Consistent_L = doc["SPH Parameters"]["Consistent Discretization for Laplacian"].GetBool();

        if (doc["SPH Parameters"].HasMember("Consistent Discretization for Gradient"))
            m_paramsH->USE_Consistent_G = doc["SPH Parameters"]["Consistent Discretization for Gradient"].GetBool();

        if (doc["SPH Parameters"].HasMember("Time steps per proximity search"))
            m_paramsH->num_proximity_search_steps = doc["SPH Parameters"]["Time steps per proximity search"].GetInt();
    }

    if (doc.HasMember("Time Stepping")) {
        if (doc["Time Stepping"].HasMember("Time step")) {
            m_paramsH->dT = doc["Time Stepping"]["Time step"].GetDouble();
            m_step = m_paramsH->dT;
        }
    }

    if (doc.HasMember("Pressure Equation")) {
        if (doc["Pressure Equation"].HasMember("Linear solver")) {
            std::string solver = doc["Pressure Equation"]["Linear solver"].GetString();
            if (solver == "Jacobi") {
                m_paramsH->LinearSolver = SolverType::JACOBI;
            } else if (solver == "BICGSTAB") {
                m_paramsH->LinearSolver = SolverType::BICGSTAB;
            } else if (solver == "GMRES")
                m_paramsH->LinearSolver = SolverType::GMRES;
        } else {
            m_paramsH->LinearSolver = SolverType::JACOBI;
        }

        if (doc["Pressure Equation"].HasMember("Poisson source term")) {
            std::string source = doc["Pressure Equation"]["Poisson source term"].GetString();
            if (source == "Density-Based")
                m_paramsH->DensityBaseProjection = true;
            else
                m_paramsH->DensityBaseProjection = false;
        }

        if (doc["Pressure Equation"].HasMember("Alpha Source Term"))
            m_paramsH->Alpha = doc["Pressure Equation"]["Alpha Source Term"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Under-relaxation"))
            m_paramsH->PPE_relaxation = doc["Pressure Equation"]["Under-relaxation"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Absolute residual"))
            m_paramsH->LinearSolver_Abs_Tol = doc["Pressure Equation"]["Absolute residual"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Relative residual"))
            m_paramsH->LinearSolver_Rel_Tol = doc["Pressure Equation"]["Relative residual"].GetDouble();

        if (doc["Pressure Equation"].HasMember("Maximum Iterations"))
            m_paramsH->LinearSolver_Max_Iter = doc["Pressure Equation"]["Maximum Iterations"].GetInt();

        if (doc["Pressure Equation"].HasMember("Verbose monitoring"))
            m_paramsH->Verbose_monitoring = doc["Pressure Equation"]["Verbose monitoring"].GetBool();

        if (doc["Pressure Equation"].HasMember("Constraint Pressure")) {
            m_paramsH->Pressure_Constraint = doc["Pressure Equation"]["Constraint Pressure"].GetBool();
            if (doc["Pressure Equation"].HasMember("Average Pressure"))
                m_paramsH->base_pressure = doc["Pressure Equation"]["Average Pressure"].GetDouble();
        }

        if (doc["Pressure Equation"].HasMember("Clamp Pressure"))
            m_paramsH->ClampPressure = doc["Pressure Equation"]["Clamp Pressure"].GetBool();
    }

    // this part is for modeling granular material dynamics using elastic SPH
    if (doc.HasMember("Elastic SPH")) {
        m_paramsH->elastic_SPH = true;

        if (doc["Elastic SPH"].HasMember("Poisson ratio"))
            m_paramsH->Nu_poisson = doc["Elastic SPH"]["Poisson ratio"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Young modulus"))
            m_paramsH->E_young = doc["Elastic SPH"]["Young modulus"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity alpha"))
            m_paramsH->Ar_vis_alpha = doc["Elastic SPH"]["Artificial viscosity alpha"].GetDouble();

        if (doc["Elastic SPH"].HasMember("I0"))
            m_paramsH->mu_I0 = doc["Elastic SPH"]["I0"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_s"))
            m_paramsH->mu_fric_s = doc["Elastic SPH"]["mu_s"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_2"))
            m_paramsH->mu_fric_2 = doc["Elastic SPH"]["mu_2"].GetDouble();

        if (doc["Elastic SPH"].HasMember("particle diameter"))
            m_paramsH->ave_diam = doc["Elastic SPH"]["particle diameter"].GetDouble();

        if (doc["Elastic SPH"].HasMember("cohesion coefficient"))
            m_paramsH->Coh_coeff = doc["Elastic SPH"]["cohesion coefficient"].GetDouble();

        if (doc["Elastic SPH"].HasMember("kernel threshold"))
            m_paramsH->C_Wi = doc["Elastic SPH"]["kernel threshold"].GetDouble();
    }

    // Geometry Information
    if (doc.HasMember("Geometry Inf")) {
        if (doc["Geometry Inf"].HasMember("BoxDimensionX"))
            m_paramsH->boxDimX = doc["Geometry Inf"]["BoxDimensionX"].GetDouble();

        if (doc["Geometry Inf"].HasMember("BoxDimensionY"))
            m_paramsH->boxDimY = doc["Geometry Inf"]["BoxDimensionY"].GetDouble();

        if (doc["Geometry Inf"].HasMember("BoxDimensionZ"))
            m_paramsH->boxDimZ = doc["Geometry Inf"]["BoxDimensionZ"].GetDouble();
    }

    if (doc.HasMember("Body Active Domain")){
        m_paramsH->use_active_domain = true;
        m_paramsH->bodyActiveDomain = LoadVectorJSON(doc["Body Active Domain"]);
    }

    if (doc.HasMember("Settling Time"))
        m_paramsH->settlingTime = doc["Settling Time"].GetDouble();

    //===============================================================
    // Material Models
    //===============================================================
    if (doc.HasMember("Material Model")) {
        m_paramsH->non_newtonian = doc["Material Model"]["Non-Newtonian"].GetBool();
        //===============================================================
        // For a simple non-newtonian flow
        //==============================================================
        if (m_paramsH->non_newtonian) {
            m_paramsH->mu_max = doc["Material Model"]["max Viscosity"].GetDouble();

            if (m_paramsH->non_newtonian) {
                if (doc["Material Model"].HasMember("HerschelBulkley")) {
                    m_paramsH->HB_k = doc["Material Model"]["HerschelBulkley"]["k"].GetDouble();
                    m_paramsH->HB_n = doc["Material Model"]["HerschelBulkley"]["n"].GetInt();
                    m_paramsH->HB_tau0 = doc["Material Model"]["HerschelBulkley"]["tau_0"].GetDouble();
                    if (doc["Material Model"]["HerschelBulkley"].HasMember("sr0"))
                        m_paramsH->HB_sr0 = doc["Material Model"]["HerschelBulkley"]["sr0"].GetDouble();
                    else
                        m_paramsH->HB_sr0 = 0.0;
                } else {
                    if (m_verbose)
                        cout << "Constants of HerschelBulkley not found. Using default Newtonian values." << endl;
                    m_paramsH->HB_k = m_paramsH->mu0;
                    m_paramsH->HB_n = 1;
                    m_paramsH->HB_tau0 = 0;
                    m_paramsH->HB_sr0 = 0.0;
                }
            }
        }
    } else {
        m_paramsH->non_newtonian = false;
    }

    // Calculate dependent parameters
    m_paramsH->ood0 = 1 / m_paramsH->d0;
    m_paramsH->h = m_paramsH->d0_multiplier * m_paramsH->d0;
    m_paramsH->ooh = 1 / m_paramsH->h;
    m_paramsH->volume0 = cube(m_paramsH->d0);
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;

    if (m_paramsH->elastic_SPH) {
        m_paramsH->G_shear = m_paramsH->E_young / (2.0 * (1.0 + m_paramsH->Nu_poisson));
        m_paramsH->INV_G_shear = 1.0 / m_paramsH->G_shear;
        m_paramsH->K_bulk = m_paramsH->E_young / (3.0 * (1.0 - 2.0 * m_paramsH->Nu_poisson));
        m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);
    } else {
        m_paramsH->Cs = 10 * m_paramsH->v_Max;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::SetBoundaryType(BoundaryType boundary_type) {
    m_paramsH->boundary_type = boundary_type;
}

void ChFsiFluidSystemSPH::SetViscosityType(ViscosityType viscosity_type) {
    m_paramsH->viscosity_type = viscosity_type;
}

void ChFsiFluidSystemSPH::SetArtificialViscosityCoefficient(double coefficient) {
    m_paramsH->Ar_vis_alpha = coefficient;
}

void ChFsiFluidSystemSPH::SetKernelType(KernelType kernel_type) {
    m_paramsH->kernel_type = kernel_type;
    switch (m_paramsH->kernel_type) {
        case KernelType::QUADRATIC:
            m_paramsH->h_multiplier = 2;
            break;
        case KernelType::CUBIC_SPLINE:
            m_paramsH->h_multiplier = 2;
            break;
        case KernelType::QUINTIC_SPLINE:
            m_paramsH->h_multiplier = 3;
            break;
        case KernelType::WENDLAND:
            m_paramsH->h_multiplier = 2;
            break;
    }
}

void ChFsiFluidSystemSPH::SetShiftingMethod(ShiftingMethod shifting_method) {
    m_paramsH->shifting_method = shifting_method;
}

void ChFsiFluidSystemSPH::SetSPHLinearSolver(SolverType lin_solver) {
    m_paramsH->LinearSolver = lin_solver;
}

void ChFsiFluidSystemSPH::SetSPHMethod(SPHMethod SPH_method) {
    m_paramsH->sph_method = SPH_method;
}

void ChFsiFluidSystemSPH::SetContainerDim(const ChVector3d& boxDim) {
    m_paramsH->boxDimX = boxDim.x();
    m_paramsH->boxDimY = boxDim.y();
    m_paramsH->boxDimZ = boxDim.z();
}

void ChFsiFluidSystemSPH::SetComputationalBoundaries(const ChVector3d& cMin,
                                                     const ChVector3d& cMax,
                                                     int periodic_sides) {
    m_paramsH->cMin = ToReal3(cMin);
    m_paramsH->cMax = ToReal3(cMax);
    m_paramsH->use_default_limits = false;
    m_paramsH->periodic_sides = periodic_sides;
}

void ChFsiFluidSystemSPH::SetActiveDomain(const ChVector3d& boxHalfDim) {
    m_paramsH->bodyActiveDomain = ToReal3(boxHalfDim);
    m_paramsH->use_active_domain = true;
}

void ChFsiFluidSystemSPH::SetActiveDomainDelay(double duration) {
    m_paramsH->settlingTime = duration;
}

void ChFsiFluidSystemSPH::SetNumBCELayers(int num_layers) {
    m_paramsH->num_bce_layers = num_layers;
}

void ChFsiFluidSystemSPH::SetInitPressure(const double height) {
    m_paramsH->pressure_height = height;
    m_paramsH->use_init_pressure = true;
}

void ChFsiFluidSystemSPH::SetGravitationalAcceleration(const ChVector3d& gravity) {
    m_paramsH->gravity.x = gravity.x();
    m_paramsH->gravity.y = gravity.y();
    m_paramsH->gravity.z = gravity.z();
}

void ChFsiFluidSystemSPH::SetBodyForce(const ChVector3d& force) {
    m_paramsH->bodyForce3.x = force.x();
    m_paramsH->bodyForce3.y = force.y();
    m_paramsH->bodyForce3.z = force.z();
}

void ChFsiFluidSystemSPH::SetInitialSpacing(double spacing) {
    m_paramsH->d0 = (Real)spacing;
    m_paramsH->ood0 = 1 / m_paramsH->d0;
    m_paramsH->volume0 = cube(m_paramsH->d0);
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;

    m_paramsH->h = m_paramsH->d0_multiplier * m_paramsH->d0;
    m_paramsH->ooh = 1 / m_paramsH->h;
}

void ChFsiFluidSystemSPH::SetKernelMultiplier(double multiplier) {
    m_paramsH->d0_multiplier = Real(multiplier);

    m_paramsH->h = m_paramsH->d0_multiplier * m_paramsH->d0;
    m_paramsH->ooh = 1 / m_paramsH->h;
}

void ChFsiFluidSystemSPH::SetDensity(double rho0) {
    m_paramsH->rho0 = rho0;
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
}

void ChFsiFluidSystemSPH::SetShiftingPPSTParameters(double push, double pull) {
    m_paramsH->shifting_ppst_push = push;
    m_paramsH->shifting_ppst_pull = pull;
}

void ChFsiFluidSystemSPH::SetShiftingXSPHParameters(double eps) {
    m_paramsH->shifting_xsph_eps = eps;
}

void ChFsiFluidSystemSPH::SetShiftingDiffusionParameters(double A, double AFSM, double AFST) {
    m_paramsH->shifting_diffusion_A = A;
    m_paramsH->shifting_diffusion_AFSM = AFSM;
    m_paramsH->shifting_diffusion_AFST = AFST;
}

void ChFsiFluidSystemSPH::SetConsistentDerivativeDiscretization(bool consistent_gradient, bool consistent_Laplacian) {
    m_paramsH->USE_Consistent_G = consistent_gradient;
    m_paramsH->USE_Consistent_L = consistent_Laplacian;
}

void ChFsiFluidSystemSPH::SetOutputLevel(OutputLevel output_level) {
    m_output_level = output_level;
}

void ChFsiFluidSystemSPH::SetCohesionForce(double Fc) {
    m_paramsH->Coh_coeff = Fc;
}

void ChFsiFluidSystemSPH::SetNumProximitySearchSteps(int steps) {
    m_paramsH->num_proximity_search_steps = steps;
}

void ChFsiFluidSystemSPH::CheckSPHParameters() {
    // Check parameter compatibility with physics problem
    if (m_paramsH->elastic_SPH) {
        if (m_paramsH->sph_method != SPHMethod::WCSPH) {
            cerr << "ERROR: Only WCSPH can be used for granular CRM problems." << endl;
            throw std::runtime_error("ISPH not supported for granular CRM problems.");
        }
        if (m_paramsH->non_newtonian) {
            cerr << "ERROR: Non-Newtonian viscosity model is not supported for granular CRM." << endl;
            throw std::runtime_error("Non-Newtonian viscosity model is not supported for granular CRM.");
        }
        if (m_paramsH->viscosity_type == ViscosityType::LAMINAR) {
            cerr << "ERROR: Viscosity type LAMINAR not supported for CRM granular. "
                    " Use ARTIFICIAL_UNILATERAL or ARTIFICIAL_BILATERAL."
                 << endl;
            throw std::runtime_error("Viscosity type LAMINAR not supported for CRM granular.");
        }
        if (m_paramsH->viscosity_type == ViscosityType::ARTIFICIAL_UNILATERAL) {
            cerr << "WARNING: Viscosity type ARTIFICIAL_UNILATERAL may be less stable for CRM granular. "
                    "Consider using ARTIFICIAL_BILATERAL or ensure the step size is small enough."
                 << endl;
        }
        if (m_paramsH->mu0 > Real(0.001)) {
            cerr << "WARNING: The Laminar viscosity parameter has been set to " << m_paramsH->mu0
                 << " but the viscosity model is not laminar. This parameter will have no influence on simulation."
                 << endl;
        }
    } else {
        if (m_paramsH->viscosity_type == ViscosityType::ARTIFICIAL_BILATERAL) {
            cerr << "ERROR: Viscosity type ARTIFICIAL_BILATERAL not supported for CFD. "
                    " Use ARTIFICIAL_UNILATERAL or LAMINAR."
                 << endl;
            throw std::runtime_error("Viscosity type ARTIFICIAL_BILATERAL not supported for CFD.");
        }
    }

    // Calculate default cMin and cMax
    Real3 default_cMin =
        mR3(-2 * m_paramsH->boxDims.x, -2 * m_paramsH->boxDims.y, -2 * m_paramsH->boxDims.z) - 10 * mR3(m_paramsH->h);
    Real3 default_cMax =
        mR3(+2 * m_paramsH->boxDims.x, +2 * m_paramsH->boxDims.y, +2 * m_paramsH->boxDims.z) + 10 * mR3(m_paramsH->h);

    // Check if user-defined cMin and cMax are much larger than defaults
    if (m_paramsH->cMin.x < 2 * default_cMin.x || m_paramsH->cMin.y < 2 * default_cMin.y ||
        m_paramsH->cMin.z < 2 * default_cMin.z || m_paramsH->cMax.x > 2 * default_cMax.x ||
        m_paramsH->cMax.y > 2 * default_cMax.y || m_paramsH->cMax.z > 2 * default_cMax.z) {
        cerr << "WARNING: User-defined cMin or cMax is much larger than the default values. "
             << "This may slow down the simulation." << endl;
    }

    // TODO: Add check for whether computational domain is larger than SPH + BCE layers
    if (m_paramsH->d0_multiplier < 1) {
        cerr << "WARNING: Kernel interaction length multiplier is less than 1. This may lead to numerical "
                "instability due to poor particle approximation."
             << endl;
    }

    if (m_paramsH->kernel_type == KernelType::CUBIC_SPLINE && m_paramsH->d0_multiplier > 1.5) {
        // Check if W3h is defined as W3h_CubicSpline
        cerr << "WARNING: Kernel interaction radius multiplier is greater than 1.5 and the cubic spline kernel is "
                "used. This may lead to pairing instability. See Pg 10. of Ha H.Bui et al. Smoothed particle "
                "hydrodynamics (SPH) and its applications in geomechanics : From solid fracture to granular "
                "behaviour and multiphase flows in porous media. You might want to switch to the Wendland kernel."
             << endl;
    }

    if (m_paramsH->num_bce_layers < 3) {
        cerr << "WARNING: Number of BCE layers is less than 3. This may cause insufficient kernel support at the "
                "boundaries and lead to leakage of particles"
             << endl;
    }

    // Check shifting method and whether defaults have changed
    if (m_paramsH->shifting_method == ShiftingMethod::NONE) {
        if (m_paramsH->shifting_xsph_eps != 0.5 || m_paramsH->shifting_ppst_push != 1.0 ||
            m_paramsH->shifting_ppst_pull != 1.0) {
            cerr << "WARNING: Shifting method is NONE, but shifting parameters have been modified. These "
                    "changes will not take effect."
                 << endl;
        }
    } else if (m_paramsH->shifting_method == ShiftingMethod::XSPH) {
        if (m_paramsH->shifting_ppst_pull != 1.0 || m_paramsH->shifting_ppst_push != 3.0) {
            cerr << "WARNING: Shifting method is XSPH, but PPST shifting parameters have been modified. These "
                    "changes will not take effect."
                 << endl;
        }
    } else if (m_paramsH->shifting_method == ShiftingMethod::PPST) {
        if (m_paramsH->shifting_xsph_eps != 0.5) {
            cerr << "WARNING: Shifting method is PPST, but XSPH shifting parameters have been modified. These "
                    "changes will not take effect."
                 << endl;
        }
    }
}

ChFsiFluidSystemSPH::FluidProperties::FluidProperties() : density(1000), viscosity(0.1), char_length(1) {}

void ChFsiFluidSystemSPH::SetCfdSPH(const FluidProperties& fluid_props) {
    m_paramsH->elastic_SPH = false;

    SetDensity(fluid_props.density);

    m_paramsH->mu0 = Real(fluid_props.viscosity);
    m_paramsH->L_Characteristic = Real(fluid_props.char_length);
}

ChFsiFluidSystemSPH::ElasticMaterialProperties::ElasticMaterialProperties()
    : density(1000),
      Young_modulus(1e6),
      Poisson_ratio(0.3),
      mu_I0(0.03),
      mu_fric_s(0.7),
      mu_fric_2(0.7),
      average_diam(0.005),
      cohesion_coeff(0) {}

void ChFsiFluidSystemSPH::SetElasticSPH(const ElasticMaterialProperties& mat_props) {
    m_paramsH->elastic_SPH = true;

    SetDensity(mat_props.density);

    m_paramsH->E_young = Real(mat_props.Young_modulus);
    m_paramsH->Nu_poisson = Real(mat_props.Poisson_ratio);
    m_paramsH->mu_I0 = Real(mat_props.mu_I0);
    m_paramsH->mu_fric_s = Real(mat_props.mu_fric_s);
    m_paramsH->mu_fric_2 = Real(mat_props.mu_fric_2);
    m_paramsH->ave_diam = Real(mat_props.average_diam);
    m_paramsH->Coh_coeff = Real(mat_props.cohesion_coeff);

    m_paramsH->G_shear = m_paramsH->E_young / (2.0 * (1.0 + m_paramsH->Nu_poisson));
    m_paramsH->INV_G_shear = 1.0 / m_paramsH->G_shear;
    m_paramsH->K_bulk = m_paramsH->E_young / (3.0 * (1.0 - 2.0 * m_paramsH->Nu_poisson));
    m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);
}

ChFsiFluidSystemSPH::SPHParameters::SPHParameters()
    : sph_method(SPHMethod::WCSPH),
      initial_spacing(0.01),
      d0_multiplier(1.2),
      max_velocity(1.0),
      shifting_xsph_eps(0.5),
      shifting_ppst_push(3.0),
      shifting_ppst_pull(1.0),
      shifting_diffusion_A(1.0),
      shifting_diffusion_AFSM(3.0),
      shifting_diffusion_AFST(2),
      min_distance_coefficient(0.01),
      density_reinit_steps(2e8),
      use_density_based_projection(false),
      num_bce_layers(3),
      consistent_gradient_discretization(false),
      consistent_laplacian_discretization(false),
      viscosity_type(ViscosityType::ARTIFICIAL_UNILATERAL),
      boundary_type(BoundaryType::ADAMI),
      kernel_type(KernelType::CUBIC_SPLINE),
      use_delta_sph(true),
      delta_sph_coefficient(0.1),
      artificial_viscosity(0.02),
      kernel_threshold(0.8),
      num_proximity_search_steps(4),
      eos_type(EosType::ISOTHERMAL) {}

void ChFsiFluidSystemSPH::SetSPHParameters(const SPHParameters& sph_params) {
    m_paramsH->sph_method = sph_params.sph_method;

    m_paramsH->eos_type = sph_params.eos_type;
    m_paramsH->viscosity_type = sph_params.viscosity_type;
    m_paramsH->boundary_type = sph_params.boundary_type;
    m_paramsH->kernel_type = sph_params.kernel_type;
    m_paramsH->shifting_method = sph_params.shifting_method;

    m_paramsH->d0 = sph_params.initial_spacing;
    m_paramsH->volume0 = cube(m_paramsH->d0);
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
    m_paramsH->d0_multiplier = sph_params.d0_multiplier;
    m_paramsH->h = m_paramsH->d0_multiplier * m_paramsH->d0;
    m_paramsH->ood0 = 1 / m_paramsH->d0;
    m_paramsH->ooh = 1 / m_paramsH->h;

    m_paramsH->v_Max = sph_params.max_velocity;
    m_paramsH->Cs = 10 * m_paramsH->v_Max;
    m_paramsH->shifting_xsph_eps = sph_params.shifting_xsph_eps;
    m_paramsH->shifting_ppst_push = sph_params.shifting_ppst_push;
    m_paramsH->shifting_ppst_pull = sph_params.shifting_ppst_pull;
    m_paramsH->shifting_beta_implicit = sph_params.shifting_beta_implicit;
    m_paramsH->shifting_diffusion_A = sph_params.shifting_diffusion_A;
    m_paramsH->shifting_diffusion_AFSM = sph_params.shifting_diffusion_AFSM;
    m_paramsH->shifting_diffusion_AFST = sph_params.shifting_diffusion_AFST;
    m_paramsH->epsMinMarkersDis = sph_params.min_distance_coefficient;

    m_paramsH->densityReinit = sph_params.density_reinit_steps;
    m_paramsH->DensityBaseProjection = sph_params.use_density_based_projection;

    m_paramsH->num_bce_layers = sph_params.num_bce_layers;

    m_paramsH->USE_Consistent_G = sph_params.consistent_gradient_discretization;
    m_paramsH->USE_Consistent_L = sph_params.consistent_laplacian_discretization;
    m_paramsH->Ar_vis_alpha = sph_params.artificial_viscosity;
    m_paramsH->USE_Delta_SPH = sph_params.use_delta_sph;
    m_paramsH->density_delta = sph_params.delta_sph_coefficient;

    m_paramsH->C_Wi = Real(sph_params.kernel_threshold);

    m_paramsH->num_proximity_search_steps = sph_params.num_proximity_search_steps;
}

ChFsiFluidSystemSPH::LinSolverParameters::LinSolverParameters()
    : type(SolverType::JACOBI), atol(0.0), rtol(0.0), max_num_iters(1000) {}

void ChFsiFluidSystemSPH::SetLinSolverParameters(const LinSolverParameters& linsolv_params) {
    m_paramsH->LinearSolver = linsolv_params.type;
    m_paramsH->LinearSolver_Abs_Tol = linsolv_params.atol;
    m_paramsH->LinearSolver_Rel_Tol = linsolv_params.rtol;
    m_paramsH->LinearSolver_Max_Iter = linsolv_params.max_num_iters;
}

//--------------------------------------------------------------------------------------------------------------------------------

PhysicsProblem ChFsiFluidSystemSPH::GetPhysicsProblem() const {
    return (m_paramsH->elastic_SPH ? PhysicsProblem::CRM : PhysicsProblem::CFD);
}

std::string ChFsiFluidSystemSPH::GetPhysicsProblemString() const {
    return (m_paramsH->elastic_SPH ? "CRM" : "CFD");
}

std::string ChFsiFluidSystemSPH::GetSphMethodTypeString() const {
    std::string method = "";
    switch (m_paramsH->sph_method) {
        case SPHMethod::WCSPH:
            method = "WCSPH";
            break;
        case SPHMethod::I2SPH:
            method = "I2SPH";
            break;
    }

    return method;
}

//--------------------------------------------------------------------------------------------------------------------------------

// Convert host data from the provided SOA to the data manager's AOS and copy to device.
void ChFsiFluidSystemSPH::LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                          const std::vector<FsiMeshState>& mesh1D_states,
                                          const std::vector<FsiMeshState>& mesh2D_states) {
    {
        size_t num_bodies = body_states.size();
        for (size_t i = 0; i < num_bodies; i++) {
            m_data_mgr->fsiBodyState_H->pos[i] = ToReal3(body_states[i].pos);
            m_data_mgr->fsiBodyState_H->lin_vel[i] = ToReal3(body_states[i].lin_vel);
            m_data_mgr->fsiBodyState_H->lin_acc[i] = ToReal3(body_states[i].lin_acc);
            m_data_mgr->fsiBodyState_H->rot[i] = ToReal4(body_states[i].rot);
            m_data_mgr->fsiBodyState_H->ang_vel[i] = ToReal3(body_states[i].lin_acc);
            m_data_mgr->fsiBodyState_H->ang_acc[i] = ToReal3(body_states[i].ang_acc);
        }

        if (num_bodies > 0)
            m_data_mgr->fsiBodyState_D->CopyFromH(*m_data_mgr->fsiBodyState_H);
    }

    {
        size_t num_meshes = mesh1D_states.size();
        int index = 0;
        for (size_t imesh = 0; imesh < num_meshes; imesh++) {
            size_t num_nodes = mesh1D_states[imesh].pos.size();
            for (size_t inode = 0; inode < num_nodes; inode++) {
                m_data_mgr->fsiMesh1DState_H->pos[index] = ToReal3(mesh1D_states[imesh].pos[inode]);
                m_data_mgr->fsiMesh1DState_H->vel[index] = ToReal3(mesh1D_states[imesh].vel[inode]);
                m_data_mgr->fsiMesh1DState_H->acc[index] = ToReal3(mesh1D_states[imesh].acc[inode]);
                if (mesh1D_states[imesh].has_node_directions)
                    m_data_mgr->fsiMesh1DState_H->dir[index] = ToReal3(mesh1D_states[imesh].dir[inode]);

                index++;
            }
        }

        if (num_meshes > 0) {
            ChDebugLog(" mesh1D | host size: " << m_data_mgr->fsiMesh1DState_H->dir.size()
                                               << " device size: " << m_data_mgr->fsiMesh1DState_D->dir.size());
            m_data_mgr->fsiMesh1DState_D->CopyFromH(*m_data_mgr->fsiMesh1DState_H);
            m_data_mgr->fsiMesh1DState_D->CopyDirectionsFromH(*m_data_mgr->fsiMesh1DState_H);
        }
    }

    {
        size_t num_meshes = mesh2D_states.size();
        int index = 0;
        for (size_t imesh = 0; imesh < num_meshes; imesh++) {
            size_t num_nodes = mesh2D_states[imesh].pos.size();
            for (size_t inode = 0; inode < num_nodes; inode++) {
                m_data_mgr->fsiMesh2DState_H->pos[index] = ToReal3(mesh2D_states[imesh].pos[inode]);
                m_data_mgr->fsiMesh2DState_H->vel[index] = ToReal3(mesh2D_states[imesh].vel[inode]);
                m_data_mgr->fsiMesh2DState_H->acc[index] = ToReal3(mesh2D_states[imesh].acc[inode]);
                if (mesh2D_states[imesh].has_node_directions)
                    m_data_mgr->fsiMesh2DState_H->dir[index] = ToReal3(mesh2D_states[imesh].dir[inode]);

                index++;
            }
        }

        if (num_meshes > 0) {
            ChDebugLog(" mesh2D | host size: " << m_data_mgr->fsiMesh2DState_H->dir.size()
                                               << " device size: " << m_data_mgr->fsiMesh2DState_D->dir.size());
            m_data_mgr->fsiMesh2DState_D->CopyFromH(*m_data_mgr->fsiMesh2DState_H);
            m_data_mgr->fsiMesh2DState_D->CopyDirectionsFromH(*m_data_mgr->fsiMesh2DState_H);
        }
    }
}

// Copy from device and convert host data from the data manager's AOS to the output SOA
void ChFsiFluidSystemSPH::StoreSolidForces(std::vector<FsiBodyForce> body_forces,
                                           std::vector<FsiMeshForce> mesh1D_forces,
                                           std::vector<FsiMeshForce> mesh2D_forces) {
    {
        auto forcesH = m_data_mgr->GetRigidForces();
        auto torquesH = m_data_mgr->GetRigidTorques();

        size_t num_bodies = body_forces.size();
        for (size_t i = 0; i < num_bodies; i++) {
            body_forces[i].force = ToChVector(forcesH[i]);
            body_forces[i].torque = ToChVector(torquesH[i]);
        }
    }

    {
        auto forces_H = m_data_mgr->GetFlex1dForces();

        size_t num_meshes = mesh1D_forces.size();
        int index = 0;
        for (size_t imesh = 0; imesh < num_meshes; imesh++) {
            size_t num_nodes = mesh1D_forces[imesh].force.size();
            for (size_t inode = 0; inode < num_nodes; inode++) {
                mesh1D_forces[imesh].force[inode] = ToChVector(forces_H[index]);
                index++;
            }
        }
    }

    {
        auto forces_H = m_data_mgr->GetFlex2dForces();

        size_t num_meshes = mesh2D_forces.size();
        int index = 0;
        for (size_t imesh = 0; imesh < num_meshes; imesh++) {
            size_t num_nodes = mesh2D_forces[imesh].force.size();
            for (size_t inode = 0; inode < num_nodes; inode++) {
                mesh2D_forces[imesh].force[inode] = ToChVector(forces_H[index]);
                index++;
            }
        }
    }
}

void ChFsiFluidSystemSPH::OnAddFsiBody(unsigned int index, FsiBody& fsi_body) {
    m_num_rigid_bodies++;
}

void ChFsiFluidSystemSPH::SetBcePattern1D(BcePatternMesh1D pattern, bool remove_center) {
    m_pattern1D = pattern;
    m_remove_center1D = remove_center;
}

void ChFsiFluidSystemSPH::SetBcePattern2D(BcePatternMesh2D pattern, bool remove_center) {
    m_pattern2D = pattern;
    m_remove_center2D = remove_center;
}

void ChFsiFluidSystemSPH::OnAddFsiMesh1D(unsigned int index, FsiMesh1D& fsi_mesh) {
    // Load index-based mesh connectivity (append to global list of 1-D flex segments)
    for (const auto& seg : fsi_mesh.contact_surface->GetSegmentsXYZ()) {
        auto node0_index = m_num_flex1D_nodes + fsi_mesh.ptr2ind_map[seg->GetNode(0)];
        auto node1_index = m_num_flex1D_nodes + fsi_mesh.ptr2ind_map[seg->GetNode(1)];
        m_data_mgr->flex1D_Nodes_H.push_back(mI2(node0_index, node1_index));
    }

    // Create the BCE markers based on the mesh contact segments
    auto num_bce = AddBCE_mesh1D(index, fsi_mesh);

    // Update total number of flex 1-D nodes and segments
    auto num_nodes = fsi_mesh.GetNumNodes();
    m_num_flex1D_nodes += num_nodes;
    auto num_elements = fsi_mesh.GetNumElements();
    m_num_flex1D_elements += num_elements;

    if (m_verbose) {
        cout << "Add mesh1D" << endl;
        cout << "  Num. nodes:       " << num_nodes << endl;
        cout << "  Num. segments:    " << num_elements << endl;
        cout << "  Num. BCE markers: " << num_bce << endl;
    }
}

void ChFsiFluidSystemSPH::OnAddFsiMesh2D(unsigned int index, FsiMesh2D& fsi_mesh) {
    // Load index-based mesh connectivity (append to global list of 1-D flex segments)
    for (const auto& tri : fsi_mesh.contact_surface->GetTrianglesXYZ()) {
        auto node0_index = m_num_flex2D_nodes + fsi_mesh.ptr2ind_map[tri->GetNode(0)];
        auto node1_index = m_num_flex2D_nodes + fsi_mesh.ptr2ind_map[tri->GetNode(1)];
        auto node2_index = m_num_flex2D_nodes + fsi_mesh.ptr2ind_map[tri->GetNode(2)];
        m_data_mgr->flex2D_Nodes_H.push_back(mI3(node0_index, node1_index, node2_index));
    }

    // Create the BCE markers based on the mesh contact surface
    auto num_bce = AddBCE_mesh2D(index, fsi_mesh);

    // Update total number of flex 2-D nodes and faces
    auto num_nodes = fsi_mesh.GetNumNodes();
    m_num_flex2D_nodes += num_nodes;
    auto num_elements = fsi_mesh.GetNumElements();
    m_num_flex2D_elements += num_elements;

    if (m_verbose) {
        cout << "Add mesh2D" << endl;
        cout << "  Num. nodes:       " << num_nodes << endl;
        cout << "  Num. faces:       " << num_elements << endl;
        cout << "  Num. BCE markers: " << num_bce << endl;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::Initialize() {
    Initialize(0, 0, 0, 0, 0, std::vector<FsiBodyState>(), std::vector<FsiMeshState>(), std::vector<FsiMeshState>(),
               false);
}

void ChFsiFluidSystemSPH::Initialize(unsigned int num_fsi_bodies,
                                     unsigned int num_fsi_nodes1D,
                                     unsigned int num_fsi_elements1D,
                                     unsigned int num_fsi_nodes2D,
                                     unsigned int num_fsi_elements2D,
                                     const std::vector<FsiBodyState>& body_states,
                                     const std::vector<FsiMeshState>& mesh1D_states,
                                     const std::vector<FsiMeshState>& mesh2D_states,
                                     bool use_node_directions) {
    // Invoke the base class method
    ChFsiFluidSystem::Initialize(num_fsi_bodies,                             //
                                 num_fsi_nodes1D, num_fsi_elements1D,        //
                                 num_fsi_nodes2D, num_fsi_elements2D,        //
                                 body_states, mesh1D_states, mesh2D_states,  //
                                 use_node_directions);                       //

    // Hack to still allow time step size specified through JSON files
    if (m_paramsH->dT < 0) {
        m_paramsH->dT = GetStepSize();
    } else {
        SetStepSize(m_paramsH->dT);
    }

    // Set kernel radius factor (based on kernel type)
    switch (m_paramsH->kernel_type) {
        case KernelType::QUADRATIC:
            m_paramsH->h_multiplier = 2;
            break;
        case KernelType::CUBIC_SPLINE:
            m_paramsH->h_multiplier = 2;
            break;
        case KernelType::QUINTIC_SPLINE:
            m_paramsH->h_multiplier = 3;
            break;
        case KernelType::WENDLAND:
            m_paramsH->h_multiplier = 2;
            break;
    }

    // Initialize SPH particle mass and number of neighbors
    {
        Real sum = 0;
        int count = 0;
        int IDX = 10;
        for (int i = -IDX; i <= IDX; i++) {
            for (int j = -IDX; j <= IDX; j++) {
                for (int k = -IDX; k <= IDX; k++) {
                    Real3 pos = mR3(i, j, k) * m_paramsH->d0;
                    Real W = W3h(m_paramsH->kernel_type, length(pos), m_paramsH->ooh);
                    sum += W;
                    if (W > 0)
                        count++;
                }
            }
        }
        m_paramsH->markerMass = m_paramsH->rho0 / sum;
        m_paramsH->num_neighbors = count;
    }

    if (m_paramsH->use_init_pressure) {
        size_t numParticles = m_data_mgr->sphMarkers_H->rhoPresMuH.size();
        for (int i = 0; i < numParticles; i++) {
            double z = m_data_mgr->sphMarkers_H->posRadH[i].z;
            double p = m_paramsH->rho0 * m_paramsH->gravity.z * (z - m_paramsH->pressure_height);
            m_data_mgr->sphMarkers_H->rhoPresMuH[i].y = p;
            if (m_paramsH->elastic_SPH) {
                m_data_mgr->sphMarkers_H->tauXxYyZzH[i].x = -p;
                m_data_mgr->sphMarkers_H->tauXxYyZzH[i].y = -p;
                m_data_mgr->sphMarkers_H->tauXxYyZzH[i].z = -p;
            }
        }
    }

    // This means boundaries have not been set - just use an approximate domain size with no periodic sides
    if (m_paramsH->use_default_limits) {
        m_paramsH->cMin =
            mR3(-2 * m_paramsH->boxDimX, -2 * m_paramsH->boxDimY, -2 * m_paramsH->boxDimZ) - 10 * mR3(m_paramsH->h);
        m_paramsH->cMax =
            mR3(+2 * m_paramsH->boxDimX, +2 * m_paramsH->boxDimY, +2 * m_paramsH->boxDimZ) + 10 * mR3(m_paramsH->h);
        m_paramsH->periodic_sides = static_cast<int>(PeriodicSide::NONE);
    }

    m_paramsH->x_periodic = (m_paramsH->periodic_sides & static_cast<int>(PeriodicSide::X)) != 0;
    m_paramsH->y_periodic = (m_paramsH->periodic_sides & static_cast<int>(PeriodicSide::Y)) != 0;
    m_paramsH->z_periodic = (m_paramsH->periodic_sides & static_cast<int>(PeriodicSide::Z)) != 0;

    // Set up subdomains for faster neighbor particle search
    m_paramsH->Apply_BC_U = false;
    int3 side0 = mI3((int)floor((m_paramsH->cMax.x - m_paramsH->cMin.x) / (m_paramsH->h_multiplier * m_paramsH->h)),
                     (int)floor((m_paramsH->cMax.y - m_paramsH->cMin.y) / (m_paramsH->h_multiplier * m_paramsH->h)),
                     (int)floor((m_paramsH->cMax.z - m_paramsH->cMin.z) / (m_paramsH->h_multiplier * m_paramsH->h)));
    Real3 binSize3 =
        mR3((m_paramsH->cMax.x - m_paramsH->cMin.x) / side0.x, (m_paramsH->cMax.y - m_paramsH->cMin.y) / side0.y,
            (m_paramsH->cMax.z - m_paramsH->cMin.z) / side0.z);
    m_paramsH->binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
    m_paramsH->binSize0 = binSize3.x;
    m_paramsH->boxDims = m_paramsH->cMax - m_paramsH->cMin;
    m_paramsH->delta_pressure = mR3(0);
    int3 SIDE = mI3(int((m_paramsH->cMax.x - m_paramsH->cMin.x) / m_paramsH->binSize0 + .1),
                    int((m_paramsH->cMax.y - m_paramsH->cMin.y) / m_paramsH->binSize0 + .1),
                    int((m_paramsH->cMax.z - m_paramsH->cMin.z) / m_paramsH->binSize0 + .1));
    Real mBinSize = m_paramsH->binSize0;
    m_paramsH->gridSize = SIDE;
    m_paramsH->worldOrigin = m_paramsH->cMin;
    m_paramsH->cellSize = mR3(mBinSize, mBinSize, mBinSize);

    // Precompute grid min and max bounds considering whether we have periodic boundaries or not
    m_paramsH->minBounds = make_int3(m_paramsH->x_periodic ? INT_MIN : 0, m_paramsH->y_periodic ? INT_MIN : 0,
                                     m_paramsH->z_periodic ? INT_MIN : 0);

    m_paramsH->maxBounds = make_int3(m_paramsH->x_periodic ? INT_MAX : m_paramsH->gridSize.x - 1,
                                     m_paramsH->y_periodic ? INT_MAX : m_paramsH->gridSize.y - 1,
                                     m_paramsH->z_periodic ? INT_MAX : m_paramsH->gridSize.z - 1);

    // Initialize the underlying FSU system: set reference arrays, set counters, and resize simulation arrays
    // Indicate if the data manager should allocate space for holding FEA mesh direction vectors
    m_data_mgr->Initialize(num_fsi_bodies,                                                            //
                           num_fsi_nodes1D, num_fsi_elements1D, num_fsi_nodes2D, num_fsi_elements2D,  //
                           use_node_directions);

    // Load the initial body and mesh node states
    ChDebugLog("load initial states");
    LoadSolidStates(body_states, mesh1D_states, mesh2D_states);

    // Create BCE and SPH worker objects
    m_bce_mgr = chrono_types::make_unique<BceManager>(*m_data_mgr, m_verbose);
    m_fluid_dynamics = chrono_types::make_unique<FluidDynamics>(*m_data_mgr, *m_bce_mgr, m_verbose);

    // Initialize worker objects
    m_bce_mgr->Initialize(m_fsi_bodies_bce_num);
    m_fluid_dynamics->Initialize();

    /// If active domains are not used then don't overly resize the arrays
    if (!m_paramsH->use_active_domain) {
        m_data_mgr->SetGrowthFactor(1.0f);
    }

    // Check if GPU is available and initialize CUDA device information
    int device;
    cudaGetDevice(&device);
    cudaCheckError();
    m_data_mgr->cudaDeviceInfo->deviceID = device;
    cudaGetDeviceProperties(&m_data_mgr->cudaDeviceInfo->deviceProp, m_data_mgr->cudaDeviceInfo->deviceID);
    cudaCheckError();

    if (m_verbose) {
        cout << "GPU device: " << m_data_mgr->cudaDeviceInfo->deviceProp.name << endl;
        cout << "  Compute capability: " << m_data_mgr->cudaDeviceInfo->deviceProp.major << "."
             << m_data_mgr->cudaDeviceInfo->deviceProp.minor << endl;
        cout << "  Total global memory: "
             << m_data_mgr->cudaDeviceInfo->deviceProp.totalGlobalMem / (1024. * 1024. * 1024.) << " GB" << endl;
        cout << "  Total constant memory: " << m_data_mgr->cudaDeviceInfo->deviceProp.totalConstMem / 1024. << " KB"
             << endl;
        cout << "  Total Static shared memory per block Available: "
             << m_data_mgr->cudaDeviceInfo->deviceProp.sharedMemPerBlock / 1024. << " KB" << endl;
        cout << "  Maximum Dynamic shared memory per block (with opt-in): "
             << m_data_mgr->cudaDeviceInfo->deviceProp.sharedMemPerBlockOptin / 1024. << " KB" << endl;
        cout << "  Total shared memory per multiprocessor: "
             << m_data_mgr->cudaDeviceInfo->deviceProp.sharedMemPerMultiprocessor / 1024. << " KB" << endl;
        cout << "  Number of multiprocessors: " << m_data_mgr->cudaDeviceInfo->deviceProp.multiProcessorCount << endl;

        cout << "Simulation parameters" << endl;
        switch (m_paramsH->viscosity_type) {
            case ViscosityType::LAMINAR:
                cout << "  Viscosity treatment: Laminar" << endl;
                break;
            case ViscosityType::ARTIFICIAL_UNILATERAL:
                cout << "  Viscosity treatment: Artificial Unilateral";
                cout << "  (coefficient: " << m_paramsH->Ar_vis_alpha << ")" << endl;
                break;
            case ViscosityType::ARTIFICIAL_BILATERAL:
                cout << "  Viscosity treatment: Artificial Bilateral";
                cout << "  (coefficient: " << m_paramsH->Ar_vis_alpha << ")" << endl;
                break;
        }
        if (m_paramsH->boundary_type == BoundaryType::ADAMI) {
            cout << "  Boundary treatment: Adami" << endl;
        } else if (m_paramsH->boundary_type == BoundaryType::HOLMES) {
            cout << "  Boundary treatment: Holmes" << endl;
        } else {
            cout << "  Boundary treatment: Adami" << endl;
        }
        switch (m_paramsH->kernel_type) {
            case KernelType::QUADRATIC:
                cout << "  Kernel type: Quadratic" << endl;
                break;
            case KernelType::CUBIC_SPLINE:
                cout << "  Kernel type: Cubic Spline" << endl;
                break;
            case KernelType::QUINTIC_SPLINE:
                cout << "  Kernel type: Quintic Spline" << endl;
                break;
            case KernelType::WENDLAND:
                cout << "  Kernel type: Wendland Quintic" << endl;
                break;
        }

        switch (m_paramsH->shifting_method) {
            case ShiftingMethod::XSPH:
                cout << "  Shifting method: XSPH" << endl;
                break;
            case ShiftingMethod::PPST:
                cout << "  Shifting method: PPST" << endl;
                break;
            case ShiftingMethod::PPST_XSPH:
                cout << "  Shifting method: PPST_XSPH" << endl;
                break;
            case ShiftingMethod::DIFFUSION:
                cout << "  Shifting method: Diffusion" << endl;
                break;
            case ShiftingMethod::DIFFUSION_XSPH:
                cout << "  Shifting method: Diffusion_XSPH" << endl;
                break;
            case ShiftingMethod::NONE:
                cout << "  Shifting method: None" << endl;
                break;
        }

        cout << "  num_neighbors: " << m_paramsH->num_neighbors << endl;
        cout << "  rho0: " << m_paramsH->rho0 << endl;
        cout << "  invrho0: " << m_paramsH->invrho0 << endl;
        cout << "  mu0: " << m_paramsH->mu0 << endl;
        cout << "  bodyForce3: " << m_paramsH->bodyForce3.x << " " << m_paramsH->bodyForce3.y << " "
             << m_paramsH->bodyForce3.z << endl;
        cout << "  gravity: " << m_paramsH->gravity.x << " " << m_paramsH->gravity.y << " " << m_paramsH->gravity.z
             << endl;

        cout << "  d0: " << m_paramsH->d0 << endl;
        cout << "  1/d0: " << m_paramsH->ood0 << endl;
        cout << "  d0_multiplier: " << m_paramsH->d0_multiplier << endl;
        cout << "  h: " << m_paramsH->h << endl;
        cout << "  1/h: " << m_paramsH->ooh << endl;

        cout << "  num_bce_layers: " << m_paramsH->num_bce_layers << endl;
        cout << "  epsMinMarkersDis: " << m_paramsH->epsMinMarkersDis << endl;
        cout << "  markerMass: " << m_paramsH->markerMass << endl;
        cout << "  volume0: " << m_paramsH->volume0 << endl;
        cout << "  gradient_type: " << m_paramsH->gradient_type << endl;

        cout << "  v_Max: " << m_paramsH->v_Max << endl;
        cout << "  Cs: " << m_paramsH->Cs << endl;

        if (m_paramsH->shifting_method == ShiftingMethod::XSPH) {
            cout << "  shifting_xsph_eps: " << m_paramsH->shifting_xsph_eps << endl;
        } else if (m_paramsH->shifting_method == ShiftingMethod::PPST) {
            cout << "  shifting_ppst_push: " << m_paramsH->shifting_ppst_push << endl;
            cout << "  shifting_ppst_pull: " << m_paramsH->shifting_ppst_pull << endl;
        } else if (m_paramsH->shifting_method == ShiftingMethod::PPST_XSPH) {
            cout << "  shifting_xsph_eps: " << m_paramsH->shifting_xsph_eps << endl;
            cout << "  shifting_ppst_push: " << m_paramsH->shifting_ppst_push << endl;
            cout << "  shifting_ppst_pull: " << m_paramsH->shifting_ppst_pull << endl;
        } else if (m_paramsH->shifting_method == ShiftingMethod::DIFFUSION) {
            cout << "  shifting_diffusion_A: " << m_paramsH->shifting_diffusion_A << endl;
            cout << "  shifting_diffusion_AFSM: " << m_paramsH->shifting_diffusion_AFSM << endl;
            cout << "  shifting_diffusion_AFST: " << m_paramsH->shifting_diffusion_AFST << endl;
        } else if (m_paramsH->shifting_method == ShiftingMethod::DIFFUSION_XSPH) {
            cout << "  shifting_xsph_eps: " << m_paramsH->shifting_xsph_eps << endl;
            cout << "  shifting_diffusion_A: " << m_paramsH->shifting_diffusion_A << endl;
            cout << "  shifting_diffusion_AFSM: " << m_paramsH->shifting_diffusion_AFSM << endl;
            cout << "  shifting_diffusion_AFST: " << m_paramsH->shifting_diffusion_AFST << endl;
        }
        cout << "  densityReinit: " << m_paramsH->densityReinit << endl;

        cout << "  Proximity search performed every " << m_paramsH->num_proximity_search_steps << " steps" << endl;
        cout << "  dT: " << m_paramsH->dT << endl;

        cout << "  non_newtonian: " << m_paramsH->non_newtonian << endl;
        cout << "  mu_of_I : " << (int)m_paramsH->mu_of_I << endl;
        cout << "  rheology_model: " << (int)m_paramsH->rheology_model << endl;
        cout << "  ave_diam: " << m_paramsH->ave_diam << endl;
        cout << "  mu_max: " << m_paramsH->mu_max << endl;
        cout << "  mu_fric_s: " << m_paramsH->mu_fric_s << endl;
        cout << "  mu_fric_2: " << m_paramsH->mu_fric_2 << endl;
        cout << "  mu_I0: " << m_paramsH->mu_I0 << endl;
        cout << "  mu_I_b: " << m_paramsH->mu_I_b << endl;
        cout << "  HB_k: " << m_paramsH->HB_k << endl;
        cout << "  HB_n: " << m_paramsH->HB_n << endl;
        cout << "  HB_tau0: " << m_paramsH->HB_tau0 << endl;
        cout << "  Coh_coeff: " << m_paramsH->Coh_coeff << endl;

        cout << "  E_young: " << m_paramsH->E_young << endl;
        cout << "  G_shear: " << m_paramsH->G_shear << endl;
        cout << "  INV_G_shear: " << m_paramsH->INV_G_shear << endl;
        cout << "  K_bulk: " << m_paramsH->K_bulk << endl;
        cout << "  C_Wi: " << m_paramsH->C_Wi << endl;

        cout << "  PPE_relaxation: " << m_paramsH->PPE_relaxation << endl;
        cout << "  Conservative_Form: " << m_paramsH->Conservative_Form << endl;
        cout << "  Pressure_Constraint: " << m_paramsH->Pressure_Constraint << endl;

        cout << "  binSize0: " << m_paramsH->binSize0 << endl;
        cout << "  boxDims: " << m_paramsH->boxDims.x << " " << m_paramsH->boxDims.y << " " << m_paramsH->boxDims.z
             << endl;
        cout << "  gridSize: " << m_paramsH->gridSize.x << " " << m_paramsH->gridSize.y << " " << m_paramsH->gridSize.z
             << endl;
        cout << "  cMin: " << m_paramsH->cMin.x << " " << m_paramsH->cMin.y << " " << m_paramsH->cMin.z << endl;
        cout << "  cMax: " << m_paramsH->cMax.x << " " << m_paramsH->cMax.y << " " << m_paramsH->cMax.z << endl;

        ////Real dt_CFL = m_paramsH->Co_number * m_paramsH->h / 2.0 / MaxVel;
        ////Real dt_nu = 0.2 * m_paramsH->h * m_paramsH->h / (m_paramsH->mu0 / m_paramsH->rho0);
        ////Real dt_body = 0.1 * sqrt(m_paramsH->h / length(m_paramsH->bodyForce3 + m_paramsH->gravity));
        ////Real dt = std::min(dt_body, std::min(dt_CFL, dt_nu));

        const auto& counters = m_data_mgr->countersH;
        cout << "Counters" << endl;
        cout << "  numFsiBodies:       " << counters->numFsiBodies << endl;
        cout << "  numFsiElements1D:   " << counters->numFsiElements1D << endl;
        cout << "  numFsiElements2D:   " << counters->numFsiElements2D << endl;
        cout << "  numFsiNodes1D:      " << counters->numFsiNodes1D << endl;
        cout << "  numFsiNodes2D:      " << counters->numFsiNodes2D << endl;
        cout << "  numGhostMarkers:    " << counters->numGhostMarkers << endl;
        cout << "  numHelperMarkers:   " << counters->numHelperMarkers << endl;
        cout << "  numFluidMarkers:    " << counters->numFluidMarkers << endl;
        cout << "  numBoundaryMarkers: " << counters->numBoundaryMarkers << endl;
        cout << "  numRigidMarkers:    " << counters->numRigidMarkers << endl;
        cout << "  numFlexMarkers1D:   " << counters->numFlexMarkers1D << endl;
        cout << "  numFlexMarkers2D:   " << counters->numFlexMarkers2D << endl;
        cout << "  numAllMarkers:      " << counters->numAllMarkers << endl;
        cout << "  startRigidMarkers:  " << counters->startRigidMarkers << endl;
        cout << "  startFlexMarkers1D: " << counters->startFlexMarkers1D << endl;
        cout << "  startFlexMarkers2D: " << counters->startFlexMarkers2D << endl;

        cout << "Reference array (size: " << m_data_mgr->referenceArray.size() << ")" << endl;
        for (size_t i = 0; i < m_data_mgr->referenceArray.size(); i++) {
            const int4& num = m_data_mgr->referenceArray[i];
            cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
        }
        cout << "Reference array FEA (size: " << m_data_mgr->referenceArray_FEA.size() << ")" << endl;
        for (size_t i = 0; i < m_data_mgr->referenceArray_FEA.size(); i++) {
            const int4& num = m_data_mgr->referenceArray_FEA[i];
            cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
        }
        cout << endl;
    }

    CheckSPHParameters();
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiFluidSystemSPH::OnSetupStepDynamics() {
    // Update particle activity
    if (m_time >= m_paramsH->settlingTime) {
        m_fluid_dynamics->UpdateActivity(m_data_mgr->sphMarkers_D);
    }
    // Resize data arrays if needed
    if (m_time < 1e-6 || int(round(m_time / m_paramsH->dT)) % m_paramsH->num_proximity_search_steps == 0) {
        m_data_mgr->ResizeData(m_first_step);
        m_first_step = false;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiFluidSystemSPH::OnDoStepDynamics(double step) {

    if (m_time < 1e-6 || int(round(m_time / m_paramsH->dT)) % m_paramsH->num_proximity_search_steps == 0) {
        m_fluid_dynamics->SortParticles();
    }

    m_data_mgr->ResetData();

    switch (m_paramsH->sph_method) {
        case SPHMethod::WCSPH: {
            m_data_mgr->CopyDeviceDataToHalfStep();
            m_fluid_dynamics->IntegrateSPH(m_data_mgr->sortedSphMarkers2_D, m_data_mgr->sortedSphMarkers1_D,  //
                                           step / 2, m_time, true);
            m_time += step / 2;
            m_fluid_dynamics->IntegrateSPH(m_data_mgr->sortedSphMarkers1_D, m_data_mgr->sortedSphMarkers2_D,  //
                                           step, m_time, false);
            m_time += step / 2;
            break;
        }

        case SPHMethod::I2SPH: {
            m_bce_mgr->updateBCEAcc();
            m_fluid_dynamics->IntegrateSPH(m_data_mgr->sortedSphMarkers2_D, m_data_mgr->sortedSphMarkers2_D,  //
                                           0.0, m_time, false);
            m_time += step;
            break;
        }
    }

    m_fluid_dynamics->CopySortedToOriginal(MarkerGroup::NON_SOLID, m_data_mgr->sortedSphMarkers2_D,
                                           m_data_mgr->sphMarkers_D);

    ChDebugLog("GPU Memory usage: " << m_data_mgr->GetCurrentGPUMemoryUsage() / 1024.0 / 1024.0 << " MB");
}

void ChFsiFluidSystemSPH::OnExchangeSolidForces() {
    m_bce_mgr->Rigid_Forces_Torques();
    m_bce_mgr->Flex1D_Forces();
    m_bce_mgr->Flex2D_Forces();
}

void ChFsiFluidSystemSPH::OnExchangeSolidStates() {
    if (m_num_rigid_bodies == 0 && m_num_flex1D_elements == 0 && m_num_flex2D_elements == 0)
        return;

    m_bce_mgr->UpdateBodyMarkerState();
    m_bce_mgr->UpdateMeshMarker1DState();
    m_bce_mgr->UpdateMeshMarker2DState();

    m_fluid_dynamics->CopySortedToOriginal(MarkerGroup::SOLID, m_data_mgr->sortedSphMarkers2_D,
                                           m_data_mgr->sphMarkers_D);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::WriteParticleFile(const std::string& filename) const {
    writeParticleFileCSV(filename, *m_data_mgr);
}

void ChFsiFluidSystemSPH::SaveParticleData(const std::string& dir) const {
    if (m_paramsH->elastic_SPH)
        saveParticleDataCRM(dir, m_output_level, *m_data_mgr);
    else
        saveParticleDataCFD(dir, m_output_level, *m_data_mgr);
}

void ChFsiFluidSystemSPH::SaveSolidData(const std::string& dir, double time) const {
    saveSolidData(dir, time, *m_data_mgr);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::AddSPHParticle(const ChVector3d& pos,
                                         double rho,
                                         double pres,
                                         double mu,
                                         const ChVector3d& vel,
                                         const ChVector3d& tauXxYyZz,
                                         const ChVector3d& tauXyXzYz) {
    m_data_mgr->AddSphParticle(ToReal3(pos), rho, pres, mu, ToReal3(vel), ToReal3(tauXxYyZz), ToReal3(tauXyXzYz));
}

void ChFsiFluidSystemSPH::AddSPHParticle(const ChVector3d& pos,
                                         const ChVector3d& vel,
                                         const ChVector3d& tauXxYyZz,
                                         const ChVector3d& tauXyXzYz) {
    AddSPHParticle(pos, m_paramsH->rho0, m_paramsH->base_pressure, m_paramsH->mu0, vel, tauXxYyZz, tauXyXzYz);
}

void ChFsiFluidSystemSPH::AddBoxSPH(const ChVector3d& boxCenter, const ChVector3d& boxHalfDim) {
    // Use a chrono sampler to create a bucket of points
    utils::ChGridSampler<> sampler(m_paramsH->d0);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        AddSPHParticle(points[i], m_paramsH->rho0, 0, m_paramsH->mu0,
                       ChVector3d(0),   // initial velocity
                       ChVector3d(0),   // tauxxyyzz
                       ChVector3d(0));  // tauxyxzyz
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::AddPlateBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector2d& size) {
    std::vector<ChVector3d> points;
    CreateBCE_Plate(size, points);
    AddPointsBCE(body, points, frame, false);
}

void ChFsiFluidSystemSPH::AddBoxContainerBCE(std::shared_ptr<ChBody> body,
                                             const ChFrame<>& frame,
                                             const ChVector3d& size,
                                             const ChVector3i faces) {
    Real spacing = m_paramsH->d0;
    Real buffer = 2 * (m_paramsH->num_bce_layers - 1) * spacing;

    ChVector3d hsize = size / 2;

    ChVector3d xn(-hsize.x(), 0, 0);
    ChVector3d xp(+hsize.x(), 0, 0);
    ChVector3d yn(0, -hsize.y(), 0);
    ChVector3d yp(0, +hsize.y(), 0);
    ChVector3d zn(0, 0, -hsize.z());
    ChVector3d zp(0, 0, +hsize.z());

    // Z- wall
    if (faces.z() == -1 || faces.z() == 2)
        AddPlateBCE(body, frame * ChFrame<>(zn, QUNIT), {size.x(), size.y()});
    // Z+ wall
    if (faces.z() == +1 || faces.z() == 2)
        AddPlateBCE(body, frame * ChFrame<>(zp, QuatFromAngleX(CH_PI)), {size.x(), size.y()});

    // X- wall
    if (faces.x() == -1 || faces.x() == 2)
        AddPlateBCE(body, frame * ChFrame<>(xn, QuatFromAngleY(+CH_PI_2)), {size.z() + buffer, size.y()});
    // X+ wall
    if (faces.x() == +1 || faces.x() == 2)
        AddPlateBCE(body, frame * ChFrame<>(xp, QuatFromAngleY(-CH_PI_2)), {size.z() + buffer, size.y()});

    // Y- wall
    if (faces.y() == -1 || faces.y() == 2)
        AddPlateBCE(body, frame * ChFrame<>(yn, QuatFromAngleX(-CH_PI_2)), {size.x() + buffer, size.z() + buffer});
    // Y+ wall
    if (faces.y() == +1 || faces.y() == 2)
        AddPlateBCE(body, frame * ChFrame<>(yp, QuatFromAngleX(+CH_PI_2)), {size.x() + buffer, size.z() + buffer});
}

size_t ChFsiFluidSystemSPH::AddBoxBCE(std::shared_ptr<ChBody> body,
                                      const ChFrame<>& frame,
                                      const ChVector3d& size,
                                      bool solid) {
    std::vector<ChVector3d> points;
    if (solid)
        CreateBCE_BoxInterior(size, points);
    else
        CreateBCE_BoxExterior(size, points);

    return AddPointsBCE(body, points, frame, solid);
}

size_t ChFsiFluidSystemSPH::AddSphereBCE(std::shared_ptr<ChBody> body,
                                         const ChFrame<>& frame,
                                         double radius,
                                         bool solid,
                                         bool polar) {
    std::vector<ChVector3d> points;
    if (solid)
        CreateBCE_SphereInterior(radius, polar, points);
    else
        CreateBCE_SphereExterior(radius, polar, points);

    return AddPointsBCE(body, points, frame, solid);
}

size_t ChFsiFluidSystemSPH::AddCylinderBCE(std::shared_ptr<ChBody> body,
                                           const ChFrame<>& frame,
                                           double radius,
                                           double height,
                                           bool solid,
                                           bool polar) {
    std::vector<ChVector3d> points;
    if (solid)
        CreateBCE_CylinderInterior(radius, height, polar, points);
    else
        CreateBCE_CylinderExterior(radius, height, polar, points);

    return AddPointsBCE(body, points, frame, solid);
}

size_t ChFsiFluidSystemSPH::AddConeBCE(std::shared_ptr<ChBody> body,
                                       const ChFrame<>& frame,
                                       double radius,
                                       double height,
                                       bool solid,
                                       bool polar) {
    std::vector<ChVector3d> points;
    if (solid)
        CreateBCE_ConeInterior(radius, height, polar, points);
    else
        CreateBCE_ConeExterior(radius, height, polar, points);

    return AddPointsBCE(body, points, frame, true);
}

size_t ChFsiFluidSystemSPH::AddCylinderAnnulusBCE(std::shared_ptr<ChBody> body,
                                                  const ChFrame<>& frame,
                                                  double radius_inner,
                                                  double radius_outer,
                                                  double height,
                                                  bool polar) {
    auto delta = m_paramsH->d0;
    std::vector<ChVector3d> points;
    CreatePoints_CylinderAnnulus(radius_inner, radius_outer, height, polar, delta, points);
    return AddPointsBCE(body, points, frame, true);
}

size_t ChFsiFluidSystemSPH::AddPointsBCE(std::shared_ptr<ChBody> body,
                                         const std::vector<ChVector3d>& points,
                                         const ChFrame<>& rel_frame,
                                         bool solid) {
    // Set BCE marker type
    MarkerType type = solid ? MarkerType::BCE_RIGID : MarkerType::BCE_WALL;

    for (const auto& p : points) {
        auto pos_body = rel_frame.TransformPointLocalToParent(p);
        auto pos_abs = body->GetFrameRefToAbs().TransformPointLocalToParent(pos_body);
        auto vel_abs = body->GetFrameRefToAbs().PointSpeedLocalToParent(pos_body);

        m_data_mgr->AddBceMarker(type, ToReal3(pos_abs), ToReal3(vel_abs));
    }

    if (solid)
        m_fsi_bodies_bce_num.push_back((int)points.size());

    return points.size();
}

//--------------------------------------------------------------------------------------------------------------------------------

const Real pi = Real(CH_PI);

void ChFsiFluidSystemSPH::CreateBCE_Plate(const ChVector2d& size, std::vector<ChVector3d>& bce) {
    Real spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Calculate actual spacing in x-y directions
    ChVector2d hsize = size / 2;
    int2 np = {(int)std::round(hsize.x() / spacing), (int)std::round(hsize.y() / spacing)};
    ChVector2d delta = {hsize.x() / np.x, hsize.y() / np.y};

    for (int il = 0; il < num_layers; il++) {
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iy = -np.y; iy <= np.y; iy++) {
                bce.push_back({ix * delta.x(), iy * delta.y(), -il * spacing});
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCE_BoxInterior(const ChVector3d& size, std::vector<ChVector3d>& bce) {
    Real spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Decide if any direction is small enough to be filled
    ChVector3<int> np(1 + (int)std::round(size.x() / spacing),  //
                      1 + (int)std::round(size.y() / spacing),  //
                      1 + (int)std::round(size.z() / spacing));
    bool fill = np[0] <= 2 * num_layers || np[1] <= 2 * num_layers || np[2] <= 2 * num_layers;

    // Adjust spacing in each direction
    ChVector3d delta(size.x() / (np.x() - 1), size.y() / (np.y() - 1), size.z() / (np.z() - 1));

    // If any direction must be filled, the box must be filled
    if (fill) {
        for (int ix = 0; ix < np.x(); ix++) {
            double x = -size.x() / 2 + ix * delta.x();
            for (int iy = 0; iy < np.y(); iy++) {
                double y = -size.y() / 2 + iy * delta.y();
                for (int iz = 0; iz < np.z(); iz++) {
                    double z = -size.z() / 2 + iz * delta.z();
                    bce.push_back({x, y, z});
                }
            }
        }
        return;
    }

    // Create interior BCE layers
    for (int il = 0; il < num_layers; il++) {
        // x faces
        double xm = -size.x() / 2 + il * delta.x();
        double xp = +size.x() / 2 - il * delta.x();
        for (int iy = 0; iy < np.y(); iy++) {
            double y = -size.y() / 2 + iy * delta.y();
            for (int iz = 0; iz < np.z(); iz++) {
                double z = -size.z() / 2 + iz * delta.z();
                bce.push_back({xm, y, z});
                bce.push_back({xp, y, z});
            }
        }

        // y faces
        double ym = -size.y() / 2 + il * delta.y();
        double yp = +size.y() / 2 - il * delta.y();
        for (int iz = 0; iz < np.z(); iz++) {
            double z = -size.z() / 2 + iz * delta.z();
            for (int ix = 0; ix < np.x(); ix++) {
                double x = -size.x() / 2 + ix * delta.x();
                bce.push_back({x, ym, z});
                bce.push_back({x, yp, z});
            }
        }

        // z faces
        double zm = -size.z() / 2 + il * delta.z();
        double zp = +size.z() / 2 - il * delta.z();
        for (int ix = 0; ix < np.x(); ix++) {
            double x = -size.x() / 2 + ix * delta.x();
            for (int iy = 0; iy < np.y(); iy++) {
                double y = -size.y() / 2 + iy * delta.y();
                bce.push_back({x, y, zm});
                bce.push_back({x, y, zp});
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCE_BoxExterior(const ChVector3d& size, std::vector<ChVector3d>& bce) {
    Real spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Adjust spacing in each direction
    ChVector3i np(1 + (int)std::round(size.x() / spacing),  //
                  1 + (int)std::round(size.y() / spacing),  //
                  1 + (int)std::round(size.z() / spacing));
    ChVector3d delta(size.x() / (np.x() - 1), size.y() / (np.y() - 1), size.z() / (np.z() - 1));

    // Inflate box
    ChVector3i Np = np + 2 * (num_layers - 1);
    ChVector3d Size = size + 2.0 * (num_layers - 1.0) * delta;

    // Create exterior BCE layers
    for (int il = 0; il < num_layers; il++) {
        // x faces
        double xm = -Size.x() / 2 + il * delta.x();
        double xp = +Size.x() / 2 - il * delta.x();
        for (int iy = 0; iy < Np.y(); iy++) {
            double y = -Size.y() / 2 + iy * delta.y();
            for (int iz = 0; iz < Np.z(); iz++) {
                double z = -Size.z() / 2 + iz * delta.z();
                bce.push_back({xm, y, z});
                bce.push_back({xp, y, z});
            }
        }

        // y faces
        double ym = -Size.y() / 2 + il * delta.y();
        double yp = +Size.y() / 2 - il * delta.y();
        for (int iz = 0; iz < Np.z(); iz++) {
            double z = -Size.z() / 2 + iz * delta.z();
            for (int ix = 0; ix < np.x(); ix++) {
                double x = -size.x() / 2 + ix * delta.x();
                bce.push_back({x, ym, z});
                bce.push_back({x, yp, z});
            }
        }

        // z faces
        double zm = -Size.z() / 2 + il * delta.z();
        double zp = +Size.z() / 2 - il * delta.z();
        for (int ix = 0; ix < np.x(); ix++) {
            double x = -size.x() / 2 + ix * delta.x();
            for (int iy = 0; iy < np.y(); iy++) {
                double y = -size.y() / 2 + iy * delta.y();
                bce.push_back({x, y, zm});
                bce.push_back({x, y, zp});
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCE_SphereInterior(double radius, bool polar, std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Use polar coordinates
    if (polar) {
        double rad_in = radius - (num_layers - 1) * spacing;

        for (int ir = 0; ir < num_layers; ir++) {
            double r = rad_in + ir * spacing;
            int np_phi = (int)std::round(pi * r / spacing);
            double delta_phi = pi / np_phi;
            for (int ip = 0; ip < np_phi; ip++) {
                double phi = ip * delta_phi;
                double cphi = std::cos(phi);
                double sphi = std::sin(phi);
                double x = r * sphi;
                double y = r * sphi;
                double z = r * cphi;
                int np_th = (int)std::round(2 * pi * r * sphi / spacing);
                double delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
                for (int it = 0; it < np_th; it++) {
                    double theta = it * delta_th;
                    bce.push_back({x * std::cos(theta), y * std::sin(theta), z});
                }
            }
        }
        return;
    }

    // Use a Cartesian grid and accept/reject points
    int np = (int)std::round(radius / spacing);
    double delta = radius / np;

    for (int iz = 0; iz <= np; iz++) {
        double z = iz * delta;
        double rz_max = std::sqrt(radius * radius - z * z);
        double rz_min = std::max(rz_max - num_layers * delta, 0.0);
        if (iz >= np - num_layers)
            rz_min = 0;
        double rz_min2 = rz_min * rz_min;
        double rz_max2 = rz_max * rz_max;
        int nq = (int)std::round(rz_max / spacing);
        for (int ix = -nq; ix <= nq; ix++) {
            double x = ix * delta;
            for (int iy = -nq; iy <= nq; iy++) {
                double y = iy * delta;
                double r2 = x * x + y * y;
                if (r2 >= rz_min2 && r2 <= rz_max2) {
                    bce.push_back({x, y, +z});
                    bce.push_back({x, y, -z});
                }
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCE_SphereExterior(double radius, bool polar, std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Use polar coordinates
    if (polar) {
        for (int ir = 0; ir < num_layers; ir++) {
            double r = radius + ir * spacing;
            int np_phi = (int)std::round(pi * r / spacing);
            double delta_phi = pi / np_phi;
            for (int ip = 0; ip < np_phi; ip++) {
                double phi = ip * delta_phi;
                double cphi = std::cos(phi);
                double sphi = std::sin(phi);
                double x = r * sphi;
                double y = r * sphi;
                double z = r * cphi;
                int np_th = (int)std::round(2 * pi * r * sphi / spacing);
                double delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
                for (int it = 0; it < np_th; it++) {
                    double theta = it * delta_th;
                    bce.push_back({x * std::cos(theta), y * std::sin(theta), z});
                }
            }
        }
        return;
    }

    // Inflate sphere and accept/reject points on a Cartesian grid
    int np = (int)std::round(radius / spacing);
    double delta = radius / np;
    np += num_layers;
    radius += num_layers * delta;

    for (int iz = 0; iz <= np; iz++) {
        double z = iz * delta;
        double rz_max = std::sqrt(radius * radius - z * z);
        double rz_min = std::max(rz_max - num_layers * delta, 0.0);
        if (iz >= np - num_layers)
            rz_min = 0;
        double rz_min2 = rz_min * rz_min;
        double rz_max2 = radius * radius - z * z;
        for (int ix = -np; ix <= np; ix++) {
            double x = ix * delta;
            for (int iy = -np; iy <= np; iy++) {
                double y = iy * delta;
                double r2 = x * x + y * y;
                if (r2 >= rz_min2 && r2 <= rz_max2) {
                    bce.push_back({x, y, +z});
                    bce.push_back({x, y, -z});
                }
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCE_CylinderInterior(double rad,
                                                     double height,
                                                     bool polar,
                                                     std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Radial direction (num divisions and adjusted spacing)
    int np_r = (int)std::round(rad / spacing);
    double delta_r = rad / np_r;

    // Axial direction (num divisions and adjusted spacing)
    int np_h = (int)std::round(height / spacing);
    double delta_h = height / np_h;

    // If the radius or height are too small, fill the entire cylinder
    bool fill = (np_r <= num_layers - 1) || (np_h <= 2 * num_layers - 1);

    // Use polar coordinates
    if (polar) {
        double rad_min = std::max(rad - (num_layers - 1) * spacing, 0.0);
        if (fill)
            rad_min = 0;
        np_r = (int)std::round((rad - rad_min) / spacing);
        delta_r = (rad - rad_min) / np_r;

        for (int ir = 0; ir <= np_r; ir++) {
            double r = rad_min + ir * delta_r;
            int np_th = std::max((int)std::round(2 * pi * r / spacing) - 1, 1);
            double delta_th = CH_2PI / np_th;
            for (int it = 0; it < np_th; it++) {
                double theta = it * delta_th;
                double x = r * cos(theta);
                double y = r * sin(theta);
                for (int iz = 0; iz <= np_h; iz++) {
                    double z = -height / 2 + iz * delta_h;
                    bce.push_back({x, y, z});
                }
            }
        }

        // Add cylinder caps (unless already filled)
        if (!fill) {
            np_r = (int)std::round(rad_min / spacing);
            delta_r = rad_min / np_r;

            for (int ir = 0; ir < np_r; ir++) {
                double r = ir * delta_r;
                int np_th = std::max((int)std::round(2 * pi * r / spacing) - 1, 1);
                double delta_th = CH_2PI / np_th;
                for (int it = 0; it < np_th; it++) {
                    double theta = it * delta_th;
                    double x = r * cos(theta);
                    double y = r * sin(theta);
                    for (int iz = 0; iz < num_layers; iz++) {
                        double z = height / 2 - iz * delta_h;
                        bce.push_back({x, y, -z});
                        bce.push_back({x, y, +z});
                    }
                }
            }
        }

        return;
    }

    // Use a Cartesian grid and accept/reject points
    double r_max = rad;
    double r_min = std::max(rad - num_layers * delta_r, 0.0);
    if (fill)
        r_min = 0;
    double r_max2 = r_max * r_max;
    double r_min2 = r_min * r_min;
    for (int ix = -np_r; ix <= np_r; ix++) {
        double x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            double y = iy * delta_r;
            double r2 = x * x + y * y;
            if (r2 >= r_min2 && r2 <= r_max2) {
                for (int iz = 0; iz <= np_h; iz++) {
                    double z = -height / 2 + iz * delta_h;
                    bce.push_back({x, y, z});
                }
            }
            // Add cylinder caps (unless already filled)
            if (!fill && r2 < r_min2) {
                for (int iz = 0; iz < num_layers; iz++) {
                    double z = height / 2 - iz * delta_h;
                    bce.push_back({x, y, -z});
                    bce.push_back({x, y, +z});
                }
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCE_CylinderExterior(double rad,
                                                     double height,
                                                     bool polar,
                                                     std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Calculate actual spacing and inflate cylinder
    int np_h = (int)std::round(height / spacing);
    double delta_h = height / np_h;
    np_h += 2 * (num_layers - 1);
    height += 2 * (num_layers - 1) * delta_h;

    // Use polar coordinates
    if (polar) {
        double rad_max = rad + num_layers * spacing;
        double rad_min = rad_max - num_layers * spacing;
        int np_r = (int)std::round((rad_max - rad_min) / spacing);
        double delta_r = (rad_max - rad_min) / np_r;

        for (int ir = 0; ir <= np_r; ir++) {
            double r = rad_min + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / spacing);
            double delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
            for (int it = 0; it < np_th; it++) {
                double theta = it * delta_th;
                double x = r * cos(theta);
                double y = r * sin(theta);
                for (int iz = 0; iz <= np_h; iz++) {
                    double z = -height / 2 + iz * delta_h;
                    bce.push_back({x, y, z});
                }
            }
        }

        // Add cylinder caps
        np_r = (int)std::round(rad_min / spacing);
        delta_r = rad_min / np_r;

        for (int ir = 0; ir < np_r; ir++) {
            double r = ir * delta_r;
            int np_th = std::max((int)std::round(2 * pi * r / spacing), 1);
            double delta_th = (2 * pi) / np_th;
            for (int it = 0; it < np_th; it++) {
                double theta = it * delta_th;
                double x = r * cos(theta);
                double y = r * sin(theta);
                for (int iz = 0; iz < num_layers; iz++) {
                    double z = -height / 2 + iz * delta_h;
                    bce.push_back({x, y, -z});
                    bce.push_back({x, y, +z});
                }
            }
        }

        return;
    }

    // Inflate cylinder and accept/reject points on a Cartesian grid
    int np_r = (int)std::round(rad / spacing);
    double delta_r = rad / np_r;
    np_r += num_layers;
    rad += num_layers * delta_r;

    double rad_max = rad;
    double rad_min = std::max(rad - num_layers * delta_r, 0.0);
    double r_max2 = rad_max * rad_max;
    double r_min2 = rad_min * rad_min;
    for (int ix = -np_r; ix <= np_r; ix++) {
        double x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            double y = iy * delta_r;
            double r2 = x * x + y * y;
            if (r2 >= r_min2 && r2 <= r_max2) {
                for (int iz = 0; iz <= np_h; iz++) {
                    double z = -height / 2 + iz * delta_h;
                    bce.push_back({x, y, z});
                }
            }
            // Add cylinder caps
            if (r2 < r_min2) {
                for (int iz = 0; iz < num_layers; iz++) {
                    double z = -height / 2 + iz * delta_h;
                    bce.push_back({x, y, -z});
                    bce.push_back({x, y, +z});
                }
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCE_ConeInterior(double rad, double height, bool polar, std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Calculate actual spacing
    int np_h = (int)std::round(height / spacing);
    double delta_h = height / np_h;

    // Use polar coordinates
    if (polar) {
        for (int iz = 0; iz < np_h; iz++) {
            double z = iz * delta_h;
            double rz = rad * (height - z) / height;
            double rad_out = rz;
            double rad_in = std::max(rad_out - num_layers * spacing, 0.0);
            if (iz >= np_h - num_layers)
                rad_in = 0;
            int np_r = (int)std::round((rad_out - rad_in) / spacing);
            double delta_r = (rad_out - rad_in) / np_r;
            for (int ir = 0; ir <= np_r; ir++) {
                double r = rad_in + ir * delta_r;
                int np_th = (int)std::round(2 * pi * r / spacing);
                double delta_th = (2 * pi) / np_th;
                for (int it = 0; it < np_th; it++) {
                    double theta = it * delta_th;
                    double x = r * cos(theta);
                    double y = r * sin(theta);
                    bce.push_back({x, y, z});
                }
            }
        }

        bce.push_back({0.0, 0.0, height});

        //// TODO: add cap

        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad / spacing);
    double delta_r = rad / np_r;

    for (int iz = 0; iz <= np_h; iz++) {
        double z = iz * delta_h;
        double rz = rad * (height - z) / height;
        double rad_out = rz;
        double rad_in = std::max(rad_out - num_layers * spacing, 0.0);
        double r_out2 = rad_out * rad_out;
        double r_in2 = rad_in * rad_in;
        for (int ix = -np_r; ix <= np_r; ix++) {
            double x = ix * delta_r;
            for (int iy = -np_r; iy <= np_r; iy++) {
                double y = iy * delta_r;
                double r2 = x * x + y * y;
                if (r2 >= r_in2 && r2 <= r_out2) {
                    bce.push_back({x, y, z});
                }
            }

            //// TODO: add cap
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCE_ConeExterior(double rad, double height, bool polar, std::vector<ChVector3d>& bce) {
    double spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Calculate actual spacing
    int np_h = (int)std::round(height / spacing);
    double delta_h = height / np_h;

    // Inflate cone
    np_h += 2 * num_layers;
    height += 2 * num_layers * delta_h;

    // Use polar coordinates
    if (polar) {
        for (int iz = 0; iz < np_h; iz++) {
            double z = iz * delta_h;
            double rz = rad * (height - z) / height;
            double rad_out = rz + num_layers * spacing;
            double rad_in = std::max(rad_out - num_layers * spacing, 0.0);
            if (iz >= np_h - num_layers)
                rad_in = 0;
            int np_r = (int)std::round((rad_out - rad_in) / spacing);
            double delta_r = (rad_out - rad_in) / np_r;
            for (int ir = 0; ir <= np_r; ir++) {
                double r = rad_in + ir * delta_r;
                int np_th = (int)std::round(2 * pi * r / spacing);
                double delta_th = (2 * pi) / np_th;
                for (int it = 0; it < np_th; it++) {
                    double theta = it * delta_th;
                    double x = r * cos(theta);
                    double y = r * sin(theta);
                    bce.push_back({x, y, z});
                }
            }
        }

        bce.push_back({0.0, 0.0, height});

        //// TODO: add cap

        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad / spacing);
    double delta_r = rad / np_r;

    for (int iz = 0; iz <= np_h; iz++) {
        double z = iz * delta_h;
        double rz = rad * (height - z) / height;
        double rad_out = rz + num_layers * spacing;
        double rad_in = std::max(rad_out - num_layers * spacing, 0.0);
        double r_out2 = rad_out * rad_out;
        double r_in2 = rad_in * rad_in;
        for (int ix = -np_r; ix <= np_r; ix++) {
            double x = ix * delta_r;
            for (int iy = -np_r; iy <= np_r; iy++) {
                double y = iy * delta_r;
                double r2 = x * x + y * y;
                if (r2 >= r_in2 && r2 <= r_out2) {
                    bce.push_back({x, y, z});
                }
            }

            //// TODO: add cap
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

unsigned int ChFsiFluidSystemSPH::AddBCE_mesh1D(unsigned int meshID, const FsiMesh1D& fsi_mesh) {
    const auto& surface = fsi_mesh.contact_surface;

    Real spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Traverse the contact segments:
    // - calculate their discretization number n
    //   (largest number that results in a discretization no coarser than the initial spacing)
    // - generate segment coordinates for a uniform grid over the segment
    // - generate locations of BCE points on segment
    unsigned int num_seg = (unsigned int)surface->GetSegmentsXYZ().size();
    unsigned int num_bce = 0;
    for (unsigned int segID = 0; segID < num_seg; segID++) {
        const auto& seg = surface->GetSegmentsXYZ()[segID];

        const auto& P0 = seg->GetNode(0)->GetPos();  // vertex 0 position (absolute coordinates)
        const auto& P1 = seg->GetNode(1)->GetPos();  // vertex 1 position (absolute coordinates)

        const auto& V0 = seg->GetNode(0)->GetPosDt();  // vertex 0 velocity (absolute coordinates)
        const auto& V1 = seg->GetNode(1)->GetPosDt();  // vertex 1 velocity (absolute coordinates)

        auto x_dir = P1 - P0;       // segment direction
        auto len = x_dir.Length();  // segment direction
        x_dir /= len;               // normalized direction

        int n = (int)std::ceil(len / spacing);  // required divisions on segment

        // Create two directions orthogonal to 'x_dir'
        ChVector3<> y_dir(-x_dir.y() - x_dir.z(), x_dir.x() - x_dir.z(), x_dir.x() + x_dir.y());
        y_dir.Normalize();
        ChVector3<> z_dir = Vcross(x_dir, y_dir);

        unsigned int n_bce = 0;  // number of BCE markers on segment
        for (int i = 0; i <= n; i++) {
            if (i == 0 && !seg->OwnsNode(0))  // segment does not own vertex 0
                continue;
            if (i == n && !seg->OwnsNode(1))  // segment does not own vertex 1
                continue;

            auto lambda = ChVector2<>(n - i, i) / n;

            auto P = P0 * lambda[0] + P1 * lambda[1];
            auto V = V0 * lambda[0] + V1 * lambda[1];

            for (int j = -num_layers + 1; j <= num_layers - 1; j += 2) {
                for (int k = -num_layers + 1; k <= num_layers - 1; k += 2) {
                    if (m_remove_center1D && j == 0 && k == 0)
                        continue;
                    if (m_pattern1D == BcePatternMesh1D::STAR && std::abs(j) + std::abs(k) > num_layers)
                        continue;
                    double y_val = j * spacing / 2;
                    double z_val = k * spacing / 2;
                    auto Q = P + y_val * y_dir + z_val * z_dir;

                    m_data_mgr->AddBceMarker(MarkerType::BCE_FLEX1D, ToReal3(Q), ToReal3(V));

                    m_data_mgr->flex1D_BCEcoords_H.push_back(ToReal3({lambda[0], y_val, z_val}));
                    m_data_mgr->flex1D_BCEsolids_H.push_back(mU3(meshID, segID, m_num_flex1D_elements + segID));
                    n_bce++;
                }
            }
        }

        // Add the number of BCE markers for this segment
        num_bce += n_bce;
    }

    return num_bce;
}

unsigned int ChFsiFluidSystemSPH::AddBCE_mesh2D(unsigned int meshID, const FsiMesh2D& fsi_mesh) {
    const auto& surface = fsi_mesh.contact_surface;

    Real spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    ////std::ofstream ofile("mesh2D.txt");
    ////ofile << mesh->GetNumTriangles() << endl;
    ////ofile << endl;

    // Traverse the contact surface faces:
    // - calculate their discretization number n
    //   (largest number that results in a discretization no coarser than the initial spacing on each edge)
    // - generate barycentric coordinates for a uniform grid over the triangular face
    // - generate locations of BCE points on triangular face
    unsigned int num_tri = (int)surface->GetTrianglesXYZ().size();
    unsigned int num_bce = 0;
    for (unsigned int triID = 0; triID < num_tri; triID++) {
        const auto& tri = surface->GetTrianglesXYZ()[triID];

        const auto& P0 = tri->GetNode(0)->GetPos();  // vertex 0 position (absolute coordinates)
        const auto& P1 = tri->GetNode(1)->GetPos();  // vertex 1 position (absolute coordinates)
        const auto& P2 = tri->GetNode(2)->GetPos();  // vertex 2 position (absolute coordinates)

        const auto& V0 = tri->GetNode(0)->GetPosDt();  // vertex 0 velocity (absolute coordinates)
        const auto& V1 = tri->GetNode(1)->GetPosDt();  // vertex 1 velocity (absolute coordinates)
        const auto& V2 = tri->GetNode(2)->GetPosDt();  // vertex 2 velocity (absolute coordinates)

        auto normal = Vcross(P1 - P0, P2 - P1);  // triangle normal
        normal.Normalize();

        int n0 = (int)std::ceil((P2 - P1).Length() / spacing);  // required divisions on edge 0
        int n1 = (int)std::ceil((P0 - P2).Length() / spacing);  // required divisions on edge 1
        int n2 = (int)std::ceil((P1 - P0).Length() / spacing);  // required divisions on edge 2

        int n_median = max(min(n0, n1), min(max(n0, n1), n2));  // number of divisions on each edge (median)
        ////int n_max = std::max(n0, std::max(n1, n2));             // number of divisions on each edge (max)

        ////cout << "(" << n0 << " " << n1 << " " << n2 << ")";
        ////cout << "  Median : " << n_median << " Max : " << n_max << endl;

        int n = n_median;

        ////ofile << P0 << endl;
        ////ofile << P1 << endl;
        ////ofile << P2 << endl;
        ////ofile << tri->OwnsNode(0) << " " << tri->OwnsNode(1) << " " << tri->OwnsNode(2) << endl;
        ////ofile << tri->OwnsEdge(0) << " " << tri->OwnsEdge(1) << " " << tri->OwnsEdge(2) << endl;
        ////ofile << n << endl;

        bool remove_center = m_remove_center2D;
        int m_start = 0;
        int m_end = 0;
        switch (m_pattern2D) {
            case BcePatternMesh2D::INWARD:
                m_start = -2 * (num_layers - 1);
                m_end = 0;
                remove_center = false;
                break;
            case BcePatternMesh2D::CENTERED:
                m_start = -(num_layers - 1);
                m_end = +(num_layers - 1);
                break;
            case BcePatternMesh2D::OUTWARD:
                m_start = 0;
                m_end = +2 * (num_layers - 1);
                remove_center = false;
                break;
        }

        ////double z_start = centered ? (num_layers - 1) * spacing / 2 : 0;  // start layer z (along normal)

        unsigned int n_bce = 0;  // number of BCE markers on triangle
        for (int i = 0; i <= n; i++) {
            if (i == n && !tri->OwnsNode(0))  // triangle does not own vertex v0
                continue;
            if (i == 0 && !tri->OwnsEdge(1))  // triangle does not own edge v1-v2 = e1
                continue;

            for (int j = 0; j <= n - i; j++) {
                int k = n - i - j;
                auto lambda = ChVector3<>(i, j, k) / n;  // barycentric coordinates of BCE marker

                if (j == n && !tri->OwnsNode(1))  // triangle does not own vertex v1
                    continue;
                if (j == 0 && !tri->OwnsEdge(2))  // triangle does not own edge v2-v0 = e2
                    continue;

                if (k == n && !tri->OwnsNode(2))  // triangle does not own vertex v2
                    continue;
                if (k == 0 && !tri->OwnsEdge(0))  // triangle does not own edge v0-v1 = e0
                    continue;

                auto P = lambda[0] * P0 + lambda[1] * P1 + lambda[2] * P2;  // absolute coordinates of BCE marker
                auto V = lambda[0] * V0 + lambda[1] * V1 + lambda[2] * V2;  // absolute velocity of BCE marker

                // Create layers in normal direction
                for (int m = m_start; m <= m_end; m += 2) {
                    if (remove_center && m == 0)
                        continue;
                    double z_val = m * spacing / 2;
                    auto Q = P + z_val * normal;

                    m_data_mgr->AddBceMarker(MarkerType::BCE_FLEX2D, ToReal3(Q), ToReal3(V));

                    m_data_mgr->flex2D_BCEcoords_H.push_back(ToReal3({lambda[0], lambda[1], z_val}));
                    m_data_mgr->flex2D_BCEsolids_H.push_back(mU3(meshID, triID, m_num_flex2D_elements + triID));
                    n_bce++;

                    ////ofile << Q << endl;
                }
            }
        }

        ////ofile << n_bce << endl;
        ////ofile << endl;

        // Add the number of BCE markers for this triangle
        num_bce += n_bce;
    }

    ////ofile.close();

    return num_bce;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::CreatePoints_CylinderAnnulus(double rad_inner,
                                                       double rad_outer,
                                                       double height,
                                                       bool polar,
                                                       double delta,
                                                       std::vector<ChVector3d>& points) {
    // Calculate actual spacing
    double hheight = height / 2;
    int np_h = (int)std::round(hheight / delta);
    double delta_h = hheight / np_h;

    // Use polar coordinates
    if (polar) {
        int np_r = (int)std::round((rad_outer - rad_inner) / delta);
        double delta_r = (rad_outer - rad_inner) / np_r;
        for (int ir = 0; ir <= np_r; ir++) {
            double r = rad_inner + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / delta);
            double delta_th = (2 * pi) / np_th;
            for (int it = 0; it < np_th; it++) {
                double theta = it * delta_th;
                double x = r * cos(theta);
                double y = r * sin(theta);
                for (int iz = -np_h; iz <= np_h; iz++) {
                    double z = iz * delta_h;
                    points.push_back({x, y, z});
                }
            }
        }
        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad_outer / delta);
    double delta_r = rad_outer / np_r;

    double r_in2 = rad_inner * rad_inner;
    double r_out2 = rad_outer * rad_outer;
    for (int ix = -np_r; ix <= np_r; ix++) {
        double x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            double y = iy * delta_r;
            double r2 = x * x + y * y;
            if (r2 >= r_in2 && r2 <= r_out2) {
                for (int iz = -np_h; iz <= np_h; iz++) {
                    double z = iz * delta_h;
                    points.push_back({x, y, z});
                }
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreatePoints_Mesh(ChTriangleMeshConnected& mesh,
                                            double delta,
                                            std::vector<ChVector3d>& points) {
    mesh.RepairDuplicateVertexes(1e-9);  // if meshes are not watertight
    auto bbox = mesh.GetBoundingBox();

    const double EPSI = 1e-6;

    ChVector3d ray_origin;
    for (double x = bbox.min.x(); x < bbox.max.x(); x += delta) {
        ray_origin.x() = x + 1e-9;
        for (double y = bbox.min.y(); y < bbox.max.y(); y += delta) {
            ray_origin.y() = y + 1e-9;
            for (double z = bbox.min.z(); z < bbox.max.z(); z += delta) {
                ray_origin.z() = z + 1e-9;

                ChVector3d ray_dir[2] = {ChVector3d(5, 0.5, 0.25), ChVector3d(-3, 0.7, 10)};
                int intersectCounter[2] = {0, 0};

                for (unsigned int i = 0; i < mesh.m_face_v_indices.size(); ++i) {
                    auto& t_face = mesh.m_face_v_indices[i];
                    auto& v1 = mesh.m_vertices[t_face.x()];
                    auto& v2 = mesh.m_vertices[t_face.y()];
                    auto& v3 = mesh.m_vertices[t_face.z()];

                    // Find vectors for two edges sharing V1
                    auto edge1 = v2 - v1;
                    auto edge2 = v3 - v1;

                    int t_inter[2] = {0, 0};

                    for (unsigned int j = 0; j < 2; j++) {
                        // Begin calculating determinant - also used to calculate uu parameter
                        auto pvec = Vcross(ray_dir[j], edge2);
                        // if determinant is near zero, ray is parallel to plane of triangle
                        double det = Vdot(edge1, pvec);
                        // NOT CULLING
                        if (det > -EPSI && det < EPSI) {
                            t_inter[j] = 0;
                            continue;
                        }
                        double inv_det = 1.0 / det;

                        // calculate distance from V1 to ray origin
                        auto tvec = ray_origin - v1;

                        /// Calculate uu parameter and test bound
                        double uu = Vdot(tvec, pvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (uu < 0.0 || uu > 1.0) {
                            t_inter[j] = 0;
                            continue;
                        }

                        // Prepare to test vv parameter
                        auto qvec = Vcross(tvec, edge1);

                        // Calculate vv parameter and test bound
                        double vv = Vdot(ray_dir[j], qvec) * inv_det;
                        // The intersection lies outside of the triangle
                        if (vv < 0.0 || ((uu + vv) > 1.0)) {
                            t_inter[j] = 0;
                            continue;
                        }

                        double tt = Vdot(edge2, qvec) * inv_det;
                        if (tt > EPSI) {  /// ray intersection
                            t_inter[j] = 1;
                            continue;
                        }

                        // No hit, no win
                        t_inter[j] = 0;
                    }

                    intersectCounter[0] += t_inter[0];
                    intersectCounter[1] += t_inter[1];
                }

                if (((intersectCounter[0] % 2) == 1) && ((intersectCounter[1] % 2) == 1))  // inside mesh
                    points.push_back(ChVector3d(x, y, z));
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

double ChFsiFluidSystemSPH::GetKernelLength() const {
    return m_paramsH->h;
}

double ChFsiFluidSystemSPH::GetInitialSpacing() const {
    return m_paramsH->d0;
}

int ChFsiFluidSystemSPH::GetNumBCELayers() const {
    return m_paramsH->num_bce_layers;
}

ChVector3d ChFsiFluidSystemSPH::GetContainerDim() const {
    return ChVector3d(m_paramsH->boxDimX, m_paramsH->boxDimY, m_paramsH->boxDimZ);
}

double ChFsiFluidSystemSPH::GetDensity() const {
    return m_paramsH->rho0;
}

double ChFsiFluidSystemSPH::GetViscosity() const {
    return m_paramsH->mu0;
}

double ChFsiFluidSystemSPH::GetBasePressure() const {
    return m_paramsH->base_pressure;
}

double ChFsiFluidSystemSPH::GetParticleMass() const {
    return m_paramsH->markerMass;
}

ChVector3d ChFsiFluidSystemSPH::GetGravitationalAcceleration() const {
    return ChVector3d(m_paramsH->gravity.x, m_paramsH->gravity.y, m_paramsH->gravity.z);
}

double ChFsiFluidSystemSPH::GetSoundSpeed() const {
    return m_paramsH->Cs;
}

ChVector3d ChFsiFluidSystemSPH::GetBodyForce() const {
    return ChVector3d(m_paramsH->bodyForce3.x, m_paramsH->bodyForce3.y, m_paramsH->bodyForce3.z);
}

int ChFsiFluidSystemSPH::GetNumProximitySearchSteps() const {
    return m_paramsH->num_proximity_search_steps;
}

size_t ChFsiFluidSystemSPH::GetNumFluidMarkers() const {
    return m_data_mgr->countersH->numFluidMarkers;
}

size_t ChFsiFluidSystemSPH::GetNumRigidBodyMarkers() const {
    return m_data_mgr->countersH->numRigidMarkers;
}

size_t ChFsiFluidSystemSPH::GetNumFlexBodyMarkers() const {
    return m_data_mgr->countersH->numFlexMarkers1D + m_data_mgr->countersH->numFlexMarkers2D;
}

size_t ChFsiFluidSystemSPH::GetNumBoundaryMarkers() const {
    return m_data_mgr->countersH->numBoundaryMarkers;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<ChVector3d> ChFsiFluidSystemSPH::GetParticlePositions() const {
    auto pos3 = GetPositions();

    std::vector<ChVector3d> pos;
    for (const auto& p : pos3)
        pos.push_back(ToChVector(p));

    return pos;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::GetParticleVelocities() const {
    auto vel3 = GetVelocities();

    std::vector<ChVector3d> vel;
    for (const auto& v : vel3)
        vel.push_back(ToChVector(v));

    return vel;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::GetParticleAccelerations() const {
    auto acc3 = GetAccelerations();

    std::vector<ChVector3d> acc;
    for (const auto& a : acc3)
        acc.push_back(ToChVector(a));

    return acc;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::GetParticleForces() const {
    auto frc3 = GetForces();

    std::vector<ChVector3d> frc;
    for (const auto& f : frc3)
        frc.push_back(ToChVector(f));

    return frc;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::GetParticleFluidProperties() const {
    auto props3 = GetProperties();

    std::vector<ChVector3d> props;
    for (const auto& p : props3)
        props.push_back(ToChVector(p));

    return props;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<int> ChFsiFluidSystemSPH::FindParticlesInBox(const ChFrame<>& frame, const ChVector3d& size) {
    const ChVector3d& Pos = frame.GetPos();
    ChVector3d Ax = frame.GetRotMat().GetAxisX();
    ChVector3d Ay = frame.GetRotMat().GetAxisY();
    ChVector3d Az = frame.GetRotMat().GetAxisZ();

    auto hsize = 0.5 * mR3(size.x(), size.y(), size.z());
    auto pos = mR3(Pos.x(), Pos.y(), Pos.z());
    auto ax = mR3(Ax.x(), Ax.y(), Ax.z());
    auto ay = mR3(Ay.x(), Ay.y(), Ay.z());
    auto az = mR3(Az.x(), Az.y(), Az.z());

    return m_data_mgr->FindParticlesInBox(hsize, pos, ax, ay, az);
}

std::vector<Real3> ChFsiFluidSystemSPH::GetPositions() const {
    return m_data_mgr->GetPositions();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetVelocities() const {
    return m_data_mgr->GetVelocities();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetAccelerations() const {
    return m_data_mgr->GetAccelerations();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetForces() const {
    return m_data_mgr->GetForces();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetProperties() const {
    return m_data_mgr->GetProperties();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetPositions(const std::vector<int>& indices) const {
    return m_data_mgr->GetPositions(indices);
}

std::vector<Real3> ChFsiFluidSystemSPH::GetVelocities(const std::vector<int>& indices) const {
    return m_data_mgr->GetVelocities(indices);
}

std::vector<Real3> ChFsiFluidSystemSPH::GetAccelerations(const std::vector<int>& indices) const {
    return m_data_mgr->GetAccelerations(indices);
}

std::vector<Real3> ChFsiFluidSystemSPH::GetForces(const std::vector<int>& indices) const {
    return m_data_mgr->GetForces(indices);
}

}  // end namespace sph
}  // end namespace fsi
}  // end namespace chrono
