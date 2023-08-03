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
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"
#include "chrono/fea/ChContactSurfaceMesh.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/physics/ChParams.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChFsiInterface.h"
#include "chrono_fsi/physics/ChFluidDynamics.cuh"
#include "chrono_fsi/physics/ChBce.cuh"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFluid.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

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

ChSystemFsi::ChSystemFsi(ChSystem* sysMBS)
    : m_sysMBS(sysMBS),
      m_verbose(true),
      m_is_initialized(false),
      m_integrate_SPH(true),
      m_time(0),
      m_write_mode(OutputMode::NONE) {
    m_paramsH = chrono_types::make_shared<SimParams>();
    m_sysFSI = chrono_types::make_unique<ChSystemFsi_impl>(m_paramsH);
    InitParams();

    m_num_flex1D_elements = 0;
    m_num_flex2D_elements = 0;

    m_num_flex1D_nodes = 0;
    m_num_flex2D_nodes = 0;

    m_fsi_interface = chrono_types::make_unique<ChFsiInterface>(*m_sysFSI, m_paramsH);
}

ChSystemFsi::~ChSystemFsi() {}

void ChSystemFsi::AttachSystem(ChSystem* sysMBS) {
    m_sysMBS = sysMBS;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::InitParams() {
    //// RADU TODO
    //// Provide default values for *all* parameters!

    m_paramsH->output_length = 1;

    // Fluid properties
    m_paramsH->rho0 = Real(1000.0);
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;
    m_paramsH->rho_solid = m_paramsH->rho0;
    m_paramsH->mu0 = Real(0.001);
    m_paramsH->bodyForce3 = mR3(0, 0, 0);
    m_paramsH->gravity = mR3(0, 0, 0);
    m_paramsH->kappa = Real(0.0);
    m_paramsH->L_Characteristic = Real(1.0);

    // SPH parameters
    m_paramsH->fluid_dynamic_type = FluidDynamics::WCSPH;
    m_paramsH->HSML = Real(0.01);
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
    m_paramsH->INITSPACE = m_paramsH->HSML;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->MULT_INITSPACE_Shells = Real(1.0);
    m_paramsH->v_Max = Real(1.0);
    m_paramsH->EPS_XSPH = Real(0.5);
    m_paramsH->beta_shifting = Real(1.0);
    m_paramsH->densityReinit = 2147483647;
    m_paramsH->Conservative_Form = true;
    m_paramsH->gradient_type = 0;
    m_paramsH->laplacian_type = 0;
    m_paramsH->USE_Consistent_L = false;
    m_paramsH->USE_Consistent_G = false;

    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;

    m_paramsH->NUM_BOUNDARY_LAYERS = 3;

    // Time stepping
    m_paramsH->Adaptive_time_stepping = false;
    m_paramsH->Co_number = Real(0.1);
    m_paramsH->Beta = Real(0.0);
    m_paramsH->dT = Real(0.0001);
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->dT_Flex = m_paramsH->dT;
    m_paramsH->dT_Max = Real(1.0);

    // Pressure equation
    m_paramsH->PPE_Solution_type = PPESolutionType::MATRIX_FREE;
    m_paramsH->Alpha = m_paramsH->HSML;
    m_paramsH->PPE_relaxation = Real(1.0);
    m_paramsH->LinearSolver = SolverType::BICGSTAB;
    m_paramsH->LinearSolver_Abs_Tol = Real(0.0);
    m_paramsH->LinearSolver_Rel_Tol = Real(0.0);
    m_paramsH->LinearSolver_Max_Iter = 1000;
    m_paramsH->Verbose_monitoring = false;
    m_paramsH->Pressure_Constraint = false;
    m_paramsH->BASEPRES = Real(0.0);
    m_paramsH->ClampPressure = false;

    m_paramsH->bceType = BceVersion::ADAMI;
    m_paramsH->bceTypeWall = BceVersion::ADAMI;

    // Elastic SPH
    m_paramsH->C_Wi = Real(0.8);

    //
    m_paramsH->bodyActiveDomain = mR3(1e10, 1e10, 1e10);
    m_paramsH->settlingTime = Real(0);

    //
    m_paramsH->Max_Pressure = Real(1e20);

    //// RADU TODO
    //// material model

    // Elastic SPH
    ElasticMaterialProperties mat_props;
    SetElasticSPH(mat_props);
    m_paramsH->elastic_SPH = false;  // default: fluid dynamics

    m_paramsH->Cs = 10 * m_paramsH->v_Max;

    m_paramsH->use_default_limits = true;
    m_paramsH->use_init_pressure = false;
}

//--------------------------------------------------------------------------------------------------------------------------------

Real3 LoadVectorJSON(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return mR3(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

void ChSystemFsi::ReadParametersFromFile(const std::string& json_file) {
    if (m_verbose)
        cout << "Reading parameters from: " << json_file << endl;

    FILE* fp = fopen(json_file.c_str(), "r");
    if (!fp) {
        cerr << "Invalid JSON file!" << endl;
        return;
    }

    char readBuffer[32768];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    fclose(fp);

    Document doc;

    doc.ParseStream<ParseFlag::kParseCommentsFlag>(is);
    if (!doc.IsObject()) {
        cerr << "Invalid JSON file!!" << endl;
        return;
    }

    if (doc.HasMember("Data Output Length"))
        m_paramsH->output_length = doc["Data Output Length"].GetInt();

    if (doc.HasMember("Physical Properties of Fluid")) {
        if (doc["Physical Properties of Fluid"].HasMember("Density"))
            m_paramsH->rho0 = doc["Physical Properties of Fluid"]["Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Solid Density"))
            m_paramsH->rho_solid = doc["Physical Properties of Fluid"]["Solid Density"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Viscosity"))
            m_paramsH->mu0 = doc["Physical Properties of Fluid"]["Viscosity"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Body Force"))
            m_paramsH->bodyForce3 = LoadVectorJSON(doc["Physical Properties of Fluid"]["Body Force"]);

        if (doc["Physical Properties of Fluid"].HasMember("Gravity"))
            m_paramsH->gravity = LoadVectorJSON(doc["Physical Properties of Fluid"]["Gravity"]);

        if (doc["Physical Properties of Fluid"].HasMember("Surface Tension Kappa"))
            m_paramsH->kappa = doc["Physical Properties of Fluid"]["Surface Tension Kappa"].GetDouble();

        if (doc["Physical Properties of Fluid"].HasMember("Characteristic Length"))
            m_paramsH->L_Characteristic = doc["Physical Properties of Fluid"]["Characteristic Length"].GetDouble();
    }

    if (doc.HasMember("SPH Parameters")) {
        if (doc["SPH Parameters"].HasMember("Method")) {
            std::string SPH = doc["SPH Parameters"]["Method"].GetString();
            if (m_verbose)
                cout << "Modeling method is: " << SPH << endl;
            if (SPH == "I2SPH")
                m_paramsH->fluid_dynamic_type = FluidDynamics::I2SPH;
            else if (SPH == "IISPH")
                m_paramsH->fluid_dynamic_type = FluidDynamics::IISPH;
            else if (SPH == "WCSPH")
                m_paramsH->fluid_dynamic_type = FluidDynamics::WCSPH;
            else {
                cerr << "Incorrect SPH method in the JSON file: " << SPH << endl;
                cerr << "Falling back to WCSPH " << endl;
                m_paramsH->fluid_dynamic_type = FluidDynamics::WCSPH;
            }
        }

        if (doc["SPH Parameters"].HasMember("Kernel h"))
            m_paramsH->HSML = doc["SPH Parameters"]["Kernel h"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Initial Spacing"))
            m_paramsH->INITSPACE = doc["SPH Parameters"]["Initial Spacing"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Initial Spacing Solid"))
            m_paramsH->MULT_INITSPACE_Shells =
                doc["SPH Parameters"]["Initial Spacing Solid"].GetDouble() / m_paramsH->HSML;

        if (doc["SPH Parameters"].HasMember("Epsilon"))
            m_paramsH->epsMinMarkersDis = doc["SPH Parameters"]["Epsilon"].GetDouble();
        else
            m_paramsH->epsMinMarkersDis = 0.01;

        if (doc["SPH Parameters"].HasMember("Maximum Velocity"))
            m_paramsH->v_Max = doc["SPH Parameters"]["Maximum Velocity"].GetDouble();

        if (doc["SPH Parameters"].HasMember("XSPH Coefficient"))
            m_paramsH->EPS_XSPH = doc["SPH Parameters"]["XSPH Coefficient"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Viscous damping"))
            m_paramsH->Vis_Dam = doc["SPH Parameters"]["Viscous damping"].GetDouble();

        if (doc["SPH Parameters"].HasMember("Shifting Coefficient"))
            m_paramsH->beta_shifting = doc["SPH Parameters"]["Shifting Coefficient"].GetDouble();

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
    }

    if (doc.HasMember("Time Stepping")) {
        if (doc["Time Stepping"].HasMember("Adaptive Time stepping"))
            m_paramsH->Adaptive_time_stepping = doc["Time Stepping"]["Adaptive Time stepping"].GetBool();

        if (doc["Time Stepping"].HasMember("CFL number"))
            m_paramsH->Co_number = doc["Time Stepping"]["CFL number"].GetDouble();

        if (doc["Time Stepping"].HasMember("Beta"))
            m_paramsH->Beta = doc["Time Stepping"]["Beta"].GetDouble();

        if (doc["Time Stepping"].HasMember("Fluid time step"))
            m_paramsH->dT = doc["Time Stepping"]["Fluid time step"].GetDouble();

        if (doc["Time Stepping"].HasMember("Solid time step"))
            m_paramsH->dT_Flex = doc["Time Stepping"]["Solid time step"].GetDouble();
        else
            m_paramsH->dT_Flex = m_paramsH->dT;

        if (doc["Time Stepping"].HasMember("Maximum time step"))
            m_paramsH->dT_Max = doc["Time Stepping"]["Maximum time step"].GetDouble();
    }

    if (doc.HasMember("Pressure Equation")) {
        if (doc["Pressure Equation"].HasMember("Linear solver")) {
            m_paramsH->PPE_Solution_type = PPESolutionType::FORM_SPARSE_MATRIX;
            std::string solver = doc["Pressure Equation"]["Linear solver"].GetString();
            if (solver == "Jacobi") {
                m_paramsH->USE_LinearSolver = false;
            } else {
                m_paramsH->USE_LinearSolver = true;
                if (solver == "BICGSTAB")
                    m_paramsH->LinearSolver = SolverType::BICGSTAB;
                if (solver == "GMRES")
                    m_paramsH->LinearSolver = SolverType::GMRES;
            }
        }

        if (doc["Pressure Equation"].HasMember("Poisson source term")) {
            std::string source = doc["Pressure Equation"]["Poisson source term"].GetString();
            if (source == "Density-Based")
                m_paramsH->DensityBaseProjetion = true;
            else
                m_paramsH->DensityBaseProjetion = false;
        }

        if (doc["Pressure Equation"].HasMember("Projection method")) {
            std::string source = doc["Pressure Equation"]["Projection method"].GetString();
            if (source == "Incremental")
                m_paramsH->USE_NonIncrementalProjection = false;
            else
                m_paramsH->USE_NonIncrementalProjection = true;
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
                m_paramsH->BASEPRES = doc["Pressure Equation"]["Average Pressure"].GetDouble();
        }

        if (doc["Pressure Equation"].HasMember("Clamp Pressure"))
            m_paramsH->ClampPressure = doc["Pressure Equation"]["Clamp Pressure"].GetBool();

        if (doc["Pressure Equation"].HasMember("Boundary Conditions")) {
            std::string BC = doc["Pressure Equation"]["Boundary Conditions"].GetString();
            if (BC == "Generalized Wall BC")
                m_paramsH->bceType = BceVersion::ADAMI;
            else
                m_paramsH->bceType = BceVersion::ORIGINAL;
        }
    }

    // this part is for modeling granular material dynamics using elastic SPH
    if (doc.HasMember("Elastic SPH")) {
        m_paramsH->elastic_SPH = true;

        if (doc["Elastic SPH"].HasMember("Poisson ratio"))
            m_paramsH->Nu_poisson = doc["Elastic SPH"]["Poisson ratio"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Young modulus"))
            m_paramsH->E_young = doc["Elastic SPH"]["Young modulus"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial stress"))
            m_paramsH->Ar_stress = doc["Elastic SPH"]["Artificial stress"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity alpha"))
            m_paramsH->Ar_vis_alpha = doc["Elastic SPH"]["Artificial viscosity alpha"].GetDouble();

        if (doc["Elastic SPH"].HasMember("Artificial viscosity beta"))
            m_paramsH->Ar_vis_beta = doc["Elastic SPH"]["Artificial viscosity beta"].GetDouble();

        if (doc["Elastic SPH"].HasMember("I0"))
            m_paramsH->mu_I0 = doc["Elastic SPH"]["I0"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_s"))
            m_paramsH->mu_fric_s = doc["Elastic SPH"]["mu_s"].GetDouble();

        if (doc["Elastic SPH"].HasMember("mu_2"))
            m_paramsH->mu_fric_2 = doc["Elastic SPH"]["mu_2"].GetDouble();

        if (doc["Elastic SPH"].HasMember("particle diameter"))
            m_paramsH->ave_diam = doc["Elastic SPH"]["particle diameter"].GetDouble();

        if (doc["Elastic SPH"].HasMember("frictional angle"))
            m_paramsH->Fri_angle = doc["Elastic SPH"]["frictional angle"].GetDouble();

        if (doc["Elastic SPH"].HasMember("dilate angle"))
            m_paramsH->Dil_angle = doc["Elastic SPH"]["dilate angle"].GetDouble();

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

    if (doc.HasMember("Body Active Domain"))
        m_paramsH->bodyActiveDomain = LoadVectorJSON(doc["Body Active Domain"]);

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
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;

    if (m_paramsH->elastic_SPH) {
        m_paramsH->G_shear = m_paramsH->E_young / (2.0 * (1.0 + m_paramsH->Nu_poisson));
        m_paramsH->INV_G_shear = 1.0 / m_paramsH->G_shear;
        m_paramsH->K_bulk = m_paramsH->E_young / (3.0 * (1.0 - 2.0 * m_paramsH->Nu_poisson));
        m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);

        Real sfri = std::sin(m_paramsH->Fri_angle);
        Real cfri = std::cos(m_paramsH->Fri_angle);
        Real sdil = std::sin(m_paramsH->Dil_angle);
        m_paramsH->Q_FA = 6 * sfri / (sqrt(3) * (3 + sfri));
        m_paramsH->Q_DA = 6 * sdil / (sqrt(3) * (3 + sdil));
        m_paramsH->K_FA = 6 * m_paramsH->Coh_coeff * cfri / (sqrt(3) * (3 + sfri));
    } else {
        m_paramsH->Cs = 10 * m_paramsH->v_Max;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::SetVerbose(bool verbose) {
    m_verbose = verbose;
    m_fsi_interface->m_verbose = verbose;
}

void ChSystemFsi::SetSPHLinearSolver(SolverType lin_solver) {
    m_paramsH->LinearSolver = lin_solver;
}

void ChSystemFsi::SetSPHMethod(FluidDynamics SPH_method, SolverType lin_solver) {
    m_paramsH->fluid_dynamic_type = SPH_method;
    m_paramsH->LinearSolver = lin_solver;
}

void ChSystemFsi::SetContainerDim(const ChVector<>& boxDim) {
    m_paramsH->boxDimX = boxDim.x();
    m_paramsH->boxDimY = boxDim.y();
    m_paramsH->boxDimZ = boxDim.z();
}

void ChSystemFsi::SetBoundaries(const ChVector<>& cMin, const ChVector<>& cMax) {
    m_paramsH->cMin = utils::ToReal3(cMin);
    m_paramsH->cMax = utils::ToReal3(cMax);
    m_paramsH->use_default_limits = false;
}

void ChSystemFsi::SetActiveDomain(const ChVector<>& boxHalfDim) {
    m_paramsH->bodyActiveDomain = utils::ToReal3(boxHalfDim);
}

void ChSystemFsi::SetActiveDomainDelay(double duration) {
    m_paramsH->settlingTime = duration;
}

void ChSystemFsi::SetNumBoundaryLayers(int num_layers) {
    m_paramsH->NUM_BOUNDARY_LAYERS = num_layers;
}

void ChSystemFsi::SetInitPressure(const double height) {
    m_paramsH->pressure_height = height;
    m_paramsH->use_init_pressure = true;
}

void ChSystemFsi::Set_G_acc(const ChVector<>& gravity) {
    m_paramsH->gravity.x = gravity.x();
    m_paramsH->gravity.y = gravity.y();
    m_paramsH->gravity.z = gravity.z();
}

void ChSystemFsi::SetBodyForce(const ChVector<>& force) {
    m_paramsH->bodyForce3.x = force.x();
    m_paramsH->bodyForce3.y = force.y();
    m_paramsH->bodyForce3.z = force.z();
}

void ChSystemFsi::SetInitialSpacing(double spacing) {
    m_paramsH->INITSPACE = (Real)spacing;
    m_paramsH->INV_INIT = 1 / m_paramsH->INITSPACE;
    m_paramsH->volume0 = cube(m_paramsH->INITSPACE);
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
}

void ChSystemFsi::SetKernelLength(double length) {
    m_paramsH->HSML = (Real)length;
    m_paramsH->MULT_INITSPACE = m_paramsH->INITSPACE / m_paramsH->HSML;
    m_paramsH->INVHSML = 1 / m_paramsH->HSML;
}

void ChSystemFsi::SetStepSize(double dT, double dT_Flex) {
    m_paramsH->dT = dT;
    m_paramsH->INV_dT = 1 / m_paramsH->dT;
    m_paramsH->dT_Flex = (dT_Flex == 0) ? m_paramsH->dT : dT_Flex;
}

void ChSystemFsi::SetMaxStepSize(double dT_max) {
    m_paramsH->dT_Max = Real(dT_max);
}

void ChSystemFsi::SetAdaptiveTimeStepping(bool adaptive) {
    m_paramsH->Adaptive_time_stepping = adaptive;
}

void ChSystemFsi::SetSPHintegration(bool runSPH) {
    m_integrate_SPH = runSPH;
}

void ChSystemFsi::SetDensity(double rho0) {
    m_paramsH->rho0 = rho0;
    m_paramsH->invrho0 = 1 / m_paramsH->rho0;
    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;
}

void ChSystemFsi::SetDiscreType(bool useGmatrix, bool useLmatrix) {
    m_paramsH->USE_Consistent_G = useGmatrix;
    m_paramsH->USE_Consistent_L = useLmatrix;
}

void ChSystemFsi::SetOutputLength(int OutputLength) {
    m_paramsH->output_length = OutputLength;
}

void ChSystemFsi::SetWallBC(BceVersion wallBC) {
    m_paramsH->bceTypeWall = wallBC;
}

void ChSystemFsi::SetRigidBodyBC(BceVersion rigidBodyBC) {
    m_paramsH->bceType = rigidBodyBC;
}

void ChSystemFsi::SetCohesionForce(double Fc) {
    m_paramsH->Coh_coeff = Fc;
}

ChSystemFsi::ElasticMaterialProperties::ElasticMaterialProperties()
    : Young_modulus(1e6),
      Poisson_ratio(0.3),
      stress(0),
      viscosity_alpha(0.5),
      viscosity_beta(0),
      mu_I0(0.03),
      mu_fric_s(0.7),
      mu_fric_2(0.7),
      average_diam(0.005),
      friction_angle(CH_C_PI / 10),
      dilation_angle(CH_C_PI / 10),
      cohesion_coeff(0),
      kernel_threshold(0.8) {}

void ChSystemFsi::SetElasticSPH(const ElasticMaterialProperties mat_props) {
    m_paramsH->elastic_SPH = true;

    m_paramsH->E_young = Real(mat_props.Young_modulus);
    m_paramsH->Nu_poisson = Real(mat_props.Poisson_ratio);
    m_paramsH->Ar_stress = Real(mat_props.stress);
    m_paramsH->Ar_vis_alpha = Real(mat_props.viscosity_alpha);
    m_paramsH->Ar_vis_beta = Real(mat_props.viscosity_beta);
    m_paramsH->mu_I0 = Real(mat_props.mu_I0);
    m_paramsH->mu_fric_s = Real(mat_props.mu_fric_s);
    m_paramsH->mu_fric_2 = Real(mat_props.mu_fric_2);
    m_paramsH->ave_diam = Real(mat_props.average_diam);
    m_paramsH->Fri_angle = Real(mat_props.friction_angle);
    m_paramsH->Dil_angle = Real(mat_props.dilation_angle);
    m_paramsH->Coh_coeff = Real(mat_props.cohesion_coeff);
    m_paramsH->C_Wi = Real(mat_props.kernel_threshold);

    m_paramsH->G_shear = m_paramsH->E_young / (2.0 * (1.0 + m_paramsH->Nu_poisson));
    m_paramsH->INV_G_shear = 1.0 / m_paramsH->G_shear;
    m_paramsH->K_bulk = m_paramsH->E_young / (3.0 * (1.0 - 2.0 * m_paramsH->Nu_poisson));
    m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);

    Real sfri = std::sin(m_paramsH->Fri_angle);
    Real cfri = std::cos(m_paramsH->Fri_angle);
    Real sdil = std::sin(m_paramsH->Dil_angle);
    m_paramsH->Q_FA = 6 * sfri / (sqrt(3) * (3 + sfri));
    m_paramsH->Q_DA = 6 * sdil / (sqrt(3) * (3 + sdil));
    m_paramsH->K_FA = 6 * m_paramsH->Coh_coeff * cfri / (sqrt(3) * (3 + sfri));
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddFsiBody(std::shared_ptr<ChBody> body) {
    m_fsi_interface->m_fsi_bodies.push_back(body);
}

void ChSystemFsi::AddFsiMesh1D(std::shared_ptr<fea::ChMesh> mesh) {
    ChFsiInterface::FsiMesh1D fsi_mesh;

    // Traverse all elements in the provided mesh and extract the ANCF cable elements.
    // Keep track of node ownership
    std::set<fea::ChNodeFEAxyz*> assigned;
    for (const auto& element : mesh->GetElements()) {
        if (auto cable_el = std::dynamic_pointer_cast<fea::ChElementCableANCF>(element)) {
            std::shared_ptr<fea::ChNodeFEAxyz> node0 = cable_el->GetNodeA();
            std::shared_ptr<fea::ChNodeFEAxyz> node1 = cable_el->GetNodeB();
            ChVector2<bool> owns_node = {false, false};
            if (assigned.count(node0.get()) == 0) {
                assigned.insert(node0.get());
                owns_node[0] = true;
            }
            if (assigned.count(node1.get()) == 0) {
                assigned.insert(node1.get());
                owns_node[1] = true;
            }
            auto segment = chrono_types::make_shared<fea::ChContactSegmentXYZ>();
            segment->SetNodes({{node0, node1}});
            segment->SetNodeOwnership(owns_node);
            fsi_mesh.segments.push_back(segment);
        }
    }

    // Create maps from pointer-based to index-based for the nodes in the mesh contact segments.
    // These maps index only the nodes that are in ANCF cable elements (and not all nodes in the given FEA mesh).
    int vertex_index = 0;
    for (const auto& seg : fsi_mesh.segments) {
        if (fsi_mesh.ptr2ind_map.insert({seg->GetNode(0), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, seg->GetNode(0)});
            ++vertex_index;
        }
        if (fsi_mesh.ptr2ind_map.insert({seg->GetNode(1), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, seg->GetNode(1)});
            ++vertex_index;
        }
    }

    // Load index-based mesh connectivity (append to global list of 1-D flex segments)
    for (const auto& seg : fsi_mesh.segments) {
        auto node0_index = fsi_mesh.ptr2ind_map[seg->GetNode(0)];
        auto node1_index = fsi_mesh.ptr2ind_map[seg->GetNode(1)];
        m_sysFSI->fsiData->flex1D_Nodes_H.push_back(mI2(node0_index, node1_index));
    }

    // Create the BCE markers based on the mesh contact segments
    unsigned int meshID = (unsigned int)m_fsi_interface->m_fsi_meshes1D.size();
    fsi_mesh.num_bce = AddBCE_mesh1D(meshID, fsi_mesh);

    // Update total number of flex 1-D segments and associated nodes
    m_num_flex1D_elements += fsi_mesh.segments.size();
    m_num_flex1D_nodes += fsi_mesh.ind2ptr_map.size();

    // Store the mesh contact surface
    m_fsi_interface->m_fsi_meshes1D.push_back(fsi_mesh);
}

void ChSystemFsi::AddFsiMesh2D(std::shared_ptr<fea::ChMesh> mesh, bool centered) {
    std::shared_ptr<fea::ChContactSurfaceMesh> contact_surface;

    // Search for a contact surface mesh associated with the FEA mesh
    for (const auto& surface : mesh->GetContactSurfaces()) {
        if (auto surface_mesh = std::dynamic_pointer_cast<fea::ChContactSurfaceMesh>(surface)) {
            contact_surface = surface_mesh;
            break;
        }
    }

    // If none found, create one with a default contact material and extract the boundary faces of the FEA mesh
    if (!contact_surface) {
        ChContactMaterialData contact_material_data;  // default contact material
        contact_surface = chrono_types::make_shared<fea::ChContactSurfaceMesh>(
            contact_material_data.CreateMaterial(ChContactMethod::SMC));
        mesh->AddContactSurface(contact_surface);
        contact_surface->AddFacesFromBoundary(0.1);
    }

    ChFsiInterface::FsiMesh2D fsi_mesh;
    fsi_mesh.contact_surface = contact_surface;

    // Create maps from pointer-based to index-based for the nodes in the mesh contact surface.
    // These maps index only the nodes that are in the contact surface (and not all nodes in the given FEA mesh).
    int vertex_index = 0;
    for (const auto& tri : contact_surface->GetTriangleList()) {
        if (fsi_mesh.ptr2ind_map.insert({tri->GetNode(0), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, tri->GetNode(0)});
            ++vertex_index;
        }
        if (fsi_mesh.ptr2ind_map.insert({tri->GetNode(1), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, tri->GetNode(1)});
            ++vertex_index;
        }
        if (fsi_mesh.ptr2ind_map.insert({tri->GetNode(2), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, tri->GetNode(2)});
            ++vertex_index;
        }
    }

    assert(fsi_mesh.ptr2ind_map.size() == contact_surface->GetNumVertices());
    assert(fsi_mesh.ind2ptr_map.size() == contact_surface->GetNumVertices());

    // Load index-based mesh connectivity (append to global list of 1-D flex segments)
    for (const auto& tri : contact_surface->GetTriangleList()) {
        auto node0_index = fsi_mesh.ptr2ind_map[tri->GetNode(0)];
        auto node1_index = fsi_mesh.ptr2ind_map[tri->GetNode(1)];
        auto node2_index = fsi_mesh.ptr2ind_map[tri->GetNode(2)];
        m_sysFSI->fsiData->flex2D_Nodes_H.push_back(mI3(node0_index, node1_index, node2_index));
    }

    // Create the BCE markers based on the mesh contact surface
    unsigned int meshID = (unsigned int)m_fsi_interface->m_fsi_meshes2D.size();
    fsi_mesh.num_bce = AddBCE_mesh2D(meshID, fsi_mesh, centered);

    // Update total number of flex 2-D faces and associated nodes
    m_num_flex2D_elements += fsi_mesh.contact_surface->GetNumTriangles();
    m_num_flex2D_nodes += fsi_mesh.ind2ptr_map.size();

    // Store the mesh contact surface
    m_fsi_interface->m_fsi_meshes2D.push_back(fsi_mesh);

    //// TODO - load necessary structures and arrays
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::Initialize() {
    // Calculate the initial neighour particle number
    m_paramsH->markerMass = utils::massCalculator(m_paramsH->HSML, m_paramsH->INITSPACE, m_paramsH->rho0);
    m_paramsH->num_neighbors = utils::IniNeiNum(m_paramsH->HSML, m_paramsH->INITSPACE);

    if (m_paramsH->use_default_limits) {
        m_paramsH->cMin =
            mR3(-2 * m_paramsH->boxDimX, -2 * m_paramsH->boxDimY, -2 * m_paramsH->boxDimZ) - 10 * mR3(m_paramsH->HSML);
        m_paramsH->cMax =
            mR3(+2 * m_paramsH->boxDimX, +2 * m_paramsH->boxDimY, +2 * m_paramsH->boxDimZ) + 10 * mR3(m_paramsH->HSML);
    }

    if (m_paramsH->use_init_pressure) {
        size_t numParticles = m_sysFSI->sphMarkers_H->rhoPresMuH.size();
        for (int i = 0; i < numParticles; i++) {
            double z = m_sysFSI->sphMarkers_H->posRadH[i].z;
            double p = m_paramsH->rho0 * m_paramsH->gravity.z * (z - m_paramsH->pressure_height);
            m_sysFSI->sphMarkers_H->rhoPresMuH[i].y = p;
            if (m_paramsH->elastic_SPH) {
                m_sysFSI->sphMarkers_H->tauXxYyZzH[i].x = -p;
                m_sysFSI->sphMarkers_H->tauXxYyZzH[i].y = -p;
                m_sysFSI->sphMarkers_H->tauXxYyZzH[i].z = -p;
            }
        }
    }

    // Set up subdomains for faster neighbor particle search
    m_paramsH->NUM_BOUNDARY_LAYERS = 3;
    m_paramsH->Apply_BC_U = false;  // You should go to custom_math.h all the way to end of file and set your function
    int3 side0 = mI3((int)floor((m_paramsH->cMax.x - m_paramsH->cMin.x) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)),
                     (int)floor((m_paramsH->cMax.y - m_paramsH->cMin.y) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)),
                     (int)floor((m_paramsH->cMax.z - m_paramsH->cMin.z) / (RESOLUTION_LENGTH_MULT * m_paramsH->HSML)));
    Real3 binSize3 =
        mR3((m_paramsH->cMax.x - m_paramsH->cMin.x) / side0.x, (m_paramsH->cMax.y - m_paramsH->cMin.y) / side0.y,
            (m_paramsH->cMax.z - m_paramsH->cMin.z) / side0.z);
    m_paramsH->binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
    m_paramsH->binSize0 = binSize3.x;
    m_paramsH->boxDims = m_paramsH->cMax - m_paramsH->cMin;
    m_paramsH->straightChannelBoundaryMin = m_paramsH->cMin;  // mR3(0, 0, 0);  // 3D channel
    m_paramsH->straightChannelBoundaryMax = m_paramsH->cMax;  // SmR3(3, 2, 3) * m_paramsH->sizeScale;
    m_paramsH->deltaPress = mR3(0);
    int3 SIDE = mI3(int((m_paramsH->cMax.x - m_paramsH->cMin.x) / m_paramsH->binSize0 + .1),
                    int((m_paramsH->cMax.y - m_paramsH->cMin.y) / m_paramsH->binSize0 + .1),
                    int((m_paramsH->cMax.z - m_paramsH->cMin.z) / m_paramsH->binSize0 + .1));
    Real mBinSize = m_paramsH->binSize0;
    m_paramsH->gridSize = SIDE;
    m_paramsH->worldOrigin = m_paramsH->cMin;
    m_paramsH->cellSize = mR3(mBinSize, mBinSize, mBinSize);

    // Print information
    if (m_verbose) {
        cout << "Simulation parameters" << endl;

        cout << "  num_neighbors: " << m_paramsH->num_neighbors << endl;
        cout << "  rho0: " << m_paramsH->rho0 << endl;
        cout << "  invrho0: " << m_paramsH->invrho0 << endl;
        cout << "  mu0: " << m_paramsH->mu0 << endl;
        cout << "  bodyForce3: " << m_paramsH->bodyForce3.x << " " << m_paramsH->bodyForce3.y << " "
             << m_paramsH->bodyForce3.z << endl;
        cout << "  gravity: " << m_paramsH->gravity.x << " " << m_paramsH->gravity.y << " " << m_paramsH->gravity.z
             << endl;

        cout << "  HSML: " << m_paramsH->HSML << endl;
        cout << "  INITSPACE: " << m_paramsH->INITSPACE << endl;
        cout << "  INV_INIT: " << m_paramsH->INV_INIT << endl;
        cout << "  MULT_INITSPACE: " << m_paramsH->MULT_INITSPACE << endl;
        cout << "  NUM_BOUNDARY_LAYERS: " << m_paramsH->NUM_BOUNDARY_LAYERS << endl;
        cout << "  epsMinMarkersDis: " << m_paramsH->epsMinMarkersDis << endl;
        cout << "  markerMass: " << m_paramsH->markerMass << endl;
        cout << "  volume0: " << m_paramsH->volume0 << endl;
        cout << "  gradient_type: " << m_paramsH->gradient_type << endl;

        cout << "  v_Max: " << m_paramsH->v_Max << endl;
        cout << "  Cs: " << m_paramsH->Cs << endl;
        cout << "  EPS_XSPH: " << m_paramsH->EPS_XSPH << endl;
        cout << "  beta_shifting: " << m_paramsH->beta_shifting << endl;
        cout << "  densityReinit: " << m_paramsH->densityReinit << endl;

        cout << "  Adaptive_time_stepping: " << m_paramsH->Adaptive_time_stepping << endl;
        cout << "  Co_number: " << m_paramsH->Co_number << endl;
        cout << "  dT: " << m_paramsH->dT << endl;
        cout << "  INV_dT: " << m_paramsH->INV_dT << endl;
        cout << "  dT_Max: " << m_paramsH->dT_Max << endl;
        cout << "  dT_Flex: " << m_paramsH->dT_Flex << endl;

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

        cout << "  bceType: " << (int)m_paramsH->bceType << endl;
        cout << "  USE_NonIncrementalProjection : " << m_paramsH->USE_NonIncrementalProjection << endl;
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
    }

    // Initialize the underlying FSU system: set reference arrays, set counters, and resize simulation arrays
    m_sysFSI->Initialize(m_fsi_interface->m_fsi_bodies.size(),          //
                         m_num_flex1D_elements, m_num_flex2D_elements,  //
                         m_num_flex1D_nodes, m_num_flex2D_nodes);

    if (m_verbose) {
        cout << "Counters" << endl;
        cout << "  numRigidBodies:     " << m_sysFSI->numObjectsH->numRigidBodies << endl;
        cout << "  numFlexBodies1D:    " << m_sysFSI->numObjectsH->numFlexBodies1D << endl;
        cout << "  numFlexBodies2D:    " << m_sysFSI->numObjectsH->numFlexBodies2D << endl;
        cout << "  numFlexNodes1D:     " << m_sysFSI->numObjectsH->numFlexNodes1D << endl;
        cout << "  numFlexNodes2D:     " << m_sysFSI->numObjectsH->numFlexNodes2D << endl;
        cout << "  numGhostMarkers:    " << m_sysFSI->numObjectsH->numGhostMarkers << endl;
        cout << "  numHelperMarkers:   " << m_sysFSI->numObjectsH->numHelperMarkers << endl;
        cout << "  numFluidMarkers:    " << m_sysFSI->numObjectsH->numFluidMarkers << endl;
        cout << "  numBoundaryMarkers: " << m_sysFSI->numObjectsH->numBoundaryMarkers << endl;
        cout << "  numRigidMarkers:    " << m_sysFSI->numObjectsH->numRigidMarkers << endl;
        cout << "  numFlexMarkers1D:   " << m_sysFSI->numObjectsH->numFlexMarkers1D << endl;
        cout << "  numFlexMarkers2D:   " << m_sysFSI->numObjectsH->numFlexMarkers2D << endl;
        cout << "  numAllMarkers:      " << m_sysFSI->numObjectsH->numAllMarkers << endl;
        cout << "  startRigidMarkers:  " << m_sysFSI->numObjectsH->startRigidMarkers << endl;
        cout << "  startFlexMarkers1D: " << m_sysFSI->numObjectsH->startFlexMarkers1D << endl;
        cout << "  startFlexMarkers2D: " << m_sysFSI->numObjectsH->startFlexMarkers2D << endl;

        cout << "Reference array (size: " << m_sysFSI->fsiData->referenceArray.size() << ")" << endl;
        for (size_t i = 0; i < m_sysFSI->fsiData->referenceArray.size(); i++) {
            const int4& num = m_sysFSI->fsiData->referenceArray[i];
            cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
        }
        cout << "Reference array FEA (size: " << m_sysFSI->fsiData->referenceArray_FEA.size() << ")" << endl;
        for (size_t i = 0; i < m_sysFSI->fsiData->referenceArray_FEA.size(); i++) {
            const int4& num = m_sysFSI->fsiData->referenceArray_FEA[i];
            cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
        }
    }

    m_fsi_interface->LoadBodyState_Chrono2Fsi(m_sysFSI->fsiBodyState1_D);
    m_fsi_interface->LoadMesh1DState_Chrono2Fsi(m_sysFSI->fsiMesh1DState_D);
    m_fsi_interface->LoadMesh2DState_Chrono2Fsi(m_sysFSI->fsiMesh2DState_D);

    // Construct midpoint rigid data
    m_sysFSI->fsiBodyState2_D = m_sysFSI->fsiBodyState1_D;

    // Create BCE and SPH worker objects
    m_bce_manager = chrono_types::make_shared<ChBce>(m_sysFSI->sortedSphMarkers_D, m_sysFSI->markersProximity_D,
                                                     m_sysFSI->fsiData, m_paramsH, m_sysFSI->numObjectsH, m_verbose);

    switch (m_paramsH->fluid_dynamic_type) {
        case FluidDynamics::IISPH:
            fluidIntegrator = TimeIntegrator::IISPH;
            break;
        case FluidDynamics::WCSPH:
            fluidIntegrator = TimeIntegrator::EXPLICITSPH;
            break;
        default:
            fluidIntegrator = TimeIntegrator::I2SPH;
            break;
    }
    m_fluid_dynamics = chrono_types::make_unique<ChFluidDynamics>(m_bce_manager, *m_sysFSI, m_paramsH,
                                                                  m_sysFSI->numObjectsH, fluidIntegrator, m_verbose);
    m_fluid_dynamics->GetForceSystem()->SetLinearSolver(m_paramsH->LinearSolver);

    // Initialize worker objects
    m_bce_manager->Initialize(m_sysFSI->sphMarkers1_D,                                  //
                              m_sysFSI->fsiBodyState1_D,                               //
                              m_sysFSI->fsiMesh1DState_D, m_sysFSI->fsiMesh2DState_D,  //
                              m_fsi_bodies_bce_num);
    m_fluid_dynamics->Initialize();

    // Mark system as initialized
    m_is_initialized = true;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::CopyDeviceDataToHalfStep() {
    thrust::copy(m_sysFSI->sphMarkers2_D->posRadD.begin(), m_sysFSI->sphMarkers2_D->posRadD.end(),
                 m_sysFSI->sphMarkers1_D->posRadD.begin());
    thrust::copy(m_sysFSI->sphMarkers2_D->velMasD.begin(), m_sysFSI->sphMarkers2_D->velMasD.end(),
                 m_sysFSI->sphMarkers1_D->velMasD.begin());
    thrust::copy(m_sysFSI->sphMarkers2_D->rhoPresMuD.begin(), m_sysFSI->sphMarkers2_D->rhoPresMuD.end(),
                 m_sysFSI->sphMarkers1_D->rhoPresMuD.begin());
    if (m_paramsH->elastic_SPH) {
        thrust::copy(m_sysFSI->sphMarkers2_D->tauXxYyZzD.begin(), m_sysFSI->sphMarkers2_D->tauXxYyZzD.end(),
                     m_sysFSI->sphMarkers1_D->tauXxYyZzD.begin());
        thrust::copy(m_sysFSI->sphMarkers2_D->tauXyXzYzD.begin(), m_sysFSI->sphMarkers2_D->tauXyXzYzD.end(),
                     m_sysFSI->sphMarkers1_D->tauXyXzYzD.begin());
    }
}

void ChSystemFsi::DoStepDynamics_FSI() {
    if (!m_is_initialized) {
        cout << "ERROR: FSI system not initialized!\n" << endl;
        throw std::runtime_error("FSI system not initialized!\n");
    }

    m_timer_step.reset();
    m_timer_step.start();

    if (m_fluid_dynamics->GetIntegratorType() == TimeIntegrator::EXPLICITSPH) {
        // The following is used to execute the Explicit WCSPH
        CopyDeviceDataToHalfStep();
        thrust::copy(m_sysFSI->fsiData->derivVelRhoD.begin(), m_sysFSI->fsiData->derivVelRhoD.end(),
                     m_sysFSI->fsiData->derivVelRhoD_old.begin());
        ChUtilsDevice::FillVector(m_sysFSI->fsiData->derivVelRhoD, mR4(0));

        if (m_integrate_SPH) {
            m_fluid_dynamics->IntegrateSPH(m_sysFSI->sphMarkers2_D, m_sysFSI->sphMarkers1_D,          //
                                           m_sysFSI->fsiBodyState2_D,                               //
                                           m_sysFSI->fsiMesh1DState_D, m_sysFSI->fsiMesh2DState_D,  //
                                           0.5 * m_paramsH->dT, m_time);
            m_fluid_dynamics->IntegrateSPH(m_sysFSI->sphMarkers1_D, m_sysFSI->sphMarkers2_D,          //
                                           m_sysFSI->fsiBodyState2_D,                               //
                                           m_sysFSI->fsiMesh1DState_D, m_sysFSI->fsiMesh2DState_D,  //
                                           1.0 * m_paramsH->dT, m_time);
        }

        m_bce_manager->Rigid_Forces_Torques(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiBodyState2_D);
        m_bce_manager->Flex1D_Forces(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiMesh1DState_D);
        m_bce_manager->Flex2D_Forces(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiMesh2DState_D);

        // Advance dynamics of the associated MBS system (if provided)
        if (m_sysMBS) {
            m_fsi_interface->ApplyBodyForce_Fsi2Chrono();
            m_fsi_interface->ApplyMesh1DForce_Fsi2Chrono();
            m_fsi_interface->ApplyMesh2DForce_Fsi2Chrono();

            if (m_paramsH->dT_Flex == 0)
                m_paramsH->dT_Flex = m_paramsH->dT;
            int sync = int(m_paramsH->dT / m_paramsH->dT_Flex);
            if (sync < 1)
                sync = 1;
            for (int t = 0; t < sync; t++) {
                m_sysMBS->DoStepDynamics(m_paramsH->dT / sync);
            }
        }

        m_fsi_interface->LoadBodyState_Chrono2Fsi(m_sysFSI->fsiBodyState2_D);
        m_bce_manager->UpdateBodyMarkerState(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiBodyState2_D);

        m_fsi_interface->LoadMesh1DState_Chrono2Fsi(m_sysFSI->fsiMesh1DState_D);
        m_bce_manager->UpdateMeshMarker1DState(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiMesh1DState_D);

        m_fsi_interface->LoadMesh2DState_Chrono2Fsi(m_sysFSI->fsiMesh2DState_D);
        m_bce_manager->UpdateMeshMarker2DState(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiMesh2DState_D);
    } else {
        // A different coupling scheme is used for implicit SPH formulations
        if (m_integrate_SPH) {
            m_fluid_dynamics->IntegrateSPH(m_sysFSI->sphMarkers2_D, m_sysFSI->sphMarkers2_D,          //
                                           m_sysFSI->fsiBodyState2_D,                               //
                                           m_sysFSI->fsiMesh1DState_D, m_sysFSI->fsiMesh2DState_D,  //
                                           0.0, m_time);
        }

        m_bce_manager->Rigid_Forces_Torques(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiBodyState2_D);
        m_bce_manager->Flex1D_Forces(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiMesh1DState_D);
        m_bce_manager->Flex2D_Forces(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiMesh2DState_D);

        // Advance dynamics of the associated MBS system (if provided)
        if (m_sysMBS) {
            m_fsi_interface->ApplyBodyForce_Fsi2Chrono();
            m_fsi_interface->ApplyMesh1DForce_Fsi2Chrono();
            m_fsi_interface->ApplyMesh2DForce_Fsi2Chrono();

            if (m_paramsH->dT_Flex == 0)
                m_paramsH->dT_Flex = m_paramsH->dT;
            int sync = int(m_paramsH->dT / m_paramsH->dT_Flex);
            if (sync < 1)
                sync = 1;
            if (m_verbose)
                cout << sync << " * Chrono StepDynamics with dt = " << m_paramsH->dT / sync << endl;
            for (int t = 0; t < sync; t++) {
                m_sysMBS->DoStepDynamics(m_paramsH->dT / sync);
            }
        }

        m_fsi_interface->LoadBodyState_Chrono2Fsi(m_sysFSI->fsiBodyState2_D);
        m_bce_manager->UpdateBodyMarkerState(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiBodyState2_D);

        m_fsi_interface->LoadMesh1DState_Chrono2Fsi(m_sysFSI->fsiMesh1DState_D);
        m_bce_manager->UpdateMeshMarker1DState(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiMesh1DState_D);

        m_fsi_interface->LoadMesh2DState_Chrono2Fsi(m_sysFSI->fsiMesh2DState_D);
        m_bce_manager->UpdateMeshMarker2DState(m_sysFSI->sphMarkers2_D, m_sysFSI->fsiMesh2DState_D);
    }

    m_time += m_paramsH->dT;

    m_timer_step.stop();
    m_RTF = m_timer_step() / m_paramsH->dT;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::WriteParticleFile(const std::string& outfilename) const {
    if (m_write_mode == OutputMode::CSV) {
        utils::WriteCsvParticlesToFile(m_sysFSI->sphMarkers2_D->posRadD, m_sysFSI->sphMarkers2_D->velMasD,
                                       m_sysFSI->sphMarkers2_D->rhoPresMuD, m_sysFSI->fsiData->referenceArray,
                                       outfilename);
    } else if (m_write_mode == OutputMode::CHPF) {
        utils::WriteChPFParticlesToFile(m_sysFSI->sphMarkers2_D->posRadD, m_sysFSI->fsiData->referenceArray,
                                        outfilename);
    }
}

void ChSystemFsi::PrintParticleToFile(const std::string& dir) const {
    utils::PrintParticleToFile(m_sysFSI->sphMarkers2_D->posRadD, m_sysFSI->sphMarkers2_D->velMasD,
                               m_sysFSI->sphMarkers2_D->rhoPresMuD, m_sysFSI->fsiData->sr_tau_I_mu_i,
                               m_sysFSI->fsiData->derivVelRhoD, m_sysFSI->fsiData->referenceArray,
                               m_sysFSI->fsiData->referenceArray_FEA, dir, m_paramsH);
}

void ChSystemFsi::PrintFsiInfoToFile(const std::string& dir, double time) const {
    utils::PrintFsiInfoToFile(                                                                 //
        m_sysFSI->fsiBodyState2_D->pos, m_sysFSI->fsiBodyState2_D->rot,                        //
        m_sysFSI->fsiBodyState2_D->lin_vel,                                                    //
        m_sysFSI->fsiMesh1DState_D->pos_fsi_fea_D, m_sysFSI->fsiMesh2DState_D->pos_fsi_fea_D,  //
        m_sysFSI->fsiMesh1DState_D->vel_fsi_fea_D, m_sysFSI->fsiMesh2DState_D->vel_fsi_fea_D,  //
        m_sysFSI->fsiData->rigid_FSI_ForcesD, m_sysFSI->fsiData->rigid_FSI_TorquesD,           //
        m_sysFSI->fsiData->flex1D_FSIforces_D, m_sysFSI->fsiData->flex2D_FSIforces_D,          //
        dir, time);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddSPHParticle(const ChVector<>& point,
                                 double rho0,
                                 double pres0,
                                 double mu0,
                                 const ChVector<>& velocity,
                                 const ChVector<>& tauXxYyZz,
                                 const ChVector<>& tauXyXzYz) {
    Real h = m_paramsH->HSML;
    m_sysFSI->AddSPHParticle(utils::ToReal4(point, h), mR4(rho0, pres0, mu0, -1), utils::ToReal3(velocity),
                             utils::ToReal3(tauXxYyZz), utils::ToReal3(tauXyXzYz));
}

void ChSystemFsi::AddSPHParticle(const ChVector<>& point,
                                 const ChVector<>& velocity,
                                 const ChVector<>& tauXxYyZz,
                                 const ChVector<>& tauXyXzYz) {
    AddSPHParticle(point, m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, velocity, tauXxYyZz, tauXyXzYz);
}

void ChSystemFsi::AddBoxSPH(const ChVector<>& boxCenter, const ChVector<>& boxHalfDim) {
    // Use a chrono sampler to create a bucket of points
    chrono::utils::GridSampler<> sampler(m_paramsH->INITSPACE);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        AddSPHParticle(points[i], m_paramsH->rho0, 0, m_paramsH->mu0,
                       ChVector<>(0),   // initial velocity
                       ChVector<>(0),   // tauxxyyzz
                       ChVector<>(0));  // tauxyxzyz
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddWallBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector2<> size) {
    thrust::host_vector<Real4> bce;
    CreateBCE_wall(mR2(size.x(), size.y()), bce);
    AddBCE_body(body, bce, frame, false, false, false);
}

void ChSystemFsi::AddBoxContainerBCE(std::shared_ptr<ChBody> body,
                                     const ChFrame<>& frame,
                                     const ChVector<>& size,
                                     const ChVector<int> faces) {
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    Real buffer = 2 * m_paramsH->NUM_BOUNDARY_LAYERS * spacing;

    ChVector<> hsize = size / 2;

    // Wall center positions
    ChVector<> xn(-hsize.x() - spacing, 0, 0);
    ChVector<> xp(+hsize.x() + spacing, 0, 0);
    ChVector<> yn(0, -hsize.y() - spacing, 0);
    ChVector<> yp(0, +hsize.y() + spacing, 0);
    ChVector<> zn(0, 0, -hsize.z() - spacing);
    ChVector<> zp(0, 0, +hsize.z() + spacing);

    // Z- wall
    if (faces.z() == -1 || faces.z() == 2)
        AddWallBCE(body, frame * ChFrame<>(zn, QUNIT), {size.x(), size.y()});
    // Z+ wall
    if (faces.z() == +1 || faces.z() == 2)
        AddWallBCE(body, frame * ChFrame<>(zp, Q_from_AngX(CH_C_PI)), {size.x(), size.y()});

    // X- wall
    if (faces.x() == -1 || faces.x() == 2)
        AddWallBCE(body, frame * ChFrame<>(xn, Q_from_AngY(+CH_C_PI_2)), {size.z() + buffer, size.y()});
    // X+ wall
    if (faces.x() == +1 || faces.x() == 2)
        AddWallBCE(body, frame * ChFrame<>(xp, Q_from_AngY(-CH_C_PI_2)), {size.z() + buffer, size.y()});

    // Y- wall
    if (faces.y() == -1 || faces.y() == 2)
        AddWallBCE(body, frame * ChFrame<>(yn, Q_from_AngX(-CH_C_PI_2)), {size.x() + buffer, size.z() + buffer});
    // Y+ wall
    if (faces.y() == +1 || faces.y() == 2)
        AddWallBCE(body, frame * ChFrame<>(yp, Q_from_AngX(+CH_C_PI_2)), {size.x() + buffer, size.z() + buffer});
}

size_t ChSystemFsi::AddBoxBCE(std::shared_ptr<ChBody> body,
                              const ChFrame<>& frame,
                              const ChVector<>& size,
                              bool solid) {
    thrust::host_vector<Real4> bce;
    CreateBCE_box(utils::ToReal3(size), solid, bce);
    AddBCE_body(body, bce, frame, solid, false, false);
    return bce.size();
}

size_t ChSystemFsi::AddSphereBCE(std::shared_ptr<ChBody> body,
                                 const ChFrame<>& frame,
                                 double radius,
                                 bool solid,
                                 bool polar) {
    thrust::host_vector<Real4> bce;
    CreateBCE_sphere(radius, solid, polar, bce);
    AddBCE_body(body, bce, frame, solid, false, false);
    return bce.size();
}

size_t ChSystemFsi::AddCylinderBCE(std::shared_ptr<ChBody> body,
                                   const ChFrame<>& frame,
                                   double radius,
                                   double height,
                                   bool solid,
                                   bool capped,
                                   bool polar) {
    thrust::host_vector<Real4> bce;
    CreateBCE_cylinder(radius, height, solid, capped, polar, bce);
    AddBCE_body(body, bce, frame, solid, false, false);
    return bce.size();
}

size_t ChSystemFsi::AddCylinderAnnulusBCE(std::shared_ptr<ChBody> body,
                                          const ChFrame<>& frame,
                                          double radius_inner,
                                          double radius_outer,
                                          double height,
                                          bool polar) {
    thrust::host_vector<Real4> bce;
    CreateBCE_cylinder_annulus(radius_inner, radius_outer, height, polar, bce);
    AddBCE_body(body, bce, frame, true, false, false);
    return bce.size();
}

size_t ChSystemFsi::AddConeBCE(std::shared_ptr<ChBody> body,
                               const ChFrame<>& frame,
                               double radius,
                               double height,
                               bool solid,
                               bool capped,
                               bool polar) {
    thrust::host_vector<Real4> bce;
    CreateBCE_cone(radius, height, solid, capped, polar, bce);
    AddBCE_body(body, bce, frame, solid, false, false);
    return bce.size();
}

size_t ChSystemFsi::AddPointsBCE(std::shared_ptr<ChBody> body,
                                 const std::vector<ChVector<>>& points,
                                 const ChFrame<>& frame,
                                 bool solid) {
    thrust::host_vector<Real4> bce;
    for (const auto& p : points)
        bce.push_back(mR4(p.x(), p.y(), p.z(), m_paramsH->HSML));
    AddBCE_body(body, bce, frame, solid, false, false);
    return bce.size();
}

//--------------------------------------------------------------------------------------------------------------------------------

const Real pi = Real(CH_C_PI);

void ChSystemFsi::CreateBCE_wall(const Real2& size, thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Calculate actual spacing in x-y directions
    Real2 hsize = size / 2;
    int2 np = {(int)std::round(hsize.x / spacing), (int)std::round(hsize.y / spacing)};
    Real2 delta = {hsize.x / np.x, hsize.y / np.y};

    for (int il = 0; il < num_layers; il++) {
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iy = -np.y; iy <= np.y; iy++) {
                bce.push_back(mR4(ix * delta.x, iy * delta.y, -il * spacing, kernel_h));
            }
        }
    }
}

void ChSystemFsi::CreateBCE_box(const Real3& size, bool solid, thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Calculate actual spacing in all 3 directions
    Real3 hsize = size / 2;
    int3 np = {(int)std::round(hsize.x / spacing), (int)std::round(hsize.y / spacing),
               (int)std::round(hsize.z / spacing)};
    Real3 delta = {hsize.x / np.x, hsize.y / np.y, hsize.z / np.z};

    // Inflate box if boundary
    if (!solid) {
        np += num_layers - 1;
        hsize = hsize + (num_layers - 1) * delta;
    }

    for (int il = 0; il < num_layers; il++) {
        // faces in Z direction
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iy = -np.y; iy <= np.y; iy++) {
                bce.push_back(mR4(ix * delta.x, iy * delta.y, -hsize.z + il * delta.z, kernel_h));
                bce.push_back(mR4(ix * delta.x, iy * delta.y, +hsize.z - il * delta.z, kernel_h));
            }
        }

        // faces in Y direction
        for (int ix = -np.x; ix <= np.x; ix++) {
            for (int iz = -np.z + num_layers; iz <= np.z - num_layers; iz++) {
                bce.push_back(mR4(ix * delta.x, -hsize.y + il * delta.y, iz * delta.z, kernel_h));
                bce.push_back(mR4(ix * delta.x, +hsize.y - il * delta.y, iz * delta.z, kernel_h));
            }
        }

        // faces in X direction
        for (int iy = -np.y + num_layers; iy <= np.y - num_layers; iy++) {
            for (int iz = -np.z + num_layers; iz <= np.z - num_layers; iz++) {
                bce.push_back(mR4(-hsize.x + il * delta.x, iy * delta.y, iz * delta.z, kernel_h));
                bce.push_back(mR4(+hsize.x - il * delta.x, iy * delta.y, iz * delta.z, kernel_h));
            }
        }
    }
}

void ChSystemFsi::CreateBCE_sphere(Real rad, bool solid, bool polar, thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Use polar coordinates
    if (polar) {
        Real rad_out = solid ? rad : rad + num_layers * spacing;
        Real rad_in = rad_out - num_layers * spacing;
        int np_r = (int)std::round((rad - rad_in) / spacing);
        Real delta_r = (rad_out - rad_in) / np_r;

        for (int ir = 0; ir <= np_r; ir++) {
            Real r = rad_in + ir * delta_r;
            int np_phi = (int)std::round(pi * r / spacing);
            Real delta_phi = pi / np_phi;
            for (int ip = 0; ip < np_phi; ip++) {
                Real phi = ip * delta_phi;
                Real cphi = std::cos(phi);
                Real sphi = std::sin(phi);
                Real x = r * sphi;
                Real y = r * sphi;
                Real z = r * cphi;
                int np_th = (int)std::round(2 * pi * r * sphi / spacing);
                Real delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
                for (int it = 0; it < np_th; it++) {
                    Real theta = it * delta_th;
                    bce.push_back(mR4(x * std::cos(theta), y * std::sin(theta), z, kernel_h));
                }
            }
        }
        return;
    }

    // Use a regular grid and accept/reject points
    int np = (int)std::round(rad / spacing);
    Real delta = rad / np;
    if (!solid) {
        np += num_layers;
        rad += num_layers * delta;
    }

    for (int iz = 0; iz <= np; iz++) {
        Real z = iz * delta;
        Real rz_max = std::sqrt(rad * rad - z * z);
        Real rz_min = std::max(rz_max - num_layers * delta, Real(0.0));
        if (iz >= np - num_layers)
            rz_min = 0;
        Real rz_min2 = rz_min * rz_min;
        Real rz_max2 = rz_max * rz_max;
        int nq = (int)std::round(rz_max / spacing);
        for (int ix = -nq; ix <= nq; ix++) {
            Real x = ix * delta;
            for (int iy = -nq; iy <= nq; iy++) {
                Real y = iy * delta;
                Real r2 = x * x + y * y;
                if (r2 >= rz_min2 && r2 <= rz_max2) {
                    bce.push_back(mR4(x, y, +z, kernel_h));
                    bce.push_back(mR4(x, y, -z, kernel_h));
                }
            }
        }
    }
}

void ChSystemFsi::CreateBCE_cylinder(Real rad,
                                     Real height,
                                     bool solid,
                                     bool capped,
                                     bool polar,
                                     thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Calculate actual spacing
    double hheight = height / 2;
    int np_h = (int)std::round(hheight / spacing);
    Real delta_h = hheight / np_h;

    // Inflate cylinder if boundary
    if (!solid && capped) {
        np_h += num_layers;
        hheight += num_layers * delta_h;
    }

    // Use polar coordinates
    if (polar) {
        Real rad_max = solid ? rad : rad + num_layers * spacing;
        Real rad_min = rad_max - num_layers * spacing;
        int np_r = (int)std::round((rad_max - rad_min) / spacing);
        Real delta_r = (rad_max - rad_min) / np_r;

        for (int ir = 0; ir <= np_r; ir++) {
            Real r = rad_min + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / spacing);
            Real delta_th = (np_th > 0) ? (2 * pi) / np_th : 1;
            for (int it = 0; it < np_th; it++) {
                Real theta = it * delta_th;
                Real x = r * cos(theta);
                Real y = r * sin(theta);
                for (int iz = -np_h; iz <= np_h; iz++) {
                    Real z = iz * delta_h;
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
        }

        if (capped) {
            rad_max = rad_min - delta_r;
            np_r = (int)std::round(rad_max / spacing);
            delta_r = rad_max / np_r;

            for (int ir = 0; ir <= np_r; ir++) {
                Real r = rad_max - ir * delta_r;
                int np_th = std::max((int)std::round(2 * pi * r / spacing), 1);
                Real delta_th = (2 * pi) / np_th;
                for (int it = 0; it < np_th; it++) {
                    Real theta = it * delta_th;
                    Real x = r * cos(theta);
                    Real y = r * sin(theta);
                    for (int iz = 0; iz <= num_layers; iz++) {
                        Real z = hheight - iz * delta_h;
                        bce.push_back(mR4(x, y, -z, kernel_h));
                        bce.push_back(mR4(x, y, +z, kernel_h));
                    }
                }
            }
        }

        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad / spacing);
    Real delta_r = rad / np_r;
    if (!solid) {
        np_r += num_layers;
        rad += num_layers * delta_r;
    }

    Real rad_max = rad;
    Real rad_min = std::max(rad - num_layers * delta_r, Real(0.0));
    Real r_max2 = rad_max * rad_max;
    Real r_min2 = rad_min * rad_min;
    for (int ix = -np_r; ix <= np_r; ix++) {
        Real x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            Real y = iy * delta_r;
            Real r2 = x * x + y * y;
            if (r2 >= r_min2 && r2 <= r_max2) {
                for (int iz = -np_h; iz <= np_h; iz++) {
                    Real z = iz * delta_h;
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
            if (capped && r2 < r_min2) {
                for (int iz = 0; iz <= num_layers; iz++) {
                    Real z = hheight - iz * delta_h;
                    bce.push_back(mR4(x, y, -z, kernel_h));
                    bce.push_back(mR4(x, y, +z, kernel_h));
                }
            }
        }
    }
}

void ChSystemFsi::CreateBCE_cylinder_annulus(Real rad_in,
                                             Real rad_out,
                                             Real height,
                                             bool polar,
                                             thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;

    // Calculate actual spacing
    double hheight = height / 2;
    int np_h = (int)std::round(hheight / spacing);
    Real delta_h = hheight / np_h;

    // Use polar coordinates
    if (polar) {
        int np_r = (int)std::round((rad_out - rad_in) / spacing);
        Real delta_r = (rad_out - rad_in) / np_r;
        for (int ir = 0; ir <= np_r; ir++) {
            Real r = rad_in + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / spacing);
            Real delta_th = (2 * pi) / np_th;
            for (int it = 0; it < np_th; it++) {
                Real theta = it * delta_th;
                Real x = r * cos(theta);
                Real y = r * sin(theta);
                for (int iz = -np_h; iz <= np_h; iz++) {
                    Real z = iz * delta_h;
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
        }
        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad_out / spacing);
    Real delta_r = rad_out / np_r;

    Real r_in2 = rad_in * rad_in;
    Real r_out2 = rad_out * rad_out;
    for (int ix = -np_r; ix <= np_r; ix++) {
        Real x = ix * delta_r;
        for (int iy = -np_r; iy <= np_r; iy++) {
            Real y = iy * delta_r;
            Real r2 = x * x + y * y;
            if (r2 >= r_in2 && r2 <= r_out2) {
                for (int iz = -np_h; iz <= np_h; iz++) {
                    Real z = iz * delta_h;
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
        }
    }
}

void ChSystemFsi::CreateBCE_cone(Real rad,
                                 Real height,
                                 bool solid,
                                 bool capped,
                                 bool polar,
                                 thrust::host_vector<Real4>& bce) {
    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Calculate actual spacing
    int np_h = (int)std::round(height / spacing);
    Real delta_h = height / np_h;

    // Inflate cone if boundary
    if (!solid) {
        np_h += num_layers;
        height += num_layers * delta_h;
        if (capped) {
            np_h += num_layers;
            height += num_layers * delta_h;
        }
    }

    // Use polar coordinates
    if (polar) {
        for (int iz = 0; iz < np_h; iz++) {
            Real z = iz * delta_h;
            Real rz = rad * (height - z) / height;
            Real rad_out = solid ? rz : rz + num_layers * spacing;
            Real rad_in = std::max(rad_out - num_layers * spacing, Real(0.0));
            if (iz >= np_h - num_layers)
                rad_in = 0;
            int np_r = (int)std::round((rad_out - rad_in) / spacing);
            Real delta_r = (rad_out - rad_in) / np_r;
            for (int ir = 0; ir <= np_r; ir++) {
                Real r = rad_in + ir * delta_r;
                int np_th = (int)std::round(2 * pi * r / spacing);
                Real delta_th = (2 * pi) / np_th;
                for (int it = 0; it < np_th; it++) {
                    Real theta = it * delta_th;
                    Real x = r * cos(theta);
                    Real y = r * sin(theta);
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }
        }

        bce.push_back(mR4(0.0, 0.0, height, kernel_h));

        if (capped) {
            //// RADU TODO
        }

        return;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad / spacing);
    Real delta_r = rad / np_r;

    for (int iz = 0; iz <= np_h; iz++) {
        Real z = iz * delta_h;
        Real rz = rad * (height - z) / height;
        Real rad_out = solid ? rz : rz + num_layers * spacing;
        Real rad_in = std::max(rad_out - num_layers * spacing, Real(0.0));
        Real r_out2 = rad_out * rad_out;
        Real r_in2 = rad_in * rad_in;
        for (int ix = -np_r; ix <= np_r; ix++) {
            Real x = ix * delta_r;
            for (int iy = -np_r; iy <= np_r; iy++) {
                Real y = iy * delta_r;
                Real r2 = x * x + y * y;
                if (r2 >= r_in2 && r2 <= r_out2) {
                    bce.push_back(mR4(x, y, z, kernel_h));
                }
            }

            if (capped) {
                //// RADU TODO
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddBCE_body(std::shared_ptr<ChBody> body,
                              const thrust::host_vector<Real4>& bce,
                              const ChFrame<>& rel_frame,
                              bool solid,
                              bool add_to_fluid_helpers,
                              bool add_to_previous) {
    // Set BCE marker type
    int type = 0;
    if (solid)
        type = 1;
    if (add_to_fluid_helpers)
        type = -3;

    for (const auto& p : bce) {
        auto pos_shape = utils::ToChVector(p);
        auto pos_body = rel_frame.TransformPointLocalToParent(pos_shape);
        auto pos_abs = body->GetFrame_REF_to_abs().TransformPointLocalToParent(pos_body);
        auto vel_abs = body->GetFrame_REF_to_abs().PointSpeedLocalToParent(pos_body);

        m_sysFSI->sphMarkers_H->posRadH.push_back(mR4(utils::ToReal3(pos_abs), p.w));
        m_sysFSI->sphMarkers_H->velMasH.push_back(utils::ToReal3(vel_abs));

        m_sysFSI->sphMarkers_H->rhoPresMuH.push_back(
            mR4(m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, (double)type));
        m_sysFSI->sphMarkers_H->tauXxYyZzH.push_back(mR3(0.0));
        m_sysFSI->sphMarkers_H->tauXyXzYzH.push_back(mR3(0.0));
    }

    if (solid)
        m_fsi_bodies_bce_num.push_back((int)bce.size());
}

unsigned int ChSystemFsi::AddBCE_mesh1D(unsigned int meshID, const ChFsiInterface::FsiMesh1D& fsi_mesh) {
    const auto& segments = fsi_mesh.segments;

    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    Real4 rhoPresMuH = {m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, 2};  // BCE markers of type 2
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    // Traverse the contact segments:
    // - calculate their discretization number n
    //   (largest number that results in a discretization no coarser than the initial spacing)
    // - generate segment coordinates for a uniform grid over the segment
    // - generate locations of BCE points on segment
    unsigned int num_seg = (unsigned int)segments.size();
    unsigned int num_bce = 0;
    for (unsigned int segID = 0; segID < num_seg; segID++) {
        const auto& seg = segments[segID];

        const auto& P0 = seg->GetNode(0)->GetPos();  // vertex 0 position (absolute coordinates)
        const auto& P1 = seg->GetNode(1)->GetPos();  // vertex 1 position (absolute coordinates)

        const auto& V0 = seg->GetNode(0)->GetPos_dt();  // vertex 0 velocity (absolute coordinates)
        const auto& V1 = seg->GetNode(1)->GetPos_dt();  // vertex 1 velocity (absolute coordinates)

        auto x_dir = P1 - P0;       // segment direction
        auto len = x_dir.Length();  // segment direction
        x_dir /= len;               // normalized direction

        int n = (int)std::ceil(len / spacing);  // required divisions on segment

        // Create two directions orthogonal to 'x_dir'
        ChVector<> y_dir(-x_dir.y() - x_dir.z(), x_dir.x() - x_dir.z(), x_dir.x() + x_dir.y());
        y_dir.Normalize();
        ChVector<> z_dir = Vcross(x_dir, y_dir);

        double yz_start = (num_layers - 1) * spacing / 2;

        unsigned int n_bce = 0;  // number of BCE markers on segment
        for (int i = 0; i <= n; i++) {
            if (i == 0 && !seg->OwnsNode(0))  // segment does not own vertex 0
                continue;
            if (i == n && !seg->OwnsNode(1))  // segment does not own vertex 1
                continue;

            auto lambda = ChVector2<>(i, n - i) / n;

            auto P = P0 * lambda[0] + P1 * lambda[1];
            auto V = V0 * lambda[0] + V1 * lambda[1];

            for (int j = 0; j < num_layers; j++) {
                double y_val = yz_start - j * spacing;
                for (int k = 0; k < num_layers; k++) {
                    auto z_val = yz_start - k * spacing;
                    auto Q = P + y_val * y_dir + z_val * z_dir;
                    m_sysFSI->sphMarkers_H->posRadH.push_back(mR4(utils::ToReal3(Q), kernel_h));
                    m_sysFSI->sphMarkers_H->velMasH.push_back(utils::ToReal3(V));
                    m_sysFSI->sphMarkers_H->rhoPresMuH.push_back(rhoPresMuH);
                    m_sysFSI->fsiData->flex1D_BCEcoords_H.push_back(utils::ToReal3({lambda[0], y_val, z_val}));
                    m_sysFSI->fsiData->flex1D_BCEsolids_H.push_back(mU3(meshID, segID, m_num_flex1D_elements + segID));
                    n_bce++;
                }
            }
        }

        // Add the number of BCE markers for this segment.
        // The maximum value on each layer is (n+1).
        num_bce += n_bce;

        //// TODO - load necessary structures and arrays
    }

    return num_bce;
}

unsigned int ChSystemFsi::AddBCE_mesh2D(unsigned int meshID, const ChFsiInterface::FsiMesh2D& fsi_mesh, bool centered) {
    const auto& surface = fsi_mesh.contact_surface;

    Real kernel_h = m_paramsH->HSML;
    Real spacing = m_paramsH->MULT_INITSPACE * m_paramsH->HSML;
    Real4 rhoPresMuH = {m_paramsH->rho0, m_paramsH->BASEPRES, m_paramsH->mu0, 3};  // BCE markers of type 3
    int num_layers = m_paramsH->NUM_BOUNDARY_LAYERS;

    ////std::ofstream ofile("mesh2D.txt");
    ////ofile << mesh->GetNumTriangles() << endl;
    ////ofile << endl;

    // Traverse the contact surface faces:
    // - calculate their discretization number n
    //   (largest number that results in a discretization no coarser than the initial spacing on each edge)
    // - generate barycentric coordinates for a uniform grid over the triangular face
    // - generate locations of BCE points on triangular face
    unsigned int num_tri = (int)surface->GetTriangleList().size();
    unsigned int num_bce = 0;
    for (unsigned int triID = 0; triID < num_tri; triID++) {
        const auto& tri = surface->GetTriangleList()[triID];

        const auto& P0 = tri->GetNode(0)->GetPos();  // vertex 0 position (absolute coordinates)
        const auto& P1 = tri->GetNode(1)->GetPos();  // vertex 1 position (absolute coordinates)
        const auto& P2 = tri->GetNode(2)->GetPos();  // vertex 2 position (absolute coordinates)

        const auto& V0 = tri->GetNode(0)->GetPos_dt();  // vertex 0 velocity (absolute coordinates)
        const auto& V1 = tri->GetNode(1)->GetPos_dt();  // vertex 1 velocity (absolute coordinates)
        const auto& V2 = tri->GetNode(2)->GetPos_dt();  // vertex 2 velocity (absolute coordinates)

        auto normal = Vcross(P1 - P0, P2 - P1);  // triangle normal
        normal.Normalize();

        int n0 = (int)std::ceil((P2 - P1).Length() / spacing);  // required divisions on edge 0
        int n1 = (int)std::ceil((P0 - P2).Length() / spacing);  // required divisions on edge 1
        int n2 = (int)std::ceil((P1 - P0).Length() / spacing);  // required divisions on edge 2
        int n = std::max(n0, std::max(n1, n2));                 // number of divisions on each edge

        ////ofile << P0 << endl;
        ////ofile << P1 << endl;
        ////ofile << P2 << endl;
        ////ofile << tri->OwnsNode(0) << " " << tri->OwnsNode(1) << " " << tri->OwnsNode(2) << endl;
        ////ofile << tri->OwnsEdge(0) << " " << tri->OwnsEdge(1) << " " << tri->OwnsEdge(2) << endl;
        ////ofile << n << endl;

        double z_start = centered ? (num_layers - 1) * spacing / 2 : 0;  // start layer z (along normal)

        unsigned int n_bce = 0;  // number of BCE markers on triangle
        for (int i = 0; i <= n; i++) {
            if (i == n && !tri->OwnsNode(0))  // triangle does not own vertex v0
                continue;
            if (i == 0 && !tri->OwnsEdge(1))  // triangle does not own edge v1-v2 = e1
                continue;

            for (int j = 0; j <= n - i; j++) {
                int k = n - i - j;
                auto lambda = ChVector<>(i, j, k) / n;  // barycentric coordinates of BCE marker

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

                // Create layers in negative normal direction
                for (int m = 0; m < num_layers; m++) {
                    auto z_val = z_start - m * spacing;
                    auto Q = P + z_val * normal;
                    m_sysFSI->sphMarkers_H->posRadH.push_back(mR4(utils::ToReal3(Q), kernel_h));
                    m_sysFSI->sphMarkers_H->velMasH.push_back(utils::ToReal3(V));
                    m_sysFSI->sphMarkers_H->rhoPresMuH.push_back(rhoPresMuH);
                    m_sysFSI->fsiData->flex2D_BCEcoords_H.push_back(utils::ToReal3({lambda[0], lambda[1], z_val}));
                    m_sysFSI->fsiData->flex2D_BCEsolids_H.push_back(mU3(meshID, triID, m_num_flex2D_elements + triID));
                    n_bce++;

                    ////ofile << Q << endl;
                }
            }
        }

        ////ofile << n_bce << endl;
        ////ofile << endl;

        // Add the number of BCE markers for this triangle.
        // The maximum value on each layer is (n+1)*(n+2)/2.
        num_bce += n_bce;

        //// TODO - load necessary structures and arrays
    }

    ////ofile.close();

    return num_bce;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::CreateMeshPoints(geometry::ChTriangleMeshConnected& mesh,
                                   double delta,
                                   std::vector<ChVector<>>& point_cloud) {
    mesh.RepairDuplicateVertexes(1e-9);  // if meshes are not watertight
    auto bbox = mesh.GetBoundingBox(ChMatrix33<>(1));

    const double EPSI = 1e-6;

    ChVector<> ray_origin;
    for (double x = bbox.min.x(); x < bbox.max.x(); x += delta) {
        ray_origin.x() = x + 1e-9;
        for (double y = bbox.min.y(); y < bbox.max.y(); y += delta) {
            ray_origin.y() = y + 1e-9;
            for (double z = bbox.min.z(); z < bbox.max.z(); z += delta) {
                ray_origin.z() = z + 1e-9;

                ChVector<> ray_dir[2] = {ChVector<>(5, 0.5, 0.25), ChVector<>(-3, 0.7, 10)};
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
                    point_cloud.push_back(ChVector<>(x, y, z));
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

double ChSystemFsi::GetKernelLength() const {
    return m_paramsH->HSML;
}

double ChSystemFsi::GetInitialSpacing() const {
    return m_paramsH->INITSPACE;
}

int ChSystemFsi::GetNumBoundaryLayers() const {
    return m_paramsH->NUM_BOUNDARY_LAYERS;
}

ChVector<> ChSystemFsi::GetContainerDim() const {
    return ChVector<>(m_paramsH->boxDimX, m_paramsH->boxDimY, m_paramsH->boxDimZ);
}

double ChSystemFsi::GetDensity() const {
    return m_paramsH->rho0;
}

double ChSystemFsi::GetViscosity() const {
    return m_paramsH->mu0;
}

double ChSystemFsi::GetBasePressure() const {
    return m_paramsH->BASEPRES;
}

double ChSystemFsi::GetParticleMass() const {
    return m_paramsH->markerMass;
}

ChVector<> ChSystemFsi::Get_G_acc() const {
    return ChVector<>(m_paramsH->gravity.x, m_paramsH->gravity.y, m_paramsH->gravity.z);
}

double ChSystemFsi::GetSoundSpeed() const {
    return m_paramsH->Cs;
}

ChVector<> ChSystemFsi::GetBodyForce() const {
    return ChVector<>(m_paramsH->bodyForce3.x, m_paramsH->bodyForce3.y, m_paramsH->bodyForce3.z);
}

double ChSystemFsi::GetStepSize() const {
    return m_paramsH->dT;
}

double ChSystemFsi::GetMaxStepSize() const {
    return m_paramsH->dT_Max;
}

bool ChSystemFsi::GetAdaptiveTimeStepping() const {
    return m_paramsH->Adaptive_time_stepping;
}

size_t ChSystemFsi::GetNumFluidMarkers() const {
    return m_sysFSI->numObjectsH->numFluidMarkers;
}

size_t ChSystemFsi::GetNumRigidBodyMarkers() const {
    return m_sysFSI->numObjectsH->numRigidMarkers;
}

size_t ChSystemFsi::GetNumFlexBodyMarkers() const {
    return m_sysFSI->numObjectsH->numFlexMarkers1D + m_sysFSI->numObjectsH->numFlexMarkers2D;
}

size_t ChSystemFsi::GetNumBoundaryMarkers() const {
    return m_sysFSI->numObjectsH->numBoundaryMarkers;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<std::shared_ptr<ChBody>>& ChSystemFsi::GetFsiBodies() const {
    return m_fsi_interface->m_fsi_bodies;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<ChVector<>> ChSystemFsi::GetParticlePositions() const {
    thrust::host_vector<Real4> posRadH = m_sysFSI->sphMarkers2_D->posRadD;
    std::vector<ChVector<>> pos;
    for (size_t i = 0; i < posRadH.size(); i++) {
        pos.push_back(utils::ToChVector(posRadH[i]));
    }
    return pos;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleFluidProperties() const {
    thrust::host_vector<Real4> rhoPresMuH = m_sysFSI->sphMarkers2_D->rhoPresMuD;
    std::vector<ChVector<>> props;
    for (size_t i = 0; i < rhoPresMuH.size(); i++) {
        props.push_back(utils::ToChVector(rhoPresMuH[i]));
    }
    return props;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleVelocities() const {
    thrust::host_vector<Real3> velH = m_sysFSI->sphMarkers2_D->velMasD;
    std::vector<ChVector<>> vel;
    for (size_t i = 0; i < velH.size(); i++) {
        vel.push_back(utils::ToChVector(velH[i]));
    }
    return vel;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleAccelerations() const {
    thrust::host_vector<Real4> accH = m_sysFSI->GetParticleAccelerations();
    std::vector<ChVector<>> acc;
    for (size_t i = 0; i < accH.size(); i++) {
        acc.push_back(utils::ToChVector(accH[i]));
    }
    return acc;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleForces() const {
    thrust::host_vector<Real4> frcH = m_sysFSI->GetParticleForces();
    std::vector<ChVector<>> frc;
    for (size_t i = 0; i < frcH.size(); i++) {
        frc.push_back(utils::ToChVector(frcH[i]));
    }
    return frc;
}

//--------------------------------------------------------------------------------------------------------------------------------

thrust::device_vector<int> ChSystemFsi::FindParticlesInBox(const ChFrame<>& frame, const ChVector<>& size) {
    const ChVector<>& Pos = frame.GetPos();
    ChVector<> Ax = frame.GetA().Get_A_Xaxis();
    ChVector<> Ay = frame.GetA().Get_A_Yaxis();
    ChVector<> Az = frame.GetA().Get_A_Zaxis();

    auto hsize = 0.5 * mR3(size.x(), size.y(), size.z());
    auto pos = mR3(Pos.x(), Pos.y(), Pos.z());
    auto ax = mR3(Ax.x(), Ax.y(), Ax.z());
    auto ay = mR3(Ay.x(), Ay.y(), Ay.z());
    auto az = mR3(Az.x(), Az.y(), Az.z());

    return m_sysFSI->FindParticlesInBox(hsize, pos, ax, ay, az);
}

thrust::device_vector<Real4> ChSystemFsi::GetParticlePositions(const thrust::device_vector<int>& indices) {
    return m_sysFSI->GetParticlePositions(indices);
}

thrust::device_vector<Real3> ChSystemFsi::GetParticleVelocities(const thrust::device_vector<int>& indices) {
    return m_sysFSI->GetParticleVelocities(indices);
}

thrust::device_vector<Real4> ChSystemFsi::GetParticleForces(const thrust::device_vector<int>& indices) {
    return m_sysFSI->GetParticleForces(indices);
}

thrust::device_vector<Real4> ChSystemFsi::GetParticleAccelerations(const thrust::device_vector<int>& indices) {
    return m_sysFSI->GetParticleAccelerations(indices);
}

}  // end namespace fsi
}  // end namespace chrono
