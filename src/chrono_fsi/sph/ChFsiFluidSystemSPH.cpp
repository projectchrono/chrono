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
////   - use ChFsiParamsSPH::free_surface_threshold (kernel threshold) for both CFD and CRM (currently, only CRM)

//// #define DEBUG_LOG

#include <cmath>
#include <algorithm>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#include "chrono_fsi/sph/physics/SphGeneral.cuh"
#include "chrono_fsi/sph/physics/SphDataManager.cuh"
#include "chrono_fsi/sph/physics/SphFluidDynamics.cuh"
#include "chrono_fsi/sph/physics/SphBceManager.cuh"

#include "chrono_fsi/sph/utils/SphUtilsLogging.cuh"

#include "chrono_fsi/sph/math/SphCustomMath.cuh"

#include "chrono_fsi/sph/utils/SphUtilsTypeConvert.cuh"
#include "chrono_fsi/sph/utils/SphUtilsPrint.cuh"
#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"

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
      m_force_proximity_search(false),
      m_check_errors(true) {
    m_paramsH = chrono_types::make_shared<ChFsiParamsSPH>();
    InitParams();

    m_data_mgr = chrono_types::make_unique<FsiDataManager>(m_paramsH);
}

ChFsiFluidSystemSPH::~ChFsiFluidSystemSPH() {}

//------------------------------------------------------------------------------

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
    m_paramsH->integration_scheme = IntegrationScheme::RK2;
    m_paramsH->eos_type = EosType::ISOTHERMAL;
    m_paramsH->viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    m_paramsH->boundary_method = BoundaryMethod::ADAMI;
    m_paramsH->kernel_type = KernelType::CUBIC_SPLINE;
    m_paramsH->shifting_method = ShiftingMethod::XSPH;
    m_paramsH->bc_type = {BCType::NONE, BCType::NONE, BCType::NONE};

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
    m_paramsH->density_reinit_steps = 2147483647;
    m_paramsH->Conservative_Form = true;
    m_paramsH->gradient_type = 0;
    m_paramsH->laplacian_type = 0;
    m_paramsH->use_consistent_laplacian_discretization = false;
    m_paramsH->use_consistent_gradient_discretization = false;

    m_paramsH->density_delta = Real(0.1);
    m_paramsH->use_delta_sph = false;

    m_paramsH->epsMinMarkersDis = Real(0.01);

    m_paramsH->markerMass = m_paramsH->volume0 * m_paramsH->rho0;

    m_paramsH->num_bce_layers = 3;

    m_paramsH->dT = Real(-1);

    // Pressure equation
    m_paramsH->use_density_based_projection = false;
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
    m_paramsH->free_surface_threshold = Real(0.8);

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
    m_paramsH->artificial_viscosity = Real(0.02);  // Does this mess with one for CRM?

    m_paramsH->Cs = 10 * m_paramsH->v_Max;

    m_paramsH->use_default_limits = true;
    m_paramsH->use_init_pressure = false;

    m_paramsH->num_proximity_search_steps = 4;
    m_paramsH->use_variable_time_step = false;
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::SetBoundaryType(BoundaryMethod boundary_method) {
    m_paramsH->boundary_method = boundary_method;
}

void ChFsiFluidSystemSPH::SetViscosityType(ViscosityMethod viscosity_method) {
    m_paramsH->viscosity_method = viscosity_method;
}

void ChFsiFluidSystemSPH::SetArtificialViscosityCoefficient(double coefficient) {
    m_paramsH->artificial_viscosity = coefficient;
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

void ChFsiFluidSystemSPH::SetIntegrationScheme(IntegrationScheme scheme) {
    m_paramsH->integration_scheme = scheme;
}

void ChFsiFluidSystemSPH::SetContainerDim(const ChVector3d& box_dim) {
    m_paramsH->boxDimX = box_dim.x();
    m_paramsH->boxDimY = box_dim.y();
    m_paramsH->boxDimZ = box_dim.z();
}

void ChFsiFluidSystemSPH::SetComputationalDomain(const ChAABB& computational_AABB, BoundaryConditions bc_type) {
    m_paramsH->cMin = ToReal3(computational_AABB.min);
    m_paramsH->cMax = ToReal3(computational_AABB.max);
    m_paramsH->use_default_limits = false;
    m_paramsH->bc_type = bc_type;
}

void ChFsiFluidSystemSPH::SetComputationalDomain(const ChAABB& computational_AABB) {
    m_paramsH->cMin = ToReal3(computational_AABB.min);
    m_paramsH->cMax = ToReal3(computational_AABB.max);
    m_paramsH->use_default_limits = false;
}

void ChFsiFluidSystemSPH::SetActiveDomain(const ChVector3d& box_dim) {
    m_paramsH->bodyActiveDomain = ToReal3(box_dim / 2);
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
    m_paramsH->use_consistent_gradient_discretization = consistent_gradient;
    m_paramsH->use_consistent_laplacian_discretization = consistent_Laplacian;
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

void ChFsiFluidSystemSPH::SetUseVariableTimeStep(bool use_variable_time_step) {
    m_paramsH->use_variable_time_step = use_variable_time_step;
}

void ChFsiFluidSystemSPH::CheckSPHParameters() {
    // Check parameter compatibility with physics problem
    if (m_paramsH->elastic_SPH) {
        if (m_paramsH->integration_scheme == IntegrationScheme::IMPLICIT_SPH) {
            cerr << "ERROR: Only WCSPH can be used for granular CRM problems." << endl;
            throw std::runtime_error("ISPH not supported for granular CRM problems.");
        }
        if (m_paramsH->non_newtonian) {
            cerr << "ERROR: Non-Newtonian viscosity model is not supported for granular CRM." << endl;
            throw std::runtime_error("Non-Newtonian viscosity model is not supported for granular CRM.");
        }
        if (m_paramsH->viscosity_method == ViscosityMethod::LAMINAR) {
            cerr << "ERROR: Viscosity type LAMINAR not supported for CRM granular. "
                    " Use ARTIFICIAL_UNILATERAL or ARTIFICIAL_BILATERAL."
                 << endl;
            throw std::runtime_error("Viscosity type LAMINAR not supported for CRM granular.");
        }
        if (m_paramsH->viscosity_method == ViscosityMethod::ARTIFICIAL_UNILATERAL) {
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
        if (m_paramsH->viscosity_method == ViscosityMethod::ARTIFICIAL_BILATERAL) {
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
}

ChFsiFluidSystemSPH::SPHParameters::SPHParameters()
    : integration_scheme(IntegrationScheme::RK2),
      initial_spacing(0.01),
      d0_multiplier(1.2),
      max_velocity(1.0),
      shifting_xsph_eps(0.5),
      shifting_ppst_push(3.0),
      shifting_ppst_pull(1.0),
      shifting_beta_implicit(1.0),
      shifting_diffusion_A(1.0),
      shifting_diffusion_AFSM(3.0),
      shifting_diffusion_AFST(2),
      min_distance_coefficient(0.01),
      density_reinit_steps(2e8),
      use_density_based_projection(false),
      num_bce_layers(3),
      use_consistent_gradient_discretization(false),
      use_consistent_laplacian_discretization(false),
      viscosity_method(ViscosityMethod::ARTIFICIAL_UNILATERAL),
      boundary_method(BoundaryMethod::ADAMI),
      kernel_type(KernelType::CUBIC_SPLINE),
      use_delta_sph(true),
      delta_sph_coefficient(0.1),
      artificial_viscosity(0.02),
      free_surface_threshold(0.8),
      num_proximity_search_steps(4),
      eos_type(EosType::ISOTHERMAL),
      use_variable_time_step(false) {}

void ChFsiFluidSystemSPH::SetSPHParameters(const SPHParameters& sph_params) {
    m_paramsH->integration_scheme = sph_params.integration_scheme;

    m_paramsH->eos_type = sph_params.eos_type;
    m_paramsH->viscosity_method = sph_params.viscosity_method;
    m_paramsH->boundary_method = sph_params.boundary_method;
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
    m_paramsH->shifting_xsph_eps = sph_params.shifting_xsph_eps;
    m_paramsH->shifting_ppst_push = sph_params.shifting_ppst_push;
    m_paramsH->shifting_ppst_pull = sph_params.shifting_ppst_pull;
    m_paramsH->shifting_beta_implicit = sph_params.shifting_beta_implicit;
    m_paramsH->shifting_diffusion_A = sph_params.shifting_diffusion_A;
    m_paramsH->shifting_diffusion_AFSM = sph_params.shifting_diffusion_AFSM;
    m_paramsH->shifting_diffusion_AFST = sph_params.shifting_diffusion_AFST;
    m_paramsH->epsMinMarkersDis = sph_params.min_distance_coefficient;

    m_paramsH->density_reinit_steps = sph_params.density_reinit_steps;
    m_paramsH->use_density_based_projection = sph_params.use_density_based_projection;

    m_paramsH->num_bce_layers = sph_params.num_bce_layers;

    m_paramsH->use_consistent_gradient_discretization = sph_params.use_consistent_gradient_discretization;
    m_paramsH->use_consistent_laplacian_discretization = sph_params.use_consistent_laplacian_discretization;
    m_paramsH->artificial_viscosity = sph_params.artificial_viscosity;
    m_paramsH->use_delta_sph = sph_params.use_delta_sph;
    m_paramsH->density_delta = sph_params.delta_sph_coefficient;

    m_paramsH->free_surface_threshold = Real(sph_params.free_surface_threshold);

    m_paramsH->num_proximity_search_steps = sph_params.num_proximity_search_steps;

    m_paramsH->use_variable_time_step = sph_params.use_variable_time_step;
}

ChFsiFluidSystemSPH::LinSolverParameters::LinSolverParameters()
    : type(SolverType::JACOBI), atol(0.0), rtol(0.0), max_num_iters(1000) {}

void ChFsiFluidSystemSPH::SetLinSolverParameters(const LinSolverParameters& linsolv_params) {
    m_paramsH->LinearSolver = linsolv_params.type;
    m_paramsH->LinearSolver_Abs_Tol = linsolv_params.atol;
    m_paramsH->LinearSolver_Rel_Tol = linsolv_params.rtol;
    m_paramsH->LinearSolver_Max_Iter = linsolv_params.max_num_iters;
}

ChFsiFluidSystemSPH::SplashsurfParameters::SplashsurfParameters()
    : smoothing_length(1.5), cube_size(0.5), surface_threshold(0.6) {}

//------------------------------------------------------------------------------

PhysicsProblem ChFsiFluidSystemSPH::GetPhysicsProblem() const {
    return (m_paramsH->elastic_SPH ? PhysicsProblem::CRM : PhysicsProblem::CFD);
}

std::string ChFsiFluidSystemSPH::GetPhysicsProblemString() const {
    return (m_paramsH->elastic_SPH ? "CRM" : "CFD");
}

std::string ChFsiFluidSystemSPH::GetSphIntegrationSchemeString() const {
    std::string method = "";
    switch (m_paramsH->integration_scheme) {
        case IntegrationScheme::EULER:
            method = "WCSPH_EULER";
            break;
        case IntegrationScheme::RK2:
            method = "WCSPH_RK2";
            break;
        case IntegrationScheme::VERLET:
            method = "WC_SPH_VERLET";
            break;
        case IntegrationScheme::SYMPLECTIC:
            method = "WCSPH_SYMPLECTIC";
            break;
        case IntegrationScheme::IMPLICIT_SPH:
            method = "ISPH";
            break;
    }

    return method;
}

//------------------------------------------------------------------------------

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
            m_data_mgr->fsiBodyState_H->ang_vel[i] = ToReal3(body_states[i].ang_vel);
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

void ChFsiFluidSystemSPH::SetBcePattern1D(BcePatternMesh1D pattern, bool remove_center) {
    m_pattern1D = pattern;
    m_remove_center1D = remove_center;
}

void ChFsiFluidSystemSPH::SetBcePattern2D(BcePatternMesh2D pattern, bool remove_center) {
    m_pattern2D = pattern;
    m_remove_center2D = remove_center;
}

//------------------------------------------------------------------------------

void PrintDeviceProperties(const cudaDeviceProp& prop) {
    cout << "GPU device: " << prop.name << endl;
    cout << "  Compute capability: " << prop.major << "." << prop.minor << endl;
    cout << "  Total global memory: " << prop.totalGlobalMem / (1024. * 1024. * 1024.) << " GB" << endl;
    cout << "  Total constant memory: " << prop.totalConstMem / 1024. << " KB" << endl;
    cout << "  Total available static shared memory per block: " << prop.sharedMemPerBlock / 1024. << " KB" << endl;
    cout << "  Max. dynamic shared memory per block: " << prop.sharedMemPerBlockOptin / 1024. << " KB" << endl;
    cout << "  Total shared memory per multiprocessor: " << prop.sharedMemPerMultiprocessor / 1024. << " KB" << endl;
    cout << "  Number of multiprocessors: " << prop.multiProcessorCount << endl;
}

void PrintParams(const ChFsiParamsSPH& params, const Counters& counters) {
    cout << "Simulation parameters" << endl;
    switch (params.viscosity_method) {
        case ViscosityMethod::LAMINAR:
            cout << "  Viscosity treatment: Laminar" << endl;
            break;
        case ViscosityMethod::ARTIFICIAL_UNILATERAL:
            cout << "  Viscosity treatment: Artificial Unilateral";
            cout << "  (coefficient: " << params.artificial_viscosity << ")" << endl;
            break;
        case ViscosityMethod::ARTIFICIAL_BILATERAL:
            cout << "  Viscosity treatment: Artificial Bilateral";
            cout << "  (coefficient: " << params.artificial_viscosity << ")" << endl;
            break;
    }
    if (params.boundary_method == BoundaryMethod::ADAMI) {
        cout << "  Boundary treatment: Adami" << endl;
    } else if (params.boundary_method == BoundaryMethod::HOLMES) {
        cout << "  Boundary treatment: Holmes" << endl;
    } else {
        cout << "  Boundary treatment: Adami" << endl;
    }
    switch (params.kernel_type) {
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

    switch (params.shifting_method) {
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

    switch (params.integration_scheme) {
        case IntegrationScheme::EULER:
            cout << "  Integration scheme: Explicit Euler" << endl;
            break;
        case IntegrationScheme::RK2:
            cout << "  Integration scheme: Runge-Kutta 2" << endl;
            break;
        case IntegrationScheme::SYMPLECTIC:
            cout << "  Integration scheme: Symplectic Euler" << endl;
            break;
        case IntegrationScheme::IMPLICIT_SPH:
            cout << "  Integration scheme: Implicit SPH" << endl;
            break;
    }

    cout << "  num_neighbors: " << params.num_neighbors << endl;
    cout << "  rho0: " << params.rho0 << endl;
    cout << "  invrho0: " << params.invrho0 << endl;
    cout << "  mu0: " << params.mu0 << endl;
    cout << "  bodyForce3: " << params.bodyForce3.x << " " << params.bodyForce3.y << " " << params.bodyForce3.z << endl;
    cout << "  gravity: " << params.gravity.x << " " << params.gravity.y << " " << params.gravity.z << endl;

    cout << "  d0: " << params.d0 << endl;
    cout << "  1/d0: " << params.ood0 << endl;
    cout << "  d0_multiplier: " << params.d0_multiplier << endl;
    cout << "  h: " << params.h << endl;
    cout << "  1/h: " << params.ooh << endl;

    cout << "  num_bce_layers: " << params.num_bce_layers << endl;
    cout << "  epsMinMarkersDis: " << params.epsMinMarkersDis << endl;
    cout << "  markerMass: " << params.markerMass << endl;
    cout << "  volume0: " << params.volume0 << endl;
    cout << "  gradient_type: " << params.gradient_type << endl;

    cout << "  v_Max: " << params.v_Max << endl;
    cout << "  Cs: " << params.Cs << endl;

    if (params.shifting_method == ShiftingMethod::XSPH) {
        cout << "  shifting_xsph_eps: " << params.shifting_xsph_eps << endl;
    } else if (params.shifting_method == ShiftingMethod::PPST) {
        cout << "  shifting_ppst_push: " << params.shifting_ppst_push << endl;
        cout << "  shifting_ppst_pull: " << params.shifting_ppst_pull << endl;
    } else if (params.shifting_method == ShiftingMethod::PPST_XSPH) {
        cout << "  shifting_xsph_eps: " << params.shifting_xsph_eps << endl;
        cout << "  shifting_ppst_push: " << params.shifting_ppst_push << endl;
        cout << "  shifting_ppst_pull: " << params.shifting_ppst_pull << endl;
    } else if (params.shifting_method == ShiftingMethod::DIFFUSION) {
        cout << "  shifting_diffusion_A: " << params.shifting_diffusion_A << endl;
        cout << "  shifting_diffusion_AFSM: " << params.shifting_diffusion_AFSM << endl;
        cout << "  shifting_diffusion_AFST: " << params.shifting_diffusion_AFST << endl;
    } else if (params.shifting_method == ShiftingMethod::DIFFUSION_XSPH) {
        cout << "  shifting_xsph_eps: " << params.shifting_xsph_eps << endl;
        cout << "  shifting_diffusion_A: " << params.shifting_diffusion_A << endl;
        cout << "  shifting_diffusion_AFSM: " << params.shifting_diffusion_AFSM << endl;
        cout << "  shifting_diffusion_AFST: " << params.shifting_diffusion_AFST << endl;
    }
    cout << "  density_reinit_steps: " << params.density_reinit_steps << endl;

    cout << "  Proximity search performed every " << params.num_proximity_search_steps << " steps" << endl;
    cout << "  use_variable_time_step: " << params.use_variable_time_step << endl;
    cout << "  dT: " << params.dT << endl;

    cout << "  non_newtonian: " << params.non_newtonian << endl;
    cout << "  mu_of_I : " << (int)params.mu_of_I << endl;
    cout << "  rheology_model: " << (int)params.rheology_model << endl;
    cout << "  ave_diam: " << params.ave_diam << endl;
    cout << "  mu_max: " << params.mu_max << endl;
    cout << "  mu_fric_s: " << params.mu_fric_s << endl;
    cout << "  mu_fric_2: " << params.mu_fric_2 << endl;
    cout << "  mu_I0: " << params.mu_I0 << endl;
    cout << "  mu_I_b: " << params.mu_I_b << endl;
    cout << "  HB_k: " << params.HB_k << endl;
    cout << "  HB_n: " << params.HB_n << endl;
    cout << "  HB_tau0: " << params.HB_tau0 << endl;
    cout << "  Coh_coeff: " << params.Coh_coeff << endl;

    cout << "  E_young: " << params.E_young << endl;
    cout << "  G_shear: " << params.G_shear << endl;
    cout << "  INV_G_shear: " << params.INV_G_shear << endl;
    cout << "  K_bulk: " << params.K_bulk << endl;
    cout << "  free_surface_threshold: " << params.free_surface_threshold << endl;

    cout << "  PPE_relaxation: " << params.PPE_relaxation << endl;
    cout << "  Conservative_Form: " << params.Conservative_Form << endl;
    cout << "  Pressure_Constraint: " << params.Pressure_Constraint << endl;

    cout << "  binSize0: " << params.binSize0 << endl;
    cout << "  boxDims: " << params.boxDims.x << " " << params.boxDims.y << " " << params.boxDims.z << endl;
    cout << "  gridSize: " << params.gridSize.x << " " << params.gridSize.y << " " << params.gridSize.z << endl;
    cout << "  cMin: " << params.cMin.x << " " << params.cMin.y << " " << params.cMin.z << endl;
    cout << "  cMax: " << params.cMax.x << " " << params.cMax.y << " " << params.cMax.z << endl;

    ////Real dt_CFL = params.Co_number * params.h / 2.0 / MaxVel;
    ////Real dt_nu = 0.2 * params.h * params.h / (params.mu0 / params.rho0);
    ////Real dt_body = 0.1 * sqrt(params.h / length(params.bodyForce3 + params.gravity));
    ////Real dt = std::min(dt_body, std::min(dt_CFL, dt_nu));

    cout << "Counters" << endl;
    cout << "  numFsiBodies:       " << counters.numFsiBodies << endl;
    cout << "  numFsiElements1D:   " << counters.numFsiElements1D << endl;
    cout << "  numFsiElements2D:   " << counters.numFsiElements2D << endl;
    cout << "  numFsiNodes1D:      " << counters.numFsiNodes1D << endl;
    cout << "  numFsiNodes2D:      " << counters.numFsiNodes2D << endl;
    cout << "  numGhostMarkers:    " << counters.numGhostMarkers << endl;
    cout << "  numHelperMarkers:   " << counters.numHelperMarkers << endl;
    cout << "  numFluidMarkers:    " << counters.numFluidMarkers << endl;
    cout << "  numBoundaryMarkers: " << counters.numBoundaryMarkers << endl;
    cout << "  numRigidMarkers:    " << counters.numRigidMarkers << endl;
    cout << "  numFlexMarkers1D:   " << counters.numFlexMarkers1D << endl;
    cout << "  numFlexMarkers2D:   " << counters.numFlexMarkers2D << endl;
    cout << "  numAllMarkers:      " << counters.numAllMarkers << endl;
    cout << "  startRigidMarkers:  " << counters.startRigidMarkers << endl;
    cout << "  startFlexMarkers1D: " << counters.startFlexMarkers1D << endl;
    cout << "  startFlexMarkers2D: " << counters.startFlexMarkers2D << endl;
}

void PrintRefArrays(const thrust::host_vector<int4>& referenceArray,
                    const thrust::host_vector<int4>& referenceArray_FEA) {
    cout << "Reference array (size: " << referenceArray.size() << ")" << endl;
    for (size_t i = 0; i < referenceArray.size(); i++) {
        const int4& num = referenceArray[i];
        cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
    }
    cout << "Reference array FEA (size: " << referenceArray_FEA.size() << ")" << endl;
    for (size_t i = 0; i < referenceArray_FEA.size(); i++) {
        const int4& num = referenceArray_FEA[i];
        cout << "  " << i << ": " << num.x << " " << num.y << " " << num.z << " " << num.w << endl;
    }
    cout << endl;
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::OnAddFsiBody(std::shared_ptr<FsiBody> fsi_body, bool check_embedded) {
    ChAssertAlways(!m_is_initialized);

    FsiSphBody b;
    b.fsi_body = fsi_body;
    b.check_embedded = check_embedded;

    CreateBCEFsiBody(fsi_body, b.bce_ids, b.bce_coords, b.bce);
    m_num_rigid_bodies++;

    m_bodies.push_back(b);
}

void ChFsiFluidSystemSPH::OnAddFsiMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh, bool check_embedded) {
    ChAssertAlways(!m_is_initialized);

    FsiSphMesh1D m;
    m.fsi_mesh = fsi_mesh;
    m.check_embedded = check_embedded;

    CreateBCEFsiMesh1D(fsi_mesh, m_pattern1D, m_remove_center1D, m.bce_ids, m.bce_coords, m.bce);
    m_num_flex1D_nodes += fsi_mesh->GetNumNodes();
    m_num_flex1D_elements += fsi_mesh->GetNumElements();

    m_meshes1D.push_back(m);
}

void ChFsiFluidSystemSPH::OnAddFsiMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh, bool check_embedded) {
    ChAssertAlways(!m_is_initialized);

    FsiSphMesh2D m;
    m.fsi_mesh = fsi_mesh;
    m.check_embedded = check_embedded;

    CreateBCEFsiMesh2D(fsi_mesh, m_pattern2D, m_remove_center2D, m.bce_ids, m.bce_coords, m.bce);
    m_num_flex2D_nodes += fsi_mesh->GetNumNodes();
    m_num_flex2D_elements += fsi_mesh->GetNumElements();

    m_meshes2D.push_back(m);
}

//// TODO FSI bodies:
////   - give control over Cartesian / polar BCE distribution (where applicable)
////   - eliminate duplicate BCE markers (from multiple volumes). Easiest if BCE created on a grid!
//// TODO FSI meshes:
////   - consider using monotone cubic Hermite interpolation instead of cubic Bezier

void ChFsiFluidSystemSPH::CreateBCEFsiBody(std::shared_ptr<FsiBody> fsi_body,
                                           std::vector<int>& bce_ids,
                                           std::vector<ChVector3d>& bce_coords,
                                           std::vector<ChVector3d>& bce) {
    const auto& geometry = fsi_body->geometry;
    if (geometry) {
        for (const auto& sphere : geometry->coll_spheres) {
            auto points = CreatePointsSphereInterior(sphere.radius, true);
            for (auto& p : points)
                p += sphere.pos;
            bce_coords.insert(bce_coords.end(), points.begin(), points.end());
        }
        for (const auto& box : geometry->coll_boxes) {
            auto points = CreatePointsBoxInterior(box.dims);
            for (auto& p : points)
                p = box.pos + box.rot.Rotate(p);
            bce_coords.insert(bce_coords.end(), points.begin(), points.end());
        }
        for (const auto& cyl : geometry->coll_cylinders) {
            auto points = CreatePointsCylinderInterior(cyl.radius, cyl.length, true);
            for (auto& p : points)
                p = cyl.pos + cyl.rot.Rotate(p);
            bce_coords.insert(bce_coords.end(), points.begin(), points.end());
        }
        for (const auto& mesh : geometry->coll_meshes) {
            auto points = CreatePointsMesh(*mesh.trimesh);
            bce_coords.insert(bce_coords.end(), points.begin(), points.end());
        }

        // Get global BCE positions
        const auto& X_G_R = fsi_body->body->GetFrameRefToAbs();
        std::transform(bce_coords.begin(), bce_coords.end(), std::back_inserter(bce),
                       [&X_G_R](ChVector3d& v) { return X_G_R.TransformPointLocalToParent(v); });

        // Get local BCE coordinates relative to centroidal frame
        bce_coords.clear();
        const auto& X_G_COM = fsi_body->body->GetFrameCOMToAbs();
        std::transform(bce.begin(), bce.end(), std::back_inserter(bce_coords),
                       [&X_G_COM](ChVector3d& v) { return X_G_COM.TransformPointParentToLocal(v); });

        // Set BCE body association
        bce_ids.resize(bce_coords.size(), fsi_body->index);
    }
}

void GetOrthogonalAxes(const ChVector3d& x, ChVector3d& y, ChVector3d& z) {
    ChVector3d y_tmp(0, 1, 0);
    if (x.y() > 0.9 || x.y() < -0.9) {
        y_tmp.x() = 1;
        y_tmp.y() = 0;
    }
    z = Vcross(x, y_tmp).GetNormalized();
    y = Vcross(z, x);
}

void ChFsiFluidSystemSPH::CreateBCEFsiMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh,
                                             BcePatternMesh1D pattern,
                                             bool remove_center,
                                             std::vector<ChVector3i>& bce_ids,
                                             std::vector<ChVector3d>& bce_coords,
                                             std::vector<ChVector3d>& bce) {
    auto meshID = fsi_mesh->index;
    const auto& surface = fsi_mesh->contact_surface;

    Real spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    // Load nodal directions if requested (from FSI mesh or calculate)
    bool use_node_directions = (m_node_directions_mode != NodeDirectionsMode::NONE);
    std::vector<ChVector3d> dir;

    if (m_node_directions_mode == NodeDirectionsMode::EXACT) {
        //// TODO - exact node directions
        //// use from FSI mesh or fall back on average
    }

    if (m_node_directions_mode == NodeDirectionsMode::AVERAGE) {
        dir.resize(fsi_mesh->GetNumNodes());
        std::fill(dir.begin(), dir.end(), VNULL);
        for (const auto& seg : surface->GetSegmentsXYZ()) {
            const auto& node0 = seg->GetNode(0);
            const auto& node1 = seg->GetNode(1);
            auto i0 = fsi_mesh->ptr2ind_map.at(node0);
            auto i1 = fsi_mesh->ptr2ind_map.at(node1);
            auto d = (node1->GetPos() - node0->GetPos()).GetNormalized();
            dir[i0] += d;
            dir[i1] += d;
        }
        for (auto& d : dir)
            d.Normalize();
    }

    // Traverse the contact segments:
    // - calculate their discretization number n
    //   (largest number that results in a discretization no coarser than the initial spacing)
    // - generate segment coordinates for a uniform grid over the segment (load `bce_coords`)
    // - generate mesh and segment association (load `bce_ids`)
    // - generate initial global BCE positions (load `bce`)
    unsigned int num_seg = (unsigned int)surface->GetSegmentsXYZ().size();
    for (unsigned int segID = 0; segID < num_seg; segID++) {
        const auto& seg = surface->GetSegmentsXYZ()[segID];

        auto i0 = fsi_mesh->ptr2ind_map.at(seg->GetNode(0));
        auto i1 = fsi_mesh->ptr2ind_map.at(seg->GetNode(1));

        const auto& P0 = seg->GetNode(0)->GetPos();  // vertex 0 position (absolute coordinates)
        const auto& P1 = seg->GetNode(1)->GetPos();  // vertex 1 position (absolute coordinates)

        ////cout << segID << "  " << P0 << "  |  " << P1 << endl;

        auto len = (P1 - P0).Length();          // segment length
        int n = (int)std::ceil(len / spacing);  // required divisions on segment

        for (int i = 0; i <= n; i++) {
            if (i == 0 && !seg->OwnsNode(0))  // segment does not own vertex 0
                continue;
            if (i == n && !seg->OwnsNode(1))  // segment does not own vertex 1
                continue;

            auto t = double(i) / n;

            ChVector3d P;
            ChVector3d D;
            if (use_node_directions) {
                auto t2 = t * t;
                auto t3 = t2 * t;

                auto a0 = 2 * t3 - 3 * t2 + 1;
                auto a1 = -2 * t3 + 3 * t2;
                auto b0 = t3 - 2 * t2 + t;
                auto b1 = t3 - t2;
                P = P0 * a0 + P1 * a1 + dir[i0] * b0 + dir[i1] * b1;

                auto a0d = 6 * t2 - 6 * t;
                auto a1d = -6 * t2 + 6 * t;
                auto b0d = 3 * t2 - 4 * t + 1;
                auto b1d = 3 * t2 - 2 * t;
                D = P0 * a0d + P1 * a1d + dir[i0] * b0d + dir[i1] * b1d;
            } else {
                P = P0 * (1 - t) + P1 * t;
                D = P1 - P0;
            }

            // Create local frame
            ChVector3d x_dir = D.GetNormalized();
            ChVector3d y_dir;
            ChVector3d z_dir;
            GetOrthogonalAxes(x_dir, y_dir, z_dir);

            for (int j = -num_layers + 1; j <= num_layers - 1; j += 2) {
                for (int k = -num_layers + 1; k <= num_layers - 1; k += 2) {
                    if (remove_center && j == 0 && k == 0)
                        continue;
                    if (pattern == BcePatternMesh1D::STAR && std::abs(j) + std::abs(k) > num_layers)
                        continue;
                    double y_val = j * spacing / 2;
                    double z_val = k * spacing / 2;

                    bce.push_back(P + y_val * y_dir + z_val * z_dir);
                    bce_coords.push_back({t, y_val, z_val});
                    bce_ids.push_back(ChVector3i(meshID, segID, m_num_flex1D_elements + segID));
                }
            }
        }
    }
}

void ChFsiFluidSystemSPH::CreateBCEFsiMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh,
                                             BcePatternMesh2D pattern,
                                             bool remove_center,
                                             std::vector<ChVector3i>& bce_ids,
                                             std::vector<ChVector3d>& bce_coords,
                                             std::vector<ChVector3d>& bce) {
    auto meshID = fsi_mesh->index;
    const auto& surface = fsi_mesh->contact_surface;

    // Load nodal directions if requested (from FSI mesh or calculate)
    bool use_node_directions = (m_node_directions_mode != NodeDirectionsMode::NONE);
    std::vector<ChVector3d> dir;

    if (m_node_directions_mode == NodeDirectionsMode::EXACT) {
        //// TODO - exact node directions
        //// use from FSI mesh or fall back on average
    }

    if (m_node_directions_mode == NodeDirectionsMode::AVERAGE) {
        dir.resize(fsi_mesh->GetNumNodes());
        std::fill(dir.begin(), dir.end(), VNULL);
        for (const auto& tri : surface->GetTrianglesXYZ()) {
            const auto& node0 = tri->GetNode(0);
            const auto& node1 = tri->GetNode(1);
            const auto& node2 = tri->GetNode(2);
            auto i0 = fsi_mesh->ptr2ind_map.at(node0);
            auto i1 = fsi_mesh->ptr2ind_map.at(node1);
            auto i2 = fsi_mesh->ptr2ind_map.at(node2);
            auto d = ChTriangle::CalcNormal(node0->GetPos(), node1->GetPos(), node2->GetPos());
            dir[i0] += d;
            dir[i1] += d;
            dir[i2] += d;
        }
        for (auto& d : dir)
            d.Normalize();
    }

    Real spacing = m_paramsH->d0;
    int num_layers = m_paramsH->num_bce_layers;

    ////std::ofstream ofile("mesh2D.txt");
    ////ofile << mesh->GetNumTriangles() << endl;
    ////ofile << endl;

    // Traverse the contact surface faces:
    // - calculate their discretization number n
    //   (largest number that results in a discretization no coarser than the initial spacing on each edge)
    // - generate barycentric coordinates for a uniform grid over the triangular face (load `bce_coords`)
    // - generate mesh and face association (load `bce_ids`)
    // - generate initial global BCE positions (load `bce`)
    unsigned int num_tri = (int)surface->GetTrianglesXYZ().size();
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

        int m_start = 0;
        int m_end = 0;
        switch (pattern) {
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

                //// TODO RADU - add cubic interpolation (position and normal) if using nodal directions
                if (use_node_directions) {
                    //// TODO
                }

                auto P = lambda[0] * P0 + lambda[1] * P1 + lambda[2] * P2;  // absolute coordinates of BCE marker

                // Create layers in normal direction
                for (int m = m_start; m <= m_end; m += 2) {
                    if (remove_center && m == 0)
                        continue;
                    double z_val = m * spacing / 2;

                    bce.push_back(P + z_val * normal);
                    bce_coords.push_back({lambda[0], lambda[1], z_val});
                    bce_ids.push_back(ChVector3i(meshID, triID, m_num_flex2D_elements + triID));

                    ////ofile << Q << endl;
                }
            }
        }

        ////ofile << endl;
    }

    ////ofile.close();
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::AddBCEFsiBody(const FsiSphBody& fsisph_body) {
    const auto& fsi_body = fsisph_body.fsi_body;

    // Add BCE markers and load their local coordinates and body associations
    auto num_bce = fsisph_body.bce.size();
    for (size_t i = 0; i < num_bce; i++) {
        m_data_mgr->AddBceMarker(MarkerType::BCE_RIGID, ToReal3(fsisph_body.bce[i]), {0, 0, 0});
        m_data_mgr->rigid_BCEcoords_H.push_back(ToReal3(fsisph_body.bce_coords[i]));
        m_data_mgr->rigid_BCEsolids_H.push_back(fsisph_body.bce_ids[i]);
    }

    m_fsi_bodies_bce_num.push_back((int)num_bce);

    if (m_verbose) {
        cout << "Add BCE for rigid body" << endl;
        cout << "  Num. BCE markers: " << num_bce << endl;
    }
}

void ChFsiFluidSystemSPH::AddBCEFsiMesh1D(const FsiSphMesh1D& fsisph_mesh) {
    const auto& fsi_mesh = fsisph_mesh.fsi_mesh;

    // Load index-based mesh connectivity (append to global list of 1-D flex segments)
    for (const auto& seg : fsi_mesh->contact_surface->GetSegmentsXYZ()) {
        auto node0_index = m_num_flex1D_nodes + fsi_mesh->ptr2ind_map.at(seg->GetNode(0));
        auto node1_index = m_num_flex1D_nodes + fsi_mesh->ptr2ind_map.at(seg->GetNode(1));
        m_data_mgr->flex1D_Nodes_H.push_back(mI2(node0_index, node1_index));
    }

    // Add BCE markers and load their local coordinates and mesh associations
    auto num_bce = fsisph_mesh.bce.size();
    for (size_t i = 0; i < num_bce; i++) {
        m_data_mgr->AddBceMarker(MarkerType::BCE_FLEX1D, ToReal3(fsisph_mesh.bce[i]), {0, 0, 0});
        m_data_mgr->flex1D_BCEcoords_H.push_back(ToReal3(fsisph_mesh.bce_coords[i]));
        m_data_mgr->flex1D_BCEsolids_H.push_back(ToUint3(fsisph_mesh.bce_ids[i]));
    }

    if (m_verbose) {
        cout << "Add BCE for 1D mesh" << endl;
        cout << "  Num. nodes:       " << fsi_mesh->GetNumNodes() << endl;
        cout << "  Num. segments:    " << fsi_mesh->GetNumElements() << endl;
        cout << "  Num. BCE markers: " << num_bce << endl;
    }
}

void ChFsiFluidSystemSPH::AddBCEFsiMesh2D(const FsiSphMesh2D& fsisph_mesh) {
    const auto& fsi_mesh = fsisph_mesh.fsi_mesh;

    // Load index-based mesh connectivity (append to global list of 1-D flex segments)
    for (const auto& tri : fsi_mesh->contact_surface->GetTrianglesXYZ()) {
        auto node0_index = m_num_flex2D_nodes + fsi_mesh->ptr2ind_map.at(tri->GetNode(0));
        auto node1_index = m_num_flex2D_nodes + fsi_mesh->ptr2ind_map.at(tri->GetNode(1));
        auto node2_index = m_num_flex2D_nodes + fsi_mesh->ptr2ind_map.at(tri->GetNode(2));
        m_data_mgr->flex2D_Nodes_H.push_back(mI3(node0_index, node1_index, node2_index));
    }

    // Add BCE markers and load their local coordinates and mesh associations
    auto num_bce = fsisph_mesh.bce.size();
    for (size_t i = 0; i < num_bce; i++) {
        m_data_mgr->AddBceMarker(MarkerType::BCE_FLEX2D, ToReal3(fsisph_mesh.bce[i]), {0, 0, 0});
        m_data_mgr->flex2D_BCEcoords_H.push_back(ToReal3(fsisph_mesh.bce_coords[i]));
        m_data_mgr->flex2D_BCEsolids_H.push_back(ToUint3(fsisph_mesh.bce_ids[i]));
    }

    if (m_verbose) {
        cout << "Add BCE for 2D mesh" << endl;
        cout << "  Num. nodes:       " << fsi_mesh->GetNumNodes() << endl;
        cout << "  Num. faces:       " << fsi_mesh->GetNumElements() << endl;
        cout << "  Num. BCE markers: " << num_bce << endl;
    }
}

void ChFsiFluidSystemSPH::Initialize(const std::vector<FsiBodyState>& body_states,
                                     const std::vector<FsiMeshState>& mesh1D_states,
                                     const std::vector<FsiMeshState>& mesh2D_states) {
    assert(body_states.size() == m_bodies.size());
    assert(mesh1D_states.size() == m_meshes1D.size());
    assert(mesh2D_states.size() == m_meshes2D.size());

    // Process FSI solids - load BCE data to data manager
    // Note: counters must be regenerated, as they are used as offsets for global indices
    m_num_rigid_bodies = 0;
    m_num_flex1D_nodes = 0;
    m_num_flex1D_elements = 0;
    m_num_flex2D_nodes = 0;
    m_num_flex2D_elements = 0;

    for (const auto& b : m_bodies) {
        AddBCEFsiBody(b);
        m_num_rigid_bodies++;
    }

    for (const auto& m : m_meshes1D) {
        AddBCEFsiMesh1D(m);
        m_num_flex1D_nodes += m.fsi_mesh->GetNumNodes();
        m_num_flex1D_elements += m.fsi_mesh->GetNumElements();
    }

    for (const auto& m : m_meshes2D) {
        AddBCEFsiMesh2D(m);
        m_num_flex2D_nodes += m.fsi_mesh->GetNumNodes();
        m_num_flex2D_elements += m.fsi_mesh->GetNumElements();
    }

    // ----------------

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

    // ----------------

    // This means boundaries have not been set - just use an approximate domain size with no periodic sides
    if (m_paramsH->use_default_limits) {
        m_paramsH->cMin =
            mR3(-2 * m_paramsH->boxDimX, -2 * m_paramsH->boxDimY, -2 * m_paramsH->boxDimZ) - 10 * mR3(m_paramsH->h);
        m_paramsH->cMax =
            mR3(+2 * m_paramsH->boxDimX, +2 * m_paramsH->boxDimY, +2 * m_paramsH->boxDimZ) + 10 * mR3(m_paramsH->h);
        m_paramsH->bc_type = BC_NONE;
    }

    m_paramsH->x_periodic = m_paramsH->bc_type.x == BCType::PERIODIC;
    m_paramsH->y_periodic = m_paramsH->bc_type.y == BCType::PERIODIC;
    m_paramsH->z_periodic = m_paramsH->bc_type.z == BCType::PERIODIC;

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
    // Update the speed of sound
    if (m_paramsH->elastic_SPH) {
        m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);
    } else {
        m_paramsH->Cs = 10 * m_paramsH->v_Max;
    }
    // ----------------

    // Hack to prevent bringing in Chrono core headers in CUDA code
    // Copy from one enum class (NodeDirectionsMode) to another (NodeDirections)
    NodeDirections node_directions_mode = NodeDirections::NONE;
    switch (m_node_directions_mode) {
        case NodeDirectionsMode::NONE:
            node_directions_mode = NodeDirections::NONE;
            break;
        case NodeDirectionsMode::AVERAGE:
            node_directions_mode = NodeDirections::AVERAGE;
            break;
        case NodeDirectionsMode::EXACT:
            node_directions_mode = NodeDirections::EXACT;
            break;
    }

    // Initialize the data manager: set reference arrays, set counters, and resize simulation arrays
    // Indicate if the data manager should allocate space for holding FEA mesh direction vectors
    m_data_mgr->Initialize(m_num_rigid_bodies,                                                                    //
                           m_num_flex1D_nodes, m_num_flex1D_elements, m_num_flex2D_nodes, m_num_flex2D_elements,  //
                           node_directions_mode);

    // Load the initial body and mesh node states
    ChDebugLog("load initial states");
    LoadSolidStates(body_states, mesh1D_states, mesh2D_states);

    // Create BCE and SPH worker objects
    m_bce_mgr = chrono_types::make_unique<SphBceManager>(*m_data_mgr, node_directions_mode, m_verbose, m_check_errors);
    m_fluid_dynamics = chrono_types::make_unique<SphFluidDynamics>(*m_data_mgr, *m_bce_mgr, m_verbose, m_check_errors);

    // Initialize worker objects
    m_bce_mgr->Initialize(m_fsi_bodies_bce_num);
    m_fluid_dynamics->Initialize();

    /// If active domains are not used then don't overly resize the arrays
    if (!m_paramsH->use_active_domain) {
        m_data_mgr->SetGrowthFactor(1.0f);
    }

    // ----------------

    // Check if GPU is available and initialize CUDA device information
    int device;
    cudaGetDevice(&device);
    cudaCheckError();
    m_data_mgr->cudaDeviceInfo->deviceID = device;
    cudaGetDeviceProperties(&m_data_mgr->cudaDeviceInfo->deviceProp, m_data_mgr->cudaDeviceInfo->deviceID);
    cudaCheckError();

    if (m_verbose) {
        PrintDeviceProperties(m_data_mgr->cudaDeviceInfo->deviceProp);
        PrintParams(*m_paramsH, *m_data_mgr->countersH);
        PrintRefArrays(m_data_mgr->referenceArray, m_data_mgr->referenceArray_FEA);
    }

    CheckSPHParameters();
}

//------------------------------------------------------------------------------
double ChFsiFluidSystemSPH::GetVariableStepSize() {
    // Variable time step requires the state from the previous time step.
    // Thus, it cannot directly be used in the frist time step.
    if (m_paramsH->use_variable_time_step && m_frame != 0) {
        return m_fluid_dynamics->computeTimeStep();
    } else {
        return GetStepSize();
    }
}

void ChFsiFluidSystemSPH::PrintStats() const {
    QuantityLogger::GetInstance().PrintStats();
}

void ChFsiFluidSystemSPH::PrintTimeSteps(const std::string& path) const {
    std::vector<std::string> quantities = {"time_step", "min_courant_viscous_time_step", "min_acceleration_time_step"};
    QuantityLogger::GetInstance().WriteQuantityValuesToFile(path, quantities);
}

void ChFsiFluidSystemSPH::OnDoStepDynamics(double time, double step) {
    SynchronizeCopyStream();
    // Update particle activity
    m_fluid_dynamics->UpdateActivity(m_data_mgr->sphMarkers_D, time);

    // Resize arrays
    bool resize_arrays = m_fluid_dynamics->CheckActivityArrayResize();
    if (m_frame == 0 || resize_arrays) {
        m_data_mgr->ResizeArrays(m_data_mgr->countersH->numExtendedParticles);
    }

    // Perform proximity search
    bool proximity_search = m_frame % m_paramsH->num_proximity_search_steps == 0 || m_force_proximity_search;
    if (proximity_search) {
        m_fluid_dynamics->ProximitySearch();
    }

    // Zero-out step data (derivatives and intermediate vectors)
    m_data_mgr->ResetData();

    // Advance fluid particle states from `time` to `time+step`
    m_fluid_dynamics->DoStepDynamics(m_data_mgr->sortedSphMarkers2_D, time, step, m_paramsH->integration_scheme);

    m_fluid_dynamics->CopySortedToOriginal(MarkerGroup::NON_SOLID, m_data_mgr->sortedSphMarkers2_D,
                                           m_data_mgr->sphMarkers_D);

    ChDebugLog("GPU Memory usage: " << m_data_mgr->GetCurrentGPUMemoryUsage() / 1024.0 / 1024.0 << " MB");

    // Reset flag for forcing a proximity search
    m_force_proximity_search = false;
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

void ChFsiFluidSystemSPH::SynchronizeCopyStream() const {
    m_fluid_dynamics->SynchronizeCopyStream();
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::WriteParticleFile(const std::string& filename) const {
    writeParticleFileCSV(filename, *m_data_mgr);
}

void ChFsiFluidSystemSPH::SaveParticleData(const std::string& dir) const {
    SynchronizeCopyStream();
    if (m_paramsH->elastic_SPH)
        saveParticleDataCRM(dir, m_output_level, *m_data_mgr);
    else
        saveParticleDataCFD(dir, m_output_level, *m_data_mgr);
}

void ChFsiFluidSystemSPH::SaveSolidData(const std::string& dir, double time) const {
    SynchronizeCopyStream();
    saveSolidData(dir, time, *m_data_mgr);
}

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

void ChFsiFluidSystemSPH::AddBCEBoundary(const std::vector<ChVector3d>& points, const ChFramed& frame) {
    for (const auto& p : points)
        m_data_mgr->AddBceMarker(MarkerType::BCE_WALL, ToReal3(frame.TransformPointLocalToParent(p)), {0, 0, 0});
}

//------------------------------------------------------------------------------

const Real pi = Real(CH_PI);

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsPlate(const ChVector2d& size) const {
    std::vector<ChVector3d> bce;

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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsBoxContainer(const ChVector3d& size,
                                                                      const ChVector3i& faces) const {
    std::vector<ChVector3d> bce;

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
    if (faces.z() == -1 || faces.z() == 2) {
        auto bce1 = CreatePointsPlate({size.x(), size.y()});
        ChFramed X(zn, QUNIT);
        std::transform(bce1.begin(), bce1.end(), std::back_inserter(bce), [&X](ChVector3d& v) { return X * v; });
    }

    // Z+ wall
    if (faces.z() == +1 || faces.z() == 2) {
        auto bce1 = CreatePointsPlate({size.x(), size.y()});
        ChFramed X(zp, QuatFromAngleX(CH_PI));
        std::transform(bce1.begin(), bce1.end(), std::back_inserter(bce), [&X](ChVector3d& v) { return X * v; });
    }

    // X- wall
    if (faces.x() == -1 || faces.x() == 2) {
        auto bce1 = CreatePointsPlate({size.z() + buffer, size.y()});
        ChFramed X(xn, QuatFromAngleY(+CH_PI_2));
        std::transform(bce1.begin(), bce1.end(), std::back_inserter(bce), [&X](ChVector3d& v) { return X * v; });
    }

    // X+ wall
    if (faces.x() == +1 || faces.x() == 2) {
        auto bce1 = CreatePointsPlate({size.z() + buffer, size.y()});
        ChFramed X(xp, QuatFromAngleY(-CH_PI_2));
        std::transform(bce1.begin(), bce1.end(), std::back_inserter(bce), [&X](ChVector3d& v) { return X * v; });
    }

    // Y- wall
    if (faces.y() == -1 || faces.y() == 2) {
        auto bce1 = CreatePointsPlate({size.x() + buffer, size.z() + buffer});
        ChFramed X(yn, QuatFromAngleX(-CH_PI_2));
        std::transform(bce1.begin(), bce1.end(), std::back_inserter(bce), [&X](ChVector3d& v) { return X * v; });
    }

    // Y+ wall
    if (faces.y() == +1 || faces.y() == 2) {
        auto bce1 = CreatePointsPlate({size.x() + buffer, size.z() + buffer});
        ChFramed X(yp, QuatFromAngleX(+CH_PI_2));
        std::transform(bce1.begin(), bce1.end(), std::back_inserter(bce), [&X](ChVector3d& v) { return X * v; });
    }

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsBoxInterior(const ChVector3d& size) const {
    std::vector<ChVector3d> bce;

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
        return bce;
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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsBoxExterior(const ChVector3d& size) const {
    std::vector<ChVector3d> bce;

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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsSphereInterior(double radius, bool polar) const {
    std::vector<ChVector3d> bce;

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

        return bce;
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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsSphereExterior(double radius, bool polar) const {
    std::vector<ChVector3d> bce;

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

        return bce;
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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsCylinderInterior(double rad, double height, bool polar) const {
    std::vector<ChVector3d> bce;

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

        return bce;
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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsCylinderExterior(double rad, double height, bool polar) const {
    std::vector<ChVector3d> bce;

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

        return bce;
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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsConeInterior(double rad, double height, bool polar) const {
    std::vector<ChVector3d> bce;

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

        return bce;
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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsConeExterior(double rad, double height, bool polar) const {
    std::vector<ChVector3d> bce;

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

        return bce;
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

    return bce;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsCylinderAnnulus(double rad_inner,
                                                                         double rad_outer,
                                                                         double height,
                                                                         bool polar) const {
    std::vector<ChVector3d> points;

    Real spacing = m_paramsH->d0;
    double hheight = height / 2;

    // Calculate actual spacing
    int np_h = (int)std::round(hheight / spacing);
    double delta_h = hheight / np_h;

    // Use polar coordinates
    if (polar) {
        int np_r = (int)std::round((rad_outer - rad_inner) / spacing);
        double delta_r = (rad_outer - rad_inner) / np_r;
        for (int ir = 0; ir <= np_r; ir++) {
            double r = rad_inner + ir * delta_r;
            int np_th = (int)std::round(2 * pi * r / spacing);
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

        return points;
    }

    // Use a regular grid and accept/reject points
    int np_r = (int)std::round(rad_outer / spacing);
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

    return points;
}

std::vector<ChVector3d> ChFsiFluidSystemSPH::CreatePointsMesh(ChTriangleMeshConnected& mesh) const {
    std::vector<ChVector3d> points;

    Real spacing = m_paramsH->d0;

    // Ensure mesh if watertight
    mesh.RepairDuplicateVertexes(1e-9);
    auto bbox = mesh.GetBoundingBox();

    const double EPSI = 1e-6;

    ChVector3d ray_origin;
    for (double x = bbox.min.x(); x < bbox.max.x(); x += spacing) {
        ray_origin.x() = x + 1e-9;
        for (double y = bbox.min.y(); y < bbox.max.y(); y += spacing) {
            ray_origin.y() = y + 1e-9;
            for (double z = bbox.min.z(); z < bbox.max.z(); z += spacing) {
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

    return points;
}

//------------------------------------------------------------------------------

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

ChAABB ChFsiFluidSystemSPH::GetComputationalDomain() const {
    return ChAABB(ToChVector(m_paramsH->cMin), ToChVector(m_paramsH->cMax));
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
    // This can be called even before Initialize() is called (For instance DepthPressurePropertiesCallback in
    // ChFsiProblemSPH) This means that we need to update Cs based on the current set of parameters.
    if (m_paramsH->elastic_SPH) {
        m_paramsH->Cs = sqrt(m_paramsH->K_bulk / m_paramsH->rho0);
    } else {
        m_paramsH->Cs = 10 * m_paramsH->v_Max;
    }
    return m_paramsH->Cs;
}

ChVector3d ChFsiFluidSystemSPH::GetBodyForce() const {
    return ChVector3d(m_paramsH->bodyForce3.x, m_paramsH->bodyForce3.y, m_paramsH->bodyForce3.z);
}

int ChFsiFluidSystemSPH::GetNumProximitySearchSteps() const {
    return m_paramsH->num_proximity_search_steps;
}

bool ChFsiFluidSystemSPH::GetUseVariableTimeStep() const {
    return m_paramsH->use_variable_time_step;
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

//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------

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
    SynchronizeCopyStream();
    return m_data_mgr->GetPositions();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetVelocities() const {
    SynchronizeCopyStream();
    return m_data_mgr->GetVelocities();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetAccelerations() const {
    SynchronizeCopyStream();
    return m_data_mgr->GetAccelerations();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetForces() const {
    SynchronizeCopyStream();
    return m_data_mgr->GetForces();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetProperties() const {
    SynchronizeCopyStream();
    return m_data_mgr->GetProperties();
}

std::vector<Real3> ChFsiFluidSystemSPH::GetPositions(const std::vector<int>& indices) const {
    SynchronizeCopyStream();
    return m_data_mgr->GetPositions(indices);
}

std::vector<Real3> ChFsiFluidSystemSPH::GetVelocities(const std::vector<int>& indices) const {
    SynchronizeCopyStream();
    return m_data_mgr->GetVelocities(indices);
}

std::vector<Real3> ChFsiFluidSystemSPH::GetAccelerations(const std::vector<int>& indices) const {
    SynchronizeCopyStream();
    return m_data_mgr->GetAccelerations(indices);
}

std::vector<Real3> ChFsiFluidSystemSPH::GetForces(const std::vector<int>& indices) const {
    SynchronizeCopyStream();
    return m_data_mgr->GetForces(indices);
}

}  // end namespace sph
}  // end namespace fsi
}  // end namespace chrono
