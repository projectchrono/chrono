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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Structure to hold the simulation parameters.
//
// For more information about these parameters see:
// "Using a half-implicit integration scheme for the SPH-based solution of
// fluid-solid interaction problems," M. Rakhsha, A. Pazouki, R. Serban, D. Negrut,
// Computer Methods in Applied Mechanics and Engineering
// =============================================================================

#ifndef CH_FSI_PARAMS_H
#define CH_FSI_PARAMS_H

#include <ctime>

#include "chrono_fsi/sph/ChFsiDefinitionsSPH.h"
#include "chrono_fsi/sph/ChFsiDataTypesSPH.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph
/// @{

/// Structure with FSI simulation parameters.
struct ChFsiParamsSPH {
    IntegrationScheme integration_scheme;  ///< Integration scheme
    EosType eos_type;                      ///< Equation of state type (Tait or isothermal)
    ViscosityMethod viscosity_method;      ///< Viscosity treatment type (physics-based laminar flow or artificial)
    BoundaryMethod boundary_method;        ///< Boundary type (Adami or Holmes)
    KernelType kernel_type;                ///< Kernel type (Quadratic, cubic spline, quintinc spline, quintic Wendland)
    ShiftingMethod shifting_method;        ///< Shifting method (NONE, PPST, XSPH, PPST_XSPH)

    bool elastic_SPH;  ///< Set physics problem: CFD (false) or CRM granular (true)

    int3 gridSize;          ///< dx, dy, dz distances between particle centers
    Real3 worldOrigin;      ///< Origin point
    Real3 cellSize;         ///< Cell size for the neighbor particle search
    uint numBodies;         ///< Number of FSI bodies.
    Real3 boxDims;          ///< Dimensions (AABB) of the domain
    Real3 zombieBoxDims;    ///< Dimensions (AABB) of the zombie domain
    Real3 zombieOrigin;     ///< Origin point of the zombie domain
    Real d0;                ///< Initial separation of SPH particles
    Real ood0;              ///< 1 / d0
    Real d0_multiplier;     ///< Multiplier to obtain the interaction length, h = d0_multiplier * d0
    Real h;                 ///< Kernel interaction length
    Real ooh;               ///< 1 / h
    Real h_multiplier;      ///< Multiplier to obtain kernel radius, r = h_multiplier * h (depends on kernel type)
    int num_neighbors;      ///< Number of neighbor particles
    Real epsMinMarkersDis;  ///< Multiplier for minimum distance between markers (d_min = eps * h)
    int num_bce_layers;     ///< Number of BCE marker layers attached to boundary and solid surfaces (default: 3)
    Real
        toleranceZone;  ///< Helps determine the particles that are in the domain but are outside the boundaries, so
                        ///< they are not considered fluid particles and are dropped at the beginning of the simulation.

    Real base_pressure;    ///< Relative value of pressure applied to the whole domain
    Real3 delta_pressure;  ///< Change in Pressure for periodic BC (when particle moves from one side to the other)

    Real3 V_in;  ///< Inlet velocity for inlet BC
    Real x_in;   ///< Inlet position for inlet BC

    Real3 gravity;     ///< Gravitational acceleration
    Real3 bodyForce3;  ///< Constant force applied to the fluid particles (solids not directly affected)

    Real rho0;     ///< Density
    Real invrho0;  ///< 1 / rho0
    Real volume0;  ///< Initial particle volume

    Real markerMass;  ///< marker mass
    Real mu0;         ///< Viscosity
    Real v_Max;  ///< Max velocity of fluid used in equation of state. Run simulation once to be able to determine it.
    Real shifting_xsph_eps;        ///< Coefficient for XSPH shifting
    Real shifting_ppst_push;       ///< Coefficient for PPST shifting - this is applied when penetration with fictitious
                                   ///< sphere is detected
    Real shifting_ppst_pull;       ///< Coefficient for PPST pulling - this is applied when penetration with fictitious
    Real shifting_beta_implicit;   ///< Coefficient for shifting used in implicit scheme
    Real shifting_diffusion_A;     ///< TODO: Add documentation
    Real shifting_diffusion_AFSM;  ///< TODO: Add documentation
    Real shifting_diffusion_AFST;  ///< TODO: Add documentation

    Real dT;  ///< Time step. Depending on the model this will vary and the only way to determine what time step to
              ///< use is to run simulations multiple time and find which one is the largest dT that produces a
              ///< stable simulation.

    Real kdT;      ///< Implicit integration parameter
    Real gammaBB;  ///< Equation of state parameter

    bool use_default_limits;  ///< true if cMin and cMax are not user-provided (default: true)
    bool use_init_pressure;   ///< true if pressure set based on height (default: false)

    Real3 cMinInit;  ///< Minimum point of the fluid domain.
    Real3 cMaxInit;  ///< Maximum point of the fluid domain.
    Real binSize0;   ///< Suggests the length of the bin each particle occupies. Normally this would be 2*hsml since
                     ///< hsml is the radius of the particle, but when we have periodic boundary condition varies a
                     ///< little from 2 hsml.This may change slightly due to the location of the periodic BC.

    double pressure_height;  ///< height for pressure initialization

    // Note: more frequent re-initialization helps in getting more accurate incompressible fluid, 
    // but more stable solution is obtained for larger values of density_reinit_steps
    int density_reinit_steps;  ///< reinitialize density after density_reinit_steps steps

    bool Conservative_Form;  ///< use conservative or consistent discretization
    int gradient_type;       ///< Type of the gradient operator
    int laplacian_type;      ///< Type of the laplacian operator

    bool use_consistent_gradient_discretization;                         ///< use consistent discretization for gradient operator
    bool use_consistent_laplacian_discretization;  ///< use consistent discretization for laplacian operator
    bool use_delta_sph;                            ///< use delta SPH
    Real density_delta;                            ///< parameter for delta SPH

    bool use_density_based_projection;  ///< Set true to use density based projetion scheme in ISPH solver

    bool Pressure_Constraint;  ///< Whether the singularity of the pressure equation should be fixed
    SolverType LinearSolver;   ///< Type of the linear solver

    Real Alpha;  ///< Poisson Pressure Equation source term constant. Used to control the noise in the FS forces

    Real LinearSolver_Abs_Tol;  ///< Poisson Pressure Equation residual
    Real LinearSolver_Rel_Tol;  ///< Poisson Pressure Equation Absolute residual
    int LinearSolver_Max_Iter;  ///< Linear Solver maximum number of iteration
    bool Verbose_monitoring;    ///< Poisson Pressure Equation Absolute residual

    Real Max_Pressure;             ///< Max Pressure in the pressure solver
    Real PPE_relaxation;           ///< PPE_relaxation
    bool ClampPressure;            ///< Clamp pressure to 0 if negative, based on the ISPH paper by Ihmsen et al. (2013)
    Real IncompressibilityFactor;  ///< Incompressibility factor (default: 1)
    Real Cs;                       ///< Speed of sound

    bool Apply_BC_U;        ///< This option lets you apply a velocity BC on the BCE markers
    Real L_Characteristic;  ///< Characteristic for Re number computation

    bool non_newtonian;       ///< Set true to model non-newtonian fluid
    Rheology rheology_model;  ///< Model of the rheology
    Real ave_diam;            ///< average particle diameter
    Real cohesion;            ///< c in the stress model sigma=(mu*p+c)/|D|
    FrictionLaw mu_of_I;      ///< Constant I in granular material dyanmcis
    Real mu_max;              ///< maximum viscosity
    Real mu_fric_s;           ///< friction mu_s
    Real mu_fric_2;           ///< mu_2 constant in mu=mu(I)
    Real mu_I0;               ///< Reference Inertia number
    Real mu_I_b;              ///< b constant in mu=mu(I)=mu_s+b*I

    Real HB_sr0;   ///< Herschel–Bulkley consistency index
    Real HB_k;     ///< Herschel–Bulkley consistency index
    Real HB_n;     ///< Herschel–Bulkley  power
    Real HB_tau0;  ///< Herschel–Bulkley yeild stress

    Real E_young;       ///< Young's modulus
    Real G_shear;       ///< Shear modulus
    Real INV_G_shear;   ///< 1.0 / G_shear
    Real K_bulk;        ///< Bulk modulus
    Real Nu_poisson;    ///< Poisson's ratio
    Real artificial_viscosity;  ///< Artifical viscosity coefficient
    Real Coh_coeff;     ///< Cohesion coefficient
    Real free_surface_threshold;          ///< Threshold of the integration of the kernel function

    Real boxDimX;  ///< Dimension of the space domain - X
    Real boxDimY;  ///< Dimension of the space domain - Y
    Real boxDimZ;  ///< Dimension of the space domain - Z

    BoundaryConditions bc_type;  ///< boundary condition types in the 3 domain directions
    bool x_periodic;             ///< periodic boundary conditions in x direction?
    bool y_periodic;             ///< periodic boundary conditions in y direction?
    bool z_periodic;             ///< periodic boundary conditions in z direction?

    int3 minBounds;  ///< Lower limit point of the grid (in grid index)
    int3 maxBounds;  ///< Upper limit point of the grid (in grid index)

    Real3 cMin;  ///< Lower limit point (in world coordinates)
    Real3 cMax;  ///< Upper limit point (in world coordinates)

    Real3 zombieMin;  ///< Lower limit point of the zombie domain -> All particles outside this will be frozen
    Real3 zombieMax;  ///< Upper limit point of the zombie domain -> All particles outside this will be frozen

    Real3 bodyActiveDomain;  ///< Size of the active domain that influenced by an FSI body
    bool use_active_domain;  ///< Set to true if active domain is used
    Real settlingTime;       ///< Time for the granular to settle down

    int num_proximity_search_steps;  ///< Number of steps between updates to neighbor lists
    bool use_variable_time_step;     ///< use variable time step (default: false)
};

/// @} fsisph

}  // namespace sph
}  // namespace fsi
}  // namespace chrono

#endif
