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
#include "chrono_fsi/sph/math/ChFsiLinearSolver.h"
#include "chrono_fsi/sph/math/CustomMath.h"
#include "chrono_fsi/sph/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsi_physics
/// @{

/// Structure with FSI simulation parameters.
struct SimParams {
    SPHMethod sph_method;  ///< SPH mehtod (WCSPH or I2SPH)
    int output_length;     ///< Output length (0:short, 1:middle, 2:long) information of SPH particles into data files

    int3 gridSize;          ///< dx, dy, dz distances between particle centers.
    Real3 worldOrigin;      ///< Origin point.
    Real3 cellSize;         ///< Size of the neighbor particle searching cell.
    uint numBodies;         ///< Number of FSI bodies.
    Real3 boxDims;          ///< Dimensions of the domain. How big is the box that the domain is in.
    Real d0;                ///< Initial separation of SPH particles
    Real ood0;              ///< 1 / d0
    Real h_multiplier;      ///< Multiplier of initial spacing to obtain the interaction radius, h
    Real h;                 ///< Kernel interaction radius, h = h_multiplier * d0
    Real ooh;               ///< 1 / h
    int num_neighbors;      ///< Number of neighbor particles.
    Real epsMinMarkersDis;  ///< epsilon mult for minimum distance between markers (d_min = eps * HSML)
    int num_bce_layers;     ///< Number of BCE marker layers attached to boundary and solid surfaces. Default value = 3.
    Real
        toleranceZone;  ///< Helps determine the particles that are in the domain but are outside the boundaries, so
                        ///< they are not considered fluid particles and are dropped at the beginning of the simulation.

    Real BASEPRES;  ///< Relative value of pressure applied to the whole domain.

    Real3 deltaPress;  ///< Change in Pressure. This is needed for periodic BC. The change in pressure of a particle
                       ///< when it moves from end boundary to beginning.

    Real3 V_in;  ///< Inlet velocity. This is needed for inlet BC.
    Real x_in;   ///< Inlet position. This is needed for inlet BC.

    Real3 gravity;     ///< Gravity. Applied to fluid, rigid and flexible.
    Real3 bodyForce3;  ///< Constant force applied to the fluid. Flexible and rigid bodies are not affected by this
                       ///< force directly, but instead they are affected indirectly through the fluid.

    Real rho0;     ///< Density
    Real invrho0;  ///< Density's inverse
    Real volume0;  ///< Initial volume of particle

    Real markerMass;  ///< marker mass
    Real mu0;         ///< Viscosity
    Real v_Max;  ///< Max velocity of fluid used in equation of state. Run simulation once to be able to determine it.
    Real EPS_XSPH;       ///< Method to modify particle velocity.
    Real beta_shifting;  ///< this is the beta coefficient in the shifting vector formula. See

    Real dT;  ///< Time step. Depending on the model this will vary and the only way to determine what time step to use
              ///< is to run simulations multiple time and find which one is the largest dT that produces a stable
              ///< simulation.

    Real kdT;      ///< Implicit integration parameter. Not very important
    Real gammaBB;  ///< Equation of state parameter.

    bool use_default_limits;  ///< true if cMin and cMax are not user-provided (default: true)
    bool use_init_pressure;   ///< true if pressure set based on height (default: false)

    Real3 cMinInit;  ///< Minimum point of the fluid domain.
    Real3 cMaxInit;  ///< Maximum point of the fluid domain.
    Real binSize0;   ///< Suggests the length of the bin each particle occupies. Normally this would be 2*hsml since
                     ///< hsml is the radius of the particle, but when we have periodic boundary condition varies a
                     ///< little from 2 hsml.This may change slightly due to the location of the periodic BC.

    double pressure_height;  ///< height for pressure initialization

    int densityReinit;  ///< Reinitialize density after densityReinit steps. Note that doing this more frequently helps
                        /// in getting more accurate incompressible fluid, but more stable solution is obtained for
                        /// larger densityReinit

    bool Conservative_Form;  ///< Whether conservative or consistent discretization should be used
    int gradient_type;       ///< Type of the gradient operator.
    int laplacian_type;      ///< Type of the laplacian operator.

    bool USE_Consistent_G;  ///< Use consistent discretization for gradient operator
    bool USE_Consistent_L;  ///< Use consistent discretization for laplacian operator
    bool USE_Delta_SPH;     ///< Use delta SPH
    Real density_delta;     ///< Parameter for delta SPH
    EosType eos_type;       ///< Equation of state type
    ViscosityType
        viscosity_type;  ///< Viscosity treatment type, physics-based laminar flow or artificial viscosity
                         ///< (Artificial Unilateral) or artificial viscosity that also opposes also particle separation
    BoundaryType boundary_type;  ///< Boundary type - Adami or Holmes

    bool DensityBaseProjection;  ///< Set true to use density based projetion scheme in ISPH solver

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
    Real IncompressibilityFactor;  ///< Incompressibility factor, default = 1
    Real Cs;                       ///< Speed of sound

    bool Apply_BC_U;        ///< This option lets you apply a velocity BC on the BCE markers
    Real L_Characteristic;  ///< Characteristic for Re number computation

    bool non_newtonian;       ///< Set true to model non-newtonian fluid.
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

    bool elastic_SPH;   ///< Handles the WCSPH solver for fluid (false) or granular (true)
    Real E_young;       ///< Young's modulus
    Real G_shear;       ///< Shear modulus
    Real INV_G_shear;   ///< 1.0 / G_shear
    Real K_bulk;        ///< Bulk modulus
    Real Nu_poisson;    ///< Poisson’s ratio
    Real Ar_vis_alpha;  ///< Artifical viscosity coefficient
    Real Coh_coeff;     ///< Cohesion coefficient
    Real C_Wi;          ///< Threshold of the integration of the kernel function

    Real boxDimX;  ///< Dimension of the space domain - X
    Real boxDimY;  ///< Dimension of the space domain - Y
    Real boxDimZ;  ///< Dimension of the space domain - Z

    Real3 cMin;  ///< Lower limit point
    Real3 cMax;  ///< Upper limit point

    Real3 bodyActiveDomain;  ///< Size of the active domain that influenced by an FSI body
    Real settlingTime;       ///< Time for the granular to settle down

    int num_proximity_search_steps;  ///< Number of steps between updates to neighbor lists
};

/// @} fsi_physics

}  // namespace sph
}  // namespace fsi
}  // namespace chrono

#endif
