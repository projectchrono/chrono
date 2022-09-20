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
// Author:Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Structure to hold the simulation parameters.
//
// For more informaiton about these parameters see the following:
//
// - Using a half-implicit integration scheme for the SPH-based solution of
//   fluid-solid interaction problems,
//   Milad Rakhsha, Arman Pazouki, Radu Serban, Dan Negrut,
//   Computer Methods in Applied Mechanics and Engineering
//
// - A Consistent Multi-Resolution Smoothed Particle Hydrodynamics Method,
//   Wei Hu, Wenxiao Pan, Milad Rakhsha, Qiang Tian, Haiyan Hu, Dan Negrut,
//   Computer Methods in Applied Mechanics and Engineering, 2018
// =============================================================================

#ifndef CH_FSI_PARAMS_H
#define CH_FSI_PARAMS_H

#include <ctime>
#include "chrono_fsi/ChDefinitionsFsi.h"
#include "chrono_fsi/math/ChFsiLinearSolver.h"
#include "chrono_fsi/math/custom_math.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Structure with FSI simulation parameters.
struct SimParams {
    FluidDynamics fluid_dynamic_type;  ///< Type of SPH mehtod (WCSPH, IISPH, or I2SPH)
    int output_length;  ///< Output length (0:short, 1:middle, 2:long) information of SPH particles into data files

    int3 gridSize;        ///< dx, dy, dz distances between particle centers.
    Real3 worldOrigin;    ///< Origin point.
    Real3 cellSize;       ///< Size of the neighbor particle searching cell.
    uint numBodies;       ///< Number of FSI bodies.
    Real3 boxDims;        ///< Dimensions of the domain. How big is the box that the domain is in.
    Real HSML;            ///< Interaction Radius (or h)
    Real INVHSML;         ///< 1.0 / h
    Real INITSPACE;       ///< Initial separation of the fluid particles
    Real INV_INIT;        ///< 1.0 / INITSPACE
    Real MULT_INITSPACE;  ///< Multiplier to hsml to determine the initial separation of the fluid particles and the
                          ///< fixed separation for the boundary particles. This means that the separation will always
                          ///< be a multiple of hsml. Default value = 1.0.
    Real MULT_INITSPACE_Cables;  ///< Multiplier to hsml in cable elements.
    Real MULT_INITSPACE_Shells;  ///< Multiplier to hsml in shell elements.
    int num_neighbors;           ///< Number of neighbor particles.
    Real epsMinMarkersDis;       ///< epsilon mult for minimum distance between markers (d_min = eps * HSML)
    int NUM_BOUNDARY_LAYERS;     ///< Number of particles layers that will be used in the boundary. Default value = 3.
    Real
        toleranceZone;  ///< Helps determine the particles that are in the domain but are outside the boundaries, so
                        ///< they are not considered fluid particles and are dropped at the beginning of the simulation.
    int NUM_BCE_LAYERS;  ///< Number of fixed particle layers to rigid/flexible bodies which act as the boundaries.
                         ///< Default value = 2.

    Real BASEPRES;    ///< Relative value of pressure applied to the whole domain.
    Real LARGE_PRES;  ///< Artificial pressure for boundary particles. Make sure fluid particles do not go through the
                      ///< boundaries.Note that if time step is not small enough particles near the boundaries might
                      ///< build up huge pressures and will make the simulation unstable.

    Real3 deltaPress;  ///< Change in Pressure. This is needed for periodic BC. The change in pressure of a particle
                       ///< when it moves from end boundary to beginning.

    Real3 V_in;  ///< Inlet velocity. This is needed for inlet BC.
    Real x_in;   ///< Inlet position. This is needed for inlet BC.

    Real3 gravity;     ///< Gravity. Applied to fluid, rigid and flexible.
    Real3 bodyForce3;  ///< Constant force applied to the fluid. Flexible and rigid bodies are not affected by this
                       ///< force directly, but instead they are affected indirectly through the fluid.

    Real rho0;       ///< Density
    Real invrho0;    ///< Density's inverse
    Real rho_solid;  ///< Solid Density
    Real volume0;    ///< Initial volume of particle

    Real markerMass;  ///< marker mass
    Real mu0;         ///< Viscosity
    Real kappa;       ///< surface tension parameter
    Real v_Max;  ///< Max velocity of fluid used in equation of state. Run simulation once to be able to determine it.
    Real EPS_XSPH;       ///< Method to modify particle velocity.
    Real beta_shifting;  ///< this is the beta coefficient in the shifting vector formula. See
    Real Vis_Dam;        ///< Viscous damping force

    Real dT;  ///< Time step. Depending on the model this will vary and the only way to determine what time step to use
              ///< is to run simulations multiple time and find which one is the largest dT that produces a stable
              ///< simulation.
    Real INV_dT;  ///< 1.0 / dT

    Real dT_Flex;    ///< Setpsize for the flexible bodies dynamics.
    Real Co_number;  ///< Constant in CFL condition.
    Real dT_Max;     ///< Maximum setpsize.

    Real kdT;      ///< Implicit integration parameter. Not very important
    Real gammaBB;  ///< Equation of state parameter.

    bool use_default_limits;  ///< true if cMin and cMax are not user-provided (default: true)
    bool use_init_pressure;   ///< true if pressure set based on height (default: false)

    Real3 cMinInit;                    ///< Minimum point of the fluid domain.
    Real3 cMaxInit;                    ///< Maximum point of the fluid domain.
    Real3 straightChannelBoundaryMin;  ///< Origin of the coordinate system (Point (0,0,0)). In this case (straigh
                                       ///< channel) this point is where the leftmost particle of the 3rd layer of
                                       ///< boundary particles is located. Everything below this point is outside the
                                       ///< tolerance zone, this means that everything below is not considered part of
                                       ///< the system
    Real3 straightChannelBoundaryMax;  ///< Upper right most part of the system, this is Point(2 * mm, 1 * mm, 3 * mm)
                                       ///< where mm is a constant defined at the beginning of main.cpp.This is also the
                                       ///< rightmost particle of the 3rd layer of the top boundaryparticles.Everything
                                       ///< above this point is not considered part of thesystem.
    Real binSize0;  ///< Suggests the length of the bin each particle occupies. Normally this would be 2*hsml since
                    ///< hsml is the radius of the particle, but when we have periodic boundary condition varies a
                    ///< little from 2 hsml.This may change slightly due to the location of the periodic BC.

    Real3 rigidRadius;  ///< Radius of rigid bodies.

    double pressure_height;  ///< height for pressure initialization

    int densityReinit;  ///< Reinitialize density after densityReinit steps. Note that doing this more frequently helps
                        /// in getting more accurate incompressible fluid, but more stable solution is obtained for
                        /// larger densityReinit

    int contactBoundary;  ///< 0: straight channel, 1: serpentine

    BceVersion bceType;      ///< Type of boundary conditions, ADAMI or ORIGINAL
    BceVersion bceTypeWall;  ///< Type of boundary conditions for fixed wall, ADAMI or ORIGINAL

    bool Conservative_Form;  ///< Whether conservative or consistent discretization should be used
    int gradient_type;       ///< Type of the gradient operator.
    int laplacian_type;      ///< Type of the laplacian operator.

    bool USE_Consistent_G;  ///< Use consistent discretization for gradient operator
    bool USE_Consistent_L;  ///< Use consistent discretization for laplacian operator

    bool USE_NonIncrementalProjection;  ///< Used in the I2SPH implementation
    bool DensityBaseProjetion;          ///< Set true to use density based projetion scheme in ISPH solver

    bool USE_LinearSolver;     ///< If a linear solver should be used to solve Ax=b, otherwise basics methods such as
                               ///< Jacobi-SOR are used
    bool Pressure_Constraint;  ///< Whether the singularity of the pressure equation should be fixed
    SolverType LinearSolver;   ///< Type of the linear solver

    Real Alpha;  ///< Poisson Pressure Equation source term constant. Used to control the noise in the FS forces
    Real Beta;   ///< Moving exponential weighted average coefficient. 1: highly damped force; 0: no force modification

    Real LinearSolver_Abs_Tol;  ///< Poisson Pressure Equation residual
    Real LinearSolver_Rel_Tol;  ///< Poisson Pressure Equation Absolute residual
    int LinearSolver_Max_Iter;  ///< Linear Solver maximum number of iteration
    bool Verbose_monitoring;    ///< Poisson Pressure Equation Absolute residual

    Real Max_Pressure;                  ///< Max Pressure in the pressure solver
    PPESolutionType PPE_Solution_type;  ///< MATRIX_FREE, FORM_SPARSE_MATRIX see Rakhsha et al. 2018 paper for details
                                        ///< omega relaxation in the pressure equation, something less than 0.5 is
                                        ///< necessary for Jacobi solver
    Real PPE_relaxation;                ///< PPE_relaxation
    bool ClampPressure;  ///< Clamp pressure to 0 if negative, based on the IISPH paper by Ihmsen et al. (2013)
    Real IncompressibilityFactor;  ///< Incompressibility factor, default = 1
    Real Cs;                       ///< Speed of sound.

    bool Adaptive_time_stepping;  ///< Works only with IISPH for now, use Co_number can be used as the criteria for
                                  ///< adaptivity. dT_Max is set either from the Co_number, or the time step the is
                                  ///< required for outputting data
    bool Apply_BC_U;              ///< This option lets you apply a velocity BC on the BCE markers
    Real L_Characteristic;        ///< Some length characteristic for Re number computation

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
    Real Ar_stress;     ///< Artifical stress
    Real Ar_vis_alpha;  ///< Artifical viscosity coefficient
    Real Ar_vis_beta;   ///< Artifical viscosity coefficient
    Real Fri_angle;     ///< Frictional angle of granular material
    Real Dil_angle;     ///< Dilate angle of granular material
    Real Coh_coeff;     ///< Cohesion coefficient
    Real Q_FA;          ///< Material constants calculate from frictional angle
    Real Q_DA;          ///< Material constants calculate from dilate angle
    Real K_FA;          ///< Material constants calculate from frictional angle and cohesion coefficient
    Real C_Wi;          ///< Threshold of the integration of the kernel function

    Real boxDimX;  ///< Dimension of the space domain - X
    Real boxDimY;  ///< Dimension of the space domain - Y
    Real boxDimZ;  ///< Dimension of the space domain - Z

    Real3 cMin;  ///< Lower limit point
    Real3 cMax;  ///< Upper limit point

    Real3 bodyActiveDomain;  ///< Size of the active domain that influenced by an FSI body
    Real settlingTime;       ///< Time for the granular to settle down
};

/// @} fsi_physics

}  // namespace fsi
}  // namespace chrono

#endif
