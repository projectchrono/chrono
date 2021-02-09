///< =============================================================================
///< PROJECT CHRONO - http://projectchrono.org
//
///< Copyright (c) 2014 projectchrono.org
///< All rights reserved.
//
///< Use of this source code is governed by a BSD-style license that can be found
///< in the LICENSE file at the top level of the distribution and at
///< http://projectchrono.org/license-chrono.txt.
//
///< =============================================================================
///< Author:Arman Pazouki, Milad Rakhsha, Wei Hu
///< =============================================================================
//
///< Structure to hold the simulation parameters.
//
///< For more informaiton about these parameters see the following:
//
///< - Using a half-implicit integration scheme for the SPH-based solution of
///<   fluid-solid interaction problems,
///<   Milad Rakhsha, Arman Pazouki, Radu Serban, Dan Negrut,
///<   Computer Methods in Applied Mechanics and Engineering
//
///< - A Consistent Multi-Resolution Smoothed Particle Hydrodynamics Method,
///<   Wei Hu, Wenxiao Pan, Milad Rakhsha, Qiang Tian, Haiyan Hu, Dan Negrut,
///<   Computer Methods in Applied Mechanics and Engineering, 2018
///< =============================================================================

#ifndef CHPARAMS_CUH_
#define CHPARAMS_CUH_

#include <ctime>
#include "chrono_fsi/math/ChFsiLinearSolver.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {

enum BceVersion { ADAMI = 0, mORIGINAL = 1 };
enum PPE_SolutionType { MATRIX_FREE, FORM_SPARSE_MATRIX };
enum rheology { Inertia_rheology, nonlocal_fluidity };
enum friction_law { constant, linear, nonlinear };
enum fluid_dynamics { IISPH, I2SPH, WCSPH };

struct SimParams {
    fluid_dynamics fluid_dynamic_type;  ///< Type of SPH mehtod
    char out_name[256];
    char demo_dir[2048];

    int3 gridSize;        ///<  dx, dy, dz distances between particle centers.
    Real3 worldOrigin;    ///<
    Real3 cellSize;       ///<
    uint numBodies;       ///<
    Real3 boxDims;        ///<  Dimensions of the domain. How big is the box that the domain is in.
    Real HSML;            ///<  Interaction Radius. (or h)
    Real MULT_INITSPACE;  ///<  Multiplier to hsml to determine the initial separation of the fluid particles and the
                          ///<  fixed
                          ///<     separation for the boundary particles. This means that the separation will always be
                          ///<     a multiple of hsml. Default value = 1.0.
    Real MULT_INITSPACE_Cables;
    Real MULT_INITSPACE_Shells;
    int num_neighbors;
    Real epsMinMarkersDis;    ///<  epsilon mult for minimum distance between markers (d_min = eps * HSML)
    int NUM_BOUNDARY_LAYERS;  ///<   Number of particles layers that will be used in the boundary. Default value = 3.
    Real toleranceZone;  ///<  Helps determine the particles that are in the domain but are outside the boundaries, so
                         ///<  they
                         ///<    are not considered fluid particles and
                         ///<        are dropped at the beginning of the
                         ///<            simulation.
    int NUM_BCE_LAYERS;  ///<  Number of fixed particle layers to rigid/flexible bodies which act as
                         ///<  the boundaries. Default
                         ///<    value = 2.
    Real solidSurfaceAdjust;  ///<
    Real BASEPRES;            ///<  Relative value of pressure applied to the whole domain.
    Real LARGE_PRES;  ///<  Artificial pressure for boundary particles. Make sure fluid particles do not go through the
                      ///<  boundaries.Note that if time step is not small enough particles near the boundaries might
                      ///<  build up huge pressures and will make the simulation unstable.

    Real3 deltaPress;  ///<  Change in Pressure. This is needed for periodic BC. The change in pressure of a particle
                       ///<  when it moves from end boundary to beginning.

    Real3 V_in;  ///<  Inlet Velocity. This is needed for inlet BC.
    Real x_in;

    Real3 gravity;     ///<  Gravity. Applied to fluid, rigid and flexible.
    Real3 bodyForce3;  ///<  Constant force applied to the fluid. Flexible and rigid bodies are not affected by this
                       ///<  force
                       ///<    directly, but instead they are affected indirectly through the fluid.

    Real rho0;       ///<  Density
    Real rho_solid;  ///<  Solid Density

    Real markerMass;  ///<  marker mass
    Real mu0;         ///<  Viscosity
    Real kappa;       ///<  surface tension parameter
    Real v_Max;  ///<  Max velocity of fluid used in equation of state. Run simulation once to be able to determine it.
    Real EPS_XSPH;       ///<  Method to modify particle velocity.
    Real beta_shifting;  ///<  this is the beta coefficient in the shifting vector formula. See
    Real Vis_Dam;        ///< Viscous damping force

    Real dT;  ///<   Time step. Depending on the model this will vary and the only way to determine what time step to
              ///<   use is to run simulations multiple time and find which one is the largest dT that produces a stable
              ///<   simulation.
    enum fluidity_model { frictional_plasticity, Inertia_rheology, nonlocal_fluidity };

    Real dT_Flex;
    Real Co_number;
    Real dT_Max;
    Real out_fps;
    Real tFinal;     ///<  Total simulation time.
    Real timePause;  ///<  Time that we let pass before applying body forces. This is done to allow the particles to
                     ///<  stabilize first.Run the fluid only during this time, with dTm = 0.1 * dT

    Real timePauseRigidFlex;  ///<  Time before letting rigid/flex move. Keep the rigid and
                              ///<  flex stationary during this time  (timePause + timePauseRigidFlex) until the fluid
                              ///<  is fully developed

    Real kdT;      ///<  Implicit integration parameter. Not very important
    Real gammaBB;  ///<  Equation of state parameter.
    Real3 cMin;    ///<  Lower leftmost part of the space shown in a simulation frame. This point is usually outside the
                   ///<  tolerance zone.
    Real3 cMax;    ///<  Upper right most part of the space shown in a simulation frame. This point is usually outside
                   ///<  thetolerance zone.

    bool ApplyInFlowOutFlow;
    Real3 inflow;
    Real3 outflow;

    Real3 cMinInit;                    ///<
    Real3 cMaxInit;                    ///<
    Real3 straightChannelBoundaryMin;  ///<  Origin of the coordinate system (Point (0,0,0)). In this case (straigh
                                       ///<  channel) this point is where the leftmost particle of the 3rd layer of
                                       ///<  boundary particles is located. Everything below this point is outside the
                                       ///<  tolerance zone, this means that everything below is not considered part of
                                       ///<  the system
    Real3
        straightChannelBoundaryMax;  ///<  Upper right most part of the system, this is Point(2 * mm, 1 * mm, 3 * mm)
                                     ///<  where mm is a constant defined at the beginning of main.cpp.This is also the
                                     ///<  rightmost particle of the 3rd layer of the top boundaryparticles.Everything
                                     ///<  above this point is not considered part of thesystem.
    Real binSize0;  ///<  Suggests the length of the bin each particle occupies. Normally this would be 2*hsml since
                    ///<  hsml is the radius of the particle, but when we have periodic boundary condition varies a
                    ///<  little from 2 hsml.This may change slightly due to the location of the periodic BC.

    Real3 rigidRadius;  ///<  Radius of rigid bodies.

    ////< Reinitialize density after densityReinit steps. Note that doing this more frequently helps in getting more
    ////< accurate incompressible fluid, but more stable solution is obtained for larger densityReinit
    int densityReinit;
    int contactBoundary;  ///<  0: straight channel, 1: serpentine

    BceVersion bceType;  ///< Type of boundary conditions, ADAMI or mORIGINAL

    ////< Whether conservative or consistent discretization should be used
    bool Conservative_Form;
    int gradient_type;
    int laplacian_type;
    ////< Used in the I2SPH implementation
    bool USE_NonIncrementalProjection;
    bool DensityBaseProjetion;

    ////< If a linear solver should be used to solve Ax=b, otherwise basics methods such as Jacobi-SOR are used
    bool USE_LinearSolver;
    bool Pressure_Constraint;                    ///< Whether the singularity of the pressure equation should be fixed
    ChFsiLinearSolver::SolverType LinearSolver;  ///< Type of the linear solver
    Real Alpha;                                  ///< Poisson Pressure Equation source term constant
    ///< This is used for controlling the noise in the FS forces, F_new=beta*F_old+(1-beta)*F_t
    ///< beta=1 highly damped force, beta=0 : no modificaiton to force
    Real Beta;  ///< moving exponential weighted average coefficient

    Real LinearSolver_Abs_Tol;  ///< Poisson Pressure Equation residual
    Real LinearSolver_Rel_Tol;  ///< Poisson Pressure Equation Absolute residual
    int LinearSolver_Max_Iter;  ///< Linear Solver maximum number of iteration
    bool Verbose_monitoring;    ///< Poisson Pressure Equation Absolute residual

    Real Max_Pressure;                   ///< Max Pressure in the pressure solver
    PPE_SolutionType PPE_Solution_type;  ///< MATRIX_FREE, FORM_SPARSE_MATRIX see Rakhsha et al. 2018 paper for details
    ///< omega relaxation in the pressure equation, something less than 0.5 is necessary for Jacobi solver
    Real PPE_relaxation;
    bool ClampPressure;  ///< Clamp pressure to 0 if negative, based on the IISPH paper by Ihmsen et al. (2013)
    Real IncompressibilityFactor;
    Real Cs;

    ///< Works only with IISPH for now, use Co_number can be used as the criteria for adaptivity
    bool Adaptive_time_stepping;
    ////< dT_Max is set either from the Co_number, or the time step the is required for outputting data
    bool Apply_BC_U;        ///< This option lets you apply a velocity BC on the BCE markers
    Real L_Characteristic;  ///< Some length characteristic for Re number computation

    bool non_newtonian;
    bool granular_material;
    rheology rheology_model;
    Real ave_diam;  ///< average particle diameter
    Real cohesion;  ///< c in the stress model sigma=(mu*p+c)/|D|
    friction_law mu_of_I;
    Real mu_max;     ///< maximum viscosity
    Real mu_fric_s;  ///< friction mu_s
    Real mu_fric_2;  ///< mu_2 constant in mu=mu(I)
    Real mu_I0;      ///< Reference Inertia number
    Real mu_I_b;     ///< b constant in mu=mu(I)=mu_s+b*I
    Real Shear_Mod;  ///< G

    Real HB_sr0;   ///< Herschel–Bulkley consistency index
    Real HB_k;     ///< Herschel–Bulkley consistency index
    Real HB_n;     ///< Herschel–Bulkley  power
    Real HB_tau0;  ///< Herschel–Bulkley yeild stress

    ///< this part is for elastic SPH
    bool elastic_SPH;
    Real E_young;       ///< Young's modulus
    Real G_shear;       ///< shear modulus
    Real K_bulk;        ///< bulk modulus
    Real Nu_poisson;    ///< Poisson’s ratio
    Real Ar_stress;     ///< Artifical stress
    Real Ar_vis_alpha;  ///< Artifical viscosity coefficient
    Real Ar_vis_beta;   ///< Artifical viscosity coefficient
    Real Fri_angle;     ///< frictional angle of granular material
    Real Dil_angle;     ///< dilate angle of granular material
    Real Coh_coeff;     ///< cohesion coefficient
    Real Q_FA;          ///< material constants calculate from frictional angle
    Real Q_DA;          ///< material constants calculate from dilate angle
    Real K_FA;          ///< material constants calculate from frictional angle and cohesion coefficient

    ////< Dimension of the space domain
    Real boxDimX;
    Real boxDimY;
    Real boxDimZ;
    ////< Dimension of the fluid domain
    Real fluidDimX;
    Real fluidDimY;
    Real fluidDimZ;

    ///< information of the body in fluid/granular material
    Real bodyDimX;
    Real bodyDimY;
    Real bodyDimZ;
    Real bodyRad;
    Real bodyLength;
    Real bodyIniPosX;
    Real bodyIniPosY;
    Real bodyIniPosZ;
    Real bodyMass;
    Real bodyDensity;
};  // namespace fsi

}  // namespace fsi
}  // namespace chrono

#endif
