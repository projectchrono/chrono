// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Miscellaneous enumerations for the SPH-based Chrono::FSI fluid solver
//
// =============================================================================

#ifndef CH_FSI_DEFINITIONS_SPH_H
#define CH_FSI_DEFINITIONS_SPH_H

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph
/// @{

/// Physics problem type.
enum class PhysicsProblem {
    CFD,  ///< incompressible fluid problem
    CRM   ///< continuous granular problem
};

//// TODO RADU: - fix the inconsistency related to solid force calculation between WCSPH and ISPH.
////            - once that is done, we can move integration_scheme out of the ChFsiParamsSPH structure.

/// Integration scheme.
/// All explicit integratioon schemes use a Weakly-Compressible SPH (WCSPH) formulation in which the density is
/// integrated and an equation of state is used to calculate the corresponding pressure. For the implicit SPH scheme,
/// the pressure is instead updated.
enum class IntegrationScheme {
    EULER,        ///< Explicit Euler
    RK2,          ///< Runge-Kutta 2
    VERLET,       ///< Velocity Verlet
    SYMPLECTIC,   ///< Symplectic Euler
    IMPLICIT_SPH  ///< Implicit SPH
};

/// Shifting Methods.
enum class ShiftingMethod { NONE, PPST, XSPH, PPST_XSPH, DIFFUSION, DIFFUSION_XSPH };

/// Equation of State type.
/// See https://pysph.readthedocs.io/en/latest/reference/equations.html#basic-wcsph-equations.
/// An equation of state is used only with an explicit WCSPH formulation.
enum class EosType { TAIT, ISOTHERMAL };

/// SPH kernel type.
enum class KernelType { QUADRATIC, CUBIC_SPLINE, QUINTIC_SPLINE, WENDLAND };

/// Visosity method type.
enum class ViscosityMethod { LAMINAR, ARTIFICIAL_UNILATERAL, ARTIFICIAL_BILATERAL };

/// Boundary method type.
enum class BoundaryMethod { ADAMI, HOLMES };

/// Rheology type.
enum class Rheology { INERTIA_RHEOLOGY, NONLOCAL_FLUIDITY };

/// Friction law in ISPH.
enum class FrictionLaw { CONSTANT, LINEAR, NONLINEAR };

/// Linear solver type.
enum class SolverType { JACOBI, BICGSTAB, GMRES, CR, CG, SAP };

// -----------------------------------------------------------------------------

/// Enumeration for specifying whether the sides of a computational domain are
/// periodic or not. Sides can be combined using binary OR operations.
namespace PeriodicSide {
enum Enum {
    NONE = 0x0000,
    X = 1 << 0,   ///< X direction (both positive and negative) is periodic
    Y = 1 << 1,   ///< Y direction (both positive and negative) is periodic
    Z = 1 << 2,   ///< Z direction (both positive and negative) is periodic
    ALL = 0xFFFF  ///< All directions (X, Y, Z) are periodic
};
}

/// Boundary conditions along directions of the computational domain.
enum class BCType {
    NONE,         ///< no boundary conditions enforced
    PERIODIC,     ///< periodic boundary conditions
    INLET_OUTLET  ///< inlet-outlet boundary conditions
};

/// Boundary condition types in all three directions of the computational domain.
struct BoundaryConditions {
    BCType x;
    BCType y;
    BCType z;
};

constexpr BoundaryConditions BC_NONE = {BCType::NONE, BCType::NONE, BCType::NONE};
constexpr BoundaryConditions BC_X_PERIODIC = {BCType::PERIODIC, BCType::NONE, BCType::NONE};
constexpr BoundaryConditions BC_Y_PERIODIC = {BCType::NONE, BCType::PERIODIC, BCType::NONE};
constexpr BoundaryConditions BC_Z_PERIODIC = {BCType::NONE, BCType::NONE, BCType::PERIODIC};
constexpr BoundaryConditions BC_ALL_PERIODIC = {BCType::PERIODIC, BCType::PERIODIC, BCType::PERIODIC};

/// Enumeration for box sides.
/// These flags are used to identify sides of a box container and can be combined using unary boolean operations.
namespace BoxSide {
enum Enum {
    NONE = 0x0000,
    X_POS = 1 << 0,
    X_NEG = 1 << 1,
    Y_POS = 1 << 2,
    Y_NEG = 1 << 3,
    Z_POS = 1 << 4,
    Z_NEG = 1 << 5,
    ALL = 0xFFFF
};
}

/// Enumeration for cylinder sides.
/// These flags are used to identify sides of a cylindrical container and can be combined using unary boolean
/// operations.
namespace CylSide {
enum Enum { NONE = 0x0000, SIDE_INT = 1 << 0, SIDE_EXT = 1 << 1, Z_NEG = 1 << 2, Z_POS = 1 << 3, ALL = 0xFFFF };
}

/// BCE pattern in cross section of 1-D flexible elements.
/// The available patterns are illustrated below (assuming 3 BCE layers):
/// <pre>
/// FULL:
///      X--X--X
///      X--X--X
///      X--X--X
/// STAR:
///      ---X---
///      X--X--X
///      ---X---
/// </pre>
enum class BcePatternMesh1D { FULL, STAR };

/// BCE pattern along normal of 2-D surface of flexible elements.
/// The choices are illustrated below (assuming 3 BCE layers):
/// <pre>
/// OUTWARD:
///    ^ n
///    |    ...--X--X--X--...
///    |    ...--X--X--X--...
/// ---|---------X--X--X-------- surface
///
/// CENTERED:
///    ^ n
///    |    ...--X--X--X--...
/// ---|---------X--X--X-------- surface
///    |    ...--X--X--X--...
///
/// INWARD:
///    ^ n
/// ---|---------X--X--X-------- surface
///    |    ...--X--X--X--...
///    |    ...--X--X--X--...
/// </pre>
enum class BcePatternMesh2D { CENTERED, OUTWARD, INWARD };

/// Output level.
enum class OutputLevel {
    STATE,           ///< marker state, velocity, and acceleration
    STATE_PRESSURE,  ///< STATE plus density and pressure
    CFD_FULL,        ///< STATE_PRESSURE plus various CFD parameters
    CRM_FULL         ///< STATE_PRESSURE plus normal and shear stress
};

/// @} fsisph

}  // namespace sph
}  // namespace fsi
}  // namespace chrono

#endif
