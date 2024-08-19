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
// Author: Radu Serban
// =============================================================================
//
// Miscellaneous enumerations for Chrono::FSI
//
// =============================================================================

#ifndef CH_DEFINITIONS_FSI_H
#define CH_DEFINITIONS_FSI_H

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

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

/// Rheology type.
enum class Rheology { INERTIA_RHEOLOGY, NONLOCAL_FLUIDITY };

/// Friction law in ISPH.
enum class FrictionLaw { CONSTANT, LINEAR, NONLINEAR };

/// SPH method.
enum class SPHMethod {
    WCSPH,  ///< Weakly Compressible SPH (explicit)
    I2SPH   ///< Implicit SPH
};

/// Linear solver type.
enum class SolverType { JACOBI, BICGSTAB, GMRES, CR, CG, SAP };

/// @} fsi_physics

}  // namespace fsi
}  // namespace chrono

#endif
