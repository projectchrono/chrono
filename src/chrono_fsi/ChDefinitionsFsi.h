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

/// Approach to handle BCE particles
enum class BceVersion { ADAMI = 0, ORIGINAL = 1 };

/// PPE solution type
enum class PPESolutionType { MATRIX_FREE, FORM_SPARSE_MATRIX };

/// Rheology type
enum class Rheology { INERTIA_RHEOLOGY, NONLOCAL_FLUIDITY };

////enum fluidity_model { frictional_plasticity, Inertia_rheology, nonlocal_fluidity };

/// Friction law in ISPH
enum class FrictionLaw { CONSTANT, LINEAR, NONLINEAR };

/// Dynamics solver type for fluid/granular
enum class FluidDynamics { IISPH, I2SPH, WCSPH };

/// Time integration method
enum class TimeIntegrator { EXPLICITSPH, IISPH, I2SPH };

/// Linear solver type
enum class SolverType { JACOBI, BICGSTAB, GMRES, CR, CG, SAP };

/// @} fsi_physics

}  // namespace fsi
}  // namespace chrono

#endif
