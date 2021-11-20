// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CH_FSI_DEFINES_H
#define CH_FSI_DEFINES_H

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Output mode
enum class CHFSI_OUTPUT_MODE { CSV, CHPF, NONE };

/// Time integration methods
enum class CHFSI_TIME_INTEGRATOR { ExplicitSPH, IISPH, I2SPH };

/// Linear solver type
enum class CHFSI_SOLVER_TYPE { JACOBI, BICGSSTAB, GMRES, CR, CG, SAP };

/// @} fsi_physics

} // namespace fsi
} // namespace chrono

#endif
