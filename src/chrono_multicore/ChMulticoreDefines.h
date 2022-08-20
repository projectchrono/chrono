// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: lots of useful definitions for thrust, includes and enums
//
// =============================================================================

#pragma once

#include "chrono_multicore/ChApiMulticore.h"

#ifdef __CDT_PARSER__
    #define BLAZE_SERIAL_SECTION
    #define CH_MULTICORE_API
    #define custom_vector std::vector
#else
    #define custom_vector std::vector
#endif

namespace chrono {

/// @addtogroup multicore_module
/// @{

/// Iterative solver type.
enum class SolverType {
    STEEPEST_DESCENT,            ///< steepest descent
    GRADIENT_DESCENT,            ///< gradient descent
    CONJUGATE_GRADIENT,          ///< conjugate gradient
    CONJUGATE_GRADIENT_SQUARED,  ///< conjugate gradient squared
    BICONJUGATE_GRADIENT,        ///< BiCG
    BICONJUGATE_GRADIENT_STAB,   ///< BiCGStab
    MINIMUM_RESIDUAL,            ///< MINRES (minimum residual)
    QUASI_MINIMUM_RESIDUAL,      ///< Quasi MINRES
    APGD,                        ///< Accelerated Projected Gradient Descent
    APGDREF,                     ///< reference implementation for APGD
    JACOBI,                      ///< Jacobi
    GAUSS_SEIDEL,                ///< Gauss-Seidel
    PDIP,                        ///< Primal-Dual Interior Point
    BB,                          ///< Barzilai-Borwein
    SPGQP                        ///< Spectral Projected Gradient (QP projection)
};

/// Enumeration for solver mode.
enum class SolverMode {
    NORMAL,    ///< solve only normal contact impulses
    SLIDING,   ///< solve for contact and sliding friction impulses
    SPINNING,  ///< solve for rolling resistance impulses
    BILATERAL  ///< solve for bilateral Lagrange multipliers
};

/// Enumeration for system type.
/// Used so that parts of the code that have been "flattened" can know what type of system is used.
enum class SystemType {
    SYSTEM_NSC,  ///< system using non-smooth (complementarity) contact
    SYSTEM_SMC   ///< system using smooth (penalty) contact
};

/// Enumeration for bilateral constraint types.
enum BilateralType {
    BODY_BODY,          ///< constraints between two rigid bodies
    SHAFT_SHAFT,        ///< constraints between two 1-D shaft elements
    SHAFT_SHAFT_SHAFT,  ///< constraints involving 3 1-D shaft elements
    SHAFT_BODY,         ///< constraints between a shaft and a rigid body
    SHAFT_SHAFT_BODY,   ///< constraints involving two shafts and one rigid body
    UNKNOWN             ///< unknow constraint type
};

/// Supported Logging Levels.
enum class LoggingLevel {
    LOG_NONE,     ///< no logging
    LOG_INFO,     ///< info
    LOG_TRACE,    ///< tracing
    LOG_WARNING,  ///< warnings
    LOG_ERROR     ///< errors
};

/// @} multicore_module

}  // end namespace chrono
