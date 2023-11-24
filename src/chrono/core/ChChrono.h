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

#ifndef CHCHRONO_H
#define CHCHRONO_H

// Definition of the main module and sub-modules in the main Chrono library

/**
    @defgroup chrono Chrono::Engine
    @brief Core Functionality
    @{
        @defgroup chrono_physics Physics objects
        @defgroup chrono_geometry Geometric objects
        @defgroup chrono_collision Collision detection
        @{
           @defgroup collision_bullet Bullet collision detection
           @defgroup collision_mc Multicore collision detection
        @}
        @defgroup chrono_fea Finite Element Analysis
        @{
           @defgroup fea_nodes Nodes
           @defgroup fea_elements Elements
           @defgroup fea_constraints Constraints
           @defgroup fea_contact Contact
           @defgroup fea_math Mathematical support
           @defgroup fea_utils Utility classes
        @}
        @defgroup chrono_assets Visual asset objects
        @defgroup chrono_linalg Linear algebra
        @defgroup chrono_solver Solver
        @defgroup chrono_timestepper Time integrators
        @defgroup chrono_functions Function objects
        @defgroup chrono_particles Particle factory
        @defgroup chrono_serialization Serialization
        @defgroup chrono_mc_math Multicore math
        @defgroup chrono_utils Utility classes
    @}
*/

/// @addtogroup chrono
/// @{

/// Main namespace for the Chrono package.
namespace chrono {
/// Namespace for FEA classes.
namespace fea {}
}

/// @} chrono

#endif
