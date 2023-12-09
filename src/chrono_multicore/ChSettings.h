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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This file contains all of the setting structures that are used
// within Chrono::Multicore. The constructor for each struct should specify
// default values for every setting parameter that is used by default
//
// =============================================================================

#pragma once

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChOpenMP.h"
#include "chrono/collision/multicore/ChBroadphase.h"
#include "chrono/collision/multicore/ChNarrowphase.h"
#include "chrono_multicore/ChMulticoreDefines.h"
#include "chrono/multicore_math/ChMulticoreMath.h"

namespace chrono {

/// @addtogroup multicore_module
/// @{

/// Chrono::Multicore collision_settings.
/// This structure that contains all settings associated with the collision detection phase.
class collision_settings {
  public:
    collision_settings()
        : use_aabb_active(false),
          collision_envelope(0),
          bins_per_axis(vec3(10, 10, 10)),
          bin_size(real3(1, 1, 1)),
          grid_density(5),
          broadphase_grid(ChBroadphase::GridType::FIXED_RESOLUTION),
          narrowphase_algorithm(ChNarrowphase::Algorithm::HYBRID) {}

    /// For stability of NSC contact, the envelope should be set to 5-10% of the smallest collision shape size (too
    /// large a value will slow down the narrowphase collision detection). The envelope is the amount by which each
    /// collision shape is inflated prior to performing the collision detection, in order to create contact constraints
    /// before shapes actually come in contact. This collision detection system uses a global envelope, used for all
    /// rigid shapes in the system.
    real collision_envelope;

    /// Flag controlling the monitoring of shapes outside active bounding box.
    /// If enabled, objects whose collision shapes exit the active bounding box are deactivated (frozen).
    /// The size of the bounding box is specified by its min and max extents.
    bool use_aabb_active;

    /// Lower corner of the axis-aligned bounding box (if set to active).
    real3 aabb_min;

    /// Upper corner of the axis-aligned bounding box (if set to active).
    real3 aabb_max;

    /// Method for controlling granularity of the broadphase collision grid.
    ChBroadphase::GridType broadphase_grid;

    /// Number of bins for the broadphase collision grid. This is the default method to control the granularity of the
    /// collision detection grid used for the broadphase. During the broadphase stage the extents of the simulation are
    /// computed and then sliced according to the variable.
    vec3 bins_per_axis;

    /// Broadphase collision grid bin size. This value  is used for dynamic tuning of the number of collision bins if
    /// the `broadphase_grid` type is set to FIXED_BIN_SIZE.
    real3 bin_size;

    /// Broadphase collision grid density. This value is used for dynamic tuning of the number of collision bins if the
    /// `broadphase_grid` type is set to FIXED_DENSITY.
    real grid_density;

    /// Algorithm for narrowphase collision detection phase.
    /// The Chrono collision detection system provides several analytical collision detection algorithms, for particular
    /// pairs of shapes (see ChNarrowphasePRIMS). For general convex shapes, the collision system relies on the
    /// Minkovski Portal Refinement algorithm (see ChNarrowphaseMPR).
    ChNarrowphase::Algorithm narrowphase_algorithm;
};

/// Chrono::Multicore solver_settings.
/// This structure contains all settings associated with the Chrono::Multicore solver.
class solver_settings {
  public:
    solver_settings() {
        tolerance = 1e-4;
        tol_speed = 1e-4;
        tolerance_objective = 1e-6;
        update_rhs = false;
        test_objective = false;

        alpha = .2;
        contact_recovery_speed = .6;
        bilateral_clamp_speed = .6;
        clamp_bilaterals = true;
        compute_N = false;
        use_full_inertia_tensor = true;
        max_iteration = 100;
        max_iteration_normal = 0;
        max_iteration_sliding = 100;
        max_iteration_spinning = 0;
        max_iteration_bilateral = 100;
        solver_type = SolverType::APGD;
        solver_mode = SolverMode::SLIDING;
        local_solver_mode = SolverMode::NORMAL;

        contact_force_model = ChSystemSMC::Hertz;
        adhesion_force_model = ChSystemSMC::Constant;
        tangential_displ_mode = ChSystemSMC::OneStep;
        use_material_properties = true;
        characteristic_vel = 1;
        min_slip_vel = 1e-4;
        min_roll_vel = 1e-4;
        min_spin_vel = 1e-4;
        cache_step_length = false;
        precondition = false;
        use_power_iteration = false;
        max_power_iteration = 15;
        power_iter_tolerance = 0.1;
        skip_residual = 1;
    }

    /// The solver type variable defines name of the solver that will be used to
    /// solve the NSC problem
    SolverType solver_type;
    /// There are three possible solver modes
    /// NORMAL will only solve for the normal and bilateral constraints.
    /// SLIDING will only solve for the normal, sliding and bilateral constraints.
    /// SPINNING will solve for all of the constraints.
    /// The purpose of this settings is to allow the user to completely ignore
    /// different types of friction. In Chrono::Multicore all constraints support
    /// friction and sliding friction so this is how you can improve performance
    /// when you know that you don't need spinning/rolling friction or want to solve
    /// a problem frictionless.
    SolverMode solver_mode;

    /// This should not be set by the user, depending on how the iterations are set
    /// The variable is used to specify what type of solve is currently being done.
    SolverMode local_solver_mode;

    /// This parameter is a constant used when solving a problem with compliance.
    real alpha;
    /// The contact recovery speed parameter controls how "hard" a contact is
    /// enforced when two objects are penetrating. The larger the value is the
    /// faster the two objects will separate.
    real contact_recovery_speed;
    /// This parameter is the same as the one for contacts, it controls how fast two
    /// objects will move in order to resolve constraint drift.
    real bilateral_clamp_speed;
    /// It is possible to disable clamping for bilaterals entirely. When set to true
    /// bilateral_clamp_speed is ignored.
    bool clamp_bilaterals;
    /// Experimental options that probably don't work for all solvers.
    bool update_rhs;
    bool compute_N;
    bool test_objective;
    bool use_full_inertia_tensor;
    bool cache_step_length;
    bool precondition;
    bool use_power_iteration;
    int max_power_iteration;
    real power_iter_tolerance;

    /// Contact force model for SMC.
    ChSystemSMC::ContactForceModel contact_force_model;
    /// Contact force model for SMC.
    ChSystemSMC::AdhesionForceModel adhesion_force_model;
    /// Tangential contact displacement history. None indicates no tangential stiffness,
    /// OneStep indicates estimating tangential displacement using only current velocity,
    /// MultiStep uses full contact history over multiple steps.
    ChSystemSMC::TangentialDisplacementModel tangential_displ_mode;
    /// Flag specifying how the stiffness and damping coefficients in the SMC contact
    /// force models are calculated. If true, these coefficients are derived from
    /// physical material properties. Otherwise, the user specifies the coefficients
    /// directly.
    bool use_material_properties;
    /// Characteristic velocity (Hooke contact force model).
    real characteristic_vel;
    /// Threshold tangential velocity.
    real min_slip_vel;
    real min_roll_vel;
    real min_spin_vel;

    /// Along with setting the solver mode, the total number of iterations for each
    /// type of constraints can be performed.
    uint max_iteration;
    /// If the normal iterations are set, iterations are performed for just the
    /// normal part of the constraints. This will essentially precondition the
    /// solver and stiffen the contacts, making objects penetrate less. For visual
    /// accuracy this is really useful in my opinion. Bilaterals are still solved.
    uint max_iteration_normal;
    /// Similarly sliding iterations are only performed on the sliding constraints.
    /// Bilaterals are still solved.
    uint max_iteration_sliding;
    /// Similarly spinning iterations are only performed on the spinning constraints
    /// Bilaterals are still solved.
    uint max_iteration_spinning;
    uint max_iteration_bilateral;

    /// This variable is the tolerance for the solver in terms of speeds.
    real tolerance;
    real tol_speed;
    /// This variable defines the tolerance if the solver is using the objective.
    /// termination condition
    real tolerance_objective;
    /// Compute residual every x iterations.
    int skip_residual;
};

/// Aggregate of all settings for Chrono::Multicore.
class settings_container {
  public:
    settings_container() {
        min_threads = 1;
#ifdef _OPENMP
        max_threads = omp_get_num_procs();
#else
        max_threads = 1;
#endif
        perform_thread_tuning = false;
        system_type = SystemType::SYSTEM_NSC;
        step_size = 0.01;
    }

    collision_settings collision;  ///< settings for collision detection
    solver_settings solver;        ///< settings for iterative solver

    real step_size;  ///< current integration step size
    real3 gravity;   ///< gravitational acceleration vector

  private:
    bool perform_thread_tuning;  ///< dynamically tune number of threads
    int min_threads;             ///< lower bound for number of threads (if dynamic tuning)
    int max_threads;             ///< maximum bound for number of threads (if dynamic tuning)
    SystemType system_type;      ///< system type (NSC or SMC)

    friend class ChSystemMulticore;
    friend class ChSystemMulticoreNSC;
    friend class ChSystemMulticoreSMC;
};

/// @} multicore_module

}  // end namespace chrono
