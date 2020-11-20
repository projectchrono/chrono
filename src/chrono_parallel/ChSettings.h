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
// within chrono parallel. The constructor for each struct should specify
// default values for every setting parameter that is used by default
//
// =============================================================================

#pragma once

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/parallel/ChOpenMP.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/other_types.h"

namespace chrono {

/// @addtogroup parallel_module
/// @{

/// Chrono::Parallel collision_settings.
/// This structure that contains all settings associated with the collision detection phase.
class collision_settings {
  public:
    /// The default values are specified in the constructor, use these as
    /// guidelines when experimenting with your own simulation setup.
    collision_settings() {
        // by default the bounding box is not active and the default values for the
        // bounding box size are not specified.
        use_aabb_active = false;
        // I chose to set the default envelope because there is no good default
        // value (in my opinion, feel free to disagree!) I suggest that the envelope
        // be set to 5-10 percent of the smallest object radius. Using too large of
        // a value will slow the collision detection down.
        collision_envelope = 0;
        // The number of slices in each direction will greatly effect the speed at
        // which the collision detection operates. I would suggest that on average
        // the number of objects in a bin/grid cell should not exceed 100.
        // NOTE!!! this really depends on the architecture that you run on and how
        // many cores you are using.
        bins_per_axis = vec3(20, 20, 20);
        narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
        grid_density = 5;
        fixed_bins = true;
    }

    real3 min_bounding_point, max_bounding_point;

    /// This parameter, similar to the one in chrono inflates each collision shape
    /// by a certain amount. This is necessary when using NSC as it creates the
    /// contact constraints before objects acutally come into contact. In general
    /// this helps with stability.
    real collision_envelope;
    /// Chrono parallel has an optional feature that allows the user to set a
    /// bounding box that automatically freezes (makes inactive) any object that
    /// exits the bounding box.
    bool use_aabb_active;
    /// The size of the bounding box (if set to active) is specified by its min and max extents.
    real3 aabb_min;
    /// The size of the bounding box (if set to active) is specified by its min and max extents.
    real3 aabb_max;
    /// This variable is the primary method to control the granularity of the
    /// collision detection grid used for the broadphase.
    /// As the name suggests, it is the number of slices along each axis. During
    /// the broadphase stage the extents of the simulation are computed and then
    /// sliced according to the variable.
    vec3 bins_per_axis;
    /// There are multiple narrowphase algorithms implemented in the collision
    /// detection code. The narrowphase_algorithm parameter can be used to change
    /// the type of narrowphase used at runtime.
    NarrowPhaseType narrowphase_algorithm;
    real grid_density;
    /// Use fixed number of bins instead of tuning them.
    bool fixed_bins;
};

/// Chrono::Parallel solver_settings.
/// This structure contains all settings associated with the parallel solver.
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
        max_iteration_fem = 0;
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
    /// different types of friction. In chrono parallel all constraints support
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
    uint max_iteration_fem;

    /// This variable is the tolerance for the solver in terms of speeds.
    real tolerance;
    real tol_speed;
    /// This variable defines the tolerance if the solver is using the objective.
    /// termination condition
    real tolerance_objective;
    /// Compute residual every x iterations.
    int skip_residual;
};

/// Aggregate of all settings for Chrono::Parallel.
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

    friend class ChSystemParallel;
    friend class ChSystemParallelNSC;
    friend class ChSystemParallelSMC;
};

/// @} parallel_module

}  // end namespace chrono
