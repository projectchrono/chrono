// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// Mechanism for testing tires over granular terrain.  The mechanism + tire
// system is co-simulated with a Chrono::Parallel system for the granular terrain.
//
// Global settings.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "Settings.h"


// Value of gravitational acceleration (Z direction), common on both systems
double gacc = -9.81;

// Specify whether or not contact coefficients are based on material properties
bool use_mat_properties = false;

// Number of OpenMP threads on each MPI node
int nthreads_rignode = 2;
int nthreads_terrainnode = 2;

// Number of cosimulation steps and step size
int num_steps = 125000;
double step_size = 1e-4;

// Output directories
std::string out_dir = "../TIRE_RIG_COSIM";
std::string rig_dir = out_dir + "/RIG";
std::string terrain_dir = out_dir + "/TERRAIN";

// Output frequency (frames per second)
double output_fps = 1000;

// Checkpointing frequency (frames per second)
double checkpoint_fps = 100;

// Problem phase.  
// In SETTLING, generate checkpointing output.
// In TESTING, initialize from checkpointing files.
PhaseType phase = SETTLING;
