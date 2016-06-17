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

#ifndef TESTRIG_SETTINGS_H
#define TESTRIG_SETTINGS_H

#include <fstream>
#include <string>

#define RIG_NODE_RANK 0
#define TERRAIN_NODE_RANK 1

// Value of gravitational acceleration (Z direction), common on both systems
extern double gacc;

// Specify whether or not contact coefficients are based on material properties
extern bool use_mat_properties;

// Cosimulation step size
extern double step_size;

// Output directories
extern std::string out_dir;
extern std::string rig_dir;
extern std::string terrain_dir;

// Output frequency (frames per second)
extern double output_fps;

// Checkpointing frequency (frames per second)
extern double checkpoint_fps;

// Problem phase.  
// In SETTLING, generate checkpointing output.
// In TESTING, initialize from checkpointing files.
enum PhaseType {SETTLING, TESTING};

#endif
