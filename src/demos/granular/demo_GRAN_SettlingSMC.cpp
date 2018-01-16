// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly
// =============================================================================
//
// Chrono::Granular demo program using SMC method for frictional contact.
//
// Basic simulation of a settling scenario; box is rectangular. 
//
// The box has square cross section.
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include<iostream>
#include "chrono/core/ChTimer.h"
#include "chrono_granular/physics/ChSettingsGranularSMC.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
// #define VISUALIZE 1

using namespace chrono;

double time_step = 1e-5;
double time_end = 6.;
std::string output_prefix = "settlingSMC";



// Material properties 
float Y = 2e5f;
float wallY = 1e7f;

double ball_radius = 0.25;


// -----------------------------------------------------------------------------
// FOR ALL PURPOSES, THIS IS A STUB.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    return 0;
}
