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
// Authors: Dan Negrut
// =============================================================================
//
// Chrono::Granular demo program using SMC method for frictional contact.
//
// Basic simulation of a settling scenario; 
//  - box is rectangular 
//  - there is no friction
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include<iostream>
#include "chrono/core/ChTimer.h"
#include "chrono_granular/physics/ChGranular.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
// #define VISUALIZE 1

using namespace chrono;

// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// There is no friction.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    double time_step = 1e-5;
    double time_end = 6.;
    std::string output_prefix = "settling_MONODISP_SPHERES_SMC";

    ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC experiment(1.f, 80000);
    experiment.setBOXdims(20.f, 20.f, 30.f);
    experiment.YoungModulus_SPH2SPH (  200000.f);
    experiment.YoungModulus_SPH2WALL(10000000.f);
    experiment.settle(10.f);

    return 0;
}
