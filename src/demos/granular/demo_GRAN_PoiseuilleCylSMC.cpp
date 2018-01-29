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
// The model simulated here represents a 3D Poiseuille flow in a cylindrical pipe.
//
// The pipe is horizontal. Flow is left to right. X-axis is along cylinder axis, left to right.
// The global reference frame has Z up.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <iostream>
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono_granular/physics/ChSettingsGranularSMC.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
// #define VISUALIZE 1

using namespace chrono;

double time_step = 1e-5;
double time_end = 6.;
std::string output_prefix = "poiseuilleCyl";

// Cylinder radius
float r1 = 1;

// Material properties
float Y = 2e5f;
float wallY = 1e7f;

double ball_radius = 0.1;

// -----------------------------------------------------------------------------
// FOR ALL PURPOSES, THIS IS A STUB.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    ChSystemSMC my_system;
    // Create the falling balls
    float ball_epsilon = .005;  // Margine between balls to ensure no overlap / DEM-splosion
    // Add epsilon
    utils::HCPSampler<float> sampler((2 + ball_epsilon) * ball_radius);

    // 2-wide box centered at origin
    ChVector<float> parCenter(0, 0, 0);
    ChVector<float> hdims{1, 1, 1};
    // This is a disgustingly-aliased std::vector<ChVector<float>>
    std::vector<ChVector<float>> points = sampler.SampleBox(parCenter, hdims);  // Vector of points
    for (unsigned int i = 0; i < points.size(); i++) {
        // safe to use index operator since we know we'll be in bounds
        // printf("Point %d is at (%f, %f, %f)\n", i, points[i].x(), points[i].y(), points[i].z());
        std::shared_ptr<ChBody> ball(my_system.NewBody());
        ball->SetPos(points[i]);

        // ball->SetBodyFixed(true);
        ball->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(ball.get(), ball_radius);
        ball->GetCollisionModel()->BuildModel();
        my_system.AddBody(ball);
    }
    my_system.Set_G_acc({0, 0, -9.8});

#if defined(CHRONO_OPENGL)
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "ballsNSC", &my_system);
    gl_window.SetCamera(ChVector<>(0, -6, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);
    while (true) {
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
        } else {
            break;
        }
    }
#endif
    return 0;
}
