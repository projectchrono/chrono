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
// Authors: Hammad Mazhar
// =============================================================================
//
// Chrono::Multicore test program for OpenGL code.
//
// A Random Set of Geometries in Space
// The global reference frame has Z up.
// =============================================================================

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

using namespace chrono;
using namespace geometry;

// -----------------------------------------------------------------------------
// Create a mixture of geometries
// -----------------------------------------------------------------------------
void AddMixture(ChSystem* sys) {
    utils::GridSampler<double> sampler(2);
    utils::Generator gen(sys);
    std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::MixtureType::BOX, 0.3);
    std::shared_ptr<utils::MixtureIngredient> m2 = gen.AddMixtureIngredient(utils::MixtureType::SPHERE, 0.4);
    std::shared_ptr<utils::MixtureIngredient> m3 = gen.AddMixtureIngredient(utils::MixtureType::CYLINDER, 0.3);
    m1->setDefaultSize(ChVector<>(1, .5, 0.7));
    m2->setDefaultSize(ChVector<>(.5, .5, .5));
    m3->setDefaultSize(ChVector<>(1, .5, 1));
    gen.CreateObjectsCylinderX(sampler, ChVector<>(0, 0, 0), 20, 20, ChVector<>(0, 0, 0));
}

// -----------------------------------------------------------------------------
// Create the system
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;

    AddMixture(&sys);

    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("benchmarkOpenGL");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::SOLID);
    vis.Initialize();
    vis.SetCameraPosition(ChVector<>(-50, -50, 0), ChVector<>(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    
    std::function<void()> step_iter = [&]() {
        vis.Render();
    };
    
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(&opengl::ChVisualSystemOpenGL::WrapRenderStep, (void*)&step_iter, 50, true);
#else
    while (vis.Run()) {
        step_iter();
    }
#endif

    return 0;
}
