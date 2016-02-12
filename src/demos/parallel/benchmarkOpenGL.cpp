// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel test program for OpenGL code.
//
// A Random Set of Geometries in Space
// The global reference frame has Z up.
// =============================================================================

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;
using namespace geometry;

// Particle generator
utils::Generator* gen;

// -----------------------------------------------------------------------------
// Create a mixture of geometries
// -----------------------------------------------------------------------------
void AddMixture(ChSystemParallelDVI* sys) {
  int Id_g = 1;

  gen = new utils::Generator(sys);
  std::shared_ptr<utils::MixtureIngredient>& m2 = gen->AddMixtureIngredient(utils::BOX, 1.0);
  m2->setDefaultSize(ChVector<>(1, .5, 1));
  gen->setBodyIdentifier(Id_g);
  gen->createObjectsCylinderX(utils::REGULAR_GRID, 2, ChVector<>(0, 0, 0), 20, 20, ChVector<>(0, 0, 0));
}

// -----------------------------------------------------------------------------
// Create the system
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
  // Create system
  // -------------
  ChSystemParallelDVI msystem;
  // Create the fixed bodies
  // ----------------------------------
  AddMixture(&msystem);
  msystem.DoFullAssembly();
  // Render everything
  // ----------------------------------
  opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
  gl_window.Initialize(1280, 720, "benchmarkOpenGL", &msystem);
  gl_window.SetCamera(ChVector<>(0, -100, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
  while (gl_window.Active()) {
    gl_window.Render();
  }

  return 0;
}
