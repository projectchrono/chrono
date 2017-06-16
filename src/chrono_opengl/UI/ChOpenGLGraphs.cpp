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
// Class to render simple plots for the UI
// =============================================================================

#include "chrono_opengl/UI/ChOpenGLGraphs.h"

namespace chrono {
namespace opengl {

using namespace glm;

ChOpenGLGraphs::ChOpenGLGraphs() {}

bool ChOpenGLGraphs::Initialize(ChOpenGLMaterial mat, ChOpenGLShader* _shader) {
    plot_data.push_back(vec3(0, 0, 0));
    plot_data.push_back(vec3(0, 0, 0));

    plots.Initialize(plot_data, mat, _shader);

    return true;
}
void ChOpenGLGraphs::Update(ChSystem* physics_system, const ivec2& window_size) {
    //  plot_data.clear();
    //  plot_data.push_back(glm::vec3(window_size.y * .1, window_size.y - window_size.y * .4, 0));
    //  plot_data.push_back(glm::vec3(window_size.y * .1, window_size.y - window_size.y * .1, 0));
    //
    //  plot_data.push_back(glm::vec3(window_size.y * .1, window_size.y - window_size.y * .4, 0));
    //  plot_data.push_back(glm::vec3(window_size.y * .6, window_size.y - window_size.y * .4, 0));
    //
    //  std::vector<double> history = std::static_pointer_cast<ChIterativeSolver>(physics_system->GetSolver())->GetViolationHistory();
    //  std::vector<double> dlambda = std::static_pointer_cast<ChIterativeSolver>(physics_system->GetSolver())->GetDeltalambdaHistory();
    //
    //  real plot_h = (window_size.y * .4 - window_size.y * .1);
    //  if (history.size() > 1) {
    //    real max_res = *std::max_element(history.begin(), history.end());
    //    real min_res = *std::min_element(history.begin(), history.end());
    //    max_res = max_res + min_res;
    //
    //    for (int i = 0; i < history.size() - 1; i++) {
    //      real value = (history[i] + min_res) / max_res * plot_h;
    //      real value_next = (history[i + 1] + min_res) / max_res * plot_h;
    //
    //      real size_seg = (window_size.y * .6 - window_size.y * .1) / history.size();
    //
    //      plot_data.push_back(glm::vec3(window_size.y * .1 + size_seg * i, window_size.y - window_size.y * .4 + value,
    //      0));
    //      plot_data.push_back(
    //          glm::vec3(window_size.y * .1 + size_seg * (i + 1), window_size.y - window_size.y * .4 + value_next, 0));
    //    }
    //  }
    //  plots.Update(plot_data);
}

void ChOpenGLGraphs::TakeDown() {
    plots.TakeDown();
}

void ChOpenGLGraphs::Draw(const mat4& projection, const mat4& modelview) {
    plots.Draw(projection, modelview);
}
}
}
