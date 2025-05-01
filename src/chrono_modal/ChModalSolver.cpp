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
// Authors: Alessandro Tasora
// =============================================================================

#include "ChModalSolver.h"

namespace chrono {
namespace modal {

int chrono::modal::ChModalSolver::GetNumRequestedModes() const {
    int num_modes = 0;
    for (const auto& span : m_freq_spans)
        num_modes += span.nmodes;

    return num_modes;
}

}  // namespace modal
}  // namespace chrono
