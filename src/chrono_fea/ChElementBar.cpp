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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono_fea/ChElementBar.h"

namespace chrono {
namespace fea {

ChElementBar::ChElementBar() {
    nodes.resize(2);

    area = 0.01 * 0.01;  // default area: 1cmx1cm
    density = 1000;      // default density: water
    E = 0.01e9;          // default stiffness: rubber
    rdamping = 0.01;     // default Rayleigh damping.

    length = 0;  // will be computed by Setup(), later
    mass = 0;    // will be computed by Setup(), later
}

ChElementBar::~ChElementBar() {}

}  // end namespace fea
}  // end namespace chrono
