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
// Authors: Radu Serban
// =============================================================================
//
// M113 track assembly subsystem.
//
// =============================================================================

#include "m113/M113_Sprocket.h"
#include "m113/M113_Idler.h"
#include "m113/M113_RoadWheel.h"
#include "m113/M113_Suspension.h"
#include "m113/M113_TrackAssembly.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_TrackAssembly::M113_TrackAssembly(VisualizationType vis_type) : ChTrackAssembly("M113_TrackAssembly") {
    m_sprocket = ChSharedPtr<M113_Sprocket>(new M113_Sprocket(vis_type));
    m_idler = ChSharedPtr<M113_Idler>(new M113_Idler(vis_type));

    for (size_t is = 0; is < 5; is++) {
        m_suspensions.push_back(ChSharedPtr<M113_Suspension>(new M113_Suspension(vis_type)));
    }
}

}  // end namespace m113
