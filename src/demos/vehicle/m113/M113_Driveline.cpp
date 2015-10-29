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
// Placeholder for an M113 driveline.
//
// =============================================================================

#include "m113/M113_Driveline.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Driveline::M113_Driveline() : chrono::vehicle::ChTrackDriveline("M113_Driveline") {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Driveline::Initialize(ChSharedPtr<ChBody> chassis,
                                ChSharedPtr<ChTrackAssembly> track_left,
                                ChSharedPtr<ChTrackAssembly> track_right) {
    // Create the driveshaft, a 1 d.o.f. object with rotational inertia which
    // represents the connection of the driveline to the transmission box.
    m_driveshaft = ChSharedPtr<ChShaft>(new ChShaft);
    m_driveshaft->SetInertia(1.0);
    chassis->GetSystem()->Add(m_driveshaft);
}

}  // end namespace hmmwv
