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

#ifndef M113_DRIVELINE_H
#define M113_DRIVELINE_H

#include "chrono_vehicle/tracked_vehicle/ChTrackDriveline.h"

namespace m113 {

class M113_Driveline : public chrono::vehicle::ChTrackDriveline {
  public:
    M113_Driveline();
    ~M113_Driveline() {}

    virtual void Initialize(
        chrono::ChSharedPtr<chrono::ChBody> chassis,                       ///< handle to the chassis body
        chrono::ChSharedPtr<chrono::vehicle::ChTrackAssembly> track_left,  ///< handle to the left track assembly
        chrono::ChSharedPtr<chrono::vehicle::ChTrackAssembly> track_right  ///< handle to the right track assembly
        ) override;
};

}  // end namespace m113

#endif
