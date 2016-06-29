//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChAssetLevel.cpp
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "assets/ChAssetLevel.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChAssetLevel> a_registration_ChAssetLevel;


void ChAssetLevel::Update(ChPhysicsItem* updater, const ChCoordsys<>& coords) {

    // Concatenate the total transformation
    ChCoordsys<> composed_coords(this->levelframe.coord >> coords);

    for (unsigned int ia = 0; ia < this->assets.size(); ++ia)
        assets[ia]->Update(updater, composed_coords);
}


}  // END_OF_NAMESPACE____
