//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//
//   ChCModelBulletBody.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCModelBulletBody.h"
#include "physics/ChBody.h"
#include "collision/bullet/btBulletCollisionCommon.h"

namespace chrono {
namespace collision {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChModelBulletBody> a_registration_ChModelBulletBody;

ChModelBulletBody::ChModelBulletBody() {
    mbody = 0;
}

ChModelBulletBody::~ChModelBulletBody() {
}



}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
