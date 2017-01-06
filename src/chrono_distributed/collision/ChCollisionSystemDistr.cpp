// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include "chrono_distributed/collision/ChCollisionSystemDistr.h"
#include "chrono_distributed/collision/ChBroadphaseDistrBasic.h"
#include "chrono_distributed/collision/ChNarrowphaseDistrBasic.h"

namespace chrono {

ChCollisionSystemDistr::ChCollisionSystemDistr(ChSystemDistr *my_sys) {
	this->my_sys = my_sys;
	broadphase = NULL;
	narrowphase = NULL;
}

ChCollisionSystemDistr::~ChCollisionSystemDistr() {}

void ChCollisionSystemDistr::SetBroadphase(ChBroadphaseDistr *bp)
{
	if (broadphase) my_sys->ErrorAbort("Cannot set the broadphase implementation more than once.");
	broadphase = bp;
}

void ChCollisionSystemDistr::SetNarrowphase(ChNarrowphaseDistr *np)
{
	if (narrowphase) my_sys->ErrorAbort("Cannot set the narrowphase implemenation more than once.");
	narrowphase = np;
}

} /* namespace chrono */
