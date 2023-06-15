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

#include <memory.h>
#include <cfloat>
#include <cmath>

#include "chrono/motion_functions/ChFunctionPosition.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChFunctionPosition) // NO! this is an abstract class, rather use for children concrete classes.

void ChFunctionPosition::Estimate_boundingbox(ChVector<>& pmin, ChVector<>& pmax) const {
    pmin.x() = pmin.y() = pmin.z() =  1e20;
    pmax.x() = pmax.y() = pmax.z() = -1e20;
	double smin, smax;
	this->Estimate_s_domain(smin, smax);
    for (double ms = smin; ms < smax; ms += (smax - smin) / 100.0) {
		ChVector<> mp = this->Get_p(ms);
        if (mp.x() < pmin.x())
            pmin.x() = mp.x();
		if (mp.y() < pmin.y())
            pmin.y() = mp.y();
		if (mp.z() < pmin.z())
            pmin.z() = mp.z();
		if (mp.x() > pmax.x())
            pmax.x() = mp.x();
		if (mp.y() > pmax.y())
            pmax.y() = mp.y();
		if (mp.z() > pmax.z())
            pmax.z() = mp.z();
    }
}


void ChFunctionPosition::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionPosition>();
}

void ChFunctionPosition::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionPosition>();
}



}  // end namespace chrono