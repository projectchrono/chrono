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

#include "chrono/functions/ChFunctionPosition.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChFunctionPosition) // NO! this is an abstract class, rather use for children concrete classes.

static const double FD_PERTURBATION = 1e-7;

ChVector3d ChFunctionPosition::Get_p_ds(double s) const {
    return ((Get_p(s + FD_PERTURBATION) - Get_p(s)) / FD_PERTURBATION);
}
ChVector3d ChFunctionPosition::Get_p_dsds(double s) const {
    return ((Get_p_ds(s + FD_PERTURBATION) - Get_p_ds(s)) / FD_PERTURBATION);
};

void ChFunctionPosition::Estimate_boundingbox(ChVector3d& pmin, ChVector3d& pmax) const {
    pmin.x() = pmin.y() = pmin.z() = 1e20;
    pmax.x() = pmax.y() = pmax.z() = -1e20;
    double smin, smax;
	this->Estimate_s_domain(smin, smax);
    for (double ms = smin; ms < smax; ms += (smax - smin) / 100.0) {
		ChVector3d mp = this->Get_p(ms);
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


void ChFunctionPosition::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionPosition>();
}

void ChFunctionPosition::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChFunctionPosition>();
}



}  // end namespace chrono