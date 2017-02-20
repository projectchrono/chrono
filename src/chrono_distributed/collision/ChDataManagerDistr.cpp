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

#include "chrono_distributed/collision/ChDataManagerDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono/physics/ChBody.h"

#include <memory>

namespace chrono {

ChDataManagerDistr::ChDataManagerDistr(std::shared_ptr<ChSystemDistr> my_sys, int max_triangles, int max_local, int max_ghost) {
	this->my_sys = my_sys;

	num_local = 0;
	num_shared = 0;
	num_ghost = 0;
	num_total = 0;

	this->max_triangles = max_triangles;
	this->max_local = max_local;
	this->max_shared = max_ghost;
	this->max_ghost = max_ghost;
}

ChDataManagerDistr::~ChDataManagerDistr() {}

// Copies memory for a new local body
void ChDataManagerDistr::AddLocal(std::shared_ptr<ChBody> body)
{
	local_bodylist.push_back(body);
	num_local++;
}

} /* namespace chrono */
