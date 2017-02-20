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

#ifndef CHRONO_DISTRIBUTED_COLLISION_CHBROADPHASEDISTRBASIC_H_
#define CHRONO_DISTRIBUTED_COLLISION_CHBROADPHASEDISTRBASIC_H_

#include "chrono_distributed/collision/ChBroadphaseDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"

#include <memory>

namespace chrono {

class ChBroadphaseDistrBasic : public ChBroadphaseDistr {
public:
	ChBroadphaseDistrBasic(std::shared_ptr<ChSystemDistr> my_sys, double bin_size);
	virtual ~ChBroadphaseDistrBasic();
	void DetectPossibleCollisions();

	int GetBinDims(int i) {return bin_dims(i);}
	int GetNumBins() {return num_bins;}
	double GetBinEdge(int xyz, int i)
	{
		if (!(xyz == 0 || xyz == 1 || xyz == 2) && (i >= 0 && i < bin_dims(xyz))) return bin_edge[xyz][i];
		else return -1;
	}

	void PrintBins();

protected:
	// Bins
	ChVector<int> bin_dims; // Number of bins along each axis of this subdomain
	int num_bins; // Total number of bins in this subdomain.

	// Borders of bins
	double **bin_edge;

	std::vector<std::shared_ptr<ChBodyDistr>> ***bins; // bin[x][y][z] is a vector of bodies ptrs in the bin
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_COLLISION_CHBROADPHASEDISTRBASIC_H_ */
