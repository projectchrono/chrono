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

#include "chrono_distributed/collision/ChBroadphaseDistrBasic.h"
#include "chrono_distributed/collision/ChBroadphaseDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"

#include <cmath>
#include <memory>

#include <thread>
#include <chrono>

namespace chrono {

ChBroadphaseDistrBasic::ChBroadphaseDistrBasic(std::shared_ptr<ChSystemDistr> my_sys, double bin_side) : ChBroadphaseDistr(my_sys)
{
	// The bin length must be large enough to include the ghost layer
	if (bin_side <= my_sys->GetGhostLayer())
	{
		my_sys->ErrorAbort("The bin side must be longer than the ghost layer.\n");
	}

	// Local copies of domain information
	double ghost_layer = my_sys->GetGhostLayer();
	ChVector<double> sublo(my_sys->GetDomain()->GetSubLo());
	ChVector<double> subhi(my_sys->GetDomain()->GetSubHi());

	// Sets the number cubic bins in each direction x=0, y=1, z=2
	for (int i = 0; i < 3; i++)
	{
		bin_dims(i) = std::ceil((subhi(i) - sublo(i) + 2 * ghost_layer)
				/ bin_side);
	}

	num_bins = bin_dims.x * bin_dims.y * bin_dims.z;

	// Allocates arrays for each bin
	bins = new std::vector<std::shared_ptr<ChBodyDistr>>** [bin_dims.x];

	for (int i = 0; i < bin_dims.x; i++)
	{
		bins[i] = new std::vector<std::shared_ptr<ChBodyDistr>>* [bin_dims.y];
		for (int j = 0; j < bin_dims.y; j++)
		{
			bins[i][j] = new std::vector<std::shared_ptr<ChBodyDistr>> [bin_dims.z];\
		}
	}

	bin_edge = new double* [3];

	for (int i = 0; i < 3; i++)
	{
		bin_edge[i] = new double[bin_dims(i) + 1];
	}

	// Sets boundaries of the bins.
	for (int i = 0; i < 3; i++)
	{
		double x = sublo(i) - ghost_layer;
		for (int j = 0; j < bin_dims(i) + 1; j++)
		{
			bin_edge[i][j] = x;
			x += bin_side;
		}
	}
}

ChBroadphaseDistrBasic::~ChBroadphaseDistrBasic() {}

// Divide the bodies into orthogonal bins
// Note: This goes through ALL bodies on this rank, and should only be used when necessary //TODO
void ChBroadphaseDistrBasic::DetectPossibleCollisions()
{
	ChVector<int> bin_index(0, 0, 0); // Indices of the bin for each body

	// Loop over all local bodies in this subdomain
	for (std::shared_ptr<ChBody> body : data->GetLocalList())
	{
		bin_index.SetNull();
		for (int j = 0; j < 3; j++)
		{
			while (bin_edge[j][bin_index(j)] < body->GetPos()(j))
			{
				bin_index(j)++;
			}
			bin_index(j)--;
		}

		// Adds the body to the bin
		bins[bin_index.x][bin_index.y][bin_index.z].push_back(body);
	}

	// Sort Shared Bodies
	for (std::shared_ptr<ChBodyDistr> body : data->GetSharedList())
	{
		bin_index.SetNull();
		for (int j = 0; j < 3; j++)
		{
			while (bin_edge[j][bin_index(j)] < body->GetPos()(j))
			{
				bin_index(j)++;
			}
			bin_index(j)--;
		}

		bins[bin_index.x][bin_index.y][bin_index.z].push_back(body);
	}

	// Sort Ghost bodies
	for (std::shared_ptr<ChBodyDistr> body : data->GetGhostList())
	{
		bin_index.SetNull();

		for (int j = 0; j < 3; j++)
		{
			while (bin_edge[j][bin_index(j)] < body->GetPos()(j))
			{
				bin_index(j)++;
			}
			bin_index(j)--;
		}

		bins[bin_index.x][bin_index.y][bin_index.z].push_back(body);
	}
}

void ChBroadphaseDistrBasic::PrintBins()
{
	GetLog() << "Bins: " << bin_dims.z << " X " << bin_dims.y << " X " << bin_dims.z << "\n";
	for (int i = 0; i < bin_dims.x; i++)
	{
		for (int j = 0; j < bin_dims.y; j++)
		{
			for (int k = 0; k < bin_dims.z; k++)
			{
				GetLog() << "Bin [" << i << "][" << j << "][" << k << "]: ";
				for (std::shared_ptr<ChBodyDistr> body : bins[i][j][k])
				{
					GetLog() << body->GetGloablId() << ", ";
				}
				GetLog() << "\n";
			}
		}
	}

	GetLog() << "\n";
}

} /* namespace chrono */
