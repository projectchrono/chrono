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

#include <forward_list>

#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/physics/ChDomainDistrLong.h"
#include "chrono_distributed/physics/ChBodyDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

// Divides the domain into equal-volume, orthogonal, axis-aligned regions along
// the longest axis.
void ChDomainDistrLong::SplitDomain()
{
	// Length of this subdomain along the long axis
	double sub_len = (boxhi[long_axis] - boxlo[long_axis]) / (double) my_sys->GetRanks();

	for (int i = 0; i < 3; i++)
	{
		if (long_axis == i)
		{
			sublo[i] = boxlo[i] + my_sys->GetMyRank() * sub_len;
			subhi[i] = sublo[i] + sub_len;
		}
		else
		{
			sublo[i] = boxlo[i];
		}
	}
	
	// Creates the list of neighboring MPI ranks
	if (my_sys->GetMyRank() != 0)
	{
		neigh_ranks.push_front(my_sys->GetMyRank() - 1);
	}
	if (my_sys->GetMyRank() != (my_sys->GetRanks() - 1))
	{
		neigh_ranks.push_front(my_sys->GetMyRank() + 1);
	}
}

bool ChDomainDistrLong::HasLeft(ChBodyDistr* body)
{
	for (int i = 0; i < 3; i++)
	{
		if (body->GetPos(i) < sublo[i] || body->GetPos(i) >= subhi[i])
		{
			return true;
		}
	}
	return false;
}

typename std::forward_list<int>::iterator ChDomainDistrLong::GetNeighItr()
{
	return neigh_ranks.begin();
}

} /* namespace chrono */
