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
#include <memory>

#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/physics/ChDomainDistrLong.h"
#include "chrono/physics/ChBody.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

#include "chrono/core/ChVector.h"

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
			subhi[i] = boxhi[i];
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

bool ChDomainDistrLong::InSub(std::shared_ptr<ChBody> body)
{
	for (int i = 0; i < 3; i++)
	{
		if (body->GetPos()[i] < sublo[i] || body->GetPos()[i] >= subhi[i])
		{
			return false;
		}
	}
	return true;
}

// Returns true only if the body is not in this subdomain,
// but is within the ghost thickness of the subdomain
bool ChDomainDistrLong::InGhost(std::shared_ptr<ChBody> body)
{
	double pos = body->GetPos()[long_axis];

	// Returns true if:
		// Above the subdomain and within the ghost skin and
		// not out of the bounding box
		// OR
		// Below the subdomain and within the ghost skin and
		// not out of the bounding box
	return (pos > subhi[long_axis] &&
			pos <= subhi[long_axis] + my_sys->ghost_layer &&
			my_sys->my_rank != my_sys->num_ranks)
			||
			(pos < sublo[long_axis] &&
			pos >= sublo[long_axis] - my_sys->ghost_layer &&
			my_sys->my_rank != 0);
}

std::forward_list<int>::iterator ChDomainDistrLong::GetNeighItr()
{
	return neigh_ranks.begin();
}

void ChDomainDistrLong::PrintDomain()
{
	GetLog() << "Domain:\n"
			"Box:\n"
				"\tX: " << boxlo.x() << " to " << boxhi.x() << "\n"
				"\tY: " << boxlo.y() << " to " << boxhi.y() << "\n"
				"\tZ: " << boxlo.z() << " to " << boxhi.z() << "\n"
			"Subdomain: Rank " << my_sys->GetMyRank() << "\n"
				"\tX: " << sublo.x() << " to " << subhi.x() << "\n"
				"\tY: " << sublo.y() << " to " << subhi.y() << "\n"
				"\tZ: " << sublo.z() << " to " << subhi.z() << "\n";
}

} /* namespace chrono */
