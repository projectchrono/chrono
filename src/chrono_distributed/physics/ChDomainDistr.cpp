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

#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

#include "chrono_parallel/ChDataManager.h"

#include "chrono/physics/ChBody.h"
#include "chrono/core/ChVector.h"

#include <iostream>
#include <stdlib.h>
#include <mpi.h>
#include <memory>

using namespace chrono;

ChDomainDistr::ChDomainDistr(ChSystemDistr *sys)
{
	this->my_sys = sys;
	long_axis = 0;
}

ChDomainDistr::~ChDomainDistr() {}

// Takes in the user specified coordinates of the bounding box for the simulation.
void ChDomainDistr::SetSimDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi)
{
	boxlo.Set(xlo, ylo, zlo);
	boxhi.Set(xhi, yhi, zhi);

	double len_x = boxhi.x() - boxlo.x();
	double len_y = boxhi.y() - boxlo.y();
	double len_z = boxhi.z() - boxlo.z();

	if (len_x <= 0 || len_y <= 0 || len_z <=0) my_sys->ErrorAbort("Invalid domain dimensions.");

	// Index of the longest domain axis 0=x, 1=y, 2=z
	long_axis = (len_x >= len_y) ? 0 : 1;
	long_axis = (len_z >= boxhi[long_axis] - boxlo[long_axis]) ? 2 : long_axis;
}

///  Divides the domain into equal-volume, orthogonal, axis-aligned regions along
/// the longest axis. Needs to be called right after the system is created so that
/// bodies are added correctly.
void ChDomainDistr::SplitDomain()
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
}

// Returns 1 if the body is in the subdomain and not shared on another rank
int ChDomainDistr::InOwned(int index)
{
	double pos = my_sys->data_manager->host_data.pos_rigid[index][long_axis];
	int my_rank = my_sys->GetMyRank();
	int ranks = my_sys->GetRanks();
	double ghost_layer = my_sys->GetGhostLayer();

	// If the position is not in the subdoman AND not within the ghost layer inside the subdomain
	if (pos >= sublo[long_axis] && pos <= subhi[long_axis])
	{
		// If there is no neighbor rank below, take ownership
		if (my_rank == 0 && pos < subhi[long_axis] - ghost_layer)
		{
			return 1;
		}
		// If there is no neighbor rank above, take ownership
		if (my_rank == ranks - 1 && pos >= sublo[long_axis] + ghost_layer)
		{
			return 1;
		}
		// If the body is not seen by other ranks
		if (pos >= sublo[long_axis] + ghost_layer && pos < subhi[long_axis] - ghost_layer)
		{
			return 1;
		}
	}

	return 0;
}

// Returns 1 if the body is in the subdomain and not shared on another rank
int ChDomainDistr::InOwned(std::shared_ptr<ChBody> body)
{
	double pos = body->GetPos()[long_axis];
	int my_rank = my_sys->GetMyRank();
	int ranks = my_sys->GetRanks();
	double ghost_layer = my_sys->GetGhostLayer();

	// If the position is not in the subdoman AND not within the ghost layer inside the subdomain
	if (pos >= sublo[long_axis] && pos <= subhi[long_axis])
	{
		// If there is no neighbor rank below, take ownership
		if (my_rank == 0 && pos < subhi[long_axis] - ghost_layer)
		{
			return 1;
		}
		// If there is no neighbor rank above, take ownership
		if (my_rank == ranks - 1 && pos >= sublo[long_axis] + ghost_layer)
		{
			return 1;
		}
		// If the body is not seen by other ranks
		if (pos >= sublo[long_axis] + ghost_layer && pos < subhi[long_axis] - ghost_layer)
		{
			return 1;
		}
	}
	return 0;
}


// Returns:
	// 1: Above the subdomain and within the ghost skin and
	// not out of the bounding box
	// 2: Below the subdomain and within the ghost skin and
	// not out of the bounding box
	// 0: Not in the ghost layer
int ChDomainDistr::InGhost(int index)
{
	double pos = my_sys->data_manager->host_data.pos_rigid[index][long_axis];
	int my_rank = my_sys->GetMyRank();
	int ranks = my_sys->GetRanks();
	double ghost_layer = my_sys->GetGhostLayer();

	if (pos >= subhi[long_axis])
	{
		// If there is no rank above to share the body, the body is ignored
		if (my_rank == ranks - 1)
		{
			return 0;
		}
		if (pos < subhi[long_axis] + ghost_layer)
		{
			return 1;
		}
	}

	else if (pos < sublo[long_axis])
	{
		// If there is no rank below to share the body, the body is ignored
		if (my_rank == 0)
		{
			return 0;
		}
		if (pos >= sublo[long_axis] - ghost_layer)
		{
			return 2;
		}
	}

	return 0;
}


// Returns:
	// 1: Above the subdomain and within the ghost skin and
	// not out of the bounding box
	// 2: Below the subdomain and within the ghost skin and
	// not out of the bounding box
	// 0: Not in the ghost layer
int ChDomainDistr::InGhost(std::shared_ptr<ChBody> body)
{
	double pos = body->GetPos()[long_axis];
	int my_rank = my_sys->GetMyRank();
	int ranks = my_sys->GetRanks();
	double ghost_layer = my_sys->GetGhostLayer();

	if (pos >= subhi[long_axis])
	{
		// If there is no rank above to share the body, the body is ignored
		if (my_rank == ranks - 1)
		{
			return 0;
		}
		if (pos < subhi[long_axis] + ghost_layer)
		{
			return 1;
		}
	}

	else if (pos < sublo[long_axis])
	{
		// If there is no rank below to share the body, the body is ignored
		if (my_rank == 0)
		{
			return 0;
		}
		if (pos >= sublo[long_axis] - ghost_layer)
		{
			return 2;
		}
	}

	return 0;
}

// Returns 1 if shared up, 2 if shared down, 0 if not shared
int ChDomainDistr::InShared(int index)
{
	double pos = my_sys->data_manager->host_data.pos_rigid[index][long_axis];
	int my_rank = my_sys->GetMyRank();
	int ranks = my_sys->GetRanks();
	double ghost_layer = my_sys->GetGhostLayer();

	if (pos >= subhi[long_axis] - ghost_layer && pos < subhi[long_axis] && my_rank != ranks)
	{
		return 1;
	}
	if (pos < sublo[long_axis] + ghost_layer && pos >= sublo[long_axis] && my_rank != 0)
	{
		return 2;
	}
	return 0;
}

// Returns 1 if shared up, 2 if shared down, 0 if not shared
int ChDomainDistr::InShared(std::shared_ptr<ChBody> body)
{
	double pos = body->GetPos()[long_axis];
	int my_rank = my_sys->GetMyRank();
	int ranks = my_sys->GetRanks();
	double ghost_layer = my_sys->GetGhostLayer();

	if (pos >= subhi[long_axis] - ghost_layer && pos < subhi[long_axis] && my_rank != ranks)
	{
		return 1;
	}
	if (pos < sublo[long_axis] + ghost_layer && pos >= sublo[long_axis] && my_rank != 0)
	{
		return 2;
	}
	return 0;
}


int ChDomainDistr::GetCommStatus(int index)
{
	int status = InShared(index);
	if (status == 1)
	{
		return chrono::SHARED_UP;
	}
	if (status == 2)
	{
		return chrono::SHARED_DOWN;
	}

	status = InOwned(index);
	if (status)
	{
		return chrono::OWNED;
	}

	status = InGhost(index);
	if (status == 1)
	{
		return chrono::GHOST_UP;
	}
	if (status == 2)
	{
		return chrono::GHOST_DOWN;
	}

	if (my_sys->data_manager->host_data.pos_rigid[index][long_axis] >= subhi[long_axis] + my_sys->GetGhostLayer())
	{
		return chrono::UNOWNED_UP;
	}

	if (my_sys->data_manager->host_data.pos_rigid[index][long_axis] < sublo[long_axis] - my_sys->GetGhostLayer())
	{
		return chrono::UNOWNED_DOWN;
	}

	// TODO: Sanity check
	GetLog() << "Error classifying body\n";
	return -1;
}

int ChDomainDistr::GetCommStatus(std::shared_ptr<ChBody> body)
{
	int status = InShared(body);
	if (status == 1)
	{
		return chrono::SHARED_UP;
	}
	if (status == 2)
	{
		return chrono::SHARED_DOWN;
	}

	status = InOwned(body);
	if (status)
	{
		return chrono::OWNED;
	}

	status = InGhost(body);
	if (status == 1)
	{
		return chrono::GHOST_UP;
	}
	if (status == 2)
	{
		return chrono::GHOST_DOWN;
	}

	if (body->GetPos()[long_axis] >= subhi[long_axis] + my_sys->GetGhostLayer())
	{
		return chrono::UNOWNED_UP;
	}

	if (body->GetPos()[long_axis] < sublo[long_axis] - my_sys->GetGhostLayer())
	{
		return chrono::UNOWNED_DOWN;
	}

	GetLog() << "Error classifying body\n";
	return -1;
}


void ChDomainDistr::PrintDomain()
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
