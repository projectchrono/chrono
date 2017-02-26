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

int ChDomainDistr::InSub(int index)
{
	double pos = my_sys->GetDataManager()->host_data.pos_rigid[index][long_axis];
	// If the position is not in the subdoman AND not within the ghost layer inside the subdomain
	return ((pos >= sublo[long_axis] + my_sys->ghost_layer) && (pos < subhi[long_axis] - my_sys->ghost_layer)) ? 1 : 0;
}

int ChDomainDistr::InSub(std::shared_ptr<ChBody> body)
{
	double pos = body->GetPos()[long_axis];
	// If the position is not in the subdoman AND not within the ghost layer inside the subdomain
	return ((pos >= sublo[long_axis] + my_sys->ghost_layer) && (pos < subhi[long_axis] - my_sys->ghost_layer)) ? 1 : 0;
}

// Returns true only if the body is not in this subdomain,
// and is within the ghost thickness of the subdomain
int ChDomainDistr::InGhost(int index)
{
	double pos = my_sys->GetDataManager()->host_data.pos_rigid[index][long_axis];

	// Returns:
		// 1: Above the subdomain and within the ghost skin and
		// not out of the bounding box
		// OR
		// 2: Below the subdomain and within the ghost skin and
		// not out of the bounding box
		// OR
		// 0: Not in the ghost layer
	if ((pos > subhi[long_axis] &&
			pos <= subhi[long_axis] + my_sys->ghost_layer &&
			my_sys->my_rank != my_sys->num_ranks))
	{
		return 1;
	}
	if ((pos < sublo[long_axis] &&
			pos >= sublo[long_axis] - my_sys->ghost_layer &&
			my_sys->my_rank != 0))
	{
		return 2;
	}
	return 0;
}

int ChDomainDistr::InGhost(std::shared_ptr<ChBody> body)
{
	double pos = body->GetPos()[long_axis];

	// Returns:
		// 1: Above the subdomain and within the ghost skin and
		// not out of the bounding box
		// OR
		// 2: Below the subdomain and within the ghost skin and
		// not out of the bounding box
		// OR
		// 0: Not in the ghost layer
	if ((pos > subhi[long_axis] &&
			pos <= subhi[long_axis] + my_sys->ghost_layer &&
			my_sys->my_rank != my_sys->num_ranks))
	{
		return 1;
	}
	if ((pos < sublo[long_axis] &&
			pos >= sublo[long_axis] - my_sys->ghost_layer &&
			my_sys->my_rank != 0))
	{
		return 2;
	}
	return 0;
}

int ChDomainDistr::InShared(int index)
{
	double pos = my_sys->data_manager->host_data.pos_rigid[index][long_axis];

	// If seen on the next rank as a ghost
	if (pos < subhi[long_axis] && subhi[long_axis] - pos < my_sys->ghost_layer)
	{
		if (my_sys->GetMyRank() == my_sys->GetRanks() - 1)
		{
			return 0;
		}
		return 1;
	}
	// If seen on the previous rank as a ghost
	else if (pos > sublo[long_axis] && sublo[long_axis] - pos < my_sys->ghost_layer)
	{
		if (my_sys->GetMyRank() == 0)
		{
			return 0;
		}
		return 2;
	}

	return 0;
}

int ChDomainDistr::InShared(std::shared_ptr<ChBody> body)
{
	double pos = body->GetPos()[long_axis];

	// If seen on the next rank as a ghost
	if (pos < subhi[long_axis] && subhi[long_axis] - pos < my_sys->ghost_layer)
	{
		if (my_sys->GetMyRank() == my_sys->GetRanks() - 1)
		{
			return 0;
		}
		return 1;
	}
	// If seen on the previous rank as a ghost
	else if (pos > sublo[long_axis] && sublo[long_axis] - pos < my_sys->ghost_layer)
	{
		if (my_sys->GetMyRank() == 0)
		{
			return 0;
		}
		return 2;
	}

	return 0;
}


// 0 none, 1 shared up, 2 shared down, 3 owned, 4 ghost up, 5 ghost down
int ChDomainDistr::GetCommStatus(int index)
{
	int status = InShared(index);
	if (status)
	{
		return status;
	}
	status = InSub(index);
	if (status)
	{
		return status + 2;
	}
	status = InGhost(index);
	if (status)
	{
		return status + 3;
	}

	return 0;
}

// 0 none, 1 shared up, 2 shared down, 3 owned, 4 ghost up, 5 ghost down
int ChDomainDistr::GetCommStatus(std::shared_ptr<ChBody> body)
{
	int status = InShared(body);
	if (status)
	{
		return status;
	}
	status = InSub(body);
	if (status)
	{
		return status + 2;
	}
	status = InGhost(body);
	if (status)
	{
		return status + 3;
	}

	return 0;
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
