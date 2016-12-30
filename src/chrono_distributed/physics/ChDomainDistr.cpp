/*
 * ChDomainDistr.cpp
 *
 *  Created on: Dec 28, 2016
 *      Author: nic
 */

#include "ChDomainDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

#include <iostream>
#include <stdlib.h>
#include <mpi.h>

namespace chrono {
namespace chrono_distributed {

ChDomainDistr::ChDomainDistr(ChSystemDistr *sys)
{
	this->my_sys = sys;
	long_axis = 0;
}

ChDomainDistr::~ChDomainDistr() {}

// Takes in the user specified coordinates of the bounding box for the simulation.
void ChDomainDistr::SetDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi)
{
	boxlo[0] = xlo;
	boxhi[0] = xhi;
	boxlo[1] = ylo;
	boxhi[1] = yhi;
	boxlo[2] = zlo;
	boxhi[2] = zhi;

	double len_x = boxhi[0] - boxlo[0];
	double len_y = boxhi[1] - boxlo[1];
	double len_z = boxhi[2] - boxlo[2];

	if (len_x <= 0 || len_y <= 0 || len_z <=0)
	{
		std::cout << "Invalid domain dimensions" << std::endl;
		MPI_Abort(my_sys->world, MPI_ERR_OTHER);
	}

	// index of the longest domain axis 0=x, 1=y, 2=z
	long_axis = (len_x >= len_y) ? 0 : 1;
	long_axis = (len_z >= boxhi[long_axis] - boxlo[long_axis]) ? 2 : long_axis;
}

// Divides the domain into equal-volume, orthogonal, axis-aligned regions along
// the longest axis.
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
		}
	}
}


} /* namespace chrono */
}
