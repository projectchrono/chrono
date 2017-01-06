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

#include <iostream>
#include <stdlib.h>
#include <mpi.h>

namespace chrono {

ChDomainDistr::ChDomainDistr(ChSystemDistr *sys)
{
	this->my_sys = sys;
	long_axis = 0;
}

ChDomainDistr::~ChDomainDistr() {}

// Takes in the user specified coordinates of the bounding box for the simulation.
void ChDomainDistr::SetSimDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi)
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

	if (len_x <= 0 || len_y <= 0 || len_z <=0) my_sys->ErrorAbort("Invalid domain dimensions.");

	// Index of the longest domain axis 0=x, 1=y, 2=z
	long_axis = (len_x >= len_y) ? 0 : 1;
	long_axis = (len_z >= boxhi[long_axis] - boxlo[long_axis]) ? 2 : long_axis;
}

} /* namespace chrono */
