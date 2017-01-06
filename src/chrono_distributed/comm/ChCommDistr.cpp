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

#include <mpi.h>

#include "chrono_distributed/comm/ChCommDistr.h"

#include "chrono_distributed/physics/ChBodyDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

ChCommDistr::ChCommDistr(ChSystemDistr * my_sys)
{
	this->my_sys = my_sys;

	send_buf = new double[(my_sys->GetMaxLocal() + my_sys->GetMaxShared()) * doubles_per_body];
	num_send = 0;

	recv_buf = new double[my_sys->GetMaxGhost() * doubles_per_body];
	num_recv = 0;

	doubles_per_body = 15;
}

ChCommDistr::~ChCommDistr()
{
	delete send_buf;
	delete recv_buf;
}

// Locate each body that has left the subdomain,
// remove it from the appropriate body list,
// and pack it for communication.
void ChCommDistr::Exchange()
{
	int num_elements = 0;

	//TODO:
	// Check local bodies
		// Send bodies that have left
		SendAll(send_buf, num_elements);
	// Recv incoming bodies.
}

void ChCommDistr::SendAll(double *buf, int size)
{
	//TODO: see liggghts MPI_Bcast(send_buf, size, MPI_DOUBLE, )
}

// Called when a sphere leaves this rank's subdomain.
// The body should have been removed from the local system's
// list before sending.
// Packages the body into buf.
// Returns the number of elements which the body took in the buffer
int ChCommDistr::PackSphere(double *buf, ChBodyDistr *sphere_body)
{
	//TODO:

	return -1;
}

// Unpacks a sphere body from the buffer into body.
// Note: body is meant to be a ptr into the data structure
// where the body should be unpacks.
void ChCommDistr::UnpackSphere(double *buf, ChBodyDistr *body)
{
	//TODO:
}





} /* namespace chrono */
