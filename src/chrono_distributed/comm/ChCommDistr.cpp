/*
 * ChCommDistr.cpp
 *
 *  Created on: Dec 29, 2016
 *      Author: nic
 */

#include "ChCommDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"

namespace chrono {

ChCommDistr::ChCommDistr(ChSystemDistr * my_sys) {
	this->my_sys = my_sys;
}

ChCommDistr::~ChCommDistr() {}


// Locate each body that has left the subdomain,
// remove it from the appropriate body list,
// and pack it for communication.
void ChCommDistr::Exchange()
{
	//TODO:
	// Check local bodies
		// Send bodies that have left
	// Recv incoming bodies.
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
