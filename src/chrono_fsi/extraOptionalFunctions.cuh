/*
 * ExtraOptionalFunctions.cuh
 *
 *  Created on: Jan 23, 2015
 *      Author: arman
 */

#ifndef EXTRAOPTIONALFUNCTIONS_CUH_
#define EXTRAOPTIONALFUNCTIONS_CUH_

#include "extraOptionalFunctions.cuh"

// Rigids
void FindPassesFromTheEnd(
		thrust::device_vector<real3> & posRigidD,
		thrust::device_vector<int> & distributionD,
		int numRigidBodies,
		real2 pipeCenter,
		real_ pipeRadius,
		int numberOfSections);


#endif /* EXTRAOPTIONALFUNCTIONS_CUH_ */
