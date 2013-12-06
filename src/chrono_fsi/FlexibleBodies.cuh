///////////////////////////////////////////////////////////////////////////////
//	collideSphereSphere.cu
//	Does the required calculations for SPH, both fluid and rigid bodies
//
//	Related Files: collideSphereSphere.cuh, SDKCollisionSystem.cu, SDKCollisionSystem.cu
//	Input File:		-
/////////////////////////////////////////////////////////////////////////////////
//	Created by Arman Pazouki, apazouki@gmail.com
//	Copyright 2013 Arman Pazouki
//
//	This file is part of SPH_FSI.
//
//	SPH_FSI is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	any later version.
//
//	SPH_FSI is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with SPH_FSI.  If not, see <http://www.gnu.org/licenses/>.
///////////////////////////////////////////////////////////////////////////////

#ifndef FLEXIBLEBODIES_CUH
#define FLEXIBLEBODIES_CUH


__device__ __host__ void shape_fun(real_* S, real_ x, real_ L);

void CalcElasticForces(
		thrust::device_vector<real3> & flex_FSI_NodesForces1,
		thrust::device_vector<real3> & flex_FSI_NodesForces2,
		const thrust::device_vector<real3> & ANCF_NodesD,
		const thrust::device_vector<real3> & ANCF_SlopesD,
		const thrust::device_vector<real3> & ANCF_NodesVelD,
		const thrust::device_vector<real3> & ANCF_SlopesVelD,
		const thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,
		const thrust::device_vector<real_> & ANCF_Beam_LengthD,
		const int numFlexBodies
	);

#endif
