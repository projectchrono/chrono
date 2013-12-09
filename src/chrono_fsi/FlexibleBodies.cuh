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

static __device__ __host__ void shape_fun(real_* S, real_ x, real_ L)
{
	real_ xi = x/L;

	S[0] = 1 - 3*xi*xi + 2*xi*xi*xi;
	S[1] = L * (xi - 2*xi*xi + xi*xi*xi);
	S[2] = 3*xi*xi - 2*xi*xi*xi;
	S[3] = L * (-xi*xi + xi*xi*xi);
}

static __device__ __host__ void shape_fun_d(real_* Sx, real_ x, real_ L)
{
	real_ xi = x/L;

	Sx[0] = (6*xi*xi-6*xi)/L;
	Sx[1] = 1-4*xi+3*xi*xi;
	Sx[2] = -(6*xi*xi-6*xi)/L;
	Sx[3] = -2*xi+3*xi*xi;
}

void Update_ANCF_Beam(
		thrust::device_vector<real3> & ANCF_NodesD,
		thrust::device_vector<real3> & ANCF_SlopesD,
		thrust::device_vector<real3> & ANCF_NodesVelD,
		thrust::device_vector<real3> & ANCF_SlopesVelD,
		thrust::device_vector<real3> & flex_FSI_NodesForces1,
		thrust::device_vector<real3> & flex_FSI_NodesForces2,
		const thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,
		const thrust::device_vector<real_> & ANCF_Beam_LengthD,
		const int numFlexBodies,
		real_ dT
		);

#endif
