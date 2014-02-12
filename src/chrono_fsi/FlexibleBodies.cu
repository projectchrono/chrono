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
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// These functions calculate the integrands required to calculate
// the generalized internal forces for an ANCF beam element:
//
// The function eps_eps_e() calculates the integrand for the bending
// elastic force (a 12-dimensional quantity)
//
// The function kappa_kappa_e() calculates the integrand for the axial
// elastic force (a 12-dimensional quantity)
//
// The two actual parts of the generalized elastic force must be calculated
// performing Gauss quadrature over the length of the ANCF elastic beam
// element.
//
// Note also that the axial part must be multiplied by E*A, while the bending
// part must be multiplied by E*I (if these quantities are constant along the
// length of the element, multiplication should be done after integration;
// otherwise they must multiply the integrand when evaluated at the Gauss
// quadrature points).
//
// The first 6 components in Q are associated with the 6 nodal coordinates at
// the "left" side of the beam element (i.e., e1...e6), while the last 6
// component are associated with the 6 nodal coordinates at the "right" of the
// element (i.e., e7...e12).
//
// Note: these functions make use of the utility functions defined in
// shapeFunctions.cpp.
/////////////////////////////////////////////////////////////////////////////
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "SPHCudaUtils.h"
#include "minv_vec.cuh"
#include "FlexibleBodies.cuh"

#include <cstdio>



// Gaussian Quadrature. Applied to the [-1, 1] interval. For other intervals it is like this:
// int_a^b f(x)dx = (b-a)/2 * sum{w_i * f( (b-a)/2*z_i + (a+b)/2 ) }
const real_ GQ3_p[3] = {-0.774596669241483	, 0					, 0.774596669241483};
const real_ GQ3_w[3] = {0.555555555555556	, 0.888888888888889	, 0.555555555555556};
const real_ GQ4_p[4] = {-0.861136311594053	, -0.339981043584856, 0.339981043584856, 0.861136311594053};
const real_ GQ4_w[4] = {0.347854845137454	, 0.652145154862546	, 0.652145154862546, 0.347854845137454};
const real_ GQ5_p[5] = {-0.906179845938664	, -0.538469310105683, 0					,	0.538469310105683, 0.906179845938664};
const real_ GQ5_w[5] = {0.236926885056189	, 0.478628670499366	, 0.568888888888889	,	0.478628670499366, 0.236926885056189};


//------------------------------------------------------------------------------
__device__ __host__ inline void shape_fun_dd(real_* Sxx, real_ x, real_ L)
{
	real_ xi = x/L;

	Sxx[0] = (12*xi-6)/(L*L);
	Sxx[1] = (-4+6*xi)/L;
	Sxx[2] = (6-12*xi)/(L*L);
	Sxx[3] = (-2+6*xi)/L;
}
//------------------------------------------------------------------------------
__device__ __host__ inline void eps_eps_e(real_* e_ee, real_ x, real_ L, real_* e)
{
	real_ e1  = e[0];
	real_ e2  = e[1];
	real_ e3  = e[2];
	real_ e4  = e[3];
	real_ e5  = e[4];
	real_ e6  = e[5];

	real_ e7  = e[6];
	real_ e8  = e[7];
	real_ e9  = e[8];
	real_ e10 = e[9];
	real_ e11 = e[10];
	real_ e12 = e[11];

	real_ Sx[4];

	shape_fun_d(Sx, x, L);

	real_ rx[3];

	rx[0] = Sx[0]*e1 + Sx[1]*e4 + Sx[2]*e7 + Sx[3]*e10;
	rx[1] = Sx[0]*e2 + Sx[1]*e5 + Sx[2]*e8 + Sx[3]*e11;
	rx[2] = Sx[0]*e3 + Sx[1]*e6 + Sx[2]*e9 + Sx[3]*e12;

	real_ rx_rx = rx[0] * rx[0] + rx[1] * rx[1] + rx[2] * rx[2];

	// Calculate eps = 0.5 * (||rx||^2 - 1)
	real_ eps = 0.5 * (rx_rx - 1);

	// Calculate the integrand for Qa (i.e. eps * eps_e), using the
	// fact that  eps_e = (Sx^T * Sx) * e
	e_ee[0]  = eps * Sx[0] * (e1*Sx[0]+e4*Sx[1]+e7*Sx[2]+e10*Sx[3]);
	e_ee[1]  = eps * Sx[0] * (e2*Sx[0]+e5*Sx[1]+e8*Sx[2]+e11*Sx[3]);
	e_ee[2]  = eps * Sx[0] * (e3*Sx[0]+e6*Sx[1]+e9*Sx[2]+e12*Sx[3]);
	e_ee[3]  = eps * Sx[1] * (e1*Sx[0]+e4*Sx[1]+e7*Sx[2]+e10*Sx[3]);
	e_ee[4]  = eps * Sx[1] * (e2*Sx[0]+e5*Sx[1]+e8*Sx[2]+e11*Sx[3]);
	e_ee[5]  = eps * Sx[1] * (e3*Sx[0]+e6*Sx[1]+e9*Sx[2]+e12*Sx[3]);
	e_ee[6]  = eps * Sx[2] * (e1*Sx[0]+e4*Sx[1]+e7*Sx[2]+e10*Sx[3]);
	e_ee[7]  = eps * Sx[2] * (e2*Sx[0]+e5*Sx[1]+e8*Sx[2]+e11*Sx[3]);
	e_ee[8]  = eps * Sx[2] * (e3*Sx[0]+e6*Sx[1]+e9*Sx[2]+e12*Sx[3]);
	e_ee[9]  = eps * Sx[3] * (e1*Sx[0]+e4*Sx[1]+e7*Sx[2]+e10*Sx[3]);
	e_ee[10] = eps * Sx[3] * (e2*Sx[0]+e5*Sx[1]+e8*Sx[2]+e11*Sx[3]);
	e_ee[11] = eps * Sx[3] * (e3*Sx[0]+e6*Sx[1]+e9*Sx[2]+e12*Sx[3]);
}

__device__ __host__ inline void eps_eps_e_ALT(real_* e_ee, real_ x, real_ L, real_* e)
{
	real_ e1  = e[0];
	real_ e2  = e[1];
	real_ e3  = e[2];
	real_ e4  = e[3];
	real_ e5  = e[4];
	real_ e6  = e[5];

	real_ e7  = e[6];
	real_ e8  = e[7];
	real_ e9  = e[8];
	real_ e10 = e[9];
	real_ e11 = e[10];
	real_ e12 = e[11];

	real_ Sx[4];

	shape_fun_d(Sx, x, L);

	real_  t2 = Sx[0];
	real_  t3 = e1*t2;
	real_  t4 = Sx[1];
	real_  t5 = e4*t4;
	real_  t6 = Sx[2];
	real_  t7 = e7*t6;
	real_  t8 = Sx[3];
	real_  t9 = e10*t8;
	real_  t10 = t3+t5+t7+t9;
	real_  t13 = e2*t2;
	real_  t14 = e5*t4;
	real_  t15 = e8*t6;
	real_  t16 = e11*t8;
	real_  t11 = t13+t14+t15+t16;
	real_  t21 = e3*t2;
	real_  t22 = e6*t4;
	real_  t23 = e9*t6;
	real_  t24 = e12*t8;
	real_  t12 = t21+t22+t23+t24;
	real_  t17 = t10*t10;
	real_  t18 = t17*(1.0/2.0);
	real_  t19 = t11*t11;
	real_  t20 = t19*(1.0/2.0);
	real_  t25 = t12*t12;
	real_  t26 = t25*(1.0/2.0);
	real_  t27 = t18+t20+t26-1.0/2.0;

	e_ee[0]  = t2*t10*t27;
	e_ee[1]  = t2*t11*t27;
	e_ee[2]  = t2*t12*t27;
	e_ee[3]  = t4*t10*t27;
	e_ee[4]  = t4*t11*t27;
	e_ee[5]  = t4*t12*t27;
	e_ee[6]  = t6*t10*t27;
	e_ee[7]  = t6*t11*t27;
	e_ee[8]  = t6*t12*t27;
	e_ee[9]  = t8*t10*t27;
	e_ee[10] = t8*t11*t27;
	e_ee[11] = t8*t12*t27;
}

__device__ __host__ inline void kappa_kappa_e(real_* k_ke, real_ x, real_ L, real_* e)
{
	real_ e1  = e[0];
	real_ e2  = e[1];
	real_ e3  = e[2];
	real_ e4  = e[3];
	real_ e5  = e[4];
	real_ e6  = e[5];

	real_ e7  = e[6];
	real_ e8  = e[7];
	real_ e9  = e[8];
	real_ e10 = e[9];
	real_ e11 = e[10];
	real_ e12 = e[11];

	real_ Sx[4];
	real_ Sxx[4];

	shape_fun_d(Sx, x, L);
	shape_fun_dd(Sxx, x, L);

	real_ rx[3];
	real_ rxx[3];

	rx[0] = Sx[0]*e1 + Sx[1]*e4 + Sx[2]*e7 + Sx[3]*e10;
	rx[1] = Sx[0]*e2 + Sx[1]*e5 + Sx[2]*e8 + Sx[3]*e11;
	rx[2] = Sx[0]*e3 + Sx[1]*e6 + Sx[2]*e9 + Sx[3]*e12;

	rxx[0] = Sxx[0]*e1 + Sxx[1]*e4 + Sxx[2]*e7 + Sxx[3]*e10;
	rxx[1] = Sxx[0]*e2 + Sxx[1]*e5 + Sxx[2]*e8 + Sxx[3]*e11;
	rxx[2] = Sxx[0]*e3 + Sxx[1]*e6 + Sxx[2]*e9 + Sxx[3]*e12;

	real_ rx_rx   =  rx[0] *  rx[0] +  rx[1] *  rx[1] +  rx[2] *  rx[2];
	real_ rx_rxx  =  rx[0] * rxx[0] +  rx[1] * rxx[1] +  rx[2] * rxx[2];
	real_ rxx_rxx = rxx[0] * rxx[0] + rxx[1] * rxx[1] + rxx[2] * rxx[2];

	// Define v = rx X rxx
	real_ v[3];

	v[0] =  rx[1]*rxx[2] - rx[2]*rxx[1];
	v[1] = -rx[0]*rxx[2] + rx[2]*rxx[0];
	v[2] =  rx[0]*rxx[1] - rx[1]*rxx[0];

	// Calculate the squared norm of v
	real_ v_v = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];

	// Define some temporary variables
	real_ inv_rx_rx_cubed = 1.0 / (rx_rx * rx_rx * rx_rx);
	real_ coef = 3 * v_v / rx_rx;


	// Calculate k
	real_ v_nrm = sqrt(v_v);
	real_ inv_rx_nrm_cubed = sqrt(inv_rx_rx_cubed);

	real_ k = v_nrm * inv_rx_nrm_cubed;

	// Calculate the integrand for Qk (this is k * k_e)
	k_ke[0]  = -inv_rx_rx_cubed * ( -(Sx[0]*rxx[1]-Sxx[0]*rx[1])*v[2]+(Sx[0]*rxx[2]-Sxx[0]*rx[2])*v[1] + coef*Sx[0]*rx[0] );
	k_ke[1]  = -inv_rx_rx_cubed * (  (Sx[0]*rxx[0]-Sxx[0]*rx[0])*v[2]-(Sx[0]*rxx[2]-Sxx[0]*rx[2])*v[0] + coef*Sx[0]*rx[1] );
	k_ke[2]  = -inv_rx_rx_cubed * ( -(Sx[0]*rxx[0]-Sxx[0]*rx[0])*v[1]+(Sx[0]*rxx[1]-Sxx[0]*rx[1])*v[0] + coef*Sx[0]*rx[2] );
	k_ke[3]  = -inv_rx_rx_cubed * ( -(Sx[1]*rxx[1]-Sxx[1]*rx[1])*v[2]+(Sx[1]*rxx[2]-Sxx[1]*rx[2])*v[1] + coef*Sx[1]*rx[0] );
	k_ke[4]  = -inv_rx_rx_cubed * (  (Sx[1]*rxx[0]-Sxx[1]*rx[0])*v[2]-(Sx[1]*rxx[2]-Sxx[1]*rx[2])*v[0] + coef*Sx[1]*rx[1] );
	k_ke[5]  = -inv_rx_rx_cubed * ( -(Sx[1]*rxx[0]-Sxx[1]*rx[0])*v[1]+(Sx[1]*rxx[1]-Sxx[1]*rx[1])*v[0] + coef*Sx[1]*rx[2] );
	k_ke[6]  = -inv_rx_rx_cubed * ( -(Sx[2]*rxx[1]-Sxx[2]*rx[1])*v[2]+(Sx[2]*rxx[2]-Sxx[2]*rx[2])*v[1] + coef*Sx[2]*rx[0] );
	k_ke[7]  = -inv_rx_rx_cubed * (  (Sx[2]*rxx[0]-Sxx[2]*rx[0])*v[2]-(Sx[2]*rxx[2]-Sxx[2]*rx[2])*v[0] + coef*Sx[2]*rx[1] );
	k_ke[8]  = -inv_rx_rx_cubed * ( -(Sx[2]*rxx[0]-Sxx[2]*rx[0])*v[1]+(Sx[2]*rxx[1]-Sxx[2]*rx[1])*v[0] + coef*Sx[2]*rx[2] );
	k_ke[9]  = -inv_rx_rx_cubed * ( -(Sx[3]*rxx[1]-Sxx[3]*rx[1])*v[2]+(Sx[3]*rxx[2]-Sxx[3]*rx[2])*v[1] + coef*Sx[3]*rx[0] );
	k_ke[10] = -inv_rx_rx_cubed * (  (Sx[3]*rxx[0]-Sxx[3]*rx[0])*v[2]-(Sx[3]*rxx[2]-Sxx[3]*rx[2])*v[0] + coef*Sx[3]*rx[1] );
	k_ke[11] = -inv_rx_rx_cubed * ( -(Sx[3]*rxx[0]-Sxx[3]*rx[0])*v[1]+(Sx[3]*rxx[1]-Sxx[3]*rx[1])*v[0] + coef*Sx[3]*rx[2] );
}
//------------------------------------------------------------------------------
__device__ __host__ inline void gravitational_force(real_* f_g, real_ L, real_ rho, real_ A, real3 g)
{
	real_ coef = rho * A * L;

	real_ t5  = coef * g.x / 2.0;
	real_ t6  = coef * g.y / 2.0;
	real_ t7  = coef * g.z / 2.0;
	real_ t8  = coef * g.x * L / 12.0;
	real_ t9  = coef * g.y * L / 12.0;
	real_ t10 = coef * g.z * L / 12.0;

	f_g[0]  =  t5;
	f_g[1]  =  t6;
	f_g[2]  =  t7;
	f_g[3]  =  t8;
	f_g[4]  =  t9;
	f_g[5]  =  t10;
	f_g[6]  =  t5;
	f_g[7]  =  t6;
	f_g[8]  =  t7;
	f_g[9]  = -t8;
	f_g[10] = -t9;
	f_g[11] = -t10;
}
//------------------------------------------------------------------------------
// sum += mult * a
__device__ __host__ inline void SumArrays(real_* sum, real_* a, real_ mult, int numComp) {
	for (int k = 0; k < numComp; k++) {
		sum[k] += mult * a[k];
	}
}
//------------------------------------------------------------------------------
__device__ __host__ inline void CopyElementNodesTo_e(real_* e, const thrust::device_vector<real3> & ANCF_NodesD, const thrust::device_vector<real3> & ANCF_SlopesD, int nodeIdx) {
	real3 ni = ANCF_NodesD[nodeIdx];
	e[0] = ni.x;
	e[1] = ni.y;
	e[2] = ni.z;
	real3 si = ANCF_SlopesD[nodeIdx];
	e[3] = si.x;
	e[4] = si.y;
	e[5] = si.z;
	real3 nj = ANCF_NodesD[nodeIdx + 1];
	e[6] = nj.x;
	e[7] = nj.y;
	e[8] = nj.z;
	real3 sj = ANCF_SlopesD[nodeIdx + 1];
	e[9] = sj.x;
	e[10] = sj.y;
	e[11] = sj.z;
}
//------------------------------------------------------------------------------
// Why -= and not += : because M*X2 + K*X = F Therefore, in an explicit method: M*X2 = F - K*X where K*X is our elastic forces (f)
__device__ __host__ inline void Add_f_ToForces(thrust::host_vector<real3> & flex_FSI_NodesForces1, thrust::host_vector<real3> & flex_FSI_NodesForces2, real_ k, real_* f, int nodeIdx) {
	flex_FSI_NodesForces1[nodeIdx] += k * R3(f[0], f[1], f[2]);
	flex_FSI_NodesForces2[nodeIdx] += k * R3(f[3], f[4], f[5]);
	flex_FSI_NodesForces1[nodeIdx + 1] += k * R3(f[6], f[7], f[8]);
	flex_FSI_NodesForces2[nodeIdx + 1] += k * R3(f[9], f[10], f[11]);
}
//------------------------------------------------------------------------------
__device__ __host__ inline void MapBeamDataTo_1D_Array(real_* e, const thrust::host_vector<real3> & beamData1, const thrust::host_vector<real3> & beamData2, int2 nodesPortion) { //beamData1: postion, beamData2: slope
	int numNodes = nodesPortion.y - nodesPortion.x;
	for (int j = 0; j < numNodes; j++) {
		int nodeIdx = nodesPortion.x + j;
		real3 data1 = beamData1[nodeIdx];
		e[6 * j + 0] = data1.x;
		e[6 * j + 1] = data1.y;
		e[6 * j + 2] = data1.z;
		real3 data2 = beamData2[nodeIdx];
		e[6 * j + 3] = data2.x;
		e[6 * j + 4] = data2.y;
		e[6 * j + 5] = data2.z;
	}
}
//------------------------------------------------------------------------------
void CalcElasticForces(
		thrust::host_vector<real3> & flex_FSI_NodesForces1,
		thrust::host_vector<real3> & flex_FSI_NodesForces2,
		const thrust::device_vector<real3> & ANCF_NodesD,
		const thrust::device_vector<real3> & ANCF_SlopesD,
		const thrust::device_vector<real3> & ANCF_NodesVelD,
		const thrust::device_vector<real3> & ANCF_SlopesVelD,
		const thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,
		const thrust::device_vector<real_> & ANCF_Beam_LengthD,
		const int numFlexBodies,
		const ANCF_Params & flexParams
	)
{
	for (int i = 0; i < numFlexBodies; i++) {
		real_ l = ANCF_Beam_LengthD[i];
		int2 nodesPortion = ANCF_ReferenceArrayNodesOnBeamsD[i];
		int numNodes = nodesPortion.y - nodesPortion.x;
		int numElements = numNodes - 1;
		real_ lE = l / numElements;
		for (int j = 0; j < numElements; j++) {
			int nodeIdx = nodesPortion.x + j;
			real_ e[12];
			CopyElementNodesTo_e(e, ANCF_NodesD, ANCF_SlopesD, nodeIdx);
			real_ f_e[12] = {0};
			real_ e_ee[12] = {0};
			real_ k_ke[12] = {0};
			real_ f_g[12] = {0};
			// Elastic Force, 1/2: tension force, GQ 5th order. Maybe 4th order is enough as well.
			for (int k = 0; k < 5; k ++) {
				real_ gqPoint = (lE - 0) / 2 * GQ5_p[k] + (lE + 0) / 2;
				eps_eps_e(e_ee, gqPoint, lE, e);
				SumArrays(f_e, e_ee, flexParams.E * flexParams.A * (lE - 0) / 2 * GQ5_w[k], 12);
			}
			// Elastic Force, 2/2: bending force, GQ 3rd order.
			for (int k = 0; k < 3; k ++) {
				real_ gqPoint = (lE - 0) / 2 * GQ3_p[k] + (lE + 0) / 2;
				kappa_kappa_e(k_ke, gqPoint, lE, e);
				SumArrays(f_e, k_ke, flexParams.E * flexParams.I  * (lE - 0) / 2 * GQ3_w[k], 12);
			}
			// Gravitational Foce
			gravitational_force(f_g, lE, flexParams.rho, flexParams.A, flexParams.gravity);
			SumArrays(f_e, f_g, -1, 12);
			// Add element forces to associated nodes
			Add_f_ToForces(flex_FSI_NodesForces1, flex_FSI_NodesForces2, -1, f_e, nodeIdx);
		}
	}
}
//------------------------------------------------------------------------------
__device__ __host__ inline void ItegrateInTime(
		thrust::device_vector<real3> & ANCF_NodesD2,
		thrust::device_vector<real3> & ANCF_SlopesD2,
		thrust::device_vector<real3> & ANCF_NodesVelD2,
		thrust::device_vector<real3> & ANCF_SlopesVelD2,

		const thrust::device_vector<real3> & ANCF_NodesVelD,
		const thrust::device_vector<real3> & ANCF_SlopesVelD,

		real_* D2Node,
		int2 nodesPortionAdjusted,
		real_ lE,
		const ANCF_Params & flexParams,
		real_ dT) {
	int numNodesAdjusted = nodesPortionAdjusted.y - nodesPortionAdjusted.x;
	for (int j = 0; j < numNodesAdjusted; j++) {
		int nodeIdx = nodesPortionAdjusted.x + j;
		ANCF_NodesD2[nodeIdx] += dT * ANCF_NodesVelD[nodeIdx];
		ANCF_SlopesD2[nodeIdx] += dT * ANCF_SlopesVelD[nodeIdx];

		ANCF_NodesVelD2[nodeIdx] += dT * (1/(flexParams.rho * flexParams.A * lE)) * R3(D2Node[6 * j + 0], D2Node[6 * j + 1], D2Node[6 * j + 2]);
		ANCF_SlopesVelD2[nodeIdx] += dT * (1/(flexParams.rho * flexParams.A * lE)) * R3(D2Node[6 * j + 3], D2Node[6 * j + 4], D2Node[6 * j + 5]);
	}
}
//------------------------------------------------------------------------------
real3 RotatePoint_Y_Direction(real3 pIn, real3 center, real_ theta) {
	real3 A1 = R3(cos(theta), 0, -sin(theta));
	real3 A2 = R3(0, 1, 0);
	real3 A3 = R3(sin(theta), 0, cos(theta));
	real3 rIn = pIn - center;
	return (center + R3(dot(A1, rIn), dot(A2, rIn), dot(A3, rIn)));
}
//------------------------------------------------------------------------------
real3 RotateY_Direction(real3 vec, real_ theta) {
	return RotatePoint_Y_Direction(vec, R3(0), theta);
}
//------------------------------------------------------------------------------

void RigidBodyRotation(
		thrust::device_vector<real3> & ANCF_NodesD,
		thrust::device_vector<real3> & ANCF_SlopesD,
		thrust::device_vector<real3> & ANCF_NodesVelD,
		thrust::device_vector<real3> & ANCF_SlopesVelD,
		int2 nodesPortion,
		real_ lE,
		real_ dT) {
	int numNodes = nodesPortion.y - nodesPortion.x;
	real3 pa = ANCF_NodesD[nodesPortion.x];
	real3 pb = ANCF_NodesD[nodesPortion.y - 1];
	real3 pCenter =  0.5 * (pa + pb);
						//	printf("pCenter %f %f %f\n", pCenter.x, pCenter.y, pCenter.z);
	real_ omega = 10;
	real3 omega3 = omega * R3(0, 1, 0);
	real_ dTheta = omega * dT;

	for (int j = 0; j < numNodes; j++) {
		int nodeIdx = nodesPortion.x + j;
		real3 pBefore = ANCF_NodesD[nodeIdx];
		ANCF_NodesD[nodeIdx] = RotatePoint_Y_Direction(pBefore, pCenter, dTheta);
		real3 nodeSlope3 = ANCF_SlopesD[nodeIdx];
		ANCF_SlopesD[nodeIdx] = RotateY_Direction(nodeSlope3, dTheta);

		real3 relR = ANCF_NodesD[nodeIdx] - pCenter;

		ANCF_NodesVelD[nodeIdx] = cross(omega3, relR);
		ANCF_SlopesVelD[nodeIdx] = cross(omega3, nodeSlope3);
	}
}
//------------------------------------------------------------------------------
void Update_ANCF_Beam(
		thrust::device_vector<real3> & ANCF_NodesD2,
		thrust::device_vector<real3> & ANCF_SlopesD2,
		thrust::device_vector<real3> & ANCF_NodesVelD2,
		thrust::device_vector<real3> & ANCF_SlopesVelD2,

		const thrust::device_vector<real3> & ANCF_NodesD,
		const thrust::device_vector<real3> & ANCF_SlopesD,
		const thrust::device_vector<real3> & ANCF_NodesVelD,
		const thrust::device_vector<real3> & ANCF_SlopesVelD,

		const thrust::device_vector<real3> & flex_FSI_NodesForces1,
		const thrust::device_vector<real3> & flex_FSI_NodesForces2,
		const thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,
		const thrust::device_vector<real_> & ANCF_Beam_LengthD,
		const thrust::host_vector<bool> & ANCF_IsCantilever,
		const int numFlexBodies,
		const ANCF_Params & flexParams,
		real_ dT)
{
	thrust::host_vector<real3> flex_FSI_NodesForcesH1 = flex_FSI_NodesForces1;
	thrust::host_vector<real3> flex_FSI_NodesForcesH2 = flex_FSI_NodesForces2;
	CalcElasticForces(flex_FSI_NodesForcesH1, flex_FSI_NodesForcesH2,
			ANCF_NodesD, ANCF_SlopesD, ANCF_NodesVelD, ANCF_SlopesVelD,
			ANCF_ReferenceArrayNodesOnBeamsD, ANCF_Beam_LengthD, numFlexBodies, flexParams);

					//	for (int i = 0; i < flex_FSI_NodesForcesH1.size(); i++) {
					//		real3 f1 = 1e10 * flex_FSI_NodesForcesH1[i];
					//		real3 f2 = 1e10 * flex_FSI_NodesForcesH2[i];
					//		printf("my f1 %f %f %f, f2 %f %f %f\n", f1.x, f1.y, f1.z, f2.x, f2.y, f2.z);
					//	}

	for (int i = 0; i < numFlexBodies; i++) {
		real_ lBeam = ANCF_Beam_LengthD[i];
		bool isCantilever = ANCF_IsCantilever[i];
		int2 nodesPortion = ANCF_ReferenceArrayNodesOnBeamsD[i];
		int numNodes = nodesPortion.y - nodesPortion.x;
		int numElements = numNodes - 1;
		real_ lE = lBeam / numElements;
		int2 nodesPortionAdjusted2 = nodesPortion;
		if (isCantilever) {
			nodesPortionAdjusted2.x = nodesPortionAdjusted2.x + 1;
		}
		int numNodesAdjusted;
		numNodesAdjusted = nodesPortionAdjusted2.y - nodesPortionAdjusted2.x;
		real_* f = new real_ [numNodesAdjusted * 6];
		MapBeamDataTo_1D_Array(f, flex_FSI_NodesForcesH1, flex_FSI_NodesForcesH2, nodesPortionAdjusted2);
		real_* D2Node = new real_ [numNodesAdjusted * 6];

										//		//ff1
										//		for (int i = 0; i < numNodesAdjusted * 6; i ++) {
										//			printf("%e\n", f[i]);
										//		}
										//		printf("\n\n\n\n\n");

		min_vec(D2Node, f, lE, numElements, isCantilever);




					//ff1 : the followin commented lines are the real algorithm
		ItegrateInTime(
				ANCF_NodesD2, ANCF_SlopesD2, ANCF_NodesVelD2, ANCF_SlopesVelD2,
				ANCF_NodesVelD, ANCF_SlopesVelD,
				D2Node, nodesPortionAdjusted2,
				lE, flexParams, dT);

//		//ff1: dummy, to check rigid body motion and such
//		RigidBodyRotation(ANCF_NodesD2, ANCF_SlopesD2, ANCF_NodesVelD2, ANCF_SlopesVelD2,
//				nodesPortion,
//				lE, dT);



		delete [] f;
		delete [] D2Node;
	}
}
