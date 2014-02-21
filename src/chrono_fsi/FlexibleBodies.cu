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
#include "SDKCollisionSystem.cuh" // for computeGridSize

#include <cstdio>

#define flexGPU true
#if flexGPU
	__constant__ ANCF_Params flexParamsD;
	__constant__ real_ dTD;
	__constant__ int numFlexBodiesD;
	__constant__ GaussQuadrature GQD;

	#define __KERNEL__(...)  <<< __VA_ARGS__ >>>
	#define __GLOBAL__ __global__
#else
	ANCF_Params flexParamsH;
	real_ dTH;
	int numFlexBodiesH;
	GaussQuadrature GQH;
	#define flexParamsD flexParamsH
	#define dTD dTH
	#define numFlexBodiesD numFlexBodiesH
	#define GQD GQH

	#define __KERNEL__(...)
	#define __GLOBAL__
#endif


//------------------------------------------------------------------------------
__device__ __host__ inline void Zero12(Elem12 & m) {
	m.e00 = 0;
	m.e01 = 0;
	m.e02 = 0;
	m.e03 = 0;
	m.e04 = 0;
	m.e05 = 0;
	m.e06 = 0;
	m.e07 = 0;
	m.e08 = 0;
	m.e09 = 0;
	m.e10 = 0;
	m.e11 = 0;
}
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
__device__ __host__ inline void eps_eps_e(Elem12 & e_ee, real_ x, real_ L, const Elem12 & e)
{
	real_ e1  = e.e00;
	real_ e2  = e.e01;
	real_ e3  = e.e02;
	real_ e4  = e.e03;
	real_ e5  = e.e04;
	real_ e6  = e.e05;
	real_ e7  = e.e06;
	real_ e8  = e.e07;
	real_ e9  = e.e08;
	real_ e10 = e.e09;
	real_ e11 = e.e10;
	real_ e12 = e.e11;

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
	e_ee.e00 = eps * Sx[0] * (e1*Sx[0]+e4*Sx[1]+e7*Sx[2]+e10*Sx[3]);
	e_ee.e01 = eps * Sx[0] * (e2*Sx[0]+e5*Sx[1]+e8*Sx[2]+e11*Sx[3]);
	e_ee.e02 = eps * Sx[0] * (e3*Sx[0]+e6*Sx[1]+e9*Sx[2]+e12*Sx[3]);
	e_ee.e03 = eps * Sx[1] * (e1*Sx[0]+e4*Sx[1]+e7*Sx[2]+e10*Sx[3]);
	e_ee.e04 = eps * Sx[1] * (e2*Sx[0]+e5*Sx[1]+e8*Sx[2]+e11*Sx[3]);
	e_ee.e05 = eps * Sx[1] * (e3*Sx[0]+e6*Sx[1]+e9*Sx[2]+e12*Sx[3]);
	e_ee.e06 = eps * Sx[2] * (e1*Sx[0]+e4*Sx[1]+e7*Sx[2]+e10*Sx[3]);
	e_ee.e07 = eps * Sx[2] * (e2*Sx[0]+e5*Sx[1]+e8*Sx[2]+e11*Sx[3]);
	e_ee.e08 = eps * Sx[2] * (e3*Sx[0]+e6*Sx[1]+e9*Sx[2]+e12*Sx[3]);
	e_ee.e09 = eps * Sx[3] * (e1*Sx[0]+e4*Sx[1]+e7*Sx[2]+e10*Sx[3]);
	e_ee.e10 = eps * Sx[3] * (e2*Sx[0]+e5*Sx[1]+e8*Sx[2]+e11*Sx[3]);
	e_ee.e11 = eps * Sx[3] * (e3*Sx[0]+e6*Sx[1]+e9*Sx[2]+e12*Sx[3]);
}

__device__ __host__ inline void eps_eps_e_ALT(Elem12 & e_ee, real_ x, real_ L, const Elem12 & e)
{
	real_ e1  = e.e00;
	real_ e2  = e.e01;
	real_ e3  = e.e02;
	real_ e4  = e.e03;
	real_ e5  = e.e04;
	real_ e6  = e.e05;
	real_ e7  = e.e06;
	real_ e8  = e.e07;
	real_ e9  = e.e08;
	real_ e10 = e.e09;
	real_ e11 = e.e10;
	real_ e12 = e.e11;

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

	e_ee.e00 = t2*t10*t27;
	e_ee.e01 = t2*t11*t27;
	e_ee.e02 = t2*t12*t27;
	e_ee.e03 = t4*t10*t27;
	e_ee.e04 = t4*t11*t27;
	e_ee.e05 = t4*t12*t27;
	e_ee.e06 = t6*t10*t27;
	e_ee.e07 = t6*t11*t27;
	e_ee.e08 = t6*t12*t27;
	e_ee.e09 = t8*t10*t27;
	e_ee.e10 = t8*t11*t27;
	e_ee.e11 = t8*t12*t27;
}

__device__ __host__ inline void kappa_kappa_e(Elem12 & k_ke, real_ x, real_ L, const Elem12 & e)
{
	real_ e1  = e.e00;
	real_ e2  = e.e01;
	real_ e3  = e.e02;
	real_ e4  = e.e03;
	real_ e5  = e.e04;
	real_ e6  = e.e05;
	real_ e7  = e.e06;
	real_ e8  = e.e07;
	real_ e9  = e.e08;
	real_ e10 = e.e09;
	real_ e11 = e.e10;
	real_ e12 = e.e11;

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
	k_ke.e00 = -inv_rx_rx_cubed * ( -(Sx[0]*rxx[1]-Sxx[0]*rx[1])*v[2]+(Sx[0]*rxx[2]-Sxx[0]*rx[2])*v[1] + coef*Sx[0]*rx[0] );
	k_ke.e01 = -inv_rx_rx_cubed * (  (Sx[0]*rxx[0]-Sxx[0]*rx[0])*v[2]-(Sx[0]*rxx[2]-Sxx[0]*rx[2])*v[0] + coef*Sx[0]*rx[1] );
	k_ke.e02 = -inv_rx_rx_cubed * ( -(Sx[0]*rxx[0]-Sxx[0]*rx[0])*v[1]+(Sx[0]*rxx[1]-Sxx[0]*rx[1])*v[0] + coef*Sx[0]*rx[2] );
	k_ke.e03 = -inv_rx_rx_cubed * ( -(Sx[1]*rxx[1]-Sxx[1]*rx[1])*v[2]+(Sx[1]*rxx[2]-Sxx[1]*rx[2])*v[1] + coef*Sx[1]*rx[0] );
	k_ke.e04 = -inv_rx_rx_cubed * (  (Sx[1]*rxx[0]-Sxx[1]*rx[0])*v[2]-(Sx[1]*rxx[2]-Sxx[1]*rx[2])*v[0] + coef*Sx[1]*rx[1] );
	k_ke.e05 = -inv_rx_rx_cubed * ( -(Sx[1]*rxx[0]-Sxx[1]*rx[0])*v[1]+(Sx[1]*rxx[1]-Sxx[1]*rx[1])*v[0] + coef*Sx[1]*rx[2] );
	k_ke.e06 = -inv_rx_rx_cubed * ( -(Sx[2]*rxx[1]-Sxx[2]*rx[1])*v[2]+(Sx[2]*rxx[2]-Sxx[2]*rx[2])*v[1] + coef*Sx[2]*rx[0] );
	k_ke.e07 = -inv_rx_rx_cubed * (  (Sx[2]*rxx[0]-Sxx[2]*rx[0])*v[2]-(Sx[2]*rxx[2]-Sxx[2]*rx[2])*v[0] + coef*Sx[2]*rx[1] );
	k_ke.e08 = -inv_rx_rx_cubed * ( -(Sx[2]*rxx[0]-Sxx[2]*rx[0])*v[1]+(Sx[2]*rxx[1]-Sxx[2]*rx[1])*v[0] + coef*Sx[2]*rx[2] );
	k_ke.e09 = -inv_rx_rx_cubed * ( -(Sx[3]*rxx[1]-Sxx[3]*rx[1])*v[2]+(Sx[3]*rxx[2]-Sxx[3]*rx[2])*v[1] + coef*Sx[3]*rx[0] );
	k_ke.e10 = -inv_rx_rx_cubed * (  (Sx[3]*rxx[0]-Sxx[3]*rx[0])*v[2]-(Sx[3]*rxx[2]-Sxx[3]*rx[2])*v[0] + coef*Sx[3]*rx[1] );
	k_ke.e11 = -inv_rx_rx_cubed * ( -(Sx[3]*rxx[0]-Sxx[3]*rx[0])*v[1]+(Sx[3]*rxx[1]-Sxx[3]*rx[1])*v[0] + coef*Sx[3]*rx[2] );
}
//------------------------------------------------------------------------------
__device__ __host__ inline void gravitational_force(Elem12 & f_g, real_ L, real_ rho, real_ A, real3 g)
{
	real_ coef = rho * A * L;

	real_ t5  = coef * g.x / 2.0;
	real_ t6  = coef * g.y / 2.0;
	real_ t7  = coef * g.z / 2.0;
	real_ t8  = coef * g.x * L / 12.0;
	real_ t9  = coef * g.y * L / 12.0;
	real_ t10 = coef * g.z * L / 12.0;

	f_g.e00 =  t5;
	f_g.e01 =  t6;
	f_g.e02 =  t7;
	f_g.e03 =  t8;
	f_g.e04 =  t9;
	f_g.e05 =  t10;
	f_g.e06 =  t5;
	f_g.e07 =  t6;
	f_g.e08 =  t7;
	f_g.e09 = -t8;
	f_g.e10 = -t9;
	f_g.e11 = -t10;
}
//------------------------------------------------------------------------------
// sum += mult * a
__device__ __host__ inline void SumArrays(real_* sum, real_* a, real_ mult, int numComp) {
	for (int k = 0; k < numComp; k++) {
		sum[k] += mult * a[k];
	}
}
//------------------------------------------------------------------------------
// sum += mult * a
__device__ __host__ inline void SumElem12(Elem12 & sum, const Elem12 & a, real_ mult) {
	sum.e00 += mult * a.e00;
	sum.e01 += mult * a.e01;
	sum.e02 += mult * a.e02;
	sum.e03 += mult * a.e03;
	sum.e04 += mult * a.e04;
	sum.e05 += mult * a.e05;
	sum.e06 += mult * a.e06;
	sum.e07 += mult * a.e07;
	sum.e08 += mult * a.e08;
	sum.e09 += mult * a.e09;
	sum.e10 += mult * a.e10;
	sum.e11 += mult * a.e11;
}
//------------------------------------------------------------------------------
__device__ __host__ inline void CopyElementNodesTo_e(Elem12 & e, const real3 * ANCF_NodesD, const real3 * ANCF_SlopesD, int nodeIdx) {
	real3 ni = ANCF_NodesD[nodeIdx];
	e.e00 = ni.x;
	e.e01 = ni.y;
	e.e02 = ni.z;
	real3 si = ANCF_SlopesD[nodeIdx];
	e.e03 = si.x;
	e.e04 = si.y;
	e.e05 = si.z;
	real3 nj = ANCF_NodesD[nodeIdx + 1];
	e.e06 = nj.x;
	e.e07 = nj.y;
	e.e08 = nj.z;
	real3 sj = ANCF_SlopesD[nodeIdx + 1];
	e.e09= sj.x;
	e.e10 = sj.y;
	e.e11 = sj.z;
}
//------------------------------------------------------------------------------
// Why -= and not += : because M*X2 + K*X = F Therefore, in an explicit method: M*X2 = F - K*X where K*X is our elastic forces (f)
__device__ __host__ inline void Add_f_ToForces(real3 * flex_NodesForcesD1, real3 * flex_NodesForcesD2, real_ k, const Elem12 & f, int nodeIdx) {
	flex_NodesForcesD1[nodeIdx		] += k * R3(f.e00, f.e01, f.e02);
	flex_NodesForcesD2[nodeIdx		] += k * R3(f.e03, f.e04, f.e05);
	flex_NodesForcesD1[nodeIdx + 1	] += k * R3(f.e06, f.e07, f.e08);
	flex_NodesForcesD2[nodeIdx + 1	] += k * R3(f.e09, f.e10, f.e11);
}
////------------------------------------------------------------------------------
//__device__ __host__ inline void MapBeamDataTo_1D_Array(real_* e, const real3 * beamData1, const real3 * beamData2, int2 nodesPortion) { //beamData1: postion, beamData2: slope
//	int numNodes = nodesPortion.y - nodesPortion.x;
//	for (int j = 0; j < numNodes; j++) {
//		int nodeIdx = nodesPortion.x + j;
//		real3 data1 = beamData1[nodeIdx];
//		e[6 * j + 0] = data1.x;
//		e[6 * j + 1] = data1.y;
//		e[6 * j + 2] = data1.z;
//		real3 data2 = beamData2[nodeIdx];
//		e[6 * j + 3] = data2.x;
//		e[6 * j + 4] = data2.y;
//		e[6 * j + 5] = data2.z;
//	}
//}

//------------------------------------------------------------------------------
__device__ __host__ inline void CalcElasticForcesKernel(
		real3 * flex_NodesForcesD1,
		real3 * flex_NodesForcesD2,
		const real3 * ANCF_NodesD,
		const real3 * ANCF_SlopesD,
		const real3 * ANCF_NodesVelD,
		const real3 * ANCF_SlopesVelD,
		const int2 * ANCF_ReferenceArrayNodesOnBeamsD,
		const real_ * ANCF_Beam_LengthD,
		const ANCF_Params & flexParams,
		const GaussQuadrature & GQ,
		uint i
	)
{
	real_ l = ANCF_Beam_LengthD[i];
	int2 nodesPortion = ANCF_ReferenceArrayNodesOnBeamsD[i];
	int numNodes = nodesPortion.y - nodesPortion.x;
	int numElements = numNodes - 1;
	real_ lE = l / numElements;
	for (int j = 0; j < numElements; j++) {
		int nodeIdx = nodesPortion.x + j;
		Elem12 e;
		CopyElementNodesTo_e(e, ANCF_NodesD, ANCF_SlopesD, nodeIdx);
		Elem12 f_e, e_ee;
		Zero12(f_e);
		Zero12(e_ee);
		// Elastic Force, 1/2: tension force, GQ 5th order. Maybe 4th order is enough as well.
		for (int k = 0; k < 5; k ++) {
			real_ gqPoint = (lE - 0) / 2 * GQ.GQ5_p[k] + (lE + 0) / 2;
			eps_eps_e(e_ee, gqPoint, lE, e);
			SumElem12(f_e, e_ee, flexParams.E * flexParams.A * (lE - 0) / 2 * GQ.GQ5_w[k]);
		}
		// Elastic Force, 2/2: bending force, GQ 3rd order.
		Elem12 k_ke;
		Zero12(k_ke);
		for (int k = 0; k < 3; k ++) {
			real_ gqPoint = (lE - 0) / 2 * GQ.GQ3_p[k] + (lE + 0) / 2;
			kappa_kappa_e(k_ke, gqPoint, lE, e);
			SumElem12(f_e, k_ke, flexParams.E * flexParams.I  * (lE - 0) / 2 * GQ.GQ3_w[k]);
		}
		// Gravitational Foce
		Elem12 f_g;
		Zero12(f_g);
		gravitational_force(f_g, lE, flexParams.rho, flexParams.A, flexParams.gravity);
		SumElem12(f_e, f_g, -1);
		// Add element forces to associated nodes
		Add_f_ToForces(flex_NodesForcesD1, flex_NodesForcesD2, -1, f_e, nodeIdx);
	}
}
//------------------------------------------------------------------------------
__GLOBAL__ void CalcElasticForcesD(
		real3 * flex_NodesForcesD1,
		real3 * flex_NodesForcesD2,
		const real3 * ANCF_NodesD,
		const real3 * ANCF_SlopesD,
		const real3 * ANCF_NodesVelD,
		const real3 * ANCF_SlopesVelD,
		const int2 * ANCF_ReferenceArrayNodesOnBeamsD,
		const real_ * ANCF_Beam_LengthD
	)
{
#if flexGPU
	uint i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= numFlexBodiesD) return;
#else
	for (int i = 0; i < numFlexBodiesD; i++) {
#endif
		CalcElasticForcesKernel(
				flex_NodesForcesD1, flex_NodesForcesD2,
				ANCF_NodesD, ANCF_SlopesD, ANCF_NodesVelD, ANCF_SlopesVelD,
				ANCF_ReferenceArrayNodesOnBeamsD, ANCF_Beam_LengthD, flexParamsD, GQD, i);
#if !flexGPU
	}
#endif
}
//------------------------------------------------------------------------------
__device__ __host__ inline void SolveForAccKernel(
		real3 * ANCF_NodesAccD,
		real3 * ANCF_SlopesAccD,

		const real3 * flex_NodesForcesD1,
		const real3 * flex_NodesForcesD2,

		const int2 * ANCF_ReferenceArrayNodesOnBeamsD,
		const real_ * ANCF_Beam_LengthD,
		const bool * ANCF_IsCantileverD,
		const ANCF_Params & flexParams,
		uint i
	)
{
	int2 nodesPortion = ANCF_ReferenceArrayNodesOnBeamsD[i];
	int numNodes = nodesPortion.y - nodesPortion.x;
	int numElements = numNodes - 1;
	real_ lBeam = ANCF_Beam_LengthD[i];
	real_ lE = lBeam / numElements;
	bool isCantilever = ANCF_IsCantileverD[i];
	real_ mult = (1/(flexParams.rho * flexParams.A * lE));

	min_vec(
			ANCF_NodesAccD,
			ANCF_SlopesAccD,
			flex_NodesForcesD1,
			flex_NodesForcesD2,
			mult,
			lE,
			nodesPortion,
			isCantilever);
}
//------------------------------------------------------------------------------
__GLOBAL__ void SolveForAccD(
		real3 * ANCF_NodesAccD,
		real3 * ANCF_SlopesAccD,

		const real3 * flex_NodesForcesD1,
		const real3 * flex_NodesForcesD2,

		const int2 * ANCF_ReferenceArrayNodesOnBeamsD,
		const real_ * ANCF_Beam_LengthD,
		const bool * ANCF_IsCantileverD
	)
{
#if flexGPU
	uint i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= numFlexBodiesD) return;
#else
	for (int i = 0; i < numFlexBodiesD; i++) {
#endif

		SolveForAccKernel(ANCF_NodesAccD, ANCF_SlopesAccD,
				flex_NodesForcesD1, flex_NodesForcesD2,
				ANCF_ReferenceArrayNodesOnBeamsD, ANCF_Beam_LengthD, ANCF_IsCantileverD, flexParamsD, i);

#if !flexGPU
	}
#endif

}
//------------------------------------------------------------------------------
__device__ __host__ inline void IntegrateInTimeKernel(
		real3 * ANCF_NodesD2,
		real3 * ANCF_SlopesD2,
		real3 * ANCF_NodesVelD2,
		real3 * ANCF_SlopesVelD2,

		const real3 * ANCF_NodesVelD,
		const real3 * ANCF_SlopesVelD,
		const real3 * ANCF_NodesAccD,
		const real3 * ANCF_SlopesAccD,

		const int2 * ANCF_ReferenceArrayNodesOnBeamsD,
		const bool * ANCF_IsCantileverD,
		real_ dT,
		uint i
	)
{
	int2 nodesPortion = ANCF_ReferenceArrayNodesOnBeamsD[i];
	int2 nodesPortionAdjusted2 = nodesPortion;
	bool isCantilever = ANCF_IsCantileverD[i];
	if (isCantilever) {
		nodesPortionAdjusted2.x = nodesPortionAdjusted2.x + 1;
	}

	int numNodesAdjusted = nodesPortionAdjusted2.y - nodesPortionAdjusted2.x;
	for (int j = 0; j < numNodesAdjusted; j++) {
		int nodeIdx = nodesPortionAdjusted2.x + j;
		ANCF_NodesD2[nodeIdx] += dT * ANCF_NodesVelD[nodeIdx];
		ANCF_SlopesD2[nodeIdx] += dT * ANCF_SlopesVelD[nodeIdx];

		ANCF_NodesVelD2[nodeIdx] += dT * ANCF_NodesAccD[nodeIdx];
		ANCF_SlopesVelD2[nodeIdx] += dT * ANCF_SlopesAccD[nodeIdx];
	}
}
//------------------------------------------------------------------------------
__GLOBAL__ void IntegrateInTimeD(
		real3 * ANCF_NodesD2,
		real3 * ANCF_SlopesD2,
		real3 * ANCF_NodesVelD2,
		real3 * ANCF_SlopesVelD2,

		const real3 * ANCF_NodesVelD,
		const real3 * ANCF_SlopesVelD,
		const real3 * ANCF_NodesAccD,
		const real3 * ANCF_SlopesAccD,

		const int2 * ANCF_ReferenceArrayNodesOnBeamsD,
		const bool * ANCF_IsCantileverD
	)
{
#if flexGPU
	uint i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= numFlexBodiesD) return;
#else
	for (int i = 0; i < numFlexBodiesD; i++) {
#endif
		IntegrateInTimeKernel(
				ANCF_NodesD2, ANCF_SlopesD2, ANCF_NodesVelD2, ANCF_SlopesVelD2,
				ANCF_NodesVelD, ANCF_SlopesVelD, ANCF_NodesAccD, ANCF_SlopesAccD,
				ANCF_ReferenceArrayNodesOnBeamsD, ANCF_IsCantileverD, dTD, i	);
#if !flexGPU
	}
#endif

}
//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
// Gaussian Quadrature. Applied to the [-1, 1] interval. For other intervals it is like this:
// int_a^b f(x)dx = (b-a)/2 * sum{w_i * f( (b-a)/2*z_i + (a+b)/2 ) }
void IntitializeGaussQuadrature(GaussQuadrature & GQ) {
	GQ.GQ3_p[0] = -0.774596669241483; 	GQ.GQ3_p[1] = 0; 					GQ.GQ3_p[2] = 0.774596669241483;
	GQ.GQ3_w[0] = 0.555555555555556; 	GQ.GQ3_w[1] = 0.888888888888889; 	GQ.GQ3_w[2] = 0.555555555555556;

	GQ.GQ4_p[0] = -0.861136311594053; 	GQ.GQ4_p[1] = -0.339981043584856; 	GQ.GQ4_p[2] = 0.339981043584856;	GQ.GQ4_p[3] = 0.861136311594053;
	GQ.GQ4_w[0] = 0.347854845137454;	GQ.GQ4_w[1] = 0.652145154862546;	GQ.GQ4_w[2] = 0.652145154862546;	GQ.GQ4_w[3] = 0.347854845137454;

	GQ.GQ5_p[0] = -0.906179845938664;	GQ.GQ5_p[1] = -0.538469310105683;	GQ.GQ5_p[2] = 0;					GQ.GQ5_p[3] = 0.538469310105683;	GQ.GQ5_p[4] = 0.906179845938664;
	GQ.GQ5_w[0] = 0.236926885056189;	GQ.GQ5_w[1] = 0.478628670499366;	GQ.GQ5_w[2] = 0.568888888888889;	GQ.GQ5_w[3] = 0.478628670499366;	GQ.GQ5_w[4] = 0.236926885056189;
}
//------------------------------------------------------------------------------
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

		const thrust::device_vector<real3> & flex_FSI_NodesForcesD1,
		const thrust::device_vector<real3> & flex_FSI_NodesForcesD2,
		const thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,
		const thrust::device_vector<real_> & ANCF_Beam_LengthD,
		const thrust::device_vector<bool> & ANCF_IsCantileverD,

		const int numFlexBodies,
		const ANCF_Params & flexParams,
		real_ dT)
{
	thrust::device_vector<real3> flex_NodesForcesD1 = flex_FSI_NodesForcesD1;
	thrust::device_vector<real3> flex_NodesForcesD2 = flex_FSI_NodesForcesD2;


	//---------------------------------------------
	int totalNumberOfFlexNodes = flex_NodesForcesD1.size();
	thrust::device_vector<real3> ANCF_NodesAccD(totalNumberOfFlexNodes);
	thrust::device_vector<real3> ANCF_SlopesAccD(totalNumberOfFlexNodes);
	thrust::fill(ANCF_NodesAccD.begin(), ANCF_NodesAccD.end(), R3(0));
	thrust::fill(ANCF_SlopesAccD.begin(), ANCF_SlopesAccD.end(), R3(0));
	//---------------------------------------------
	GaussQuadrature GQ;
	IntitializeGaussQuadrature(GQ);
	//---------------------------------------------
#if flexGPU
	cutilSafeCall( cudaMemcpyToSymbolAsync(flexParamsD, &flexParams, sizeof(ANCF_Params)));
	cutilSafeCall( cudaMemcpyToSymbolAsync(dTD, &dT, sizeof(real_)));
	cutilSafeCall( cudaMemcpyToSymbolAsync(numFlexBodiesD, &numFlexBodies, sizeof(int))); //This can be updated in the future, to work with numObjectsD
	cutilSafeCall( cudaMemcpyToSymbolAsync(GQD, &GQ, sizeof(GaussQuadrature))); //This can be updated in the future, to work with numObjects
#else

	flexParamsD = flexParams;
	dTD = dT;
	numFlexBodiesD = numFlexBodies;
	GQD = GQ;
#endif

	//---------------------------------------------
	//####### Calculate Forces
	uint nBlock_FlexBodies;
	uint nThreads_FlexBodies;
	computeGridSize(numFlexBodies, 128, nBlock_FlexBodies, nThreads_FlexBodies);

#if flexGPU
	CalcElasticForcesD __KERNEL__(nBlock_FlexBodies, nThreads_FlexBodies)(
			R3CAST(flex_NodesForcesD1), R3CAST(flex_NodesForcesD2),
			R3CAST(ANCF_NodesD), R3CAST(ANCF_SlopesD), R3CAST(ANCF_NodesVelD), R3CAST(ANCF_SlopesVelD),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsD), R1CAST(ANCF_Beam_LengthD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: CalcElasticForcesD");

	SolveForAccD __KERNEL__(nBlock_FlexBodies, nThreads_FlexBodies)(
			R3CAST(ANCF_NodesAccD), R3CAST(ANCF_SlopesAccD),
			R3CAST(flex_NodesForcesD1), R3CAST(flex_NodesForcesD2),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsD), R1CAST(ANCF_Beam_LengthD), BCAST(ANCF_IsCantileverD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: SolveAndIntegrateInTime");

	IntegrateInTimeD __KERNEL__(nBlock_FlexBodies, nThreads_FlexBodies)(
			R3CAST(ANCF_NodesD2), R3CAST(ANCF_SlopesD2), R3CAST(ANCF_NodesVelD2), R3CAST(ANCF_SlopesVelD2),
			R3CAST(ANCF_NodesVelD), R3CAST(ANCF_SlopesVelD), R3CAST(ANCF_NodesAccD), R3CAST(ANCF_SlopesAccD),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsD), BCAST(ANCF_IsCantileverD));
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: IntegrateInTimeD");
#else
	thrust::host_vector<real3> ANCF_NodesH2 = ANCF_NodesD2;
	thrust::host_vector<real3> ANCF_SlopesH2 = ANCF_SlopesD2;
	thrust::host_vector<real3> ANCF_NodesVelH2 = ANCF_NodesVelD2;
	thrust::host_vector<real3> ANCF_SlopesVelH2 = ANCF_SlopesVelD2;

	thrust::host_vector<real3> ANCF_NodesH = ANCF_NodesD;
	thrust::host_vector<real3> ANCF_SlopesH = ANCF_SlopesD;
	thrust::host_vector<real3> ANCF_NodesVelH = ANCF_NodesVelD;
	thrust::host_vector<real3> ANCF_SlopesVelH = ANCF_SlopesVelD;

	thrust::host_vector<real3> flex_NodesForcesH1 = flex_FSI_NodesForcesD1;
	thrust::host_vector<real3> flex_NodesForcesH2 = flex_FSI_NodesForcesD2;
	thrust::host_vector<int2>  ANCF_ReferenceArrayNodesOnBeamsH = ANCF_ReferenceArrayNodesOnBeamsD;
	thrust::host_vector<real_> ANCF_Beam_LengthH = ANCF_Beam_LengthD;
	thrust::host_vector<bool> ANCF_IsCantileverH = ANCF_IsCantileverD;

	thrust::host_vector<real3> ANCF_NodesAccH = ANCF_NodesAccD;
	thrust::host_vector<real3> ANCF_SlopesAccH = ANCF_SlopesAccD;
	//------------------ CPU code should work on CPU data
	CalcElasticForcesD(
			R3CAST(flex_NodesForcesH1), R3CAST(flex_NodesForcesH2),
			R3CAST(ANCF_NodesH), R3CAST(ANCF_SlopesH), R3CAST(ANCF_NodesVelH), R3CAST(ANCF_SlopesVelH),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsH), R1CAST(ANCF_Beam_LengthH));

	SolveForAccD(
			R3CAST(ANCF_NodesAccH), R3CAST(ANCF_SlopesAccH),
			R3CAST(flex_NodesForcesH1), R3CAST(flex_NodesForcesH2),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsH), R1CAST(ANCF_Beam_LengthH), BCAST(ANCF_IsCantileverH));

	IntegrateInTimeD(
			R3CAST(ANCF_NodesH2), R3CAST(ANCF_SlopesH2), R3CAST(ANCF_NodesVelH2), R3CAST(ANCF_SlopesVelH2),
			R3CAST(ANCF_NodesVelH), R3CAST(ANCF_SlopesVelH), R3CAST(ANCF_NodesAccH), R3CAST(ANCF_SlopesAccH),
			I2CAST(ANCF_ReferenceArrayNodesOnBeamsH), BCAST(ANCF_IsCantileverH));

	//---------------------------------------------
	thrust::copy(ANCF_NodesH2.begin(), ANCF_NodesH2.end(), ANCF_NodesD2.begin());
	thrust::copy(ANCF_SlopesH2.begin(), ANCF_SlopesH2.end(), ANCF_SlopesD2.begin());
	thrust::copy(ANCF_NodesVelH2.begin(), ANCF_NodesVelH2.end(), ANCF_NodesVelD2.begin());
	thrust::copy(ANCF_SlopesVelH2.begin(), ANCF_SlopesVelH2.end(), ANCF_SlopesVelD2.begin());
	//---------------------------------------------
	//------------------ CPU code should work on CPU data
	ANCF_NodesH2.clear();
	ANCF_SlopesH2.clear();
	ANCF_NodesVelH2.clear();
	ANCF_SlopesVelH2.clear();

	ANCF_NodesH.clear();
	ANCF_SlopesH.clear();
	ANCF_NodesVelH.clear();
	ANCF_SlopesVelH.clear();

	flex_NodesForcesH1.clear();
	flex_NodesForcesH2.clear();
	ANCF_ReferenceArrayNodesOnBeamsH.clear();
	ANCF_Beam_LengthH.clear();
	ANCF_IsCantileverH.clear();

	ANCF_NodesAccH.clear();
	ANCF_SlopesAccH.clear();
#endif


	flex_NodesForcesD1.clear();
	flex_NodesForcesD2.clear();

	ANCF_NodesAccD.clear();
	ANCF_SlopesAccD.clear();

}
