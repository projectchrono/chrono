// This function calculates edd = Mhat^(-1)*v for a "noodle"
// made up of 3 ANCF identical beam elements.
//
// As such, the input vector 'v' and the output vector 'edd'
// are assumed to be of length (3+1)*6 = 24
//
// Note that the matrix Mhat used for this calculation is just
//    Mhat = \int_0^1 { S^T(\xi) S(\xi) d \xi } 
// As such, the result must be muliplied by 1/(rho * A * lE)
// where lE is element length: lE = L / numElements
// Created by Arman Pazouki apazouki@gmail.com

//
#include <stdio.h>
#include "SPHCudaUtils.h"


#ifndef MINV_VEC_CUH
#define MINV_VEC_CUH


__device__ __host__ inline void minv_vec_1(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_2(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_3(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_4(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_5(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_weld_1(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_weld_2(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_weld_3(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_weld_4(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_weld_5(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_fix_1(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_fix_2(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_fix_3(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_fix_4(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);
__device__ __host__ inline void minv_vec_fix_5(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion);


// mult = (1/(flexParamsD.rho * flexParamsD.A * lE))
__device__ __host__ inline void min_vec(
		real3 * ANCF_NodesAccD,
		real3 * ANCF_SlopesAccD,
		const real3 * flex_FSI_NodesForcesD1,
		const real3 * flex_FSI_NodesForcesD2,
		real_ mult,
		real_ lE,
		const int2 & nodesPortion,
		bool isCantilever)
{
	int nE = nodesPortion.y - nodesPortion.x;
	if (isCantilever) {
		switch(nE) {
		case 1: minv_vec_weld_1(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		case 2: minv_vec_weld_2(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		case 3: minv_vec_weld_3(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		case 4: minv_vec_weld_4(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		case 5: minv_vec_weld_5(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		default: printf("Error!!: Number of beam elements equal to %d is not supported. Code exits!\n"); break;
		}
	} else {
		switch(nE) {
		case 1: minv_vec_1(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		case 2: minv_vec_2(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		case 3: minv_vec_3(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		case 4: minv_vec_4(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		case 5: minv_vec_5(ANCF_NodesAccD, ANCF_SlopesAccD, flex_FSI_NodesForcesD1, flex_FSI_NodesForcesD2, mult, lE, nodesPortion); break;
		default: printf("Error!!: Number of beam elements equal to %d is not supported. Code exits!\n"); break;
		}
	}
}
//******************* Functions Implementations ******************************
__device__ __host__ inline void minv_vec_1(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p0 = f1[nodesPortion.x + 0];
	real_ v1  = p0.x;
	real_ v2  = p0.y;
	real_ v3  = p0.z;
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v4  = s0.x;
	real_ v5  = s0.y;
	real_ v6  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v7  = p1.x;
	real_ v8  = p1.y;
	real_ v9  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v10  = s1.x;
	real_ v11  = s1.y;
	real_ v12  = s1.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 dp0, ds0;
	dp0.x = v1*1.6E1-v7*4.0-t2*(v4*2.0+v10)*6.0E1;
	dp0.y = v2*1.6E1-v8*4.0-t2*(v5*2.0+v11)*6.0E1;
	dp0.z = v3*1.6E1-v9*4.0-t2*(v6*2.0+v12)*6.0E1;
	ds0.x = -t2*(v1*1.2E2-v7*6.0E1)+t3*(v4*1.2E3+v10*8.4E2);
	ds0.y = -t2*(v2*1.2E2-v8*6.0E1)+t3*(v5*1.2E3+v11*8.4E2);
	ds0.z = -t2*(v3*1.2E2-v9*6.0E1)+t3*(v6*1.2E3+v12*8.4E2);
	d2_1[nodesPortion.x + 0] = mult * dp0;
	d2_2[nodesPortion.x + 0] = mult * ds0;

	real3 dp1, ds1;
	dp1.x = v1*-4.0+v7*1.6E1+t2*(v4+v10*2.0)*6.0E1;
	dp1.y = v2*-4.0+v8*1.6E1+t2*(v5+v11*2.0)*6.0E1;
	dp1.z = v3*-4.0+v9*1.6E1+t2*(v6+v12*2.0)*6.0E1;
	ds1.x = -t2*(v1*6.0E1-v7*1.2E2)+t3*(v4*8.4E2+v10*1.2E3);
	ds1.y = -t2*(v2*6.0E1-v8*1.2E2)+t3*(v5*8.4E2+v11*1.2E3);
	ds1.z = -t2*(v3*6.0E1-v9*1.2E2)+t3*(v6*8.4E2+v12*1.2E3);
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;
}

// Same as above for a 2-element beam
__device__ __host__ inline void minv_vec_2(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p0 = f1[nodesPortion.x + 0];
	real_ v1  = p0.x;
	real_ v2  = p0.y;
	real_ v3  = p0.z;
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v4  = s0.x;
	real_ v5  = s0.y;
	real_ v6  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v7  = p1.x;
	real_ v8  = p1.y;
	real_ v9  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v10  = s1.x;
	real_ v11  = s1.y;
	real_ v12  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v13  = p2.x;
	real_ v14  = p2.y;
	real_ v15  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v16  = s2.x;
	real_ v17  = s2.y;
	real_ v18  = s2.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);
	real_ t4 = v10*3.0E1;
	real_ t5 = v11*3.0E1;
	real_ t6 = v12*3.0E1;
	real_ t7 = v7*1.2E1;
	real_ t8 = v10*1.95E2;
	real_ t9 = v8*1.2E1;
	real_ t10 = v11*1.95E2;
	real_ t11 = v9*1.2E1;
	real_ t12 = v12*1.95E2;

	real3 dp0, ds0;
	dp0.x = v1*1.4E1+v7-v13-t2*(t4+v4*1.83E2+v16*2.7E1)*(1.0/2.0);
	dp0.y = v2*1.4E1+v8-v14-t2*(t5+v5*1.83E2+v17*2.7E1)*(1.0/2.0);
	dp0.z = v3*1.4E1+v9-v15-t2*(t6+v6*1.83E2+v18*2.7E1)*(1.0/2.0);
	ds0.x = -t2*(t7+v1*(1.83E2/2.0)-v13*(2.7E1/2.0))+t3*(t8+v4*7.935E2+v16*(3.63E2/2.0));
	ds0.y = -t2*(t9+v2*(1.83E2/2.0)-v14*(2.7E1/2.0))+t3*(t10+v5*7.935E2+v17*(3.63E2/2.0));
	ds0.z = -t2*(t11+v3*(1.83E2/2.0)-v15*(2.7E1/2.0))+t3*(t12+v6*7.935E2+v18*(3.63E2/2.0));
	d2_1[nodesPortion.x + 0] = mult * dp0;
	d2_2[nodesPortion.x + 0] = mult * ds0;

	real3 dp1, ds1;
	dp1.x = v1+v7*2.0+v13-t2*(v4-v16)*1.2E1;
	dp1.y = v2+v8*2.0+v14-t2*(v5-v17)*1.2E1;
	dp1.z = v3+v9*2.0+v15-t2*(v6-v18)*1.2E1;
	ds1.x = t3*(v4*1.95E2+v10*1.5E2+v16*1.95E2)-t2*(v1*1.5E1-v13*1.5E1);
	ds1.y = t3*(v5*1.95E2+v11*1.5E2+v17*1.95E2)-t2*(v2*1.5E1-v14*1.5E1);
	ds1.z = t3*(v6*1.95E2+v12*1.5E2+v18*1.95E2)-t2*(v3*1.5E1-v15*1.5E1);
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = -v1+v7+v13*1.4E1+t2*(t4+v4*2.7E1+v16*1.83E2)*(1.0/2.0);
	dp2.y = -v2+v8+v14*1.4E1+t2*(t5+v5*2.7E1+v17*1.83E2)*(1.0/2.0);
	dp2.z = -v3+v9+v15*1.4E1+t2*(t6+v6*2.7E1+v18*1.83E2)*(1.0/2.0);
	ds2.x = t2*(t7-v1*(2.7E1/2.0)+v13*(1.83E2/2.0))+t3*(t8+v4*(3.63E2/2.0)+v16*7.935E2);
	ds2.y = t2*(t9-v2*(2.7E1/2.0)+v14*(1.83E2/2.0))+t3*(t10+v5*(3.63E2/2.0)+v17*7.935E2);
	ds2.z = t2*(t11-v3*(2.7E1/2.0)+v15*(1.83E2/2.0))+t3*(t12+v6*(3.63E2/2.0)+v18*7.935E2);
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;
}

__device__ __host__ inline void minv_vec_3(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p0 = f1[nodesPortion.x + 0];
	real_ v1  = p0.x;
	real_ v2  = p0.y;
	real_ v3  = p0.z;
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v4  = s0.x;
	real_ v5  = s0.y;
	real_ v6  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v7  = p1.x;
	real_ v8  = p1.y;
	real_ v9  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v10  = s1.x;
	real_ v11  = s1.y;
	real_ v12  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v13  = p2.x;
	real_ v14  = p2.y;
	real_ v15  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v16  = s2.x;
	real_ v17  = s2.y;
	real_ v18  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v19  = p3.x;
	real_ v20  = p3.y;
	real_ v21  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v22  = s3.x;
	real_ v23  = s3.y;
	real_ v24  = s3.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 dp0, ds0;
	dp0.x = t2*(v4*5.8745E5+v10*8.4965E4+v16*2.609E4+v22*2.4605E4-lE*v1*9.0852E4-lE*v7*7.397E3-lE*v13*1.552E3+lE*v19*1.833E3)*(-1.525611197986193E-4);
	dp0.y = t2*(v5*5.8745E5+v11*8.4965E4+v17*2.609E4+v23*2.4605E4-lE*v2*9.0852E4-lE*v8*7.397E3-lE*v14*1.552E3+lE*v20*1.833E3)*(-1.525611197986193E-4);
	dp0.z = t2*(v6*5.8745E5+v12*8.4965E4+v18*2.609E4+v24*2.4605E4-lE*v3*9.0852E4-lE*v9*7.397E3-lE*v15*1.552E3+lE*v21*1.833E3)*(-1.525611197986193E-4);
	ds0.x = t3*(v4*1.0071E6+v10*2.19678E5+v16*6.99E4+v22*6.6042E4-lE*v1*1.1749E5-lE*v7*1.7999E4-lE*v13*4.138E3+lE*v19*4.921E3)*7.628055989930966E-4;
	ds0.y = t3*(v5*1.0071E6+v11*2.19678E5+v17*6.99E4+v23*6.6042E4-lE*v2*1.1749E5-lE*v8*1.7999E4-lE*v14*4.138E3+lE*v20*4.921E3)*7.628055989930966E-4;
	ds0.z = t3*(v6*1.0071E6+v12*2.19678E5+v18*6.99E4+v24*6.6042E4-lE*v3*1.1749E5-lE*v9*1.7999E4-lE*v15*4.138E3+lE*v21*4.921E3)*7.628055989930966E-4;
	d2_1[nodesPortion.x + 0] = mult * dp0;
	d2_2[nodesPortion.x + 0] = mult * ds0;

	real3 dp1, ds1;
	dp1.x = t2*(v4*-8.9995E4-v10*1.231E4+v16*2.066E4+v22*2.069E4+lE*v1*7.397E3+lE*v7*1.2332E4-lE*v13*1.028E3+lE*v19*1.552E3)*1.525611197986193E-4;
	dp1.y = t2*(v5*-8.9995E4-v11*1.231E4+v17*2.066E4+v23*2.069E4+lE*v2*7.397E3+lE*v8*1.2332E4-lE*v14*1.028E3+lE*v20*1.552E3)*1.525611197986193E-4;
	dp1.z = t2*(v6*-8.9995E4-v12*1.231E4+v18*2.066E4+v24*2.069E4+lE*v3*7.397E3+lE*v9*1.2332E4-lE*v15*1.028E3+lE*v21*1.552E3)*1.525611197986193E-4;
	ds1.x = t3*(v4*2.19678E5+v10*1.5762E5+v16*7.284E4+v22*6.99E4-lE*v1*1.6993E4-lE*v7*2.462E3-lE*v13*4.132E3+lE*v19*5.218E3)*7.628055989930966E-4;
	ds1.y = t3*(v5*2.19678E5+v11*1.5762E5+v17*7.284E4+v23*6.99E4-lE*v2*1.6993E4-lE*v8*2.462E3-lE*v14*4.132E3+lE*v20*5.218E3)*7.628055989930966E-4;
	ds1.z = t3*(v6*2.19678E5+v12*1.5762E5+v18*7.284E4+v24*6.99E4-lE*v3*1.6993E4-lE*v9*2.462E3-lE*v15*4.132E3+lE*v21*5.218E3)*7.628055989930966E-4;
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t2*(v4*-2.069E4-v10*2.066E4+v16*1.231E4+v22*8.9995E4+lE*v1*1.552E3-lE*v7*1.028E3+lE*v13*1.2332E4+lE*v19*7.397E3)*1.525611197986193E-4;
	dp2.y = t2*(v5*-2.069E4-v11*2.066E4+v17*1.231E4+v23*8.9995E4+lE*v2*1.552E3-lE*v8*1.028E3+lE*v14*1.2332E4+lE*v20*7.397E3)*1.525611197986193E-4;
	dp2.z = t2*(v6*-2.069E4-v12*2.066E4+v18*1.231E4+v24*8.9995E4+lE*v3*1.552E3-lE*v9*1.028E3+lE*v15*1.2332E4+lE*v21*7.397E3)*1.525611197986193E-4;
	ds2.x = t3*(v4*6.99E4+v10*7.284E4+v16*1.5762E5+v22*2.19678E5-lE*v1*5.218E3+lE*v7*4.132E3+lE*v13*2.462E3+lE*v19*1.6993E4)*7.628055989930966E-4;
	ds2.y = t3*(v5*6.99E4+v11*7.284E4+v17*1.5762E5+v23*2.19678E5-lE*v2*5.218E3+lE*v8*4.132E3+lE*v14*2.462E3+lE*v20*1.6993E4)*7.628055989930966E-4;
	ds2.z = t3*(v6*6.99E4+v12*7.284E4+v18*1.5762E5+v24*2.19678E5-lE*v3*5.218E3+lE*v9*4.132E3+lE*v15*2.462E3+lE*v21*1.6993E4)*7.628055989930966E-4;
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = t2*(v4*2.4605E4+v10*2.609E4+v16*8.4965E4+v22*5.8745E5-lE*v1*1.833E3+lE*v7*1.552E3+lE*v13*7.397E3+lE*v19*9.0852E4)*1.525611197986193E-4;
	dp3.y = t2*(v5*2.4605E4+v11*2.609E4+v17*8.4965E4+v23*5.8745E5-lE*v2*1.833E3+lE*v8*1.552E3+lE*v14*7.397E3+lE*v20*9.0852E4)*1.525611197986193E-4;
	dp3.z = t2*(v6*2.4605E4+v12*2.609E4+v18*8.4965E4+v24*5.8745E5-lE*v3*1.833E3+lE*v9*1.552E3+lE*v15*7.397E3+lE*v21*9.0852E4)*1.525611197986193E-4;
	ds3.x = t3*(v4*6.6042E4+v10*6.99E4+v16*2.19678E5+v22*1.0071E6-lE*v1*4.921E3+lE*v7*4.138E3+lE*v13*1.7999E4+lE*v19*1.1749E5)*7.628055989930966E-4;
	ds3.y = t3*(v5*6.6042E4+v11*6.99E4+v17*2.19678E5+v23*1.0071E6-lE*v2*4.921E3+lE*v8*4.138E3+lE*v14*1.7999E4+lE*v20*1.1749E5)*7.628055989930966E-4;
	ds3.z = t3*(v6*6.6042E4+v12*6.99E4+v18*2.19678E5+v24*1.0071E6-lE*v3*4.921E3+lE*v9*4.138E3+lE*v15*1.7999E4+lE*v21*1.1749E5)*7.628055989930966E-4;
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;
}

// Same as above, but for a "noodle" made up of 4 ANCF beam elements
// In this case, 'v' and 'edd' are of length (4+1)*6 = 30
//
__device__ __host__ inline void minv_vec_4(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p0 = f1[nodesPortion.x + 0];
	real_ v1  = p0.x;
	real_ v2  = p0.y;
	real_ v3  = p0.z;
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v4  = s0.x;
	real_ v5  = s0.y;
	real_ v6  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v7  = p1.x;
	real_ v8  = p1.y;
	real_ v9  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v10  = s1.x;
	real_ v11  = s1.y;
	real_ v12  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v13  = p2.x;
	real_ v14  = p2.y;
	real_ v15  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v16  = s2.x;
	real_ v17  = s2.y;
	real_ v18  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v19  = p3.x;
	real_ v20  = p3.y;
	real_ v21  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v22  = s3.x;
	real_ v23  = s3.y;
	real_ v24  = s3.z;

	real3 p4 = f1[nodesPortion.x + 4];
	real_ v25  = p4.x;
	real_ v26  = p4.y;
	real_ v27  = p4.z;
	real3 s4 = f2[nodesPortion.x + 4];
	real_ v28  = s4.x;
	real_ v29  = s4.y;
	real_ v30  = s4.z;


	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real_ t4 = v16*8.0937E4;
	real_ t5 = v17*8.0937E4;
	real_ t6 = v18*8.0937E4;
	real_ t7 = v16*9.5749E4;
	real_ t8 = v17*9.5749E4;
	real_ t9 = v18*9.5749E4;
	real_ t10 = v16*1.03155E5;
	real_ t11 = v17*1.03155E5;
	real_ t12 = v18*1.03155E5;
	real_ t13 = v16*9.20989E5;
	real_ t14 = v17*9.20989E5;
	real_ t15 = v18*9.20989E5;

	real3 dp0, ds0;
	dp0.x = t2*(t10+v4*2.650575E6+v10*3.7935E5+v22*3.327E4+v28*3.1455E4-lE*v1*4.10276E5-lE*v7*3.3706E4-lE*v13*8.246E3-lE*v19*1.966E3+lE*v25*2.344E3)*(-3.375641371860654E-5);
	dp0.y = t2*(t11+v5*2.650575E6+v11*3.7935E5+v23*3.327E4+v29*3.1455E4-lE*v2*4.10276E5-lE*v8*3.3706E4-lE*v14*8.246E3-lE*v20*1.966E3+lE*v26*2.344E3)*(-3.375641371860654E-5);
	dp0.z = t2*(t12+v6*2.650575E6+v12*3.7935E5+v24*3.327E4+v30*3.1455E4-lE*v3*4.10276E5-lE*v9*3.3706E4-lE*v15*8.246E3-lE*v21*1.966E3+lE*v27*2.344E3)*(-3.375641371860654E-5);
	ds0.x = t3*(t13+v4*1.5132601E7+v10*3.26785E6+v22*2.9761E5+v28*2.81401E5-lE*v1*1.76705E6-lE*v7*2.73618E5-lE*v13*7.336E4-lE*v19*1.7582E4+lE*v25*2.097E4)*5.06346205779098E-5;
	ds0.y = t3*(t14+v5*1.5132601E7+v11*3.26785E6+v23*2.9761E5+v29*2.81401E5-lE*v2*1.76705E6-lE*v8*2.73618E5-lE*v14*7.336E4-lE*v20*1.7582E4+lE*v26*2.097E4)*5.06346205779098E-5;
	ds0.z = t3*(t15+v6*1.5132601E7+v12*3.26785E6+v24*2.9761E5+v30*2.81401E5-lE*v3*1.76705E6-lE*v9*2.73618E5-lE*v15*7.336E4-lE*v21*1.7582E4+lE*v27*2.097E4)*5.06346205779098E-5;
	d2_1[nodesPortion.x + 0] = mult * dp0;
	d2_2[nodesPortion.x + 0] = mult * ds0;

	real3 dp1, ds1;
	dp1.x = t2*(t4-v4*4.10427E5-v10*5.955E4+v22*2.781E4+v28*2.6373E4+lE*v1*3.3706E4+lE*v7*5.5502E4-lE*v13*5.684E3-lE*v19*1.63E3+lE*v25*1.966E3)*3.375641371860654E-5;
	dp1.y = t2*(t5-v5*4.10427E5-v11*5.955E4+v23*2.781E4+v29*2.6373E4+lE*v2*3.3706E4+lE*v8*5.5502E4-lE*v14*5.684E3-lE*v20*1.63E3+lE*v26*1.966E3)*3.375641371860654E-5;
	dp1.z = t2*(t6-v6*4.10427E5-v12*5.955E4+v24*2.781E4+v30*2.6373E4+lE*v3*3.3706E4+lE*v9*5.5502E4-lE*v15*5.684E3-lE*v21*1.63E3+lE*v27*1.966E3)*3.375641371860654E-5;
	ds1.x = t3*(t7+v4*3.26785E5+v10*2.3305E5+v22*3.145E4+v28*2.9761E4-lE*v1*2.529E4-lE*v7*3.97E3-lE*v13*7.392E3-lE*v19*1.854E3+lE*v25*2.218E3)*5.06346205779098E-4;
	ds1.y = t3*(t8+v5*3.26785E5+v11*2.3305E5+v23*3.145E4+v29*2.9761E4-lE*v2*2.529E4-lE*v8*3.97E3-lE*v14*7.392E3-lE*v20*1.854E3+lE*v26*2.218E3)*5.06346205779098E-4;
	ds1.z = t3*(t9+v6*3.26785E5+v12*2.3305E5+v24*3.145E4+v30*2.9761E4-lE*v3*2.529E4-lE*v9*3.97E3-lE*v15*7.392E3-lE*v21*1.854E3+lE*v27*2.218E3)*5.06346205779098E-4;
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t2*(v4*-7.86E3-v10*7.92E3+v22*7.92E3+v28*7.86E3+lE*v1*5.89E2-lE*v7*4.06E2+lE*v13*3.649E3-lE*v19*4.06E2+lE*v25*5.89E2)*4.725897920604915E-4;
	dp2.y = t2*(v5*-7.86E3-v11*7.92E3+v23*7.92E3+v29*7.86E3+lE*v2*5.89E2-lE*v8*4.06E2+lE*v14*3.649E3-lE*v20*4.06E2+lE*v26*5.89E2)*4.725897920604915E-4;
	dp2.z = t2*(v6*-7.86E3-v12*7.92E3+v24*7.92E3+v30*7.86E3+lE*v3*5.89E2-lE*v9*4.06E2+lE*v15*3.649E3-lE*v21*4.06E2+lE*v27*5.89E2)*4.725897920604915E-4;
	ds2.x = t3*(v4*1.741E3+v10*1.81E3+v16*3.649E3+v22*1.81E3+v28*1.741E3-lE*v1*1.3E2+lE*v7*1.02E2-lE*v19*1.02E2+lE*v25*1.3E2)*(3.0/1.12E2);
	ds2.y = t3*(v5*1.741E3+v11*1.81E3+v17*3.649E3+v23*1.81E3+v29*1.741E3-lE*v2*1.3E2+lE*v8*1.02E2-lE*v20*1.02E2+lE*v26*1.3E2)*(3.0/1.12E2);
	ds2.z = t3*(v6*1.741E3+v12*1.81E3+v18*3.649E3+v24*1.81E3+v30*1.741E3-lE*v3*1.3E2+lE*v9*1.02E2-lE*v21*1.02E2+lE*v27*1.3E2)*(3.0/1.12E2);
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = t2*(t4+v4*2.6373E4+v10*2.781E4-v22*5.955E4-v28*4.10427E5-lE*v1*1.966E3+lE*v7*1.63E3+lE*v13*5.684E3-lE*v19*5.5502E4-lE*v25*3.3706E4)*(-3.375641371860654E-5);
	dp3.y = t2*(t5+v5*2.6373E4+v11*2.781E4-v23*5.955E4-v29*4.10427E5-lE*v2*1.966E3+lE*v8*1.63E3+lE*v14*5.684E3-lE*v20*5.5502E4-lE*v26*3.3706E4)*(-3.375641371860654E-5);
	dp3.z = t2*(t6+v6*2.6373E4+v12*2.781E4-v24*5.955E4-v30*4.10427E5-lE*v3*1.966E3+lE*v9*1.63E3+lE*v15*5.684E3-lE*v21*5.5502E4-lE*v27*3.3706E4)*(-3.375641371860654E-5);
	ds3.x = t3*(t7+v4*2.9761E4+v10*3.145E4+v22*2.3305E5+v28*3.26785E5-lE*v1*2.218E3+lE*v7*1.854E3+lE*v13*7.392E3+lE*v19*3.97E3+lE*v25*2.529E4)*5.06346205779098E-4;
	ds3.y = t3*(t8+v5*2.9761E4+v11*3.145E4+v23*2.3305E5+v29*3.26785E5-lE*v2*2.218E3+lE*v8*1.854E3+lE*v14*7.392E3+lE*v20*3.97E3+lE*v26*2.529E4)*5.06346205779098E-4;
	ds3.z = t3*(t9+v6*2.9761E4+v12*3.145E4+v24*2.3305E5+v30*3.26785E5-lE*v3*2.218E3+lE*v9*1.854E3+lE*v15*7.392E3+lE*v21*3.97E3+lE*v27*2.529E4)*5.06346205779098E-4;
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;

	real3 dp4, ds4;
	dp4.x = t2*(t10+v4*3.1455E4+v10*3.327E4+v22*3.7935E5+v28*2.650575E6-lE*v1*2.344E3+lE*v7*1.966E3+lE*v13*8.246E3+lE*v19*3.3706E4+lE*v25*4.10276E5)*3.375641371860654E-5;
	dp4.y = t2*(t11+v5*3.1455E4+v11*3.327E4+v23*3.7935E5+v29*2.650575E6-lE*v2*2.344E3+lE*v8*1.966E3+lE*v14*8.246E3+lE*v20*3.3706E4+lE*v26*4.10276E5)*3.375641371860654E-5;
	dp4.z = t2*(t12+v6*3.1455E4+v12*3.327E4+v24*3.7935E5+v30*2.650575E6-lE*v3*2.344E3+lE*v9*1.966E3+lE*v15*8.246E3+lE*v21*3.3706E4+lE*v27*4.10276E5)*3.375641371860654E-5;
	ds4.x = t3*(t13+v4*2.81401E5+v10*2.9761E5+v22*3.26785E6+v28*1.5132601E7-lE*v1*2.097E4+lE*v7*1.7582E4+lE*v13*7.336E4+lE*v19*2.73618E5+lE*v25*1.76705E6)*5.06346205779098E-5;
	ds4.y = t3*(t14+v5*2.81401E5+v11*2.9761E5+v23*3.26785E6+v29*1.5132601E7-lE*v2*2.097E4+lE*v8*1.7582E4+lE*v14*7.336E4+lE*v20*2.73618E5+lE*v26*1.76705E6)*5.06346205779098E-5;
	ds4.z = t3*(t15+v6*2.81401E5+v12*2.9761E5+v24*3.26785E6+v30*1.5132601E7-lE*v3*2.097E4+lE*v9*1.7582E4+lE*v15*7.336E4+lE*v21*2.73618E5+lE*v27*1.76705E6)*5.06346205779098E-5;
	d2_1[nodesPortion.x + 4] = mult * dp4;
	d2_2[nodesPortion.x + 4] = mult * ds4;
}



// Same as above, but for a "noodle" made up of 5 ANCF beam elements
// In this case, 'v' and 'edd' are of length (5+1)*6 = 36
//
__device__ __host__ inline void minv_vec_5(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p0 = f1[nodesPortion.x + 0];
	real_ v1  = p0.x;
	real_ v2  = p0.y;
	real_ v3  = p0.z;
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v4  = s0.x;
	real_ v5  = s0.y;
	real_ v6  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v7  = p1.x;
	real_ v8  = p1.y;
	real_ v9  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v10  = s1.x;
	real_ v11  = s1.y;
	real_ v12  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v13  = p2.x;
	real_ v14  = p2.y;
	real_ v15  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v16  = s2.x;
	real_ v17  = s2.y;
	real_ v18  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v19  = p3.x;
	real_ v20  = p3.y;
	real_ v21  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v22  = s3.x;
	real_ v23  = s3.y;
	real_ v24  = s3.z;

	real3 p4 = f1[nodesPortion.x + 4];
	real_ v25  = p4.x;
	real_ v26  = p4.y;
	real_ v27  = p4.z;
	real3 s4 = f2[nodesPortion.x + 4];
	real_ v28  = s4.x;
	real_ v29  = s4.y;
	real_ v30  = s4.z;

	real3 p5 = f1[nodesPortion.x + 5];
	real_ v31  = p5.x;
	real_ v32  = p5.y;
	real_ v33  = p5.z;
	real3 s5 = f2[nodesPortion.x + 5];
	real_ v34  = s5.x;
	real_ v35  = s5.y;
	real_ v36  = s5.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 dp0, ds0;
	dp0.x  = t2*(v4*5.37103929E9+v10*7.68050805E8+v16*2.0672385E8+v22*5.9057325E7+v28*1.909737E7+v34*1.8057885E7-lE*v1*8.31427196E8-lE*v7*6.8354341E7-lE*v13*1.6897736E7-lE*v19*4.697911E6-lE*v25*1.128116E6+lE*v31*1.345679E6)*(-1.665637337492874E-8);
	dp0.y  = t2*(v5*5.37103929E9+v11*7.68050805E8+v17*2.0672385E8+v23*5.9057325E7+v29*1.909737E7+v35*1.8057885E7-lE*v2*8.31427196E8-lE*v8*6.8354341E7-lE*v14*1.6897736E7-lE*v20*4.697911E6-lE*v26*1.128116E6+lE*v32*1.345679E6)*(-1.665637337492874E-8);
	dp0.z  = t2*(v6*5.37103929E9+v12*7.68050805E8+v18*2.0672385E8+v24*5.9057325E7+v30*1.909737E7+v36*1.8057885E7-lE*v3*8.31427196E8-lE*v9*6.8354341E7-lE*v15*1.6897736E7-lE*v21*4.697911E6-lE*v27*1.128116E6+lE*v33*1.345679E6)*(-1.665637337492874E-8);
	ds0.x  = t3*(v4*3.06619054E9+v10*6.6160009E8+v16*1.845631E8+v22*5.282779E7+v28*1.708462E7+v34*1.615477E7-lE*v1*3.58069286E8-lE*v7*5.5492261E7-lE*v13*1.5033886E7-lE*v19*4.201581E6-lE*v25*1.009206E6+lE*v31*1.203859E6)*2.498456006239311E-7;
	ds0.y  = t3*(v5*3.06619054E9+v11*6.6160009E8+v17*1.845631E8+v23*5.282779E7+v29*1.708462E7+v35*1.615477E7-lE*v2*3.58069286E8-lE*v8*5.5492261E7-lE*v14*1.5033886E7-lE*v20*4.201581E6-lE*v26*1.009206E6+lE*v32*1.203859E6)*2.498456006239311E-7;
	ds0.z  = t3*(v6*3.06619054E9+v12*6.6160009E8+v18*1.845631E8+v24*5.282779E7+v30*1.708462E7+v36*1.615477E7-lE*v3*3.58069286E8-lE*v9*5.5492261E7-lE*v15*1.5033886E7-lE*v21*4.201581E6-lE*v27*1.009206E6+lE*v33*1.203859E6)*2.498456006239311E-7;
	d2_1[nodesPortion.x + 0] = mult * dp0;
	d2_2[nodesPortion.x + 0] = mult * ds0;

	real3 dp1, ds1;
	dp1.x = t2*(v4*-7.5671265E7-v10*1.102893E7+v16*1.47339E7+v22*4.48575E6+v28*1.45518E6+v34*1.37619E6+lE*v1*6.214031E6+lE*v7*1.0222276E7-lE*v13*1.061404E6-lE*v19*3.54704E5-lE*v25*8.5924E4+lE*v31*1.02556E5)*1.832201071242161E-7;
	dp1.y = t2*(v5*-7.5671265E7-v11*1.102893E7+v17*1.47339E7+v23*4.48575E6+v29*1.45518E6+v35*1.37619E6+lE*v2*6.214031E6+lE*v8*1.0222276E7-lE*v14*1.061404E6-lE*v20*3.54704E5-lE*v26*8.5924E4+lE*v32*1.02556E5)*1.832201071242161E-7;
	dp1.z = t2*(v6*-7.5671265E7-v12*1.102893E7+v18*1.47339E7+v24*4.48575E6+v30*1.45518E6+v36*1.37619E6+lE*v3*6.214031E6+lE*v9*1.0222276E7-lE*v15*1.061404E6-lE*v21*3.54704E5-lE*v27*8.5924E4+lE*v33*1.02556E5)*1.832201071242161E-7;
	ds1.x = t3*(v4*6.6160009E8+v10*4.7159446E8+v16*1.918402E8+v22*5.58193E7+v28*1.806724E7+v34*1.708462E7-lE*v1*5.1203387E7-lE*v7*8.087882E6-lE*v13*1.5156972E7-lE*v19*4.432522E6-lE*v25*1.067132E6+lE*v31*1.273158E6)*2.498456006239311E-7;
	ds1.y = t3*(v5*6.6160009E8+v11*4.7159446E8+v17*1.918402E8+v23*5.58193E7+v29*1.806724E7+v35*1.708462E7-lE*v2*5.1203387E7-lE*v8*8.087882E6-lE*v14*1.5156972E7-lE*v20*4.432522E6-lE*v26*1.067132E6+lE*v32*1.273158E6)*2.498456006239311E-7;
	ds1.z = t3*(v6*6.6160009E8+v12*4.7159446E8+v18*1.918402E8+v24*5.58193E7+v30*1.806724E7+v36*1.708462E7-lE*v3*5.1203387E7-lE*v9*8.087882E6-lE*v15*1.5156972E7-lE*v21*4.432522E6-lE*v27*1.067132E6+lE*v33*1.273158E6)*2.498456006239311E-7;
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t2*(v4*2.2550829E8+v10*2.2735458E8+v16*8.17785E6-v22*1.95387675E8-v28*6.648783E7-v34*6.3023715E7-lE*v1*1.6897736E7+lE*v7*1.1675444E7-lE*v13*1.02880676E8+lE*v19*1.4019899E7+lE*v25*3.901744E6-lE*v31*4.697911E6)*(-1.665637337492874E-8);
	dp2.y = t2*(v5*2.2550829E8+v11*2.2735458E8+v17*8.17785E6-v23*1.95387675E8-v29*6.648783E7-v35*6.3023715E7-lE*v2*1.6897736E7+lE*v8*1.1675444E7-lE*v14*1.02880676E8+lE*v20*1.4019899E7+lE*v26*3.901744E6-lE*v32*4.697911E6)*(-1.665637337492874E-8);
	dp2.z = t2*(v6*2.2550829E8+v12*2.2735458E8+v18*8.17785E6-v24*1.95387675E8-v30*6.648783E7-v36*6.3023715E7-lE*v3*1.6897736E7+lE*v9*1.1675444E7-lE*v15*1.02880676E8+lE*v21*1.4019899E7+lE*v27*3.901744E6-lE*v33*4.697911E6)*(-1.665637337492874E-8);
	ds2.x = t3*(v4*3.691262E7+v10*3.836804E7+v16*7.68737E7+v22*3.3906002E7+v28*1.116386E7+v34*1.0565558E7-lE*v1*2.756318E6+lE*v7*2.160972E6-lE*v13*1.09038E5-lE*v19*2.605169E6-lE*v25*6.5791E5+lE*v31*7.87431E5)*1.249228003119655E-6;
	ds2.y = t3*(v5*3.691262E7+v11*3.836804E7+v17*7.68737E7+v23*3.3906002E7+v29*1.116386E7+v35*1.0565558E7-lE*v2*2.756318E6+lE*v8*2.160972E6-lE*v14*1.09038E5-lE*v20*2.605169E6-lE*v26*6.5791E5+lE*v32*7.87431E5)*1.249228003119655E-6;
	ds2.z = t3*(v6*3.691262E7+v12*3.836804E7+v18*7.68737E7+v24*3.3906002E7+v30*1.116386E7+v36*1.0565558E7-lE*v3*2.756318E6+lE*v9*2.160972E6-lE*v15*1.09038E5-lE*v21*2.605169E6-lE*v27*6.5791E5+lE*v33*7.87431E5)*1.249228003119655E-6;
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = t2*(v4*6.3023715E7+v10*6.648783E7+v16*1.95387675E8-v22*8.17785E6-v28*2.2735458E8-v34*2.2550829E8-lE*v1*4.697911E6+lE*v7*3.901744E6+lE*v13*1.4019899E7-lE*v19*1.02880676E8+lE*v25*1.1675444E7-lE*v31*1.6897736E7)*(-1.665637337492874E-8);
	dp3.y = t2*(v5*6.3023715E7+v11*6.648783E7+v17*1.95387675E8-v23*8.17785E6-v29*2.2735458E8-v35*2.2550829E8-lE*v2*4.697911E6+lE*v8*3.901744E6+lE*v14*1.4019899E7-lE*v20*1.02880676E8+lE*v26*1.1675444E7-lE*v32*1.6897736E7)*(-1.665637337492874E-8);
	dp3.z = t2*(v6*6.3023715E7+v12*6.648783E7+v18*1.95387675E8-v24*8.17785E6-v30*2.2735458E8-v36*2.2550829E8-lE*v3*4.697911E6+lE*v9*3.901744E6+lE*v15*1.4019899E7-lE*v21*1.02880676E8+lE*v27*1.1675444E7-lE*v33*1.6897736E7)*(-1.665637337492874E-8);
	ds3.x = t3*(v4*1.0565558E7+v10*1.116386E7+v16*3.3906002E7+v22*7.68737E7+v28*3.836804E7+v34*3.691262E7-lE*v1*7.87431E5+lE*v7*6.5791E5+lE*v13*2.605169E6+lE*v19*1.09038E5-lE*v25*2.160972E6+lE*v31*2.756318E6)*1.249228003119655E-6;
	ds3.y = t3*(v5*1.0565558E7+v11*1.116386E7+v17*3.3906002E7+v23*7.68737E7+v29*3.836804E7+v35*3.691262E7-lE*v2*7.87431E5+lE*v8*6.5791E5+lE*v14*2.605169E6+lE*v20*1.09038E5-lE*v26*2.160972E6+lE*v32*2.756318E6)*1.249228003119655E-6;
	ds3.z = t3*(v6*1.0565558E7+v12*1.116386E7+v18*3.3906002E7+v24*7.68737E7+v30*3.836804E7+v36*3.691262E7-lE*v3*7.87431E5+lE*v9*6.5791E5+lE*v15*2.605169E6+lE*v21*1.09038E5-lE*v27*2.160972E6+lE*v33*2.756318E6)*1.249228003119655E-6;
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;

	real3 dp4, ds4;
	dp4.x = t2*(v4*1.37619E6+v10*1.45518E6+v16*4.48575E6+v22*1.47339E7-v28*1.102893E7-v34*7.5671265E7-lE*v1*1.02556E5+lE*v7*8.5924E4+lE*v13*3.54704E5+lE*v19*1.061404E6-lE*v25*1.0222276E7-lE*v31*6.214031E6)*(-1.832201071242161E-7);
	dp4.y = t2*(v5*1.37619E6+v11*1.45518E6+v17*4.48575E6+v23*1.47339E7-v29*1.102893E7-v35*7.5671265E7-lE*v2*1.02556E5+lE*v8*8.5924E4+lE*v14*3.54704E5+lE*v20*1.061404E6-lE*v26*1.0222276E7-lE*v32*6.214031E6)*(-1.832201071242161E-7);
	dp4.z = t2*(v6*1.37619E6+v12*1.45518E6+v18*4.48575E6+v24*1.47339E7-v30*1.102893E7-v36*7.5671265E7-lE*v3*1.02556E5+lE*v9*8.5924E4+lE*v15*3.54704E5+lE*v21*1.061404E6-lE*v27*1.0222276E7-lE*v33*6.214031E6)*(-1.832201071242161E-7);
	ds4.x = t3*(v4*1.708462E7+v10*1.806724E7+v16*5.58193E7+v22*1.918402E8+v28*4.7159446E8+v34*6.6160009E8-lE*v1*1.273158E6+lE*v7*1.067132E6+lE*v13*4.432522E6+lE*v19*1.5156972E7+lE*v25*8.087882E6+lE*v31*5.1203387E7)*2.498456006239311E-7;
	ds4.y = t3*(v5*1.708462E7+v11*1.806724E7+v17*5.58193E7+v23*1.918402E8+v29*4.7159446E8+v35*6.6160009E8-lE*v2*1.273158E6+lE*v8*1.067132E6+lE*v14*4.432522E6+lE*v20*1.5156972E7+lE*v26*8.087882E6+lE*v32*5.1203387E7)*2.498456006239311E-7;
	ds4.z = t3*(v6*1.708462E7+v12*1.806724E7+v18*5.58193E7+v24*1.918402E8+v30*4.7159446E8+v36*6.6160009E8-lE*v3*1.273158E6+lE*v9*1.067132E6+lE*v15*4.432522E6+lE*v21*1.5156972E7+lE*v27*8.087882E6+lE*v33*5.1203387E7)*2.498456006239311E-7;
	d2_1[nodesPortion.x + 4] = mult * dp4;
	d2_2[nodesPortion.x + 4] = mult * ds4;

	real3 dp5, ds5;
	dp5.x = t2*(v4*1.8057885E7+v10*1.909737E7+v16*5.9057325E7+v22*2.0672385E8+v28*7.68050805E8+v34*5.37103929E9-lE*v1*1.345679E6+lE*v7*1.128116E6+lE*v13*4.697911E6+lE*v19*1.6897736E7+lE*v25*6.8354341E7+lE*v31*8.31427196E8)*1.665637337492874E-8;
	dp5.y = t2*(v5*1.8057885E7+v11*1.909737E7+v17*5.9057325E7+v23*2.0672385E8+v29*7.68050805E8+v35*5.37103929E9-lE*v2*1.345679E6+lE*v8*1.128116E6+lE*v14*4.697911E6+lE*v20*1.6897736E7+lE*v26*6.8354341E7+lE*v32*8.31427196E8)*1.665637337492874E-8;
	dp5.z = t2*(v6*1.8057885E7+v12*1.909737E7+v18*5.9057325E7+v24*2.0672385E8+v30*7.68050805E8+v36*5.37103929E9-lE*v3*1.345679E6+lE*v9*1.128116E6+lE*v15*4.697911E6+lE*v21*1.6897736E7+lE*v27*6.8354341E7+lE*v33*8.31427196E8)*1.665637337492874E-8;
	ds5.x = t3*(v4*1.615477E7+v10*1.708462E7+v16*5.282779E7+v22*1.845631E8+v28*6.6160009E8+v34*3.06619054E9-lE*v1*1.203859E6+lE*v7*1.009206E6+lE*v13*4.201581E6+lE*v19*1.5033886E7+lE*v25*5.5492261E7+lE*v31*3.58069286E8)*2.498456006239311E-7;
	ds5.y = t3*(v5*1.615477E7+v11*1.708462E7+v17*5.282779E7+v23*1.845631E8+v29*6.6160009E8+v35*3.06619054E9-lE*v2*1.203859E6+lE*v8*1.009206E6+lE*v14*4.201581E6+lE*v20*1.5033886E7+lE*v26*5.5492261E7+lE*v32*3.58069286E8)*2.498456006239311E-7;
	ds5.z = t3*(v6*1.615477E7+v12*1.708462E7+v18*5.282779E7+v24*1.845631E8+v30*6.6160009E8+v36*3.06619054E9-lE*v3*1.203859E6+lE*v9*1.009206E6+lE*v15*4.201581E6+lE*v21*1.5033886E7+lE*v27*5.5492261E7+lE*v33*3.58069286E8)*2.498456006239311E-7;
	d2_1[nodesPortion.x + 5] = mult * dp5;
	d2_2[nodesPortion.x + 5] = mult * ds5;
}
//*****************************************************************************************
__device__ __host__ inline void minv_vec_weld_1(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p1 = f1[nodesPortion.x + 1];
	real_ v1  = p1.x;
	real_ v2  = p1.y;
	real_ v3  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v4  = s1.x;
	real_ v5  = s1.y;
	real_ v6  = s1.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 dp1, ds1;
	dp1.x = v1*1.2E1+t2*v4*6.6E1;
	dp1.y = v2*1.2E1+t2*v5*6.6E1;
	dp1.z = v3*1.2E1+t2*v6*6.6E1;
	ds1.x = t3*(v4*7.8E1+lE*v1*1.1E1)*6.0;
	ds1.y = t3*(v5*7.8E1+lE*v2*1.1E1)*6.0;
	ds1.z = t3*(v6*7.8E1+lE*v3*1.1E1)*6.0;
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;
}

// Same as above for a 2-element noodle.
// In this case, the input vector 'v' and the output vector 'edd'
// are assumed to be of length (2+1)*6 - 6 = 12
__device__ __host__ inline void minv_vec_weld_2(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p1 = f1[nodesPortion.x + 1];
	real_ v1  = p1.x;
	real_ v2  = p1.y;
	real_ v3  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v4  = s1.x;
	real_ v5  = s1.y;
	real_ v6  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v7  = p2.x;
	real_ v8  = p2.y;
	real_ v9  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v10  = s2.x;
	real_ v11  = s2.y;
	real_ v12  = s2.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 dp1, ds1;
	dp1.x = v1*1.775828994244999+v7*1.266100301452453+t2*(v4*2.3E2+v10*9.47E2)*1.644286105782406E-2;
	dp1.y = v2*1.775828994244999+v8*1.266100301452453+t2*(v5*2.3E2+v11*9.47E2)*1.644286105782406E-2;
	dp1.z = v3*1.775828994244999+v9*1.266100301452453+t2*(v6*2.3E2+v12*9.47E2)*1.644286105782406E-2;
	ds1.x = t2*(v1*3.781858043299534+v7*1.047410249383393E1)+t3*(v4*8.58317347218416E1+v10*1.342724033981913E2);
	ds1.y = t2*(v2*3.781858043299534+v8*1.047410249383393E1)+t3*(v5*8.58317347218416E1+v11*1.342724033981913E2);
	ds1.z = t2*(v3*3.781858043299534+v9*1.047410249383393E1)+t3*(v6*8.58317347218416E1+v12*1.342724033981913E2);
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = v1*1.266100301452453+v7*1.368046040010962E1+t2*(v4*4.9E1+v10*4.08E2)*2.137571937517128E-1;
	dp2.y = v2*1.266100301452453+v8*1.368046040010962E1+t2*(v5*4.9E1+v11*4.08E2)*2.137571937517128E-1;
	dp2.z = v3*1.266100301452453+v9*1.368046040010962E1+t2*(v6*4.9E1+v12*4.08E2)*2.137571937517128E-1;
	ds2.x = t2*(v1*1.557138942175939E1+v7*8.721293505069882E1)+t3*(v4*1.342724033981913E2+v10*7.35982460948205E2);
	ds2.y = t2*(v2*1.557138942175939E1+v8*8.721293505069882E1)+t3*(v5*1.342724033981913E2+v11*7.35982460948205E2);
	ds2.z = t2*(v3*1.557138942175939E1+v9*8.721293505069882E1)+t3*(v6*1.342724033981913E2+v12*7.35982460948205E2);
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;
}

__device__ __host__ inline void minv_vec_weld_3(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p1 = f1[nodesPortion.x + 1];
	real_ v1  = p1.x;
	real_ v2  = p1.y;
	real_ v3  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v4  = s1.x;
	real_ v5  = s1.y;
	real_ v6  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v7  = p2.x;
	real_ v8  = p2.y;
	real_ v9  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v10  = s2.x;
	real_ v11  = s2.y;
	real_ v12  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v13  = p3.x;
	real_ v14  = p3.y;
	real_ v15  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v16  = s3.x;
	real_ v17  = s3.y;
	real_ v18  = s3.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 dp1, ds1;
	dp1.x = t2*(v4*1.4519E4+v10*3.155E4+v16*3.1091E4+lE*v1*1.1218E4-lE*v7*1.654E3+lE*v13*2.328E3)*1.399743380380264E-4;
	dp1.y = t2*(v5*1.4519E4+v11*3.155E4+v17*3.1091E4+lE*v2*1.1218E4-lE*v8*1.654E3+lE*v14*2.328E3)*1.399743380380264E-4;
	dp1.z = t2*(v6*1.4519E4+v12*3.155E4+v18*3.1091E4+lE*v3*1.1218E4-lE*v9*1.654E3+lE*v15*2.328E3)*1.399743380380264E-4;
	ds1.x = t3*(v4*5.06802E5+v10*2.829E5+v16*2.73078E5+lE*v1*1.4519E4-lE*v7*1.5782E4+lE*v13*2.0399E4)*1.399743380380264E-4;
	ds1.y = t3*(v5*5.06802E5+v11*2.829E5+v17*2.73078E5+lE*v2*1.4519E4-lE*v8*1.5782E4+lE*v14*2.0399E4)*1.399743380380264E-4;
	ds1.z = t3*(v6*5.06802E5+v12*2.829E5+v18*2.73078E5+lE*v3*1.4519E4-lE*v9*1.5782E4+lE*v15*2.0399E4)*1.399743380380264E-4;
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t2*(v4*-7.891E3+v10*7.8E3+v16*5.0076E4-lE*v1*8.27E2+lE*v7*6.656E3+lE*v13*4.108E3)*2.799486760760527E-4;
	dp2.y = t2*(v5*-7.891E3+v11*7.8E3+v17*5.0076E4-lE*v2*8.27E2+lE*v8*6.656E3+lE*v14*4.108E3)*2.799486760760527E-4;
	dp2.z = t2*(v6*-7.891E3+v12*7.8E3+v18*5.0076E4-lE*v3*8.27E2+lE*v9*6.656E3+lE*v15*4.108E3)*2.799486760760527E-4;
	ds2.x = t3*(v4*5.658E3+v10*1.644E4+v16*2.3244E4+lE*v1*6.31E2+lE*v7*3.12E2+lE*v13*1.8E3)*6.998716901901318E-3;
	ds2.y = t3*(v5*5.658E3+v11*1.644E4+v17*2.3244E4+lE*v2*6.31E2+lE*v8*3.12E2+lE*v14*1.8E3)*6.998716901901318E-3;
	ds2.z = t3*(v6*5.658E3+v12*1.644E4+v18*2.3244E4+lE*v3*6.31E2+lE*v9*3.12E2+lE*v15*1.8E3)*6.998716901901318E-3;
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = t2*(v4*2.0399E4+v10*9.0E4+v16*6.37811E5+lE*v1*2.328E3+lE*v7*8.216E3+lE*v13*9.8838E4)*1.399743380380264E-4;
	dp3.y = t2*(v5*2.0399E4+v11*9.0E4+v17*6.37811E5+lE*v2*2.328E3+lE*v8*8.216E3+lE*v14*9.8838E4)*1.399743380380264E-4;
	dp3.z = t2*(v6*2.0399E4+v12*9.0E4+v18*6.37811E5+lE*v3*2.328E3+lE*v9*8.216E3+lE*v15*9.8838E4)*1.399743380380264E-4;
	ds3.x = t3*(v4*2.73078E5+v10*1.1622E6+v16*5.455242E6+lE*v1*3.1091E4+lE*v7*1.00152E5+lE*v13*6.37811E5)*1.399743380380264E-4;
	ds3.y = t3*(v5*2.73078E5+v11*1.1622E6+v17*5.455242E6+lE*v2*3.1091E4+lE*v8*1.00152E5+lE*v14*6.37811E5)*1.399743380380264E-4;
	ds3.z = t3*(v6*2.73078E5+v12*1.1622E6+v18*5.455242E6+lE*v3*3.1091E4+lE*v9*1.00152E5+lE*v15*6.37811E5)*1.399743380380264E-4;
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;
}



// Same as above, but for a "noodle" made up of 4 ANCF beam elements
// In this case, 'v' and 'edd' are of length (4+1)*6 - 6 = 24
//
__device__ __host__ inline void minv_vec_weld_4(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p1 = f1[nodesPortion.x + 1];
	real_ v1  = p1.x;
	real_ v2  = p1.y;
	real_ v3  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v4  = s1.x;
	real_ v5  = s1.y;
	real_ v6  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v7  = p2.x;
	real_ v8  = p2.y;
	real_ v9  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v10  = s2.x;
	real_ v11  = s2.y;
	real_ v12  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v13  = p3.x;
	real_ v14  = p3.y;
	real_ v15  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v16  = s3.x;
	real_ v17  = s3.y;
	real_ v18  = s3.z;

	real3 p4 = f1[nodesPortion.x + 4];
	real_ v19  = p4.x;
	real_ v20  = p4.y;
	real_ v21  = p4.z;
	real3 s4 = f2[nodesPortion.x + 4];
	real_ v22  = s4.x;
	real_ v23  = s4.y;
	real_ v24  = s4.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 dp1, ds1;
	dp1.x = t2*(v4*6.52415E6+v10*1.3219763E7+v16*4.46471E6+v22*4.230587E6+lE*v1*5.337092E6-lE*v7*9.64147E5-lE*v13*2.62256E5+lE*v19*3.15343E5)*2.914133299285895E-7;
	dp1.y = t2*(v5*6.52415E6+v11*1.3219763E7+v17*4.46471E6+v23*4.230587E6+lE*v2*5.337092E6-lE*v8*9.64147E5-lE*v14*2.62256E5+lE*v20*3.15343E5)*2.914133299285895E-7;
	dp1.z = t2*(v6*6.52415E6+v12*1.3219763E7+v18*4.46471E6+v24*4.230587E6+lE*v3*5.337092E6-lE*v9*9.64147E5-lE*v15*2.62256E5+lE*v21*3.15343E5)*2.914133299285895E-7;
	ds1.x = t3*(v4*2.394843E8+v10*1.18904934E8+v16*3.92943E7+v22*3.7195026E7+lE*v1*6.52415E6-lE*v7*9.069827E6-lE*v13*2.314594E6+lE*v19*2.772133E6)*2.914133299285895E-7;
	ds1.y = t3*(v5*2.394843E8+v11*1.18904934E8+v17*3.92943E7+v23*3.7195026E7+lE*v2*6.52415E6-lE*v8*9.069827E6-lE*v14*2.314594E6+lE*v20*2.772133E6)*2.914133299285895E-7;
	ds1.z = t3*(v6*2.394843E8+v12*1.18904934E8+v18*3.92943E7+v24*3.7195026E7+lE*v3*6.52415E6-lE*v9*9.069827E6-lE*v15*2.314594E6+lE*v21*2.772133E6)*2.914133299285895E-7;
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t2*(v4*-9.069827E6+v10*1.083576E6+v16*1.319448E7+v22*1.3078104E7-lE*v1*9.64147E5+lE*v7*5.831488E6-lE*v13*6.7912E5+lE*v19*9.79888E5)*2.914133299285895E-7;
	dp2.y = t2*(v5*-9.069827E6+v11*1.083576E6+v17*1.319448E7+v23*1.3078104E7-lE*v2*9.64147E5+lE*v8*5.831488E6-lE*v14*6.7912E5+lE*v20*9.79888E5)*2.914133299285895E-7;
	dp2.z = t2*(v6*-9.069827E6+v12*1.083576E6+v18*1.319448E7+v24*1.3078104E7-lE*v3*9.64147E5+lE*v9*5.831488E6-lE*v15*6.7912E5+lE*v21*9.79888E5)*2.914133299285895E-7;
	ds2.x = t3*(v4*1.18904934E8+v10*3.2177532E8+v16*1.6196088E8+v22*1.5585852E8+lE*v1*1.3219763E7+lE*v7*1.083576E6-lE*v13*9.115104E6+lE*v19*1.1638536E7)*2.914133299285895E-7;
	ds2.y = t3*(v5*1.18904934E8+v11*3.2177532E8+v17*1.6196088E8+v23*1.5585852E8+lE*v2*1.3219763E7+lE*v8*1.083576E6-lE*v14*9.115104E6+lE*v20*1.1638536E7)*2.914133299285895E-7;
	ds2.z = t3*(v6*1.18904934E8+v12*3.2177532E8+v18*1.6196088E8+v24*1.5585852E8+lE*v3*1.3219763E7+lE*v9*1.083576E6-lE*v15*9.115104E6+lE*v21*1.1638536E7)*2.914133299285895E-7;
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = t2*(v4*2.314594E6+v10*9.115104E6-v16*6.98231E6-v22*4.7622227E7+lE*v1*2.62256E5+lE*v7*6.7912E5-lE*v13*6.424204E6-lE*v19*3.910333E6)*(-2.914133299285895E-7);
	dp3.y = t2*(v5*2.314594E6+v11*9.115104E6-v17*6.98231E6-v23*4.7622227E7+lE*v2*2.62256E5+lE*v8*6.7912E5-lE*v14*6.424204E6-lE*v20*3.910333E6)*(-2.914133299285895E-7);
	dp3.z = t2*(v6*2.314594E6+v12*9.115104E6-v18*6.98231E6-v24*4.7622227E7+lE*v3*2.62256E5+lE*v9*6.7912E5-lE*v15*6.424204E6-lE*v21*3.910333E6)*(-2.914133299285895E-7);
	ds3.x = t3*(v4*3.92943E7+v10*1.6196088E8+v16*4.0351098E8+v22*5.66458134E8+lE*v1*4.46471E6+lE*v7*1.319448E7+lE*v13*6.98231E6+lE*v19*4.3842253E7)*2.914133299285895E-7;
	ds3.y = t3*(v5*3.92943E7+v11*1.6196088E8+v17*4.0351098E8+v23*5.66458134E8+lE*v2*4.46471E6+lE*v8*1.319448E7+lE*v14*6.98231E6+lE*v20*4.3842253E7)*2.914133299285895E-7;
	ds3.z = t3*(v6*3.92943E7+v12*1.6196088E8+v18*4.0351098E8+v24*5.66458134E8+lE*v3*4.46471E6+lE*v9*1.319448E7+lE*v15*6.98231E6+lE*v21*4.3842253E7)*2.914133299285895E-7;
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;

	real3 dp4, ds4;
	dp4.x = t2*(v4*3.96019E5+v10*1.662648E6+v16*6.263179E6+v22*4.384848E7+lE*v1*4.5049E4+lE*v7*1.39984E5+lE*v13*5.58619E5+lE*v19*6.788288E6)*2.039893309500127E-6;
	dp4.y = t2*(v5*3.96019E5+v11*1.662648E6+v17*6.263179E6+v23*4.384848E7+lE*v2*4.5049E4+lE*v8*1.39984E5+lE*v14*5.58619E5+lE*v20*6.788288E6)*2.039893309500127E-6;
	dp4.z = t2*(v6*3.96019E5+v12*1.662648E6+v18*6.263179E6+v24*4.384848E7+lE*v3*4.5049E4+lE*v9*1.39984E5+lE*v15*5.58619E5+lE*v21*6.788288E6)*2.039893309500127E-6;
	ds4.x = t3*(v4*3.7195026E7+v10*1.5585852E8+v16*5.66458134E8+v22*2.62809528E9+lE*v1*4.230587E6+lE*v7*1.3078104E7+lE*v13*4.7622227E7+lE*v19*3.0693936E8)*2.914133299285895E-7;
	ds4.y = t3*(v5*3.7195026E7+v11*1.5585852E8+v17*5.66458134E8+v23*2.62809528E9+lE*v2*4.230587E6+lE*v8*1.3078104E7+lE*v14*4.7622227E7+lE*v20*3.0693936E8)*2.914133299285895E-7;
	ds4.z = t3*(v6*3.7195026E7+v12*1.5585852E8+v18*5.66458134E8+v24*2.62809528E9+lE*v3*4.230587E6+lE*v9*1.3078104E7+lE*v15*4.7622227E7+lE*v21*3.0693936E8)*2.914133299285895E-7;
	d2_1[nodesPortion.x + 4] = mult * dp4;
	d2_2[nodesPortion.x + 4] = mult * ds4;
}


// Same as above, but for a "noodle" made up of 5 ANCF beam elements
// In this case, 'v' and 'edd' are of length (5+1)*6 - 6 = 30
//
__device__ __host__ inline void minv_vec_weld_5(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 p1 = f1[nodesPortion.x + 1];
	real_ v1  = p1.x;
	real_ v2  = p1.y;
	real_ v3  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v4  = s1.x;
	real_ v5  = s1.y;
	real_ v6  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v7  = p2.x;
	real_ v8  = p2.y;
	real_ v9  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v10  = s2.x;
	real_ v11  = s2.y;
	real_ v12  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v13  = p3.x;
	real_ v14  = p3.y;
	real_ v15  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v16  = s3.x;
	real_ v17  = s3.y;
	real_ v18  = s3.z;

	real3 p4 = f1[nodesPortion.x + 4];
	real_ v19  = p4.x;
	real_ v20  = p4.y;
	real_ v21  = p4.z;
	real3 s4 = f2[nodesPortion.x + 4];
	real_ v22  = s4.x;
	real_ v23  = s4.y;
	real_ v24  = s4.z;

	real3 p5 = f1[nodesPortion.x + 5];
	real_ v25  = p5.x;
	real_ v26  = p5.y;
	real_ v27  = p5.z;
	real3 s5 = f2[nodesPortion.x + 5];
	real_ v28  = s5.x;
	real_ v29  = s5.y;
	real_ v30  = s5.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 dp1, ds1;
	dp1.x = t2*(v4*7.80502331E8+v10*1.57213715E9+v16*4.70408999E8+v22*1.5247187E8+v28*1.44189371E8+lE*v1*6.41540802E8-lE*v7*1.17508246E8-lE*v13*3.7256368E7-lE*v19*9.00401E6+lE*v25*1.0745182E7)*2.422459633916196E-9;
	dp1.y = t2*(v5*7.80502331E8+v11*1.57213715E9+v17*4.70408999E8+v23*1.5247187E8+v29*1.44189371E8+lE*v2*6.41540802E8-lE*v8*1.17508246E8-lE*v14*3.7256368E7-lE*v20*9.00401E6+lE*v26*1.0745182E7)*2.422459633916196E-9;
	dp1.z = t2*(v6*7.80502331E8+v12*1.57213715E9+v18*4.70408999E8+v24*1.5247187E8+v30*1.44189371E8+lE*v3*6.41540802E8-lE*v9*1.17508246E8-lE*v15*3.7256368E7-lE*v21*9.00401E6+lE*v27*1.0745182E7)*2.422459633916196E-9;
	ds1.x = t3*(v4*6.69094146E8+v10*3.289359E8+v16*9.6296334E7+v22*3.117822E7+v28*2.9482986E7+lE*v1*1.8151217E7-lE*v7*2.5685426E7-lE*v13*7.642303E6-lE*v19*1.84145E6+lE*v25*2.197097E6)*1.041657642583964E-7;
	ds1.y = t3*(v5*6.69094146E8+v11*3.289359E8+v17*9.6296334E7+v23*3.117822E7+v29*2.9482986E7+lE*v2*1.8151217E7-lE*v8*2.5685426E7-lE*v14*7.642303E6-lE*v20*1.84145E6+lE*v26*2.197097E6)*1.041657642583964E-7;
	ds1.z = t3*(v6*6.69094146E8+v12*3.289359E8+v18*9.6296334E7+v24*3.117822E7+v30*2.9482986E7+lE*v3*1.8151217E7-lE*v9*2.5685426E7-lE*v15*7.642303E6-lE*v21*1.84145E6+lE*v27*2.197097E6)*1.041657642583964E-7;
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t2*(v4*-5.52236659E8+v10*3.70734E7+v16*6.90401244E8+v22*2.3461932E8+v28*2.22381276E8-lE*v1*5.8754123E7+lE*v7*3.48393344E8-lE*v13*4.9684388E7-lE*v19*1.377064E7+lE*v25*1.6576612E7)*4.844919267832393E-9;
	dp2.y = t2*(v5*-5.52236659E8+v11*3.70734E7+v17*6.90401244E8+v23*2.3461932E8+v29*2.22381276E8-lE*v2*5.8754123E7+lE*v8*3.48393344E8-lE*v14*4.9684388E7-lE*v20*1.377064E7+lE*v26*1.6576612E7)*4.844919267832393E-9;
	dp2.z = t2*(v6*-5.52236659E8+v12*3.70734E7+v18*6.90401244E8+v24*2.3461932E8+v30*2.22381276E8-lE*v3*5.8754123E7+lE*v9*3.48393344E8-lE*v15*4.9684388E7-lE*v21*1.377064E7+lE*v27*1.6576612E7)*4.844919267832393E-9;
	ds2.x = t3*(v4*1.41442437E9+v10*3.8039286E9+v16*1.702545708E9+v22*5.60847E8+v28*5.30802012E8+lE*v1*1.57213715E8+lE*v7*7.41468E6-lE*v13*1.30691544E8-lE*v19*3.3049848E7+lE*v25*3.9559776E7)*2.422459633916196E-8;
	ds2.y = t3*(v5*1.41442437E9+v11*3.8039286E9+v17*1.702545708E9+v23*5.60847E8+v29*5.30802012E8+lE*v2*1.57213715E8+lE*v8*7.41468E6-lE*v14*1.30691544E8-lE*v20*3.3049848E7+lE*v26*3.9559776E7)*2.422459633916196E-8;
	ds2.z = t3*(v6*1.41442437E9+v12*3.8039286E9+v18*1.702545708E9+v24*5.60847E8+v30*5.30802012E8+lE*v3*1.57213715E8+lE*v9*7.41468E6-lE*v15*1.30691544E8-lE*v21*3.3049848E7+lE*v27*3.9559776E7)*2.422459633916196E-8;
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = t2*(v4*3.28619029E8+v10*1.30691544E9-v16*6.6696479E7-v22*1.56663227E9-v28*1.553753291E9+lE*v1*3.7256368E7+lE*v7*9.9368776E7-lE*v13*7.06555622E8+lE*v19*8.047811E7-lE*v25*1.16424172E8)*(-2.422459633916196E-9);
	dp3.y = t2*(v5*3.28619029E8+v11*1.30691544E9-v17*6.6696479E7-v23*1.56663227E9-v29*1.553753291E9+lE*v2*3.7256368E7+lE*v8*9.9368776E7-lE*v14*7.06555622E8+lE*v20*8.047811E7-lE*v26*1.16424172E8)*(-2.422459633916196E-9);
	dp3.z = t2*(v6*3.28619029E8+v12*1.30691544E9-v18*6.6696479E7-v24*1.56663227E9-v30*1.553753291E9+lE*v3*3.7256368E7+lE*v9*9.9368776E7-lE*v15*7.06555622E8+lE*v21*8.047811E7-lE*v27*1.16424172E8)*(-2.422459633916196E-9);
	ds3.x = t3*(v4*4.140742362E9+v10*1.702545708E10+v16*3.9511055598E10+v22*1.974328494E10+v28*1.8995062242E10+lE*v1*4.70408999E8+lE*v7*1.380802488E9+lE*v13*6.6696479E7-lE*v19*1.11186803E9+lE*v25*1.418394479E9)*2.422459633916196E-9;
	ds3.y = t3*(v5*4.140742362E9+v11*1.702545708E10+v17*3.9511055598E10+v23*1.974328494E10+v29*1.8995062242E10+lE*v2*4.70408999E8+lE*v8*1.380802488E9+lE*v14*6.6696479E7-lE*v20*1.11186803E9+lE*v26*1.418394479E9)*2.422459633916196E-9;
	ds3.z = t3*(v6*4.140742362E9+v12*1.702545708E10+v18*3.9511055598E10+v24*1.974328494E10+v30*1.8995062242E10+lE*v3*4.70408999E8+lE*v9*1.380802488E9+lE*v15*6.6696479E7-lE*v21*1.11186803E9+lE*v27*1.418394479E9)*2.422459633916196E-9;
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;

	real3 dp4, ds4;
	dp4.x = t2*(v4*7.918235E6+v10*3.3049848E7+v16*1.11186803E8-v22*8.349744E7-v28*5.72408304E8+lE*v1*9.00401E5+lE*v7*2.754128E6+lE*v13*8.047811E6-lE*v19*7.7310272E7-lE*v25*4.700488E7)*(-2.422459633916196E-8);
	dp4.y = t2*(v5*7.918235E6+v11*3.3049848E7+v17*1.11186803E8-v23*8.349744E7-v29*5.72408304E8+lE*v2*9.00401E5+lE*v8*2.754128E6+lE*v14*8.047811E6-lE*v20*7.7310272E7-lE*v26*4.700488E7)*(-2.422459633916196E-8);
	dp4.z = t2*(v6*7.918235E6+v12*3.3049848E7+v18*1.11186803E8-v24*8.349744E7-v30*5.72408304E8+lE*v3*9.00401E5+lE*v9*2.754128E6+lE*v15*8.047811E6-lE*v21*7.7310272E7-lE*v27*4.700488E7)*(-2.422459633916196E-8);
	ds4.x = t3*(v4*1.34066346E8+v10*5.60847E8+v16*1.974328494E9+v22*4.86251448E9+v28*6.822253464E9+lE*v1*1.5247187E7+lE*v7*4.6923864E7+lE*v13*1.56663227E8+lE*v19*8.349744E7+lE*v25*5.28000168E8)*2.422459633916196E-8;
	ds4.y = t3*(v5*1.34066346E8+v11*5.60847E8+v17*1.974328494E9+v23*4.86251448E9+v29*6.822253464E9+lE*v2*1.5247187E7+lE*v8*4.6923864E7+lE*v14*1.56663227E8+lE*v20*8.349744E7+lE*v26*5.28000168E8)*2.422459633916196E-8;
	ds4.z = t3*(v6*1.34066346E8+v12*5.60847E8+v18*1.974328494E9+v24*4.86251448E9+v30*6.822253464E9+lE*v3*1.5247187E7+lE*v9*4.6923864E7+lE*v15*1.56663227E8+lE*v21*8.349744E7+lE*v27*5.28000168E8)*2.422459633916196E-8;
	d2_1[nodesPortion.x + 4] = mult * dp4;
	d2_2[nodesPortion.x + 4] = mult * ds4;

	real3 dp5, ds5;
	dp5.x = t2*(v4*9.4475171E7+v10*3.9559776E8+v16*1.418394479E9+v22*5.28000168E9+v28*3.6929331011E10+lE*v1*1.0745182E7+lE*v7*3.3153224E7+lE*v13*1.16424172E8+lE*v19*4.700488E8+lE*v25*5.716667482E9)*2.422459633916196E-9;
	dp5.y = t2*(v5*9.4475171E7+v11*3.9559776E8+v17*1.418394479E9+v23*5.28000168E9+v29*3.6929331011E10+lE*v2*1.0745182E7+lE*v8*3.3153224E7+lE*v14*1.16424172E8+lE*v20*4.700488E8+lE*v26*5.716667482E9)*2.422459633916196E-9;
	dp5.z = t2*(v6*9.4475171E7+v12*3.9559776E8+v18*1.418394479E9+v24*5.28000168E9+v30*3.6929331011E10+lE*v3*1.0745182E7+lE*v9*3.3153224E7+lE*v15*1.16424172E8+lE*v21*4.700488E8+lE*v27*5.716667482E9)*2.422459633916196E-9;
	ds5.x = t3*(v4*1.267768398E9+v10*5.30802012E9+v16*1.8995062242E10+v22*6.822253464E10+v28*3.16225866918E11+lE*v1*1.44189371E8+lE*v7*4.44762552E8+lE*v13*1.553753291E9+lE*v19*5.72408304E9+lE*v25*3.6929331011E10)*2.422459633916196E-9;
	ds5.y = t3*(v5*1.267768398E9+v11*5.30802012E9+v17*1.8995062242E10+v23*6.822253464E10+v29*3.16225866918E11+lE*v2*1.44189371E8+lE*v8*4.44762552E8+lE*v14*1.553753291E9+lE*v20*5.72408304E9+lE*v26*3.6929331011E10)*2.422459633916196E-9;
	ds5.z = t3*(v6*1.267768398E9+v12*5.30802012E9+v18*1.8995062242E10+v24*6.822253464E10+v30*3.16225866918E11+lE*v3*1.44189371E8+lE*v9*4.44762552E8+lE*v15*1.553753291E9+lE*v21*5.72408304E9+lE*v27*3.6929331011E10)*2.422459633916196E-9;
	d2_1[nodesPortion.x + 5] = mult * dp5;
	d2_2[nodesPortion.x + 5] = mult * ds5;

}
//*****************************************************************************************
// This function calculates edd = Mhat^(-1)*v for a "noodle"
// made up of 1 ANCF beam element fixed at its left end (i.e.
// constrained by a spherical joint)
//
// As such, the input vector 'v' and the output vector 'edd'
// are assumed to be of length (1+1)*6 - 3 = 9
//
// Note that the matrix Mhat used for this calculation is just
//    Mhat = \int_0^1 { S^T(\xi) S(\xi) d \xi }
// As such, the result must be muliplied by 1/(rho * A * lE)
//
__device__ __host__ inline void minv_vec_fix_1(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v1  = s0.x;
	real_ v2  = s0.y;
	real_ v3  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v4  = p1.x;
	real_ v5  = p1.y;
	real_ v6  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v7  = s1.x;
	real_ v8  = s1.y;
	real_ v9  = s1.z;

	real_ t2 = 1.0/(lE*lE);
	real_ t3 = 1.0/lE;

	real3 ds0;
	ds0.x = t2*(v1*1.0E1+v7*1.3E1+lE*v4)*3.0E1;
	ds0.y = t2*(v2*1.0E1+v8*1.3E1+lE*v5)*3.0E1;
	ds0.z = t2*(v3*1.0E1+v9*1.3E1+lE*v6)*3.0E1;
	d2_2[nodesPortion.x + 0] = ds0;

	real3 dp1, ds1;
	dp1.x = v4*1.5E1+t3*(v1*2.0+v7*7.0)*1.5E1;
	dp1.y = v5*1.5E1+t3*(v2*2.0+v8*7.0)*1.5E1;
	dp1.z = v6*1.5E1+t3*(v3*2.0+v9*7.0)*1.5E1;
	ds1.x = t2*(v1*2.6E1+v7*6.5E1+lE*v4*7.0)*1.5E1;
	ds1.y = t2*(v2*2.6E1+v8*6.5E1+lE*v5*7.0)*1.5E1;
	ds1.z = t2*(v3*2.6E1+v9*6.5E1+lE*v6*7.0)*1.5E1;
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;
}

// Same as above for a 2-element noodle.
// In this case, the input vector 'v' and the output vector 'edd'
// are assumed to be of length (2+1)*6 - 3 = 15
__device__ __host__ inline void minv_vec_fix_2(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v1  = s0.x;
	real_ v2  = s0.y;
	real_ v3  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v4  = p1.x;
	real_ v5  = p1.y;
	real_ v6  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v7  = s1.x;
	real_ v8  = s1.y;
	real_ v9  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v10  = p2.x;
	real_ v11  = p2.y;
	real_ v12  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v13  = s2.x;
	real_ v14  = s2.y;
	real_ v15  = s2.z;

	real_ t2 = 1.0/(lE*lE);
	real_ t3 = 1.0/lE;
	real_ t4 = v4*(1.5E1/1.4E1);
	real_ t5 = v10*(1.95E2/1.4E1);
	real_ t6 = v5*(1.5E1/1.4E1);
	real_ t7 = v11*(1.95E2/1.4E1);
	real_ t8 = v6*(1.5E1/1.4E1);
	real_ t9 = v12*(1.95E2/1.4E1);

	real3 ds0;
	ds0.x  = t2*(v1*1.954821428571429E2+v7*9.696428571428571E1+v13*9.326785714285714E1)-t3*(v4*(1.53E2/2.8E1)-v10*(1.95E2/2.8E1));
	ds0.y  = t2*(v2*1.954821428571429E2+v8*9.696428571428571E1+v14*9.326785714285714E1)-t3*(v5*(1.53E2/2.8E1)-v11*(1.95E2/2.8E1));
	ds0.z  = t2*(v3*1.954821428571429E2+v9*9.696428571428571E1+v15*9.326785714285714E1)-t3*(v6*(1.53E2/2.8E1)-v12*(1.95E2/2.8E1));
	d2_2[nodesPortion.x + 0] = ds0;

	real3 dp1, ds1;
	dp1.x  = v4*(2.7E1/1.4E1)+v10*(1.5E1/1.4E1)+t3*(v1*-1.53E2+v7*3.0E1+v13*3.63E2)*(1.0/2.8E1);
	dp1.y  = v5*(2.7E1/1.4E1)+v11*(1.5E1/1.4E1)+t3*(v2*-1.53E2+v8*3.0E1+v14*3.63E2)*(1.0/2.8E1);
	dp1.z  = v6*(2.7E1/1.4E1)+v12*(1.5E1/1.4E1)+t3*(v3*-1.53E2+v9*3.0E1+v15*3.63E2)*(1.0/2.8E1);
	ds1.x  = t3*(t4+t5)+t2*(v1*9.696428571428571E1+v7*1.339285714285714E2+v13*1.805357142857143E2);
	ds1.y  = t3*(t6+t7)+t2*(v2*9.696428571428571E1+v8*1.339285714285714E2+v14*1.805357142857143E2);
	ds1.z  = t3*(t8+t9)+t2*(v3*9.696428571428571E1+v9*1.339285714285714E2+v15*1.805357142857143E2);
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t4+t5+t3*(v1*1.95E2+v7*3.9E2+v13*2.535E3)*(1.0/2.8E1);
	dp2.y = t6+t7+t3*(v2*1.95E2+v8*3.9E2+v14*2.535E3)*(1.0/2.8E1);
	dp2.z = t8+t9+t3*(v3*1.95E2+v9*3.9E2+v15*2.535E3)*(1.0/2.8E1);
	ds2.x = t2*(v1*9.326785714285714E1+v7*1.805357142857143E2+v13*7.804821428571429E2)+t3*(v4*(3.63E2/2.8E1)+v10*9.053571428571429E1);
	ds2.y = t2*(v2*9.326785714285714E1+v8*1.805357142857143E2+v14*7.804821428571429E2)+t3*(v5*(3.63E2/2.8E1)+v11*9.053571428571429E1);
	ds2.z = t2*(v3*9.326785714285714E1+v9*1.805357142857143E2+v15*7.804821428571429E2)+t3*(v6*(3.63E2/2.8E1)+v12*9.053571428571429E1);
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;
}


// Same as above for a 3-element noodle.
// In this case, the input vector 'v' and the output vector 'edd'
// are assumed to be of length (3+1)*6 - 3 = 21
__device__ __host__ inline void minv_vec_fix_3(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v1  = s0.x;
	real_ v2  = s0.y;
	real_ v3  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v4  = p1.x;
	real_ v5  = p1.y;
	real_ v6  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v7  = s1.x;
	real_ v8  = s1.y;
	real_ v9  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v10  = p2.x;
	real_ v11  = p2.y;
	real_ v12  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v13  = s2.x;
	real_ v14  = s2.y;
	real_ v15  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v16  = p3.x;
	real_ v17  = p3.y;
	real_ v18  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v19  = s3.x;
	real_ v20  = s3.y;
	real_ v21  = s3.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 ds0;
	ds0.x  = t3*(v1*1.887245189979307E2+v7*8.375687932021309E1+v13*2.758332232642099E1+v19*2.610531413727821E1)-t2*(v4*6.432879848544886+v10*1.625500814511513-v16*1.945581825386343);
	ds0.y  = t3*(v2*1.887245189979307E2+v8*8.375687932021309E1+v14*2.758332232642099E1+v20*2.610531413727821E1)-t2*(v5*6.432879848544886+v11*1.625500814511513-v17*1.945581825386343);
	ds0.z  = t3*(v3*1.887245189979307E2+v9*8.375687932021309E1+v15*2.758332232642099E1+v21*2.610531413727821E1)-t2*(v6*6.432879848544886+v12*1.625500814511513-v18*1.945581825386343);
	d2_2[nodesPortion.x + 0] = ds0;

	real3 dp1, ds1;
	dp1.x  = v4*1.78950380839167-v10*1.761105974552019E-1+v16*2.595429929996038E-1-t2*(v1*1.4611E5+v7*1.8685E4-v13*7.895E4-v19*7.8635E4)*4.402764936380047E-5;
	dp1.y  = v5*1.78950380839167-v11*1.761105974552019E-1+v17*2.595429929996038E-1-t2*(v2*1.4611E5+v8*1.8685E4-v14*7.895E4-v20*7.8635E4)*4.402764936380047E-5;
	dp1.z  = v6*1.78950380839167-v12*1.761105974552019E-1+v18*2.595429929996038E-1-t2*(v3*1.4611E5+v9*1.8685E4-v15*7.895E4-v21*7.8635E4)*4.402764936380047E-5;
	ds1.x  = t3*(v1*8.375687932021309E1+v7*1.081109937040461E2+v13*5.184035574340686E1+v19*4.980958041650156E1)-t2*(v4*8.226566283626117E-1+v10*2.930480341654559-v16*3.718795403513406);
	ds1.y  = t3*(v2*8.375687932021309E1+v8*1.081109937040461E2+v14*5.184035574340686E1+v20*4.980958041650156E1)-t2*(v5*8.226566283626117E-1+v11*2.930480341654559-v17*3.718795403513406);
	ds1.z  = t3*(v3*8.375687932021309E1+v9*1.081109937040461E2+v15*5.184035574340686E1+v21*4.980958041650156E1)-t2*(v6*8.226566283626117E-1+v12*2.930480341654559-v18*3.718795403513406);
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = v4*(-1.761105974552019E-1)+v10*1.877338968872452+v16*1.133271694624224-t2*(v1*3.692E4+v7*6.656E4-v13*4.42E4-v19*3.133E5)*4.402764936380047E-5;
	dp2.y = v5*(-1.761105974552019E-1)+v11*1.877338968872452+v17*1.133271694624224-t2*(v2*3.692E4+v8*6.656E4-v14*4.42E4-v20*3.133E5)*4.402764936380047E-5;
	dp2.z = v6*(-1.761105974552019E-1)+v12*1.877338968872452+v18*1.133271694624224-t2*(v3*3.692E4+v9*6.656E4-v15*4.42E4-v21*3.133E5)*4.402764936380047E-5;
	ds2.x = t3*(v1*2.758332232642099E1+v7*5.184035574340686E1+v13*1.190903887641439E2+v19*1.664936380046669E2)+t2*(v4*3.475982917272047+v10*1.946022101879981+v16*1.288204992735438E1);
	ds2.y = t3*(v2*2.758332232642099E1+v8*5.184035574340686E1+v14*1.190903887641439E2+v20*1.664936380046669E2)+t2*(v5*3.475982917272047+v11*1.946022101879981+v17*1.288204992735438E1);
	ds2.z = t3*(v3*2.758332232642099E1+v9*5.184035574340686E1+v15*1.190903887641439E2+v21*1.664936380046669E2)+t2*(v6*3.475982917272047+v12*1.946022101879981+v18*1.288204992735438E1);
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = v4*2.595429929996038E-1+v10*1.133271694624224+v16*1.385484084004755E1+t2*(v1*1.473E4+v7*2.8155E4+v13*9.753E4+v19*6.77955E5)*1.320829480914014E-4;
	dp3.y = v5*2.595429929996038E-1+v11*1.133271694624224+v17*1.385484084004755E1+t2*(v2*1.473E4+v8*2.8155E4+v14*9.753E4+v20*6.77955E5)*1.320829480914014E-4;
	dp3.z = v6*2.595429929996038E-1+v12*1.133271694624224+v18*1.385484084004755E1+t2*(v3*1.473E4+v9*2.8155E4+v15*9.753E4+v21*6.77955E5)*1.320829480914014E-4;
	ds3.x = t3*(v1*2.610531413727821E1+v7*4.980958041650156E1+v13*1.664936380046669E2+v19*7.672049046801391E2)+t2*(v4*3.46211420772245+v10*1.379386254567869E1+v16*8.954629507330604E1);
	ds3.y = t3*(v2*2.610531413727821E1+v8*4.980958041650156E1+v14*1.664936380046669E2+v20*7.672049046801391E2)+t2*(v5*3.46211420772245+v11*1.379386254567869E1+v17*8.954629507330604E1);
	ds3.z = t3*(v3*2.610531413727821E1+v9*4.980958041650156E1+v15*1.664936380046669E2+v21*7.672049046801391E2)+t2*(v6*3.46211420772245+v12*1.379386254567869E1+v18*8.954629507330604E1);
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;
}



// Same as above, but for a "noodle" made up of 4 ANCF beam elements
// In this case, 'v' and 'edd' are of length (4+1)*6 - 3 = 27
//
__device__ __host__ inline void minv_vec_fix_4(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v1  = s0.x;
	real_ v2  = s0.y;
	real_ v3  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v4  = p1.x;
	real_ v5  = p1.y;
	real_ v6  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v7  = s1.x;
	real_ v8  = s1.y;
	real_ v9  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v10  = p2.x;
	real_ v11  = p2.y;
	real_ v12  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v13  = s2.x;
	real_ v14  = s2.y;
	real_ v15  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v16  = p3.x;
	real_ v17  = p3.y;
	real_ v18  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v19  = s3.x;
	real_ v20  = s3.y;
	real_ v21  = s3.z;

	real3 p4 = f1[nodesPortion.x + 4];
	real_ v22  = p4.x;
	real_ v23  = p4.y;
	real_ v24  = p4.z;
	real3 s4 = f2[nodesPortion.x + 4];
	real_ v25  = s4.x;
	real_ v26  = s4.y;
	real_ v27  = s4.z;

	real_ t2 = 1.0/lE;
	real_ t3 = 1.0/(lE*lE);

	real3 ds0;
	ds0.x  = -t2*(v4*6.503863862375572+v10*1.916249670953212+v16*4.61508228607084E-1-v22*5.506232389903382E-1)+t3*(v1*1.881901948322593E2+v7*8.273684970605154E1+v13*2.413765580974758E1+v19*7.813773532938802+v25*7.388856221177939);
	ds0.y  = -t2*(v5*6.503863862375572+v11*1.916249670953212+v17*4.61508228607084E-1-v23*5.506232389903382E-1)+t3*(v2*1.881901948322593E2+v8*8.273684970605154E1+v14*2.413765580974758E1+v20*7.813773532938802+v26*7.388856221177939);
	ds0.z  = -t2*(v6*6.503863862375572+v12*1.916249670953212+v18*4.61508228607084E-1-v24*5.506232389903382E-1)+t3*(v3*1.881901948322593E2+v9*8.273684970605154E1+v15*2.413765580974758E1+v21*7.813773532938802+v27*7.388856221177939);
	d2_2[nodesPortion.x + 0] = ds0;

	real3 dp1, ds1;
	dp1.x  = t2*(v1*-3.557839E6-v7*5.2415E5+v13*1.651069E6+v19*5.6401E5+v25*5.34721E5+lE*v4*9.73762E5-lE*v10*1.1747E5-lE*v16*3.3082E4+lE*v22*3.986E4)*1.82803771119929E-6;
	dp1.y  = t2*(v2*-3.557839E6-v8*5.2415E5+v14*1.651069E6+v20*5.6401E5+v26*5.34721E5+lE*v5*9.73762E5-lE*v11*1.1747E5-lE*v17*3.3082E4+lE*v23*3.986E4)*1.82803771119929E-6;
	dp1.z  = t2*(v3*-3.557839E6-v9*5.2415E5+v15*1.651069E6+v21*5.6401E5+v27*5.34721E5+lE*v6*9.73762E5-lE*v12*1.1747E5-lE*v18*3.3082E4+lE*v24*3.986E4)*1.82803771119929E-6;
	ds1.x  = t3*(v1*8.273684970605154E1+v7*1.061637470873266E2+v13*4.526247879476255E1+v19*1.488616809172362E1+v25*1.408759895777477E1)-t2*(v4*9.58165966325108E-1+v10*3.485537784320799+v16*8.774032602443233E-1-v22*1.0499151790502);
	ds1.y  = t3*(v2*8.273684970605154E1+v8*1.061637470873266E2+v14*4.526247879476255E1+v20*1.488616809172362E1+v26*1.408759895777477E1)-t2*(v5*9.58165966325108E-1+v11*3.485537784320799+v17*8.774032602443233E-1-v23*1.0499151790502);
	ds1.z  = t3*(v3*8.273684970605154E1+v9*1.061637470873266E2+v15*4.526247879476255E1+v21*1.488616809172362E1+v27*1.408759895777477E1)-t2*(v6*9.58165966325108E-1+v12*3.485537784320799+v18*8.774032602443233E-1-v24*1.0499151790502);
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t2*(v1*-2.09651E5-v7*3.81342E5+v13*7.657E3+v19*4.1197E5+v25*4.08733E5-lE*v4*2.3494E4+lE*v10*1.88058E5-lE*v16*2.1138E4+lE*v22*3.0628E4)*9.140188555996451E-6;
	dp2.y = t2*(v2*-2.09651E5-v8*3.81342E5+v14*7.657E3+v20*4.1197E5+v26*4.08733E5-lE*v5*2.3494E4+lE*v11*1.88058E5-lE*v17*2.1138E4+lE*v23*3.0628E4)*9.140188555996451E-6;
	dp2.z = t2*(v3*-2.09651E5-v9*3.81342E5+v15*7.657E3+v21*4.1197E5+v27*4.08733E5-lE*v6*2.3494E4+lE*v12*1.88058E5-lE*v18*2.1138E4+lE*v24*3.0628E4)*9.140188555996451E-6;
	ds2.x = t3*(v1*2.413765580974758E1+v7*4.526247879476255E1+v13*9.686556214597003E1+v19*4.819976978911757E1+v25*4.636695998547319E1)+t2*(v4*3.018216395792101+v10*6.998642377326483E-2-v16*2.715456790063274+v22*3.46224858388012);
	ds2.y = t3*(v2*2.413765580974758E1+v8*4.526247879476255E1+v14*9.686556214597003E1+v20*4.819976978911757E1+v26*4.636695998547319E1)+t2*(v5*3.018216395792101+v11*6.998642377326483E-2-v17*2.715456790063274+v23*3.46224858388012);
	ds2.z = t3*(v3*2.413765580974758E1+v9*4.526247879476255E1+v15*9.686556214597003E1+v21*4.819976978911757E1+v27*4.636695998547319E1)+t2*(v6*3.018216395792101+v12*6.998642377326483E-2-v18*2.715456790063274+v24*3.46224858388012);
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = t2*(v1*2.52461E5+v7*4.7997E5+v13*1.485449E6-v19*1.10259E6-v25*7.581699E6+lE*v4*3.3082E4+lE*v10*1.0569E5-lE*v16*1.024722E6-lE*v22*6.2262E5)*(-1.82803771119929E-6);
	dp3.y = t2*(v2*2.52461E5+v8*4.7997E5+v14*1.485449E6-v20*1.10259E6-v26*7.581699E6+lE*v5*3.3082E4+lE*v11*1.0569E5-lE*v17*1.024722E6-lE*v23*6.2262E5)*(-1.82803771119929E-6);
	dp3.z = t2*(v3*2.52461E5+v9*4.7997E5+v15*1.485449E6-v21*1.10259E6-v27*7.581699E6+lE*v6*3.3082E4+lE*v12*1.0569E5-lE*v18*1.024722E6-lE*v24*6.2262E5)*(-1.82803771119929E-6);
	ds3.x = t3*(v1*7.813773532938802+v7*1.488616809172362E1+v13*4.819976978911757E1+v19*1.179129110647467E2+v25*1.65380240984118E2)+t2*(v4*1.031031549493512+v10*3.765483479413858+v16*2.015576099991225+v22*1.279907915647028E1);
	ds3.y = t3*(v2*7.813773532938802+v8*1.488616809172362E1+v14*4.819976978911757E1+v20*1.179129110647467E2+v26*1.65380240984118E2)+t2*(v5*1.031031549493512+v11*3.765483479413858+v17*2.015576099991225+v23*1.279907915647028E1);
	ds3.z = t3*(v3*7.813773532938802+v9*1.488616809172362E1+v15*4.819976978911757E1+v21*1.179129110647467E2+v27*1.65380240984118E2)+t2*(v6*1.031031549493512+v12*3.765483479413858+v18*2.015576099991225+v24*1.279907915647028E1);
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;

	real3 dp4, ds4;
	dp4.x = t2*(v1*3.0121E4+v7*5.7434E4+v13*1.89397E5+v19*7.00154E5+v25*4.894201E6+lE*v4*3.986E3+lE*v10*1.5314E4+lE*v16*6.2262E4+lE*v22*7.57588E5)*1.82803771119929E-5;
	dp4.y = t2*(v2*3.0121E4+v8*5.7434E4+v14*1.89397E5+v20*7.00154E5+v26*4.894201E6+lE*v5*3.986E3+lE*v11*1.5314E4+lE*v17*6.2262E4+lE*v23*7.57588E5)*1.82803771119929E-5;
	dp4.z = t2*(v3*3.0121E4+v9*5.7434E4+v15*1.89397E5+v21*7.00154E5+v27*4.894201E6+lE*v6*3.986E3+lE*v12*1.5314E4+lE*v18*6.2262E4+lE*v24*7.57588E5)*1.82803771119929E-5;
	ds4.x = t3*(v1*7.388856221177939+v7*1.408759895777477E1+v13*4.636695998547319E1+v19*1.65380240984118E2+v25*7.661521034011251E2)+t2*(v4*9.774901529701957E-1+v10*3.735896689058097+v16*1.385963168696195E1+v22*8.946783994189277E1);
	ds4.y = t3*(v2*7.388856221177939+v8*1.408759895777477E1+v14*4.636695998547319E1+v20*1.65380240984118E2+v26*7.661521034011251E2)+t2*(v5*9.774901529701957E-1+v11*3.735896689058097+v17*1.385963168696195E1+v23*8.946783994189277E1);
	ds4.z = t3*(v3*7.388856221177939+v9*1.408759895777477E1+v15*4.636695998547319E1+v21*1.65380240984118E2+v27*7.661521034011251E2)+t2*(v6*9.774901529701957E-1+v12*3.735896689058097+v18*1.385963168696195E1+v24*8.946783994189277E1);
	d2_1[nodesPortion.x + 4] = mult * dp4;
	d2_2[nodesPortion.x + 4] = mult * ds4;
}


// Same as above, but for a "noodle" made up of 5 ANCF beam elements
// In this case, 'v' and 'edd' are of length (5+1)*6 - 3 = 33
//
__device__ __host__ inline void minv_vec_fix_5(real3 * d2_1, real3 * d2_2, const real3 * f1, const real3 * f2, real_ mult, real_ lE, const int2 & nodesPortion)
{
	real3 s0 = f2[nodesPortion.x + 0];
	real_ v1  = s0.x;
	real_ v2  = s0.y;
	real_ v3  = s0.z;

	real3 p1 = f1[nodesPortion.x + 1];
	real_ v4  = p1.x;
	real_ v5  = p1.y;
	real_ v6  = p1.z;
	real3 s1 = f2[nodesPortion.x + 1];
	real_ v7  = s1.x;
	real_ v8  = s1.y;
	real_ v9  = s1.z;

	real3 p2 = f1[nodesPortion.x + 2];
	real_ v10  = p2.x;
	real_ v11  = p2.y;
	real_ v12  = p2.z;
	real3 s2 = f2[nodesPortion.x + 2];
	real_ v13  = s2.x;
	real_ v14  = s2.y;
	real_ v15  = s2.z;

	real3 p3 = f1[nodesPortion.x + 3];
	real_ v16  = p3.x;
	real_ v17  = p3.y;
	real_ v18  = p3.z;
	real3 s3 = f2[nodesPortion.x + 3];
	real_ v19  = s3.x;
	real_ v20  = s3.y;
	real_ v21  = s3.z;

	real3 p4 = f1[nodesPortion.x + 4];
	real_ v22  = p4.x;
	real_ v23  = p4.y;
	real_ v24  = p4.z;
	real3 s4 = f2[nodesPortion.x + 4];
	real_ v25  = s4.x;
	real_ v26  = s4.y;
	real_ v27  = s4.z;

	real3 p5 = f1[nodesPortion.x + 5];
	real_ v28  = p5.x;
	real_ v29  = p5.y;
	real_ v30  = p5.z;
	real3 s5 = f2[nodesPortion.x + 5];
	real_ v31  = s5.x;
	real_ v32  = s5.y;
	real_ v33  = s5.z;

	real_ t2 = 1.0/(lE*lE);
	real_ t3 = 1.0/lE;

	real3 ds0;
	ds0.x  = t2*(v1*1.881473562757477E2+v7*8.265517149571199E1+v13*2.386867556831759E1+v19*6.844189400445428+v25*2.213629348378043+v31*2.09315958967146)-t3*(v4*6.50953158583498+v10*1.937944341038472+v16*5.442486518402458E-1+v22*1.307597605366659E-1-v28*1.559830545694713E-1);
	ds0.y  = t2*(v2*1.881473562757477E2+v8*8.265517149571199E1+v14*2.386867556831759E1+v20*6.844189400445428+v26*2.213629348378043+v32*2.09315958967146)-t3*(v5*6.50953158583498+v11*1.937944341038472+v17*5.442486518402458E-1+v23*1.307597605366659E-1-v29*1.559830545694713E-1);
	ds0.z  = t2*(v3*1.881473562757477E2+v9*8.265517149571199E1+v15*2.386867556831759E1+v21*6.844189400445428+v27*2.213629348378043+v33*2.09315958967146)-t3*(v6*6.50953158583498+v12*1.937944341038472+v18*5.442486518402458E-1+v24*1.307597605366659E-1-v30*1.559830545694713E-1);
	d2_2[nodesPortion.x + 0] = ds0;

	real3 dp1, ds1;
	dp1.x  = t3*(v1*-1.713863838E9-v7*2.55116153E8+v13*7.8528235E8+v19*2.37681103E8+v25*7.708195E7+v31*7.2896767E7+lE*v4*4.68469761E8-lE*v10*5.729348E7-lE*v16*1.8804389E7-lE*v22*4.551628E6+lE*v28*5.432381E6)*3.798161465050399E-9;
	dp1.y  = t3*(v2*-1.713863838E9-v8*2.55116153E8+v14*7.8528235E8+v20*2.37681103E8+v26*7.708195E7+v32*7.2896767E7+lE*v5*4.68469761E8-lE*v11*5.729348E7-lE*v17*1.8804389E7-lE*v23*4.551628E6+lE*v29*5.432381E6)*3.798161465050399E-9;
	dp1.z  = t3*(v3*-1.713863838E9-v9*2.55116153E8+v15*7.8528235E8+v21*2.37681103E8+v27*7.708195E7+v33*7.2896767E7+lE*v6*4.68469761E8-lE*v12*5.729348E7-lE*v18*1.8804389E7-lE*v24*4.551628E6+lE*v30*5.432381E6)*3.798161465050399E-9;
	ds1.x  = t2*(v1*8.265517149571199E1+v7*1.060080151813911E2+v13*4.474962712189174E1+v19*1.303750776159676E1+v25*4.220174463781471+v31*3.99066544872863)-t3*(v4*9.689723414365018E-1+v10*3.526901969101384+v16*1.035160660372671+v22*2.492602256978803E-1-v28*2.973873335292069E-1);
	ds1.y  = t2*(v2*8.265517149571199E1+v8*1.060080151813911E2+v14*4.474962712189174E1+v20*1.303750776159676E1+v26*4.220174463781471+v32*3.99066544872863)-t3*(v5*9.689723414365018E-1+v11*3.526901969101384+v17*1.035160660372671+v23*2.492602256978803E-1-v29*2.973873335292069E-1);
	ds1.z  = t2*(v3*8.265517149571199E1+v9*1.060080151813911E2+v15*4.474962712189174E1+v21*1.303750776159676E1+v27*4.220174463781471+v33*3.99066544872863)-t3*(v6*9.689723414365018E-1+v12*3.526901969101384+v18*1.035160660372671+v24*2.492602256978803E-1-v30*2.973873335292069E-1);
	d2_1[nodesPortion.x + 1] = mult * dp1;
	d2_2[nodesPortion.x + 1] = mult * ds1;

	real3 dp2, ds2;
	dp2.x = t3*(v1*1.27558054E8+v7*2.32145342E8+v13*4.35955E6-v19*2.15528105E8-v25*7.331909E7-v31*6.9498065E7+lE*v4*1.432337E7-lE*v10*1.12416148E8+lE*v16*1.5475317E7+lE*v22*4.302792E6-lE*v28*5.180513E6)*(-1.51926458602016E-8);
	dp2.y = t3*(v2*1.27558054E8+v8*2.32145342E8+v14*4.35955E6-v20*2.15528105E8-v26*7.331909E7-v32*6.9498065E7+lE*v5*1.432337E7-lE*v11*1.12416148E8+lE*v17*1.5475317E7+lE*v23*4.302792E6-lE*v29*5.180513E6)*(-1.51926458602016E-8);
	dp2.z = t3*(v3*1.27558054E8+v9*2.32145342E8+v15*4.35955E6-v21*2.15528105E8-v27*7.331909E7-v33*6.9498065E7+lE*v6*1.432337E7-lE*v12*1.12416148E8+lE*v18*1.5475317E7+lE*v24*4.302792E6-lE*v30*5.180513E6)*(-1.51926458602016E-8);
	ds2.x = -t3*(v4*(-2.98262916095422)+v10*6.623309925984187E-2+v16*3.234994155760091+v22*8.17207619944152E-1-v28*9.781078655021528E-1)+t2*(v1*2.386867556831759E1+v7*4.474962712189174E1+v13*9.517665332660107E1+v19*4.211174737661576E1+v25*1.386711675474229E1+v31*1.312400606150006E1);
	ds2.y = -t3*(v5*(-2.98262916095422)+v11*6.623309925984187E-2+v17*3.234994155760091+v23*8.17207619944152E-1-v29*9.781078655021528E-1)+t2*(v2*2.386867556831759E1+v8*4.474962712189174E1+v14*9.517665332660107E1+v20*4.211174737661576E1+v26*1.386711675474229E1+v32*1.312400606150006E1);
	ds2.z = -t3*(v6*(-2.98262916095422)+v12*6.623309925984187E-2+v18*3.234994155760091+v24*8.17207619944152E-1-v30*9.781078655021528E-1)+t2*(v3*2.386867556831759E1+v9*4.474962712189174E1+v15*9.517665332660107E1+v21*4.211174737661576E1+v27*1.386711675474229E1+v33*1.312400606150006E1);
	d2_1[nodesPortion.x + 2] = mult * dp2;
	d2_2[nodesPortion.x + 2] = mult * ds2;

	real3 dp3, ds3;
	dp3.x = t3*(v1*1.43292658E8+v7*2.72542563E8+v13*8.51726338E8-v19*3.7326357E7-v25*9.9750897E8-v31*9.89386533E8+lE*v4*1.8804389E7+lE*v10*6.1901268E7-lE*v16*4.51054233E8+lE*v22*5.1229188E7-lE*v28*7.4136303E7)*(-3.798161465050399E-9);
	dp3.y = t3*(v2*1.43292658E8+v8*2.72542563E8+v14*8.51726338E8-v20*3.7326357E7-v26*9.9750897E8-v32*9.89386533E8+lE*v5*1.8804389E7+lE*v11*6.1901268E7-lE*v17*4.51054233E8+lE*v23*5.1229188E7-lE*v29*7.4136303E7)*(-3.798161465050399E-9);
	dp3.z = t3*(v3*1.43292658E8+v9*2.72542563E8+v15*8.51726338E8-v21*3.7326357E7-v27*9.9750897E8-v33*9.89386533E8+lE*v6*1.8804389E7+lE*v12*6.1901268E7-lE*v18*4.51054233E8+lE*v24*5.1229188E7-lE*v30*7.4136303E7)*(-3.798161465050399E-9);
	ds3.x = t3*(v4*9.027512063852748E-1+v10*3.274442172185345+v16*1.417715307881142E-1-v22*2.698212036076363+v28*3.441677526975523)+t2*(v1*6.844189400445428+v7*1.303750776159676E1+v13*4.211174737661576E1+v19*9.596290664845758E1+v25*4.790783545013705E1+v31*4.609091386871998E1);
	ds3.y = t3*(v5*9.027512063852748E-1+v11*3.274442172185345+v17*1.417715307881142E-1-v23*2.698212036076363+v29*3.441677526975523)+t2*(v2*6.844189400445428+v8*1.303750776159676E1+v14*4.211174737661576E1+v20*9.596290664845758E1+v26*4.790783545013705E1+v32*4.609091386871998E1);
	ds3.z = t3*(v6*9.027512063852748E-1+v12*3.274442172185345+v18*1.417715307881142E-1-v24*2.698212036076363+v30*3.441677526975523)+t2*(v3*6.844189400445428+v9*1.303750776159676E1+v15*4.211174737661576E1+v21*9.596290664845758E1+v27*4.790783545013705E1+v33*4.609091386871998E1);
	d2_1[nodesPortion.x + 3] = mult * dp3;
	d2_2[nodesPortion.x + 3] = mult * ds3;

	real3 dp4, ds4;
	dp4.x = t3*(v1*8.60678E6+v7*1.6406637E7+v13*5.3789684E7+v19*1.77599877E8-v25*1.3303498E8-v31*9.1260637E8+lE*v4*1.137907E6+lE*v10*4.302792E6+lE*v16*1.2807297E7-lE*v22*1.23276816E8-lE*v28*7.4941906E7)*(-1.51926458602016E-8);
	dp4.y = t3*(v2*8.60678E6+v8*1.6406637E7+v14*5.3789684E7+v20*1.77599877E8-v26*1.3303498E8-v32*9.1260637E8+lE*v5*1.137907E6+lE*v11*4.302792E6+lE*v17*1.2807297E7-lE*v23*1.23276816E8-lE*v29*7.4941906E7)*(-1.51926458602016E-8);
	dp4.z = t3*(v3*8.60678E6+v9*1.6406637E7+v15*5.3789684E7+v21*1.77599877E8-v27*1.3303498E8-v33*9.1260637E8+lE*v6*1.137907E6+lE*v12*4.302792E6+lE*v18*1.2807297E7-lE*v24*1.23276816E8-lE*v30*7.4941906E7)*(-1.51926458602016E-8);
	ds4.x = t3*(v4*2.927696921409416E-1+v10*1.113910969162248+v16*3.788700130896115+v22*2.021153338159002+v28*1.279242614020708E1)+t2*(v1*2.213629348378043+v7*4.220174463781471+v13*1.386711675474229E1+v19*4.790783545013705E1+v25*1.178184947112758E2+v31*1.652909631536125E2);
	ds4.y = t3*(v5*2.927696921409416E-1+v11*1.113910969162248+v17*3.788700130896115+v23*2.021153338159002+v29*1.279242614020708E1)+t2*(v2*2.213629348378043+v8*4.220174463781471+v14*1.386711675474229E1+v20*4.790783545013705E1+v26*1.178184947112758E2+v32*1.652909631536125E2);
	ds4.z = t3*(v6*2.927696921409416E-1+v12*1.113910969162248+v18*3.788700130896115+v24*2.021153338159002+v30*1.279242614020708E1)+t2*(v3*2.213629348378043+v9*4.220174463781471+v15*1.386711675474229E1+v21*4.790783545013705E1+v27*1.178184947112758E2+v33*1.652909631536125E2);
	d2_1[nodesPortion.x + 4] = mult * dp4;
	d2_2[nodesPortion.x + 4] = mult * ds4;

	real3 dp5, ds5;
	dp5.x = t3*(v1*4.1068042E7+v7*7.8297707E7+v13*2.57521402E8+v19*9.06143027E8+v25*3.368057482E9+v31*2.3553908867E10+lE*v4*5.432381E6+lE*v10*2.0722052E7+lE*v16*7.4136303E7+lE*v22*2.99767624E8+lE*v28*3.646112905E9)*3.798161465050399E-9;
	dp5.y = t3*(v2*4.1068042E7+v8*7.8297707E7+v14*2.57521402E8+v20*9.06143027E8+v26*3.368057482E9+v32*2.3553908867E10+lE*v5*5.432381E6+lE*v11*2.0722052E7+lE*v17*7.4136303E7+lE*v23*2.99767624E8+lE*v29*3.646112905E9)*3.798161465050399E-9;
	dp5.z = t3*(v3*4.1068042E7+v9*7.8297707E7+v15*2.57521402E8+v21*9.06143027E8+v27*3.368057482E9+v33*2.3553908867E10+lE*v6*5.432381E6+lE*v12*2.0722052E7+lE*v18*7.4136303E7+lE*v24*2.99767624E8+lE*v30*3.646112905E9)*3.798161465050399E-9;
	ds5.x = t3*(v4*2.768736913461576E-1+v10*1.055859489514271+v16*3.757849803680415+v22*1.386490538917411E1+v28*8.94615490099483E1)+t2*(v1*2.09315958967146+v7*3.99066544872863+v13*1.312400606150006E1+v19*4.609091386871998E1+v25*1.652909631536125E2+v31*7.660676844347409E2);
	ds5.y = t3*(v5*2.768736913461576E-1+v11*1.055859489514271+v17*3.757849803680415+v23*1.386490538917411E1+v29*8.94615490099483E1)+t2*(v2*2.09315958967146+v8*3.99066544872863+v14*1.312400606150006E1+v20*4.609091386871998E1+v26*1.652909631536125E2+v32*7.660676844347409E2);
	ds5.z = t3*(v6*2.768736913461576E-1+v12*1.055859489514271+v18*3.757849803680415+v24*1.386490538917411E1+v30*8.94615490099483E1)+t2*(v3*2.09315958967146+v9*3.99066544872863+v15*1.312400606150006E1+v21*4.609091386871998E1+v27*1.652909631536125E2+v33*7.660676844347409E2);
	d2_1[nodesPortion.x + 5] = mult * dp5;
	d2_2[nodesPortion.x + 5] = mult * ds5;
}

#endif
