#ifndef CHCUDA_H
#define CHCUDA_H

//////////////////////////////////////////////////
//
//   ChCuda.h
//
///////////////////////////////////////////////////
#include <time.h>
#include <iostream>
#include <cutil.h>
#include <cutil_math.h>
#include <cutil_inline.h>
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/constant_iterator.h>
#include "ChApiGPU.h"

using namespace std;

typedef unsigned int uint;

#define CH_REALNUMBER4 float4
#define CH_REALNUMBER3 float3
#define CH_REALNUMBER2 float2
#define CH_REALNUMBER  float 

#define F3	make_float3
#define F4	make_float4
#define I3	make_int3
#define U3	make_uint3
#define I3F make_int3f

//defines to cast thrust vectors as raw pointers
#define CASTU1(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define CASTI1(x) (int*)thrust::raw_pointer_cast(&x[0])
#define CASTI2(x) (int2*)thrust::raw_pointer_cast(&x[0])
#define CASTI3(x) (int3*)thrust::raw_pointer_cast(&x[0])
#define CASTI4(x) (int4*)thrust::raw_pointer_cast(&x[0])
#define CASTF1(x) (float*)thrust::raw_pointer_cast(&x[0])
#define CASTF2(x) (float2*)thrust::raw_pointer_cast(&x[0])
#define CASTF3(x) (float3*)thrust::raw_pointer_cast(&x[0])
#define CASTF4(x) (float4*)thrust::raw_pointer_cast(&x[0])

#define BDCAST(x) (bodyData*)thrust::raw_pointer_cast(&x[0])

struct __align__(16) int3f
{
  int x,y,z;
  float w;
};


static __inline__ __host__ __device__ int3f make_int3f(int x, int y, int z, float w)
{
  int3f t; 
  t.x = x; 
  t.y = y; 
  t.z = z; 
  t.w = w; 
  return t;
}

#define CASTI3F(x) (int3f*)thrust::raw_pointer_cast(&x[0])

//custom version of ceil used for float3's
inline __host__ __device__ float3 ceil(float3 v)
{
	return make_float3(ceil(v.x), ceil(v.y), ceil(v.z));
}


//////////////////////////////////////////////////

#define CH_CONTACT_VSIZE 10
#define CH_CONTACT_HSIZE sizeof(CH_REALNUMBER4)

#define CH_BODY_VSIZE 7
#define CH_BODY_HSIZE sizeof(CH_REALNUMBER4)

#define CH_BILATERAL_VSIZE 5
#define CH_BILATERAL_HSIZE sizeof(CH_REALNUMBER4)

#define CH_REDUCTION_VSIZE 2
#define CH_REDUCTION_HSIZE sizeof(CH_REALNUMBER4)


#define CH_PREPROCESSING_SH_MEM_BLOCK_SIZE 13 //note that this odd number will ensure no bank conflicts

#ifndef CH_CUDAGPUEMULATION
	//***ALEX*** TO DO: FIND OPTIMAL VALUES FOR THESE DEFs, SEE OCCUPANCY & .cubin

   // optimized values for cuda 1.1 (to be improved!!)
 #define CH_PREPROCESSING_TPB 128   
 #define CH_LCPADDFORCES_TPB 128
 #define CH_LCPITERATION_TPB 128
 #define CH_LCPITERATIONBILATERALS_TPB 128
 #define CH_LCPINTEGRATE_TPB 128
 #define CH_REDUCTION_TPB 128
 #define CH_SPEEDUPDATE_TPB 128
#else
 // for device emulation (using too may threads slow down things too much..)
 #define CH_PREPROCESSING_TPB 8
 #define CH_LCPADDFORCES_TPB 8
 #define CH_LCPITERATION_TPB 8
 #define CH_LCPITERATIONBILATERALS_TPB 8
 #define CH_LCPINTEGRATE_TPB 8
 #define CH_REDUCTION_TPB 8
 #define CH_SPEEDUPDATE_TPB 8
#endif



#endif

// Kernels for solving the CCP complementarity problem in GPU. 
//
//  These kernels expects to find the data arranged as blocks of
//  float4 data (columns of N 'four4' structures) in horizontal
//  buffers called 'contacts', 'bilaterals, 'bodies' etc.
//
//  The data model is represented below, using some ascii-art.
//  In the following schemes, we use these symbols:
//   B1 and B2 are indexes pointing to bodies in body buffer. For inactive body, B1=-1 or B2=-1
//   R1 and R2 are indexes pointing to reduction buffer cell. Senseless for inactive bodies.
//   n1 and n2 tell the repetition index of the body (0,1,2,..) in reduction buffer.
// 
//   
//  'contacts' buffer is made with an horizontal array of:
//		[             , bx ]      0
//		[ matr.J12(x) ,  0 ]      1
//		[_____________,  0 ]      2
//		[             , B1 ]	  3
//		[ matr.J1(w)  ,    ]	  4
//		[_____________,    ]	  5
//		[             , B2 ]	  6
//		[ matr.J2(w)  ,    ]	  7
//		[_____________, et ]	  8
//		[ gx,  gy,  gz, mu ]      9
//   
//  'bilaterals' buffer is made with an horizontal array of:
//		[ matr.J1(x)  , B1 ]      0
//		[ matr.J2(x)  , B2 ]      1
//		[ matr.J1(w)  ,    ]      2
//		[ matr.J2(w)  ,    ]      3
//      [ eta, b, g   ,  u ]      4      u=1 if unilateral, 0 if bilateral
//      [ R1 , R2, n1 , n2 ]      5      index to fill the reduction buffer, and n repetition
//
//  'bodies' buffer is made with an horizontal array of:  
//		[ vx, vy, vz  , S  ]       0     S= state of body (active or not)
//		[ wx, wy, wz  ,    ]       1
//		[ xx, xy, xz  , -  ]       2
//		[ q0, q1, q2 , q3  ]       3
//		[iJ1,iJ2,iJ3  ,im  ]       4
//		[ fx, fy, fz  , -  ]       5
//		[ cx, cy, cz  , -  ]       6
//
//  'reduction' buffer is made with an horizontal array of:
//		[ vx, vy, vz  , - ]       0
//		[ wx, wy, wz  , n ]       1   n=repetition (0=no repetition, also 0=final accumulators)
//
//  'variables' buffer is made with an horizontal array of: (optional to bodies for future development)
//		[ vx, vy, vz  , R  ]       0     R= body index in reduction buffer
//		[ wx, wy, wz  ,    ]       1
//		[ xx, xy, xz  , -  ]       2
//		[ q0, q1, q2 , q3  ]       3
//		[iJ1,iJ2,iJ3  ,im  ]       4
//		[ fx, fy, fz  , -  ]       5
//		[ cx, cy, cz  , -  ]       6
//
//  Note that the contacts are uploaded to the GPU in a compressed format, 
//  that is later transformed in the above 'contact' buffer thank to the ChKernelContactsPreprocess
//  kernel. Such kernel prepares the 'contacts' buffer in-place, starting from the state below:
//
//  'contacts' buffer  before preprocessing:
//		[   Normal    , 0  ]      0
//		[  -   -   -  ,  - ]      1
//		[  -   -   -  ,  - ]      2
//		[     P1      , B1 ]	  3
//		[  -   -   -  ,  - ]	  4
//		[  -   -   -  ,  - ]	  5
//		[     P2      , B2 ]	  6
//		[  -   -   -  ,  - ]	  7
//		[  -   -   -  ,  - ]	  8
//		[ gx,  gy,  gz, mu ]      9
//      [  -   -   -  ,  - ]     10  