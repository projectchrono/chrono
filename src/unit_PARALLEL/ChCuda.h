#ifndef CHCUDA_H
#define CHCUDA_H

//////////////////////////////////////////////////
//
//   ChCuda.h
//
///////////////////////////////////////////////////
#define THRUST_DEVICE_BACKEND THRUST_DEVICE_BACKEND_CUDA
//#define THRUST_DEBUG
#include <time.h>
#include <iostream>
#include <cutil.h>
#include <cutil_math.h>
#include <cutil_inline.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/count.h>
#include <thrust/scan.h>
#include <thrust/sequence.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/set_operations.h>
#include <thrust/functional.h>
#include "ChApiGPU.h"
#include <omp.h>
using namespace std;
using namespace thrust;
typedef unsigned int uint;
#ifdef __CDT_PARSER__
#define __host__
#define __device__
#define __global__
#define __constant__
#define __shared__
#define CUDA_KERNEL_DIM(...) ()
#else
#define CUDA_KERNEL_DIM(...)  <<< __VA_ARGS__ >>>
#endif

#define Zero_Vector make_float3(0,0,0)
#define PI  3.1415926535897932384626433832795
#define PI_2   (PI / 2.0f)
#define PI_180  (PI / 180.0f)

#define CH_REALNUMBER4 float4
#define CH_REALNUMBER3 float3
#define CH_REALNUMBER2 float2
#define CH_REALNUMBER  float

#define F3	make_float3
#define F4	make_float4
#define F2	make_float2
#define I4  make_int4
#define I3	make_int3
#define I2	make_int2
#define U3	make_uint3
#define I3F make_int3f

//defines to cast thrust vectors as raw pointers
#define CASTC1(x) (char*)thrust::raw_pointer_cast(&x[0])
#define CASTU1(x) (uint*)thrust::raw_pointer_cast(&x[0])
#define CASTU2(x) (uint2*)thrust::raw_pointer_cast(&x[0])
#define CASTU3(x) (uint3*)thrust::raw_pointer_cast(&x[0])
#define CASTI1(x) (int*)thrust::raw_pointer_cast(&x[0])
#define CASTLL(x) (long long*)thrust::raw_pointer_cast(&x[0])
#define CASTI2(x) (int2*)thrust::raw_pointer_cast(&x[0])
#define CASTI3(x) (int3*)thrust::raw_pointer_cast(&x[0])
#define CASTI4(x) (int4*)thrust::raw_pointer_cast(&x[0])
#define CASTF1(x) (float*)thrust::raw_pointer_cast(&x[0])
#define CASTF2(x) (float2*)thrust::raw_pointer_cast(&x[0])
#define CASTF3(x) (float3*)thrust::raw_pointer_cast(&x[0])
#define CASTF4(x) (float4*)thrust::raw_pointer_cast(&x[0])

#define OBJCAST(x)  (object*)thrust::raw_pointer_cast(&x[0])
#define AABBCAST(x) (AABB*)thrust::raw_pointer_cast(&x[0])
#define CONTCAST(x)	(contactGPU*)thrust::raw_pointer_cast(&x[0])

#define CHVECCAST(v) ChVector<>(v.x,v.y,v.z)
#define CHQUATCAST(q) ChQuaternion<>(q.x,q.y,q.z,q.w)

#define THREADS							128
#define MAXBLOCK						65535
#define BLOCKS(x)						max((int)ceil(x/(float)THREADS),1)
#define BLOCKS_T(x,y)					max((int)ceil(x/(float)y),1)
#define BLOCKS2D(x)						dim3(min(MAXBLOCK,BLOCKS(x)),ceil(BLOCKS(x)/(float)MAXBLOCK),1)
#define COPY_TO_CONST_MEM(x)			cudaMemcpyToSymbolAsync(x##_const,	&x,	sizeof(x),0,cudaMemcpyHostToDevice)
#define START_TIMING(x,y,z) 			cudaEventCreate(&x); cudaEventCreate(&y); cudaEventRecord(x, 0); z=0;
#define STOP_TIMING(x,y,z) 				cudaThreadSynchronize(); cudaEventRecord(y, 0); cudaEventSynchronize(y); cudaEventElapsedTime(&z,x , y); cudaEventDestroy(x);  cudaEventDestroy(y);
#define BIND_TEXF4(x)					cudaBindTexture(NULL, x##_tex,   CASTF4(x),   x.size()*sizeof(float4));
#define BIND_TEXU1(x)					cudaBindTexture(NULL, x##_tex,   CASTU1(x),   x.size()*sizeof(uint1));
#define UNBIND_TEX(x)					cudaUnbindTexture( x##_tex );

#define Thrust_Inclusive_Scan_Sum(x,y)	thrust::inclusive_scan(x.begin(),x.end(), x.begin()); y=x.back();
#define Thrust_Sort_By_Key(x,y)			thrust::sort_by_key(x.begin(),x.end(),y.begin())
#define Thrust_Reduce_By_KeyA(x,y,z)x=  thrust::reduce_by_key(y.begin(),y.end(),thrust::constant_iterator<uint>(1),y.begin(),z.begin()).first-y.begin()
#define Thrust_Reduce_By_KeyB(x,y,z,w)x=thrust::reduce_by_key(y.begin(),y.end(),thrust::constant_iterator<uint>(1),z.begin(),w.begin()).first-z.begin()
#define Thrust_Inclusive_Scan(x)		thrust::inclusive_scan(x.begin(), x.end(), x.begin())
#define Thrust_Fill(x,y)				thrust::fill(x.begin(),x.end(),y)
#define Thrust_Sort(x)					thrust::sort(x.begin(),x.end())
#define Thrust_Count(x,y)				thrust::count(x.begin(),x.end(),y)
#define Thrust_Sequence(x)				thrust::sequence(x.begin(),x.end())
#define Thrust_Equal(x,y)				thrust::equal(x.begin(),x.end(), y.begin())
#define Thrust_Max(x)					x[thrust::max_element(x.begin(),x.end())-x.begin()]
#define Thrust_Min(x)					x[thrust::max_element(x.begin(),x.end())-x.begin()]
#define Thrust_Total(x)					thrust::reduce(x.begin(),x.end())
#define DBG(x)							printf(x);CUT_CHECK_ERROR(x);

#define	_SPHERE 0
#define	_ELLIPSOID 1
#define	_BOX 2
#define	_CYLINDER 3
#define	_CONVEXHULL 4
#define	_TRIANGLEMESH 5
#define	_BARREL 6
#define	_RECT 7				//Currently implemented on GPU only
#define	_DISC 8				//Currently implemented on GPU only
#define	_ELLIPSE 9			//Currently implemented on GPU only
#define	_CAPSULE 10			//Currently implemented on GPU only
#define	_CONE 11				//Currently implemented on GPU only

#define Vector_ZERO_EPSILON 1e-8
#define MIN_ZERO_EPSILON 1.1754943508222875E-38
#define EPS FLT_EPSILON

__device__ __host__ bool operator ==(const uint3 &a, const uint3 &b) {
	return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}

__device__ __host__ inline void Swap(float3& a, float3& b) {
	float3 tmp = a;
	a = b;
	b = tmp;
}

__device__ __host__ inline float4 inv(const float4& a) {
	return (1.0f / (dot(a, a))) * F4(a.x, -a.y, -a.z, -a.w);
}

//__device__ __host__ inline float4 operator ~(const float4& a)
//{
//	return 1.0/(dot(a,a))*F4(a.x, -a.y, -a.z, -a.w);
//}

inline __host__  __device__ float4 make_float4(float w, float3 a) {
	return make_float4(w, a.x, a.y, a.z);
}

__device__ __host__ inline float4 mult(const float4 &a, const float4 &b) {
	float w0 = a.x;
	float w1 = b.x;
	float3 v0 = F3(a.y, a.z, a.w);
	float3 v1 = F3(b.y, b.z, b.w);
	float4 quat = F4(w0 * w1 - dot(v0, v1), w0 * v1 + w1 * v0 + cross(v0, v1));

	//quat.x = a.x * b.x - a.y * b.y - a.z * b.z - a.w * b.w;
	//quat.y = a.x * b.y + a.y * b.x - a.w * b.z + a.z * b.w;
	//quat.z = a.x * b.z + a.z * b.x + a.w * b.y - a.y * b.w;
	//quat.w = a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z;
	return quat;
}

__device__ __host__ bool IsZero3(const float3 &v) {
	return (v.x < Vector_ZERO_EPSILON && v.x > -Vector_ZERO_EPSILON && v.y < Vector_ZERO_EPSILON && v.y > -Vector_ZERO_EPSILON && v.z < Vector_ZERO_EPSILON && v.z > -Vector_ZERO_EPSILON);
}
__device__ __host__ bool IsZero(const float &val) {
	return fabs(val) < 1E-10;
}
__device__ __host__ bool isEqual(const float& _a, const float& _b) {
	float ab;

	ab = fabs(_a - _b);
	if (fabs(ab) < 1E-10) return 1;

	float a, b;
	a = fabs(_a);
	b = fabs(_b);
	if (b > a) {
		return ab < 1E-10 * b;
	} else {
		return ab < 1E-10 * a;
	}
}

__device__ __host__ inline float3 quatRotate(const float3 &v, const float4 &q) {
	float4 r = mult(mult(q, F4(0, v.x, v.y, v.z)), inv(q));
	return F3(r.y, r.z, r.w);
}

__device__ __host__ uint nearest_pow(uint num) {
	uint n = num > 0 ? num - 1 : 0;

	n |= n >> 1;
	n |= n >> 2;
	n |= n >> 4;
	n |= n >> 8;
	n |= n >> 16;
	n++;

	return n;
}

//Round a / b to nearest higher integer value
uint iDivUp(uint a, uint b){
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

struct __align__(16) int3f {
		int x, y, z;
		float w;
};
struct updateGPU {
		float3 vel, omega;
};

static __host__  __device__    __inline__ int3f make_int3f(int x, int y, int z, float w) {
	int3f t;
	t.x = x;
	t.y = y;
	t.z = z;
	t.w = w;
	return t;
}

#define CASTI3F(x) (int3f*)thrust::raw_pointer_cast(&x[0])

//custom version of ceil used for float3's
__host__ __device__ inline float3 ceil(float3 v) {
	return make_float3(ceil(v.x), ceil(v.y), ceil(v.z));
}
template<class T>
inline __host__ __device__ float max3(T a) {
	return max(a.x, max(a.y, a.z));
}
template<class T>
inline __host__ __device__ float min3(T a) {
	return min(a.x, min(a.y, a.z));
}

float __host_int_as_float(int a) {
	union {
			int a;
			float b;
	} u;
	u.a = a;
	return u.b;
}
//////////////////////////////////////////////////

#define CH_CONTACT_VSIZE 4
#define CH_CONTACT_HSIZE sizeof(CH_REALNUMBER4)

#define CH_BODY_VSIZE 8
#define CH_BODY_HSIZE sizeof(CH_REALNUMBER4)

#define CH_BILATERAL_VSIZE 5
#define CH_BILATERAL_HSIZE sizeof(CH_REALNUMBER4)

#define CH_REDUCTION_VSIZE 2
#define CH_REDUCTION_HSIZE sizeof(CH_REALNUMBER4)

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
//  'contacts' buffer is made with an horizontal array of://*OLD, there is no more preprocess stage, merged with iteration//*
//		[             , bx ]      0
//		[ matr.J12(x) , -  ]      1
//		[_____________, -  ]      2
//		[             , B1 ]	  3
//		[ matr.J1(w)  , -  ]	  4
//		[_____________, -  ]	  5
//		[             , B2 ]	  6
//		[ matr.J2(w)  , -  ]	  7
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
//		[ wx, wy, wz  , mu ]       1
//		[ xx, xy, xz  , -  ]       2
//		[ q0, q1, q2 , q3  ]       3
//		[iJ1,iJ2,iJ3  ,im  ]       4
//		[ fx, fy, fz  , -  ]       5
//		[ cx, cy, cz  , -  ]       6
//		[ ax, ay, az  , -  ]       7
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
//  'contacts' buffer  before preprocessing://*OLD//*
//		[   Normal    ,  0 ]      0
//		[  -   -   -  ,  - ]      1
//		[  -   -   -  ,  - ]      2
//		[     P1      , B1 ]	  3
//		[  -   -   -  ,  - ]	  4
//		[  -   -   -  ,  - ]	  5
//		[     P2      , B2 ]	  6
//		[  -   -   -  ,  - ]	  7
//		[  -   -   -  ,  - ]	  8
//		[ gx,  gy,  gz, mu ]      9

//*NEW//*
//  'contacts' buffer:
//		[ d ,  B1,  B2]   3
//		[   Normal    ]   0
//		[     P1      ]	  1
//		[     P2      ]	  2
//		[ gx,  gy,  gz]   4
