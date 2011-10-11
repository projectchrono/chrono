#ifndef CHC_COLLISIONGPU_CUH
#define CHC_COLLISIONGPU_CUH
using namespace chrono::collision;
//__constant__ float		bin_size_const;
__constant__ float3 global_origin_const;
__constant__ float3 bin_size_vec_const;
__constant__ uint last_active_bin_const;
__constant__ float collision_envelope_const;
__constant__ uint number_of_models_const;


//1.f/16384.f;
#define Vector_ZERO_EPSILON 0.0000001
#define MIN_ZERO_EPSILON 1.1754943508222875E-38
#define kCollideEpsilon  1e-5f

__device__ __host__ inline void Swap(float3& a, float3& b) {
	float3 tmp = a;
	a = b;
	b = tmp;
}

__device__ __host__ void ComputeAABBSphere(const float &radius,const float3 &position, float3 &minp, float3 &maxp) {
	minp=position-F3(radius);
	maxp=position+F3(radius);
}

__device__ __host__ void ComputeAABBTriangle(const float3 &A,const float3 &B,const float3 &C, float3 &minp, float3 &maxp) {
	minp.x=min(A.x,min(B.x,C.x));
	minp.y=min(A.y,min(B.y,C.y));
	minp.z=min(A.z,min(B.z,C.z));
	maxp.x=max(A.x,max(B.x,C.x));
	maxp.y=max(A.y,max(B.y,C.y));
	maxp.z=max(A.z,max(B.z,C.z));
}

__device__ __host__ void ComputeAABBBox(const float3 &dim,const float3 &positon,const float4 &rotation, float3 &minp, float3 &maxp) {
	float3 p1=F3(-dim.x,-dim.y,-dim.z);
	float3 p2=F3(-dim.x,-dim.y,dim.z);
	float3 p3=F3(-dim.x,dim.y,-dim.z);
	float3 p4=F3(dim.x,-dim.y,-dim.z);
	float3 p5=F3(dim.x,dim.y,-dim.z);
	float3 p6=F3(dim.x,-dim.y,dim.z);
	float3 p7=F3(-dim.x,dim.y,dim.z);
	float3 p8=F3(dim.x,dim.y,dim.z);

	p1=quatRotate(p1,rotation);
	p2=quatRotate(p2,rotation);
	p3=quatRotate(p3,rotation);
	p4=quatRotate(p4,rotation);
	p5=quatRotate(p5,rotation);
	p6=quatRotate(p6,rotation);
	p7=quatRotate(p7,rotation);
	p8=quatRotate(p8,rotation);

	minp.x=fminf(p1.x,fminf(p2.x,fminf(p3.x,fminf(p4.x,fminf(p5.x,fminf(p6.x,fminf(p7.x,p8.x)))))));
	minp.y=fminf(p1.y,fminf(p2.y,fminf(p3.y,fminf(p4.y,fminf(p5.y,fminf(p6.y,fminf(p7.y,p8.y)))))));
	minp.z=fminf(p1.z,fminf(p2.z,fminf(p3.z,fminf(p4.z,fminf(p5.z,fminf(p6.z,fminf(p7.z,p8.z)))))));
	maxp.x=fmaxf(p1.x,fmaxf(p2.x,fmaxf(p3.x,fmaxf(p4.x,fmaxf(p5.x,fmaxf(p6.x,fmaxf(p7.x,p8.x)))))));
	maxp.y=fmaxf(p1.y,fmaxf(p2.y,fmaxf(p3.y,fmaxf(p4.y,fmaxf(p5.y,fmaxf(p6.y,fmaxf(p7.y,p8.y)))))));
	maxp.z=fmaxf(p1.z,fmaxf(p2.z,fmaxf(p3.z,fmaxf(p4.z,fmaxf(p5.z,fmaxf(p6.z,fmaxf(p7.z,p8.z)))))));
	minp+=positon;
	maxp+=positon;
}

#define EPS FLT_EPSILON
template <class T>
__device__ inline uint3 Hash(const T &A) {
	return U3(A.x*bin_size_vec_const.x,A.y*bin_size_vec_const.y,A.z*bin_size_vec_const.z);
}
template <class T>
__device__ inline uint Hash_Index(const T &A) {
	return ((A.x * 73856093) ^ (A.y * 19349663) ^ (A.z * 83492791));
}

#endif
