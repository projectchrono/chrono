#ifndef CHC_COLLISIONGPU_CUH
#define CHC_COLLISIONGPU_CUH
using namespace chrono::collision;
__constant__ float		bin_size_const;
__constant__ float3     global_origin_const;
__constant__ uint		last_active_bin_const;
__constant__ float		collision_envelope_const;
__constant__ uint		number_of_objects_const;

#define Zero_Vector make_float3(0,0,0)
#define PI  3.1415926535897932384626433832795f
#define PI_2   (PI / 2.0f)   
#define PI_180  (PI / 180.0f)
//1.f/16384.f;
#define Vector_ZERO_EPSILON 0.00000000001
#define MIN_ZERO_EPSILON 1.1754943508222875E-38
#define kCollideEpsilon  1e-3f

__device__ __host__ inline float4 operator ~(const float4& a){
	return (1.0f/(dot(a,a)))*(F4(-1*F3(a),a.w));
}
__device__ __host__  inline float4 mult(const float4 &a, const float4 &b){
	return F4(a.w*F3(b)+b.w*F3(a)+cross(F3(a),F3(b)),a.w*b.w-dot(F3(a),F3(b)));
}

__device__ __host__ inline float3 quatRotate(const float3 &v, const float4 &q){
	return make_float3(mult(mult(q,make_float4(v,0)),~(q)));
}

__device__ __host__ inline void Swap(float3& a, float3& b){
	float3 tmp = a;
	a = b;
	b = tmp;
}

__device__ __host__ void ComputeAABBSphere(const object &C, float4 &minp, float4 &maxp){
	float4 S=C.A;
	minp=F4(F3(S)-F3(S.w),0);
	maxp=F4(F3(S)+F3(S.w),0);
}

__device__ __host__ void ComputeAABBTriangle(const object &C,float4 &minp, float4 &maxp){
	minp.x=min(C.A.x,min(C.B.x,C.C.x));
	minp.y=min(C.A.y,min(C.B.y,C.C.y));
	minp.z=min(C.A.z,min(C.B.z,C.C.z));
	maxp.x=max(C.A.x,max(C.B.x,C.C.x));
	maxp.y=max(C.A.y,max(C.B.y,C.C.y));
	maxp.z=max(C.A.z,max(C.B.z,C.C.z));
}

__device__ __host__ void ComputeAABBBox(const object &C,float4 &minp, float4 &maxp){
	float3 dim=F3(C.B);
	float3 p1=F3(-dim.x,-dim.y,-dim.z);
	float3 p2=F3(-dim.x,-dim.y,dim.z);
	float3 p3=F3(-dim.x,dim.y,-dim.z);
	float3 p4=F3(dim.x,-dim.y,-dim.z);
	float3 p5=F3(dim.x,dim.y,-dim.z);
	float3 p6=F3(dim.x,-dim.y,dim.z);
	float3 p7=F3(-dim.x,dim.y,dim.z);
	float3 p8=F3(dim.x,dim.y,dim.z);

	p1=quatRotate(p1,C.C);
	p2=quatRotate(p2,C.C);
	p3=quatRotate(p3,C.C);
	p4=quatRotate(p4,C.C);
	p5=quatRotate(p5,C.C);
	p6=quatRotate(p6,C.C);
	p7=quatRotate(p7,C.C);
	p8=quatRotate(p8,C.C);

	minp.x=fminf(p1.x,fminf(p2.x,fminf(p3.x,fminf(p4.x,fminf(p5.x,fminf(p6.x,fminf(p7.x,p8.x)))))));
	minp.y=fminf(p1.y,fminf(p2.y,fminf(p3.y,fminf(p4.y,fminf(p5.y,fminf(p6.y,fminf(p7.y,p8.y)))))));
	minp.z=fminf(p1.z,fminf(p2.z,fminf(p3.z,fminf(p4.z,fminf(p5.z,fminf(p6.z,fminf(p7.z,p8.z)))))));
	maxp.x=fmaxf(p1.x,fmaxf(p2.x,fmaxf(p3.x,fmaxf(p4.x,fmaxf(p5.x,fmaxf(p6.x,fmaxf(p7.x,p8.x)))))));
	maxp.y=fmaxf(p1.y,fmaxf(p2.y,fmaxf(p3.y,fmaxf(p4.y,fmaxf(p5.y,fmaxf(p6.y,fmaxf(p7.y,p8.y)))))));
	maxp.z=fmaxf(p1.z,fmaxf(p2.z,fmaxf(p3.z,fmaxf(p4.z,fmaxf(p5.z,fmaxf(p6.z,fmaxf(p7.z,p8.z)))))));
	minp+=F4(F3(C.A));
	maxp+=F4(F3(C.A));
}
#endif
