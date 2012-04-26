#ifndef CHC_COLLISIONGPU_CUH
#define CHC_COLLISIONGPU_CUH
using namespace chrono::collision;

//1.f/16384.f;

#define kCollideEpsilon  1e-5f

__device__ __host__ void ComputeAABBSphere(const float &radius, const float3 &position, float3 &minp, float3 &maxp) {
	minp = position - F3(radius);
	maxp = position + F3(radius);
}

__device__ __host__ void ComputeAABBTriangle(const float3 &A, const float3 &B, const float3 &C, float3 &minp, float3 &maxp) {
	minp.x = min(A.x, min(B.x, C.x));
	minp.y = min(A.y, min(B.y, C.y));
	minp.z = min(A.z, min(B.z, C.z));
	maxp.x = max(A.x, max(B.x, C.x));
	maxp.y = max(A.y, max(B.y, C.y));
	maxp.z = max(A.z, max(B.z, C.z));
}

__device__ __host__ void ComputeAABBBox(const float3 &dim, const float3 &lpositon, const float3 &positon, const float4 &lrotation, const float4 &rotation, float3 &minp, float3 &maxp) {

	float3 pos = quatRotate(lpositon, rotation) + positon; //new position

	float4 q1 = mult(rotation, lrotation); //full rotation

	float4 q=F4(q1.y,q1.z,q1.w,q1.x);

	float t[3] = { pos.x, pos.y, pos.z };

	float mina[3] = { -dim.x, -dim.y, -dim.z };
	float maxa[3] = { dim.x, dim.y, dim.z };

	float minb[3] = { 0, 0, 0 };
	float maxb[3] = { 0, 0, 0 };

	float m[3][3];

	float qx2=q.x*q.x;
	float qy2=q.y*q.y;
	float qz2=q.z*q.z;


	m[0][0]=1 - 2*qy2 - 2*qz2;
	m[1][0]=2*q.x*q.y + 2*q.z*q.w;
	m[2][0]=2*q.x*q.z - 2*q.y*q.w;


	m[0][1]=2*q.x*q.y - 2*q.z*q.w;
	m[1][1]=1 - 2*qx2 - 2*qz2;
	m[2][1]=2*q.y*q.z + 2*q.x*q.w 	;


	m[0][2]=2*q.x*q.z + 2*q.y*q.w;
	m[1][2]=2*q.y*q.z - 2*q.x*q.w;
	m[2][2]=1 - 2*qx2 - 2*qy2;

	// For all three axes
	for (int i = 0; i < 3; i++) {
		// Start by adding in translation
		minb[i] = maxb[i] = t[i];
		// Form extent by summing smaller and larger terms respectively
		for (int j = 0; j < 3; j++) {
			float e = m[i][j] * mina[j];
			float f = m[i][j] * maxa[j];
			if (e < f) {
				minb[i] += e;
				maxb[i] += f;
			} else {
				minb[i] += f;
				maxb[i] += e;
			}
		}
	}

	minp = F3(minb[0],minb[1],minb[2]);
	maxp = F3(maxb[0],maxb[1],maxb[2]);

}

typedef thrust::pair<float3, float3> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction: public thrust::binary_function<bbox, bbox, bbox> {
	__host__      __device__
	bbox operator()(bbox a, bbox b) {
		float3 ll = F3(fminf(a.first.x, b.first.x), fminf(a.first.y, b.first.y), fminf(a.first.z, b.first.z)); // lower left corner
		float3 ur = F3(fmaxf(a.second.x, b.second.x), fmaxf(a.second.y, b.second.y), fmaxf(a.second.z, b.second.z)); // upper right corner
		return bbox(ll, ur);
	}
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation: public thrust::unary_function<float3, bbox> {
	__host__      __device__
	bbox operator()(float3 point) {
		return bbox(point, point);
	}
};

#endif
