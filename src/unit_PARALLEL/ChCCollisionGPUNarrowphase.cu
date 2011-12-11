#include "ChCCollisionGPU.h"
#include "ChCCollisionGPU.cuh"
#include <thrust/remove.h>

//__global__ void Sphere_Sphere(object * object_data, int3 * Pair,
//		uint* Contact_Number, contactGPU* CData, uint totalPossibleConts) {
//	uint index = blockIdx.x * blockDim.x + threadIdx.x;
//	if (index >= totalPossibleConts) {
//		return;
//	}
//	int3 pair = Pair[index];
//	if (pair.z == 0) {
//		object A = object_data[pair.x];
//		object B = object_data[pair.y];
//		float3 N = F3(B.A) - F3(A.A);
//		float centerDist = dot(N, N);
//		float rAB = B.B.x + A.B.x;
//		if (centerDist <= (rAB) * (rAB)) {
//			float dist = sqrtf(centerDist);
//			N = N / dist;
//			AddContact(CData, index, A.A.w, B.A.w, F3(A.A) + A.B.x * N, F3(B.A)
//					- B.B.x * N, N, -dist);
//			Contact_Number[index] = index;
//		}
//	}
//}


__device__ __host__ inline float3 GetSupportPoint_Sphere(const float3 &B, const float3 &n) {
	return (B.x) * n;
}
__device__ __host__ inline float3 GetSupportPoint_Triangle(const float3 &A, const float3 &B, const float3 &C, const float3 &n) {
	float dist = dot(A, n);
	float3 point = A;
	if (dot(B, n) > dist) {
		dist = dot(B, n);
		point = B;
	}
	if (dot(C, n) > dist) {
		dist = dot(C, n);
		point = C;
	}
	return point;
}
__device__ __host__ inline float3 GetSupportPoint_Box(const float3 &B, const float3 &n) {
	float3 result = F3(0, 0, 0);
	result.x = n.x >= 0 ? B.x : -B.x;
	result.y = n.y >= 0 ? B.y : -B.y;
	result.z = n.z >= 0 ? B.z : -B.z;
	return result;
}
__device__ __host__ inline float3 GetSupportPoint_Ellipsoid(const float3 &B, const float3 &n) {
	return B * B * n / length(n * B);
}
__device__ __host__ float sign(float x) {
	if (x < 0) {
		return -1;
	} else {
		return 1;
	}
}

__device__ __host__ inline float3 GetSupportPoint_Cylinder(const float3 &B, const float3 &n) {
	//return make_float3(0,0,0);
	float3 u = F3(0, 1, 0);
	float3 w = n - (dot(u, n)) * u;
	float3 result;
	if (length(w) != 0) {
		result = sign(dot(u, n)) * B.y * u + B.x * normalize(w);
	} else {
		result = sign(dot(u, n)) * B.y * u;
	}
	return result;
}
__device__ __host__ inline float3 GetSupportPoint_Plane(const float3 &B, const float3 &n) {
	float3 result = B;
	if (n.x < 0) result.x = -result.x;
	if (n.y < 0) result.y = -result.y;
	return result;
}
__device__ __host__ inline float3 GetSupportPoint_Cone(const float3 &B, const float3 &n) {
	return make_float3(0, 0, 0);
}
__device__ __host__ inline float3 GetCenter_Sphere() {
	return Zero_Vector;
}
__device__ __host__ inline float3 GetCenter_Triangle(const float3 &A, const float3 &B, const float3 &C) {
	return make_float3((A.x + B.x + C.x) / 3.0f, (A.y + B.y + C.y) / 3.0f, (A.z + B.z + C.z) / 3.0f);
}
__device__ __host__ inline float3 GetCenter_Box() {
	return Zero_Vector;
}
__device__ __host__ inline float3 GetCenter_Ellipsoid() {
	return Zero_Vector;
}
__device__ __host__ inline float3 GetCenter_Cylinder() {
	return Zero_Vector;
}
__device__ __host__ inline float3 GetCenter_Plane() {
	return Zero_Vector;
}
__device__ __host__ inline float3 GetCenter_Cone() {
	return Zero_Vector;
}
__device__ __host__ bool IsZero3(const float3 &v) {
	return (v.x < Vector_ZERO_EPSILON && v.x > -Vector_ZERO_EPSILON && v.y < Vector_ZERO_EPSILON && v.y > -Vector_ZERO_EPSILON && v.z
			< Vector_ZERO_EPSILON && v.z > -Vector_ZERO_EPSILON);
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

__device__ __host__ float3 GetCenter(const int &type, const float3 &A, const float3 &B, const float3 &C) {
	if (type == 5) {
		return GetCenter_Triangle(A, B, C);
	} //triangle
	else {
		return make_float3(0, 0, 0) + A;
	} //All other shapes assumed to be locally centered
}
__device__ __host__ float3 TransformSupportVert(const int &type, const float3 &A, const float3 &B, const float3 &C, const float4 &R, const float3& b) {
	float3 localSupport;
	float3 n = normalize(b);
	if (type == 5) {//triangle
		return GetSupportPoint_Triangle(A, B, C, n);
	} else if (type == 0) {//sphere
		localSupport = GetSupportPoint_Sphere(B, quatRotate(n, inv(R)));
	} else if (type == 1) {//ellipsoid
		localSupport = GetSupportPoint_Ellipsoid(B, quatRotate(n, inv(R)));
	} else if (type == 2) {//box
		localSupport = GetSupportPoint_Box(B, quatRotate(n, inv(R)));
	} else if (type == 3) {//cylinder
		localSupport = GetSupportPoint_Cylinder(B, quatRotate(n, inv(R)));
	} else if (type == 6) {//plane
		localSupport = GetSupportPoint_Plane(B, quatRotate(n, inv(R)));
	} else if (type == 7) {//cone
		localSupport = GetSupportPoint_Cone(B, quatRotate(n, inv(R)));
	}
	return quatRotate(localSupport, R) + A; //globalSupport
}

__device__ __host__ float dist_line(float3 & P, float3 &x0, float3 &b, float3& witness) {
	float dist, t;
	float3 d, a;

	d = b - x0; // direction of segment
	a = x0 - P; // precompute vector from P to x0
	t = -(1.f) * dot(a, d);
	t /= dot(d, d);

	if (t < 0.0f || IsZero(t)) {
		dist = dot(x0 - P, x0 - P);
		witness = x0;
	} else if (t > 1.0f || isEqual(t, 1.0f)) {
		dist = dot(b - P, b - P);
		witness = b;
	} else {
		witness = d;
		witness *= t;
		witness += x0;
		dist = dot(witness - P, witness - P);
	}

	return dist;
}
__device__ __host__ float find_dist(float3 & P, float3 &x0, float3 &B, float3 &C, float3& witness) {
	float3 d1, d2, a;
	float u, v, w, p, q, r;
	float s, t, dist, dist2;
	float3 witness2;

	d1 = B - x0;
	d2 = C - x0;
	a = x0 - P;

	u = dot(a, a);
	v = dot(d1, d1);
	w = dot(d2, d2);
	p = dot(a, d1);
	q = dot(a, d2);
	r = dot(d1, d2);

	s = (q * r - w * p) / (w * v - r * r);
	t = (-s * r - q) / w;

	if ((IsZero(s) || s > 0.0f) && (isEqual(s, 1.0f) || s < 1.0f) && (IsZero(t) || t > 0.0f) && (isEqual(t, 1.0f) || t < 1.0f) && (isEqual(
																																			t + s,
																																			1.0f)
			|| t + s < 1.0f)) {
		d1 *= s;
		d2 *= t;
		witness = x0;
		witness += d1;
		witness += d2;
		dist = dot(witness - P, witness - P);
	} else {
		dist = dist_line(P, x0, B, witness);

		dist2 = dist_line(P, x0, C, witness2);
		if (dist2 < dist) {
			dist = dist2;
			(witness = witness2);
		}

		dist2 = dist_line(P, B, C, witness2);
		if (dist2 < dist) {
			dist = dist2;
			(witness = witness2);
		}
	}
	return dist;
}

//Code for Convex-Convex Collision detection, adopted from xeno-collide
__device__ __host__ bool CollideAndFindPoint(
												int typeA,
												float3 A_X,
												float3 A_Y,
												float3 A_Z,
												float4 A_R,
												int typeB,
												float3 B_X,
												float3 B_Y,
												float3 B_Z,
												float4 B_R,
												float3& returnNormal,
												float3& point1,
												float3& point2,
												float& depth) {
	float3 v01, v02, v0, n, v11, v12, v1, v21, v22, v2;
	// v0 = center of Minkowski sum
	v01 = GetCenter(typeA, A_X, A_Y, A_Z);
	v02 = GetCenter(typeB, B_X, B_Y, B_Z);
	v0 = v02 - v01;

	// Avoid case where centers overlap -- any direction is fine in this case
	if (IsZero3(v0)) v0 = make_float3(1, 0, 0);

	// v1 = support in direction of origin
	n = normalize(-v0);
	v11 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
	v12 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
	v1 = v12 - v11;
	if (dot(v1, n) <= 0) {
		return false;
	}

	// v2 - support perpendicular to v1,v0
	n = cross(v1, v0);
	if (IsZero3(n)) {
		n = v1 - v0;
		n = normalize(n);
		returnNormal = n;
		point1 = v11;
		point2 = v12;
		return true;
	}
	v21 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
	v22 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
	v2 = v22 - v21;
	if (dot(v2, n) <= 0) {
		return false;
	}

	// Determine whether origin is on + or - side of plane (v1,v0,v2)
	n = normalize(cross((v1 - v0), (v2 - v0)));
	// If the origin is on the - side of the plane, reverse the direction of the plane
	if (dot(n, v0) > 0) {
		Swap(v1, v2);
		Swap(v11, v21);
		Swap(v12, v22);
		n = -n;
	}
	// Phase One: Identify a portal
	float3 v31, v32, v3;
	while (1) {
		// Obtain the support point in a direction perpendicular to the existing plane
		// Note: This point is guaranteed to lie off the plane
		v31 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
		v32 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
		v3 = v32 - v31;
		if (dot(v3, n) <= 0) {
			return false;
		}
		// If origin is outside (v1,v0,v3), then eliminate v2 and loop
		else if (dot(cross(v1, v3), v0) < 0) {
			v2 = v3;
			v21 = v31;
			v22 = v32;
			n = cross((v1 - v0), (v3 - v0));
			continue;
		}
		// If origin is outside (v3,v0,v2), then eliminate v1 and loop
		else if (dot(cross(v3, v2), v0) < 0) {
			v1 = v3;
			v11 = v31;
			v12 = v32;
			n = cross((v3 - v0), (v2 - v0));
			continue;
		}
		break;
	}
	bool hit = false;

	// Phase Two: Refine the portal
	// We are now inside of a wedge...
	int phase2 = 0;
	while (1) {
		phase2++;
		// Compute normal of the wedge face
		n = cross((v2 - v1), (v3 - v1));
		n = normalize(n);

		// Compute distance from origin to wedge face
		// If the origin is inside the wedge, we have a hit
		if (dot(n, v1) >= 0. && !hit) {
			// Compute the barycentric coordinates of the origin
			float b0 = dot(cross(v1, v2), v3);
			float b1 = dot(cross(v3, v2), v0);
			float b2 = dot(cross(v0, v1), v3);
			float b3 = dot(cross(v2, v1), v0);

			float sum = b0 + b1 + b2 + b3;

			if (sum <= 0.) {
				b0 = 0;
				b1 = dot(cross(v2, v3), n);
				b2 = dot(cross(v3, v1), n);
				b3 = dot(cross(v1, v2), n);

				sum = b1 + b2 + b3;
			}
			float inv = 1.0f / sum;
			point1 = (b0 * v01 + b1 * v11 + b2 * v21 + b3 * v31) * inv;
			point2 = (b0 * v02 + b1 * v12 + b2 * v22 + b3 * v32) * inv;
			//point1+=point2;
			//point1*=.5;

			hit = true;// HIT!!!
		}
		// Find the support point in the direction of the wedge face
		float3 v41 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
		float3 v42 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
		float3 v4 = v42 - v41;

		float delta = dot((v4 - v3), n);
		float separation = -dot(v4, n);

		// If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex, then we can terminate
		if (delta <= kCollideEpsilon || separation >= 0. || phase2 > 100) {
			float3 O = F3(0, 0, 0);
			//depth=find_dist(O,v1,v2,v3,n);
			returnNormal = normalize(n);
			return hit;
		}
		if (dot(cross(v4, v1), v0) < 0.) { // Compute the tetrahedron dividing face (v4,v0,v1)
			if (dot(cross(v4, v2), v0) < 0) { // Compute the tetrahedron dividing face (v4,v0,v2)
				v1 = v4;
				v11 = v41;
				v12 = v42; // Inside d1 & inside d2 ==> eliminate v1
			} else {
				v3 = v4;
				v31 = v41;
				v32 = v42; // Inside d1 & outside d2 ==> eliminate v3
			}
		} else {
			if (dot(cross(v4, v3), v0) < 0.) { // Compute the tetrahedron dividing face (v4,v0,v3)
				v2 = v4;
				v21 = v41;
				v22 = v42; // Outside d1 & inside d3 ==> eliminate v2
			} else {
				v1 = v4;
				v11 = v41;
				v12 = v42; // Outside d1 & outside d3 ==> eliminate v1
			}
		}
	}
}
__global__ void MPR_GPU_Store(
								float3* pos,
								float4* rot,
								float3* obA,
								float3* obB,
								float3* obC,
								float4* obR,
								int3* typ,
								long long * Pair,
								uint* Contact_Number,
								float3* norm,
								float3* ptA,
								float3* ptB,
								float* contactDepth,
								int2* ids,

								uint totalPossibleConts) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= totalPossibleConts) {
		return;
	}
	if (Contact_Number[index] != 0xFFFFFFFF) {
		return;
	}
	long long p = Pair[index];
	int2 pair = I2(int(p >> 32), int(p & 0xffffffff));
	//if(pair.z<4){return;}

	int3 A_T = typ[pair.x], B_T = typ[pair.y];

	float3 posA = pos[A_T.z], posB = pos[B_T.z];
	float4 rotA = rot[A_T.z], rotB = rot[B_T.z];

	float3 A_X = obA[A_T.y], B_X = obA[B_T.y];
	float3 A_Y = obB[A_T.y], B_Y = obB[B_T.y];
	float3 A_Z = obC[A_T.y], B_Z = obC[B_T.y];

	float4 A_R = normalize(rotA + obR[A_T.y]);
	float4 B_R = normalize(rotB + obR[B_T.y]);

	if (A_T.x == 0 || A_T.x == 1 || A_T.x == 2 || A_T.x == 3) {
		A_X = A_X + posA;
	} else if (A_T.x == 5) {
		A_X = quatRotate(A_X + posA, A_R);
		A_Y = quatRotate(A_Y + posA, A_R);
		A_Z = quatRotate(A_Z + posA, A_R);
	}

	if (B_T.x == 0 || B_T.x == 1 || B_T.x == 2 || B_T.x == 3) {
		B_X = B_X + posB;
	} else if (B_T.x == 5) {
		B_X = quatRotate(B_X + posB, B_R);
		B_Y = quatRotate(B_Y + posB, B_R);
		B_Z = quatRotate(B_Z + posB, B_R);
	}

	float3 N, p1, p2;
	float depth = 0;
	if (!CollideAndFindPoint(A_T.x, A_X, A_Y, A_Z, A_R, B_T.x, B_X, B_Y, B_Z, B_R, N, p1, p2, depth)) {
		return;
	};

	p1 = (TransformSupportVert(A_T.x, A_X, A_Y, A_Z, A_R, -N) - p1) * N * N + p1;
	p2 = (TransformSupportVert(B_T.x, B_X, B_Y, B_Z, B_R, N) - p2) * N * N + p2;
	depth = sqrtf(dot((p2 - p1), (p2 - p1)));
	norm[index] = -N;
	ptA[index] = p1;
	ptB[index] = p2;
	contactDepth[index] = -depth;
	ids[index] = I2(A_T.z, B_T.z);
	Contact_Number[index] = 0;
}
__global__ void CopyGamma(int* to, float3* oldG, float3* newG, int contacts) {
	uint i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i >= contacts) {
		return;
	}
	newG[to[i]] = oldG[i];
}
void ChCCollisionGPU::Narrowphase_HOST(ChGPUDataManager * data_container) {
	data_container->number_of_contacts = 0;
	data_container->host_norm_data.clear();
	data_container->host_cpta_data.clear();
	data_container->host_cptb_data.clear();
	data_container->host_dpth_data.clear();
	data_container->host_bids_data.clear();
	for (int index = 0; index < data_container->number_of_contacts_possible; index++) {

		long long p = data_container->host_pair_data[index];
		int2 pair = I2(int(p >> 32), int(p & 0xffffffff));

		int3 A_T = data_container->host_typ_data[pair.x], B_T = data_container->host_typ_data[pair.y];

		float3 posA = data_container->host_pos_data[A_T.z], posB = data_container->host_pos_data[B_T.z];
		float4 rotA = data_container->host_rot_data[A_T.z], rotB = data_container->host_rot_data[B_T.z];

		float3 A_X = data_container->host_ObA_data[A_T.y], B_X = data_container->host_ObA_data[B_T.y];
		float3 A_Y = data_container->host_ObB_data[A_T.y], B_Y = data_container->host_ObB_data[B_T.y];
		float3 A_Z = data_container->host_ObC_data[A_T.y], B_Z = data_container->host_ObC_data[B_T.y];

		float4 A_R = rotA;// + obR[A_T.y];
		float4 B_R = rotB;// + obR[B_T.y];

		if (A_T.x == 0 || A_T.x == 1 || A_T.x == 2 || A_T.x == 3) {
			A_X = A_X + posA;
		} else if (A_T.x == 5) {
			A_X = quatRotate(A_X + posA, A_R);
			A_Y = quatRotate(A_Y + posA, A_R);
			A_Z = quatRotate(A_Z + posA, A_R);
		}

		if (B_T.x == 0 || B_T.x == 1 || B_T.x == 2 || B_T.x == 3) {
			B_X = B_X + posB;
		} else if (B_T.x == 5) {
			B_X = quatRotate(B_X + posB, B_R);
			B_Y = quatRotate(B_Y + posB, B_R);
			B_Z = quatRotate(B_Z + posB, B_R);
		}

		float3 N, p1, p2;
		float depth = 0;
		if (!CollideAndFindPoint(A_T.x, A_X, A_Y, A_Z, A_R, B_T.x, B_X, B_Y, B_Z, B_R, N, p1, p2, depth)) {
			continue;
		};

		p1 = (TransformSupportVert(A_T.x, A_X, A_Y, A_Z, A_R, -N) - p1) * N * N + p1;
		p2 = (TransformSupportVert(B_T.x, B_X, B_Y, B_Z, B_R, N) - p2) * N * N + p2;
		depth = sqrtf(dot((p2 - p1), (p2 - p1)));
		data_container->host_norm_data.push_back(-N);
		data_container->host_cpta_data.push_back(p1);
		data_container->host_cptb_data.push_back(p2);
		data_container->host_dpth_data.push_back(-depth);
		data_container->host_bids_data.push_back(I2(A_T.z, B_T.z));
		data_container->number_of_contacts++;
	}
	data_container->host_gam_data.resize(data_container->number_of_contacts);
	thrust::fill(data_container->host_gam_data.begin(), data_container->host_gam_data.end(), F3(0));
}
void ChCCollisionGPU::Narrowphase(gpu_container & gpu_data) {
	thrust::device_vector<uint> generic_counter(gpu_data.number_of_contacts_possible, 0xFFFFFFFF);
	uint number_of_contacts_possible=gpu_data.number_of_contacts_possible;
	gpu_data.device_norm_data.resize(gpu_data.number_of_contacts_possible);
	gpu_data.device_cpta_data.resize(gpu_data.number_of_contacts_possible);
	gpu_data.device_cptb_data.resize(gpu_data.number_of_contacts_possible);
	gpu_data.device_dpth_data.resize(gpu_data.number_of_contacts_possible);
	gpu_data.device_bids_data.resize(gpu_data.number_of_contacts_possible);

	MPR_GPU_Store CUDA_KERNEL_DIM(BLOCKS(number_of_contacts_possible),THREADS) (
			CASTF3(gpu_data.device_pos_data),
			CASTF4(gpu_data.device_rot_data),
			CASTF3(gpu_data.device_ObA_data),
			CASTF3(gpu_data.device_ObB_data),
			CASTF3(gpu_data.device_ObC_data),
			CASTF4(gpu_data.device_ObR_data),
			CASTI3(gpu_data.device_typ_data),
			CASTLL(gpu_data.device_pair_data),
			CASTU1(generic_counter),
			CASTF3(gpu_data.device_norm_data),
			CASTF3(gpu_data.device_cpta_data),
			CASTF3(gpu_data.device_cptb_data),
			CASTF1(gpu_data.device_dpth_data),
			CASTI2(gpu_data.device_bids_data),
			number_of_contacts_possible);

	thrust::sort_by_key(generic_counter.begin(), generic_counter.end(), thrust::make_zip_iterator(
					thrust::make_tuple(
							gpu_data.device_norm_data.begin(),
							gpu_data.device_cpta_data.begin(),
							gpu_data.device_cptb_data.begin(),
							gpu_data.device_dpth_data.begin(),
							gpu_data.device_bids_data.begin(),
							gpu_data.device_pair_data.begin())));

	gpu_data.number_of_contacts = number_of_contacts_possible- Thrust_Count(generic_counter,0xFFFFFFFF);

	//thrust::device_vector<float3> old_gamma = data_container->device_gam_data;

	gpu_data.device_norm_data.resize(gpu_data.number_of_contacts);
	gpu_data.device_cpta_data.resize(gpu_data.number_of_contacts);
	gpu_data.device_cptb_data.resize(gpu_data.number_of_contacts);
	gpu_data.device_dpth_data.resize(gpu_data.number_of_contacts);
	gpu_data.device_bids_data.resize(gpu_data.number_of_contacts);
	gpu_data.device_gam_data.resize(gpu_data.number_of_contacts);
	Thrust_Fill(gpu_data.device_gam_data,F3(0));
	//	contact_pair.resize(number_of_contacts);
	//
	//	thrust::sort_by_key(contact_pair.begin(), contact_pair.end(), thrust::make_zip_iterator(thrust::make_tuple(data_container->device_norm_data.begin(), data_container->device_cpta_data.begin(),
	//			data_container->device_cptb_data.begin(), data_container->device_dpth_data.begin(), data_container->device_bids_data.begin())));
	//	if (old_contact_pair.size() != 0) {
	//		thrust::device_vector<int> res(old_contact_pair.size());
	//
	//		thrust::binary_search(contact_pair.begin(), contact_pair.end(), old_contact_pair.begin(), old_contact_pair.end(), res.begin());//list of persistent contacts
	//		thrust::sort_by_key(res.begin(), res.end(), old_contact_pair.begin(),thrust::greater<int>() );//index of common contacts from old list
	//
	//		int numP = Thrust_Count(res,1);
	//		old_contact_pair.resize(numP);
	//		if (numP > 0) {cout<<numP<<"\t";
	//			thrust::device_vector<int> temporaryB(numP);
	//			thrust::lower_bound(contact_pair.begin(), contact_pair.end(), old_contact_pair.begin(), old_contact_pair.end(), temporaryB.begin());//return index of common new contact
	//CopyGamma		<<<BLOCKS(numP),THREADS>>>(
	//				CASTI1(temporaryB),
	//				CASTF3(old_gamma),
	//				CASTF3(data_container->device_gam_data),
	//				numP);
	//		//				for(int i=0; i<old_contact_pair.size(); i++){cout<<old_contact_pair[i]<<endl;}
	//		//				cout<<"------------------------"<<endl;
	//		//				for(int i=0; i<contact_pair.size(); i++){cout<<contact_pair[i]<<endl;}
	//		//				cout<<"------------------------"<<endl;
	//		//				for(int i=0; i<res1.size(); i++){cout<<res1[i]<<endl;}
	//		//				cout<<"------------------------"<<endl;
	//		//				for(int i=0; i<temporaryA.size(); i++){cout<<temporaryA[i]<<endl;}
	//		//				cout<<"------------------------"<<endl;
	//		//				for(int i=0; i<temporaryB.size(); i++){cout<<temporaryB[i]<<endl;}
	//		//
	//		//				exit(0);
	//	}
	//}
	//old_contact_pair = contact_pair;
}
