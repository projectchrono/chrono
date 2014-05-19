#include "collision/ChCCollisionModel.h"

#include "math/ChParallelMath.h"
#include "collision/ChCNarrowphaseMPR.h"
#include "collision/ChCNarrowphaseMPRUtils.h"

using namespace chrono::collision;

__constant__ uint total_possible_contacts_const;
__constant__ real collision_envelope_const;

__device__ __host__ real3 GetCenter(const shape_type &type, const real3 &A, const real3 &B, const real3 &C) {
	if (type == TRIANGLEMESH) {
		return GetCenter_Triangle(A, B, C);     //triangle center
	} else {
		return R3(0, 0, 0) + A;     //All other shapes assumed to be locally centered
	}
}

__device__ __host__ real3 TransformSupportVert(const shape_type &type, const real3 &A, const real3 &B, const real3 &C, const real4 &R, const real3 &b) {
	real3 localSupport;
	real3 n = normalize(b);

//M33 orientation = AMat(R);
//M33 invorientation = AMatT(R);
	real3 rotated_n = quatRotateMatT(n, R);
	switch (type) {
		case chrono::collision::SPHERE:
			localSupport = GetSupportPoint_Sphere(B, rotated_n);
			break;
		case chrono::collision::ELLIPSOID:
			localSupport = GetSupportPoint_Ellipsoid(B, rotated_n);
			break;
		case chrono::collision::BOX:
			localSupport = GetSupportPoint_Box(B, rotated_n);
			break;
		case chrono::collision::CYLINDER:
			localSupport = GetSupportPoint_Cylinder(B, rotated_n);
			break;
		case chrono::collision::RECT:
			localSupport = GetSupportPoint_Plane(B, rotated_n);
			break;
		case chrono::collision::CONE:
			localSupport = GetSupportPoint_Cone(B, rotated_n);
			break;
		case chrono::collision::TRIANGLEMESH:
			return GetSupportPoint_Triangle(A, B, C, n);
			break;
	}

	return quatRotateMat(localSupport, R) + A;     //globalSupport
}

__device__ __host__ real dist_line(real3 &P, real3 &x0, real3 &b, real3 &witness) {
	real dist, t;
	real3 d, a;
	d = b - x0;     // direction of segment
	a = x0 - P;     // precompute vector from P to x0
	t = -(1.f) * dot(a, d);
	t = t / dot(d, d);

	if (t < 0.0f || IsZero(t)) {
		dist = dot(x0 - P, x0 - P);
		witness = x0;
	} else if (t > 1.0f || isEqual(t, 1.0f)) {
		dist = dot(b - P, b - P);
		witness = b;
	} else {
		witness = d;
		witness = witness * t;
		witness = witness + x0;
		dist = dot(witness - P, witness - P);
	}

	return dist;
}
__device__ __host__ real find_dist(real3 &P, real3 &x0, real3 &B, real3 &C, real3 &witness) {
	real3 d1, d2, a;
	real u, v, w, p, q, r;
	real s, t, dist, dist2;
	real3 witness2;
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

	if ((IsZero(s) || s > 0.0f) && (isEqual(s, 1.0f) || s < 1.0f) && (IsZero(t) || t > 0.0f) && (isEqual(t, 1.0f) || t < 1.0f) && (isEqual(t + s, 1.0f) || t + s < 1.0f)) {
		d1 = d1 * s;
		d2 = d2 * t;
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
shape_type typeA, real3 A_X, real3 A_Y, real3 A_Z, real4 A_R,
shape_type typeB, real3 B_X, real3 B_Y, real3 B_Z, real4 B_R, real3 &returnNormal, real3 &point, real &depth) {
	real3 v01, v02, v0, n, v11, v12, v1, v21, v22, v2;
// v0 = center of Minkowski sum
	v01 = GetCenter(typeA, A_X, A_Y, A_Z);
	v02 = GetCenter(typeB, B_X, B_Y, B_Z);
	v0 = v02 - v01;

// Avoid case where centers overlap -- any direction is fine in this case
	if (IsZero(v0))
		v0 = R3(1, 0, 0);

// v1 = support in direction of origin
//n = normalize(-v0);
	n = -v0;
	v11 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
	v12 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
	v1 = v12 - v11;

	if (dot(v1, n) <= 0.0) {
		//cout << "FAIL A" << endl;
		return false;
	}

// v2 - support perpendicular to v1,v0
	n = cross(v1, v0);

	if (IsZero(n)) {
		n = v1 - v0;
		n = normalize(n);
		returnNormal = n;
		//point1 = v11;
		//point2 = v12;
		point = (v11 + v12) * .5;

		depth = dot((v12 - v11), n);
		return true;
	}

	v21 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
	v22 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
	v2 = v22 - v21;

	if (dot(v2, n) <= 0.0) {
		//cout << "FAIL B" << endl;
		return false;
	}

// Determine whether origin is on + or - side of plane (v1,v0,v2)
	n = (cross((v1 - v0), (v2 - v0)));

// If the origin is on the - side of the plane, reverse the direction of the plane
	if (dot(n, v0) > 0.0) {
		Swap(v1, v2);
		Swap(v11, v21);
		Swap(v12, v22);
		n = -n;
	}

// Phase One: Identify a portal
	real3 v31, v32, v3;
	bool hit = false;
	int phase1 = 0;
	int phase2 = 0;
	int max_iterations = 30;

	while (true) {
		if (phase1 > max_iterations) {
			//cout << "FAIL PHASE 1 MAX ITER" << endl;
			return false;
		}
		phase1++;

		// Obtain the support point in a direction perpendicular to the existing plane
		// Note: This point is guaranteed to lie off the plane
		v31 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
		v32 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
		v3 = v32 - v31;

		if (dot(v3, n) <= 0.0) {
			//cout << "FAIL C" << endl;
			return false;
		}
		// If origin is outside (v1,v0,v3), then eliminate v2 and loop
		if (dot(cross(v1, v3), v0) < 0.0) {
			v2 = v3;
			v21 = v31;
			v22 = v32;
			n = cross((v1 - v0), (v3 - v0));
			continue;
		}
		// If origin is outside (v3,v0,v2), then eliminate v1 and loop
		if (dot(cross(v3, v2), v0) < 0.0) {
			v1 = v3;
			v11 = v31;
			v12 = v32;
			n = cross((v3 - v0), (v2 - v0));
			continue;
		}

		//break;

		// Phase Two: Refine the portal
		// We are now inside of a wedge...

		while (true) {
			phase2++;

			// Compute normal of the wedge face
			n = cross((v2 - v1), (v3 - v1));
			//if (IsZero(n)) {
			//	return true;
			//}
			n = normalize(n);

			// Compute distance from origin to wedge face
			// If the origin is inside the wedge, we have a hit
			if (dot(n, v1) >= 0.0 && !hit) {
				hit = true;     // HIT!!!
			}

			// Find the support point in the direction of the wedge face
			real3 v41 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
			real3 v42 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
			real3 v4 = v42 - v41;
			real delta = dot((v4 - v3), n);
			depth = dot(v4, n);

			// If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex, then we can terminate
			if (delta <= 1e-20 || depth <= 0.0 || phase2 > max_iterations) {
				//cout << "ITERS MAX" << delta << " " << depth << " " << phase2 <<" "<<hit<< endl;
				if (hit) {
					//cout << "HIT" << endl;
					// Compute the barycentric coordinates of the origin
					real b0 = dot(cross(v1, v2), v3);
					real b1 = dot(cross(v3, v2), v0);
					real b2 = dot(cross(v0, v1), v3);
					real b3 = dot(cross(v2, v1), v0);
					real sum = b0 + b1 + b2 + b3;

					if (sum <= 0.) {
						b0 = 0;
						b1 = dot(cross(v2, v3), n);
						b2 = dot(cross(v3, v1), n);
						b3 = dot(cross(v1, v2), n);
						sum = b1 + b2 + b3;
					}

					real inv = 1.0f / sum;
					point = (b0 * v01 + b1 * v11 + b2 * v21 + b3 * v31) + (b0 * v02 + b1 * v12 + b2 * v22 + b3 * v32);
					point = point * inv * .5;
					returnNormal = normalize(n);
				}

				return hit;
			}

			if (dot(cross(v4, v1), v0) < 0) {     // Compute the tetrahedron dividing face (v4,v0,v1)
				if (dot(cross(v4, v2), v0) < 0) {     // Compute the tetrahedron dividing face (v4,v0,v2)
					v1 = v4;
					v11 = v41;
					v12 = v42;     // Inside d1 & inside d2 ==> eliminate v1
				} else {
					v3 = v4;
					v31 = v41;
					v32 = v42;     // Inside d1 & outside d2 ==> eliminate v3
				}
			} else {
				if (dot(cross(v4, v3), v0) < 0) {     // Compute the tetrahedron dividing face (v4,v0,v3)
					v2 = v4;
					v21 = v41;
					v22 = v42;     // Outside d1 & inside d3 ==> eliminate v2
				} else {
					v1 = v4;
					v11 = v41;
					v12 = v42;     // Outside d1 & outside d3 ==> eliminate v1
				}
			}
		}
	}
}

__host__ __device__ void function_MPR_Store(const uint &index, const shape_type *obj_data_T, const real3 *obj_data_A, const real3 *obj_data_B, const real3 *obj_data_C, const real4 *obj_data_R,
		const uint *obj_data_ID, const bool * obj_active, const real3 *body_pos, const real4 *body_rot, const real & collision_envelope, long long *contact_pair, uint *contact_active, real3 *norm,
		real3 *ptA, real3 *ptB, real *contactDepth, int2 *ids

		) {

	long long p = contact_pair[index];
	int2 pair = I2(int(p >> 32), int(p & 0xffffffff));
	shape_type A_T = obj_data_T[pair.x], B_T = obj_data_T[pair.y];     //Get the type data for each object in the collision pair
	uint ID_A = obj_data_ID[pair.x];
	uint ID_B = obj_data_ID[pair.y];

	if (obj_active[ID_A] == false && obj_active[ID_B] == false) {
		return;
	}
	if (ID_A == ID_B) {
		return;
	}

	real3 posA = body_pos[ID_A], posB = body_pos[ID_B];     //Get the global object position
	real4 rotA = body_rot[ID_A], rotB = body_rot[ID_B];     //Get the global object rotation
	real3 A_X = obj_data_A[pair.x], B_X = obj_data_A[pair.y];
	real3 A_Y = obj_data_B[pair.x], B_Y = obj_data_B[pair.y];
	real3 A_Z = obj_data_C[pair.x], B_Z = obj_data_C[pair.y];
	real4 A_R = (mult(rotA, obj_data_R[pair.x]));
	real4 B_R = (mult(rotB, obj_data_R[pair.y]));

	real envelope = collision_envelope;

	if (A_T == SPHERE || A_T == ELLIPSOID || A_T == BOX || A_T == CYLINDER || A_T == CONE) {
		A_X = quatRotate(A_X, rotA) + posA;
	} else if (A_T == TRIANGLEMESH) {
		envelope = 0;
		A_X = quatRotate(A_X, rotA) + posA;
		A_Y = quatRotate(A_Y, rotA) + posA;
		A_Z = quatRotate(A_Z, rotA) + posA;
	}

	if (B_T == SPHERE || B_T == ELLIPSOID || B_T == BOX || B_T == CYLINDER || B_T == CONE) {
		B_X = quatRotate(B_X, rotB) + posB;
	} else if (B_T == TRIANGLEMESH) {
		envelope = 0;
		B_X = quatRotate(B_X, rotB) + posB;
		B_Y = quatRotate(B_Y, rotB) + posB;
		B_Z = quatRotate(B_Z, rotB) + posB;
	}

	real3 N = R3(1, 0, 0), p1 = R3(0), p2 = R3(0), p0 = R3(0);
	real depth = 0;

	if (A_T == SPHERE && B_T == SPHERE) {

		real3 relpos = B_X - A_X;
		real d2 = dot(relpos, relpos);
		real collide_dist = A_Y.x + B_Y.x;
		if (d2 <= collide_dist * collide_dist) {
			real dist = A_Y.x + B_Y.x;
			N = normalize(relpos);
			p1 = A_X + N * A_Y.x;
			p2 = B_X - N * B_Y.x;
			//depth = length(relpos) - dist;

			//if (ID_A == 0 && ID_B == 36) {

			//cout << "OMG: " << depth << endl;

			//}

		} else {
			return;
		}

	} else {

		if (!CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, N, p0, depth)) {
//
//					if (ID_A == 1 && ID_B == 1004) {
//						cout << A_X.x << " " << A_X.y << " " << A_X.z << " | ";
//						cout << B_X.x << " " << B_X.y << " " << B_X.z << " || ";
//						cout << A_Y.x << " " << A_Y.y << " " << A_Y.z << " | ";
//						cout << B_Y.x << " " << B_Y.y << " " << B_Y.z << " || ";
//						cout << N.x << " " << N.y << " " << N.z << " | " << depth;
//
//					}
			return;

		}
		p1 = dot((TransformSupportVert(A_T, A_X, A_Y, A_Z, A_R, -N) - p0), N) * N + p0;
		p2 = dot((TransformSupportVert(B_T, B_X, B_Y, B_Z, B_R, N) - p0), N) * N + p0;
		N = -N;

	}
//cout << p1.x << " " << p1.y << " " << p1.z << "||" << p2.x << " " << p2.y << " " << p2.z
//		<< "||" << p0.x << " " << p0.y << " " << p0.z
//		<< "||" << N.x << " " << N.y << " " << N.z<< endl;

	p1 = p1 - (N) * envelope;
	p2 = p2 + (N) * envelope;

	depth = dot(N, p2 - p1);
//depth = -(depth - envelope - envelope);
	norm[index] = N;
	ptA[index] = p1-posA;
	ptB[index] = p2-posB;
	contactDepth[index] = depth;
	ids[index] = I2(ID_A, ID_B);
	contact_active[index] = 0;
}

void ChCNarrowphaseMPR::host_MPR_Store(const shape_type *obj_data_T, const real3 *obj_data_A, const real3 *obj_data_B, const real3 *obj_data_C, const real4 *obj_data_R, const uint *obj_data_ID,
		const bool * obj_active, const real3 *body_pos, const real4 *body_rot, long long *contact_pair, uint *contact_active, real3 *norm, real3 *ptA, real3 *ptB, real *contactDepth, int2 *ids) {
#pragma omp parallel for

	for (int index = 0; index < total_possible_contacts; index++) {
		function_MPR_Store(index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, obj_active, body_pos, body_rot, collision_envelope, contact_pair, contact_active, norm, ptA,
				ptB, contactDepth, ids);
	}
}

__host__ __device__ void function_MPR_Update(const uint &index, const shape_type *obj_data_T, const real3 *obj_data_A, const real3 *obj_data_B, const real3 *obj_data_C, const real4 *obj_data_R,
		const uint *obj_data_ID, const bool * obj_active, const real3 *body_pos, const real4 *body_rot, const real & collision_envelope, real3 *norm, real3 *ptA, real3 *ptB, real *contactDepth,
		int2 *ids) {

	int2 pair = ids[index];
	shape_type A_T = obj_data_T[pair.x], B_T = obj_data_T[pair.y];     //Get the type data for each object in the collision pair
	uint ID_A = obj_data_ID[pair.x];
	uint ID_B = obj_data_ID[pair.y];

	if (obj_active[ID_A] == false && obj_active[ID_B] == false) {
		return;
	}
	if (ID_A == ID_B) {
		return;
	}

	real3 posA = body_pos[ID_A], posB = body_pos[ID_B];     //Get the global object position
	real4 rotA = body_rot[ID_A], rotB = body_rot[ID_B];     //Get the global object rotation
	real3 A_X = obj_data_A[pair.x], B_X = obj_data_A[pair.y];
	real3 A_Y = obj_data_B[pair.x], B_Y = obj_data_B[pair.y];
	real3 A_Z = obj_data_C[pair.x], B_Z = obj_data_C[pair.y];
	real4 A_R = (mult(rotA, obj_data_R[pair.x]));
	real4 B_R = (mult(rotB, obj_data_R[pair.y]));

	real envelope = collision_envelope;

	if (A_T == SPHERE || A_T == ELLIPSOID || A_T == BOX || A_T == CYLINDER || A_T == CONE) {
		A_X = quatRotate(A_X, rotA) + posA;
	} else if (A_T == TRIANGLEMESH) {
		envelope = 0;
		A_X = quatRotate(A_X, rotA) + posA;
		A_Y = quatRotate(A_Y, rotA) + posA;
		A_Z = quatRotate(A_Z, rotA) + posA;
	}

	if (B_T == SPHERE || B_T == ELLIPSOID || B_T == BOX || B_T == CYLINDER || B_T == CONE) {
		B_X = quatRotate(B_X, rotB) + posB;
	} else if (B_T == TRIANGLEMESH) {
		envelope = 0;
		B_X = quatRotate(B_X, rotB) + posB;
		B_Y = quatRotate(B_Y, rotB) + posB;
		B_Z = quatRotate(B_Z, rotB) + posB;
	}

	real3 N = R3(1, 0, 0), p1 = R3(0), p2 = R3(0), p0 = R3(0);
	real depth = 0;

	if (A_T == SPHERE && B_T == SPHERE) {

		real3 relpos = B_X - A_X;
		real d2 = dot(relpos, relpos);
		real collide_dist = A_Y.x + B_Y.x;
		if (d2 <= collide_dist * collide_dist) {
			real dist = A_Y.x + B_Y.x;
			N = normalize(relpos);
			p1 = A_X + N * A_Y.x;
			p2 = B_X - N * B_Y.x;
			//depth = length(relpos) - dist;

			//if (ID_A == 0 && ID_B == 36) {

			//cout << "OMG: " << depth << endl;

			//}

		} else {
			//contactDepth[index] = 0;
			return;
		}
	} else {

		if (!CollideAndFindPoint(A_T, A_X, A_Y, A_Z, A_R, B_T, B_X, B_Y, B_Z, B_R, N, p0, depth)) {
			//contactDepth[index] = 0;
			return;
		}

		p1 = dot((TransformSupportVert(A_T, A_X, A_Y, A_Z, A_R, -N) - p0), N) * N + p0;
		p2 = dot((TransformSupportVert(B_T, B_X, B_Y, B_Z, B_R, N) - p0), N) * N + p0;
		N = -N;

	}

	p1 = p1 - (N) * envelope;
	p2 = p2 + (N) * envelope;

	depth = dot(N, p2 - p1);
	norm[index] = N;
	ptA[index] = p1;
	ptB[index] = p2;

	//contactDepth[index] = depth;

}

void ChCNarrowphaseMPR::host_MPR_Update(const shape_type *obj_data_T, const real3 *obj_data_A, const real3 *obj_data_B, const real3 *obj_data_C, const real4 *obj_data_R, const uint *obj_data_ID,
		const bool * obj_active, const real3 *body_pos, const real4 *body_rot, real3 *norm, real3 *ptA, real3 *ptB, real *contactDepth, int2 *ids) {
//#pragma omp parallel for
	for (int index = 0; index < total_possible_contacts; index++) {
		function_MPR_Update(index, obj_data_T, obj_data_A, obj_data_B, obj_data_C, obj_data_R, obj_data_ID, obj_active, body_pos, body_rot, collision_envelope, norm, ptA, ptB, contactDepth, ids);
	}
}


void ChCNarrowphaseMPR::DoNarrowphase(const custom_vector<shape_type> &obj_data_T,
const custom_vector<real3> &obj_data_A,
const custom_vector<real3> &obj_data_B,
const custom_vector<real3> &obj_data_C,
const custom_vector<real4> &obj_data_R,
const custom_vector<uint> &obj_data_ID,
const custom_vector<bool> & obj_active,
const custom_vector<real3> &body_pos,
const custom_vector<real4> &body_rot,
custom_vector<long long> &potentialCollisions,
custom_vector<real3> &norm_data,
custom_vector<real3> &cpta_data,
custom_vector<real3> &cptb_data,
custom_vector<real> &dpth_data,
custom_vector<int2> &bids_data,
uint & number_of_contacts
) {

	total_possible_contacts = potentialCollisions.size();

#ifdef PRINT_LEVEL_2
	cout << "Number of total_possible_contacts: " << total_possible_contacts << endl;
#endif
	custom_vector<uint> generic_counter(total_possible_contacts);
	thrust::fill(generic_counter.begin(), generic_counter.end(), 1);
	norm_data.resize(total_possible_contacts);
	cpta_data.resize(total_possible_contacts);
	cptb_data.resize(total_possible_contacts);
	dpth_data.resize(total_possible_contacts);
	bids_data.resize(total_possible_contacts);
#ifdef SIM_ENABLE_GPU_MODE
	COPY_TO_CONST_MEM(total_possible_contacts);
	COPY_TO_CONST_MEM(collision_envelope);
	device_MPR_Store __KERNEL__(BLOCKS(total_possible_contacts), THREADS)(
	CASTS(obj_data_T),
	CASTR3(obj_data_A),
	CASTR3(obj_data_B),
	CASTR3(obj_data_C),
	CASTR4(obj_data_R),
	CASTU1(obj_data_ID),
	CASTB1(obj_active),
	CASTR3(body_pos),
	CASTR4(body_rot),
	CASTLL(potentialCollisions),
	CASTU1(generic_counter),
	CASTR3(norm_data),
	CASTR3(cpta_data),
	CASTR3(cptb_data),
	CASTR1(dpth_data),
	CASTI2(bids_data));
#else
	host_MPR_Store(
	obj_data_T.data(),
	obj_data_A.data(),
	obj_data_B.data(),
	obj_data_C.data(),
	obj_data_R.data(),
	obj_data_ID.data(),
	obj_active.data(),
	body_pos.data(),
	body_rot.data(),
	potentialCollisions.data(),
	generic_counter.data(),
	norm_data.data(),
	cpta_data.data(),
	cptb_data.data(),
	dpth_data.data(),
	bids_data.data());
#endif
	number_of_contacts = total_possible_contacts - thrust::count(generic_counter.begin(),generic_counter.end(),1);
#ifdef PRINT_LEVEL_2
	cout << "Number of number_of_contacts: " << number_of_contacts << endl;
#endif
	thrust::remove_if(norm_data.begin(), norm_data.end(), generic_counter.begin(), thrust::identity<int>());
	thrust::remove_if(cpta_data.begin(), cpta_data.end(), generic_counter.begin(), thrust::identity<int>());
	thrust::remove_if(cptb_data.begin(), cptb_data.end(), generic_counter.begin(), thrust::identity<int>());
	thrust::remove_if(dpth_data.begin(), dpth_data.end(), generic_counter.begin(), thrust::identity<int>());
	thrust::remove_if(bids_data.begin(), bids_data.end(), generic_counter.begin(), thrust::identity<int>());
	thrust::remove_if(potentialCollisions.begin(), potentialCollisions.end(), generic_counter.begin(), thrust::identity<int>());

	potentialCollisions.resize(number_of_contacts);
	norm_data.resize(number_of_contacts);
	cpta_data.resize(number_of_contacts);
	cptb_data.resize(number_of_contacts);
	dpth_data.resize(number_of_contacts);
	bids_data.resize(number_of_contacts);

//	thrust::sort_by_key(thrust::omp::par,
//	potentialCollisions.begin(),
//	potentialCollisions.end(),
//	thrust::make_zip_iterator(thrust::make_tuple(norm_data.begin(), cpta_data.begin(), cptb_data.begin(), dpth_data.begin(), bids_data.begin()))
//	);

}

void ChCNarrowphaseMPR::UpdateNarrowphase(const custom_vector<shape_type> &obj_data_T,
const custom_vector<real3> &obj_data_A,
const custom_vector<real3> &obj_data_B,
const custom_vector<real3> &obj_data_C,
const custom_vector<real4> &obj_data_R,
const custom_vector<uint> &obj_data_ID,
const custom_vector<bool> & obj_active,
const custom_vector<real3> &body_pos,
const custom_vector<real4> &body_rot,
const uint & number_of_contacts,
custom_vector<real3> &norm_data,
custom_vector<real3> &cpta_data,
custom_vector<real3> &cptb_data,
custom_vector<real> &dpth_data,
custom_vector<int2> &bids_data) {
	total_possible_contacts = number_of_contacts;

	host_MPR_Update(
	obj_data_T.data(),
	obj_data_A.data(),
	obj_data_B.data(),
	obj_data_C.data(),
	obj_data_R.data(),
	obj_data_ID.data(),
	obj_active.data(),
	body_pos.data(),
	body_rot.data(),
	norm_data.data(),
	cpta_data.data(),
	cptb_data.data(),
	dpth_data.data(),
	bids_data.data());

}

