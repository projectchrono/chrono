#include "ChCCollisionGPU.h"
#include "ChCCollisionGPU.cuh"
//__global__ void Sphere_Sphere(object * object_data, int3 * Pair,
//      uint* Contact_Number, contactGPU* CData, uint totalPossibleConts) {
//  uint index = blockIdx.x * blockDim.x + threadIdx.x;
//  if (index >= totalPossibleConts) {
//      return;
//  }
//  int3 pair = Pair[index];
//  if (pair.z == 0) {
//      object A = object_data[pair.x];
//      object B = object_data[pair.y];
//      real3 N = R3(B.A) - R3(A.A);
//      real centerDist = dot(N, N);
//      real rAB = B.B.x + A.B.x;
//      if (centerDist <= (rAB) * (rAB)) {
//          real dist = sqrtf(centerDist);
//          N = N / dist;
//          AddContact(CData, index, A.A.w, B.A.w, R3(A.A) + A.B.x * N, R3(B.A)
//                  - B.B.x * N, N, -dist);
//          Contact_Number[index] = index;
//      }
//  }
//}

__device__ __host__ inline real3 GetSupportPoint_Sphere(const real3 &B, const real3 &n) {
    return (B.x) * n;
}
__device__ __host__ inline real3 GetSupportPoint_Triangle(const real3 &A, const real3 &B, const real3 &C, const real3 &n) {
    real dist = dot(A, n);
    real3 point = A;

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
__device__ __host__ inline real3 GetSupportPoint_Box(const real3 &B, const real3 &n) {
    real3 result = R3(0, 0, 0);
    result.x = n.x <= 0 ? -B.x : B.x;
    result.y = n.y <= 0 ? -B.y : B.y;
    result.z = n.z <= 0 ? -B.z : B.z;
    return result;
}
__device__ __host__ inline real3 GetSupportPoint_Ellipsoid(const real3 &B, const real3 &n) {
    return B * B * n / length(n * B);
}
__device__ __host__ real sign(real x) {
    if (x < 0) {
        return -1;
    } else {
        return 1;
    }
}

__device__ __host__ inline real3 GetSupportPoint_Cylinder(const real3 &B, const real3 &n) {
    //return real3(0,0,0);
    real3 u = R3(0, 1, 0);
    real3 w = n - (dot(u, n)) * u;
    real3 result;

    if (length(w) != 0) {
        result = sign(dot(u, n)) * B.y * u + B.x * normalize(w);
    } else {
        result = sign(dot(u, n)) * B.y * u;
    }

    return result;
}
__device__ __host__ inline real3 GetSupportPoint_Plane(const real3 &B, const real3 &n) {
    real3 result = B;

    if (n.x < 0)
        result.x = -result.x;

    if (n.y < 0)
        result.y = -result.y;

    return result;
}
__device__ __host__ inline real3 GetSupportPoint_Cone(const real3 &B, const real3 &n) {
    return real3(0, 0, 0);
}
__device__ __host__ inline real3 GetCenter_Sphere() {
    return Zero_Vector;
}
__device__ __host__ inline real3 GetCenter_Triangle(const real3 &A, const real3 &B, const real3 &C) {
    return real3((A.x + B.x + C.x) / 3.0f, (A.y + B.y + C.y) / 3.0f, (A.z + B.z + C.z) / 3.0f);
}
__device__ __host__ inline real3 GetCenter_Box() {
    return Zero_Vector;
}
__device__ __host__ inline real3 GetCenter_Ellipsoid() {
    return Zero_Vector;
}
__device__ __host__ inline real3 GetCenter_Cylinder() {
    return Zero_Vector;
}
__device__ __host__ inline real3 GetCenter_Plane() {
    return Zero_Vector;
}
__device__ __host__ inline real3 GetCenter_Cone() {
    return Zero_Vector;
}

__device__ __host__ real3 GetCenter(const int &type, const real3 &A, const real3 &B, const real3 &C) {
    if (type == _TRIANGLEMESH) {
        return GetCenter_Triangle(A, B, C);
    } //triangle
    else {
        return real3(0, 0, 0) + A;
    } //All other shapes assumed to be locally centered
}
__device__ __host__ real3 TransformSupportVert(const int &type, const real3 &A, const real3 &B, const real3 &C, const real4 &R, const real3 &b) {
    real3 localSupport;
    real3 n = normalize(b);

    if (type == _TRIANGLEMESH) { //triangle
        return GetSupportPoint_Triangle(A, B, C, n);
    } else if (type == _SPHERE) { //sphere
        localSupport = GetSupportPoint_Sphere(B, quatRotate(n, inv(R)));
    } else if (type == _ELLIPSOID) { //ellipsoid
        localSupport = GetSupportPoint_Ellipsoid(B, quatRotate(n, inv(R)));
    } else if (type == _BOX) { //box
        localSupport = GetSupportPoint_Box(B, quatRotate(n, inv(R)));
    } else if (type == _CYLINDER) { //cylinder
        localSupport = GetSupportPoint_Cylinder(B, quatRotate(n, inv(R)));
    } else if (type == _RECT) { //plane
        localSupport = GetSupportPoint_Plane(B, quatRotate(n, inv(R)));
    } else if (type == _CONE) { //cone
        localSupport = GetSupportPoint_Cone(B, quatRotate(n, inv(R)));
    }

    return quatRotate(localSupport, R) + A; //globalSupport
}

__device__ __host__ real dist_line(real3 &P, real3 &x0, real3 &b, real3 &witness) {
    real dist, t;
    real3 d, a;
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
        witness =witness* t;
        witness += x0;
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
        d1 =d1* s;
        d2 =d2* t;
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
__device__ __host__ bool CollideAndFindPoint(int typeA, real3 A_X, real3 A_Y, real3 A_Z, real4 A_R, int typeB, real3 B_X, real3 B_Y, real3 B_Z, real4 B_R, real3 &returnNormal, real3 &point,
        real &depth) {
    real3 v01, v02, v0, n, v11, v12, v1, v21, v22, v2;
    // v0 = center of Minkowski sum
    v01 = GetCenter(typeA, A_X, A_Y, A_Z);
    v02 = GetCenter(typeB, B_X, B_Y, B_Z);
    v0 = v02 - v01;

    // Avoid case where centers overlap -- any direction is fine in this case
    if (IsZero3(v0))
        v0 = real3(1, 0, 0);

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
        //point1 = v11;
        //point2 = v12;
        point = (v11 + v12) * .5;
        depth = dot((v12 - v11), n);
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
    real3 v31, v32, v3;

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
            hit = true; // HIT!!!
        }

        // Find the support point in the direction of the wedge face
        real3 v41 = TransformSupportVert(typeA, A_X, A_Y, A_Z, A_R, -n);
        real3 v42 = TransformSupportVert(typeB, B_X, B_Y, B_Z, B_R, n);
        real3 v4 = v42 - v41;
        real delta = dot((v4 - v3), n);
        depth = -dot(v4, n);

        // If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex, then we can terminate
        if (delta <= kCollideEpsilon || depth >= 0. || phase2 > 100) {
            if (hit) {
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
                point =point* inv * .5;
                returnNormal = normalize(n);
            }

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

__host__ __device__ inline real4 Quat_from_AngAxis(const real &angle, const real3 &v) {
    real sinhalf = sinf(angle * 0.5f);
    real4 quat;
    quat.x = cosf(angle * 0.5f);
    quat.y = v.x * sinhalf;
    quat.z = v.y * sinhalf;
    quat.w = v.z * sinhalf;
    return quat;
}
__host__ __device__
unsigned int hash(unsigned int a) {
    a = (a + 0x7ed55d16) + (a << 12);
    a = (a ^ 0xc761c23c) ^ (a >> 19);
    a = (a + 0x165667b1) + (a << 5);
    a = (a + 0xd3a2646c) ^ (a << 9);
    a = (a + 0xfd7046c5) + (a << 3);
    a = (a ^ 0xb55a4f09) ^ (a >> 16);
    return a;
}
__global__ void MPR_GPU_Store(real3 *pos, real4 *rot, real3 *obA, real3 *obB, real3 *obC, real4 *obR, int3 *typ, long long *Pair, uint *Contact_Number, real3 *norm, real3 *ptA, real3 *ptB,
                              real *contactDepth, int2 *ids, real3 *aux, uint totalPossibleConts) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;

    if (index >= totalPossibleConts) {
        return;
    }

    //if (Contact_Number[index] != 0xFFFFFFFF) {
    //  return;
    //}
    long long p = Pair[index];
    int2 pair = I2(int(p >> 32), int(p & 0xffffffff));
    //if(pair.z<4){return;}
    int3 A_T = typ[pair.x], B_T = typ[pair.y]; //Get the type data for each object in the collision pair
    real3 posA = pos[A_T.z], posB = pos[B_T.z]; //Get the global object position
    real4 rotA = rot[A_T.z], rotB = rot[B_T.z]; //Get the global object rotation
    real3 A_X = obA[A_T.y], B_X = obA[B_T.y];
    real3 A_Y = obB[A_T.y], B_Y = obB[B_T.y];
    real3 A_Z = obC[A_T.y], B_Z = obC[B_T.y];
    real4 A_R = mult(rotA, obR[A_T.y]);
    real4 B_R = mult(rotB, obR[B_T.y]);

    if (A_T.x == _SPHERE || A_T.x == _ELLIPSOID || A_T.x == _BOX || A_T.x == _CYLINDER) {
        A_X = quatRotate(A_X, rotA) + posA;
    } else if (A_T.x == _TRIANGLEMESH) {
        A_X = quatRotate(A_X + posA, A_R);
        A_Y = quatRotate(A_Y + posA, A_R);
        A_Z = quatRotate(A_Z + posA, A_R);
    }

    if (B_T.x == _SPHERE || B_T.x == _ELLIPSOID || B_T.x == _BOX || B_T.x == _CYLINDER) {
        B_X = quatRotate(B_X, rotB) + posB;
    } else if (B_T.x == _TRIANGLEMESH) {
        B_X = quatRotate(B_X + posB, B_R);
        B_Y = quatRotate(B_Y + posB, B_R);
        B_Z = quatRotate(B_Z + posB, B_R);
    }

    //unsigned int seed = hash(threadIdx.x) ;
    //thrust::default_random_engine rng(seed);
    //thrust::uniform_real_distribution<real> u01(-.01, .01);
    int num = 0;
    //if (A_T.x == _SPHERE || B_T.x == _SPHERE) {
    //  num = 2;
    //}
    real4 A_R_T = A_R, B_R_T = B_R;
    //real3 vect1,vect2;
//  uint counter=0;
//  real3 p1_old, p2_old;
//  while(counter<100) {
//      counter++;
    //if (num == 0) {
//          vect1 = normalize(R3(u01(rng), u01(rng), u01(rng)));
//          vect2 = normalize(R3(u01(rng), u01(rng), u01(rng)));
    //}
    //if (num == 1) {
    //  vect = R3(0, 1, 0);
    //}
    //if (num == 2) {
    //  vect = R3(0, 0, 1);
    //}
//      real4 rand1 = Quat_from_AngAxis(u01(rng), vect1);
//      real4 rand2 = Quat_from_AngAxis(u01(rng), vect2);
//      if (aux[A_T.z].x == 1) {
//          A_R_T = mult(A_R, rand1);
//      } else if (aux[B_T.z].x == 1) {
//          B_R_T = mult(B_R, rand2);
//      }
    real3 N, p1, p2, p0;
    real depth = 0;

    if (!CollideAndFindPoint(A_T.x, A_X, A_Y, A_Z, A_R_T, B_T.x, B_X, B_Y, B_Z, B_R_T, N, p0, depth)) {
        return;
    };

    p1 = dot((TransformSupportVert(A_T.x, A_X, A_Y, A_Z, A_R, -N) - p0), N) * N + p0;

    p2 = dot((TransformSupportVert(B_T.x, B_X, B_Y, B_Z, B_R, N) - p0), N) * N + p0;

//      if(num>1){
//
//          if(isEqual(p1.x,p1_old.x)&&isEqual(p1.y,p1_old.y)&&isEqual(p1.z,p1_old.z)){
//              continue;
//          }
//          if(isEqual(p2.x,p2_old.x)&&isEqual(p2.y,p2_old.y)&&isEqual(p2.z,p2_old.z)){
//              continue;
//          }
//
//      }
    norm[index + num * totalPossibleConts] = -N;

    ptA[index + num * totalPossibleConts] = p1;

    ptB[index + num * totalPossibleConts] = p2;

    contactDepth[index + num * totalPossibleConts] = -depth;

    ids[index + num * totalPossibleConts] = I2(A_T.z, B_T.z);

    Contact_Number[index + num * totalPossibleConts] = 0;

//      p1_old=p1;
//      p2_old=p2;
//      num++;
//
//      if(num>3){return;}
//  }
}
__global__ void CopyGamma(int *to, real3 *oldG, real3 *newG, int contacts) {
    uint i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i >= contacts) {
        return;
    }

    newG[to[i]] = oldG[i];
}

void ChCCollisionGPU::Narrowphase(gpu_container &gpu_data) {
    //DBG("C ");
    gpu_data.generic_counter.resize(gpu_data.number_of_contacts_possible);
    thrust::fill(gpu_data.generic_counter.begin(), gpu_data.generic_counter.end(), 1);
    uint number_of_contacts_possible = gpu_data.number_of_contacts_possible;
    gpu_data.device_norm_data.resize(gpu_data.number_of_contacts_possible);
    gpu_data.device_cpta_data.resize(gpu_data.number_of_contacts_possible);
    gpu_data.device_cptb_data.resize(gpu_data.number_of_contacts_possible);
    gpu_data.device_dpth_data.resize(gpu_data.number_of_contacts_possible);
    gpu_data.device_bids_data.resize(gpu_data.number_of_contacts_possible);
//DBG(" X ")
    //cout << "  POSSIBLE  " << number_of_contacts_possible << "  ";
    MPR_GPU_Store CUDA_KERNEL_DIM(BLOCKS(number_of_contacts_possible), THREADS)(CASTR3(gpu_data.device_pos_data), CASTR4(gpu_data.device_rot_data), CASTR3(gpu_data.device_ObA_data),
            CASTR3(gpu_data.device_ObB_data), CASTR3(gpu_data.device_ObC_data), CASTR4(gpu_data.device_ObR_data), CASTI3(gpu_data.device_typ_data), CASTLL(gpu_data.device_pair_data),
            CASTU1(gpu_data.generic_counter), CASTR3(gpu_data.device_norm_data), CASTR3(gpu_data.device_cpta_data), CASTR3(gpu_data.device_cptb_data), CASTR1(gpu_data.device_dpth_data),
            CASTI2(gpu_data.device_bids_data), CASTR3(gpu_data.device_aux_data), number_of_contacts_possible);
    gpu_data.number_of_contacts = number_of_contacts_possible - Thrust_Count(gpu_data.generic_counter, 1);
    //DBG(" Y ")
    //thrust::remove_if(gpu_data.device_norm_data.begin(),gpu_data.device_norm_data.end(),gpu_data.generic_counter.begin(),thrust::identity<int>());
    //thrust::remove_if(gpu_data.device_cpta_data.begin(),gpu_data.device_cpta_data.end(),gpu_data.generic_counter.begin(),thrust::identity<int>());
    //thrust::remove_if(gpu_data.device_cptb_data.begin(),gpu_data.device_cptb_data.end(),gpu_data.generic_counter.begin(),thrust::identity<int>());
    //thrust::remove_if(gpu_data.device_dpth_data.begin(),gpu_data.device_dpth_data.end(),gpu_data.generic_counter.begin(),thrust::identity<int>());
    //thrust::remove_if(gpu_data.device_bids_data.begin(),gpu_data.device_bids_data.end(),gpu_data.generic_counter.begin(),thrust::identity<int>());
    //thrust::remove_if(gpu_data.device_pair_data.begin(),gpu_data.device_pair_data.end(),gpu_data.generic_counter.begin(),thrust::identity<int>());
    thrust::sort_by_key(
        gpu_data.generic_counter.begin(),
        gpu_data.generic_counter.end(),
        thrust::make_zip_iterator(
            thrust::make_tuple(gpu_data.device_norm_data.begin(), gpu_data.device_cpta_data.begin(), gpu_data.device_cptb_data.begin(), gpu_data.device_dpth_data.begin(),
                               gpu_data.device_bids_data.begin(), gpu_data.device_pair_data.begin()))
    );
    //DBG(" D");
//
//
//
//  //thrust::device_vector<real3> old_gamma = data_container->device_gam_data;
//
    gpu_data.device_norm_data.resize(gpu_data.number_of_contacts);
    gpu_data.device_cpta_data.resize(gpu_data.number_of_contacts);
    gpu_data.device_cptb_data.resize(gpu_data.number_of_contacts);
    gpu_data.device_dpth_data.resize(gpu_data.number_of_contacts);
    gpu_data.device_bids_data.resize(gpu_data.number_of_contacts);
    gpu_data.device_gam_data.resize(gpu_data.number_of_contacts);
    Thrust_Fill(gpu_data.device_gam_data, R3(0));
    //  contact_pair.resize(number_of_contacts);
    //
    //  thrust::sort_by_key(contact_pair.begin(), contact_pair.end(), thrust::make_zip_iterator(thrust::make_tuple(data_container->device_norm_data.begin(), data_container->device_cpta_data.begin(),
    //          data_container->device_cptb_data.begin(), data_container->device_dpth_data.begin(), data_container->device_bids_data.begin())));
    //  if (old_contact_pair.size() != 0) {
    //      thrust::device_vector<int> res(old_contact_pair.size());
    //
    //      thrust::binary_search(contact_pair.begin(), contact_pair.end(), old_contact_pair.begin(), old_contact_pair.end(), res.begin());//list of persistent contacts
    //      thrust::sort_by_key(res.begin(), res.end(), old_contact_pair.begin(),thrust::greater<int>() );//index of common contacts from old list
    //
    //      int numP = Thrust_Count(res,1);
    //      old_contact_pair.resize(numP);
    //      if (numP > 0) {cout<<numP<<"\t";
    //          thrust::device_vector<int> temporaryB(numP);
    //          thrust::lower_bound(contact_pair.begin(), contact_pair.end(), old_contact_pair.begin(), old_contact_pair.end(), temporaryB.begin());//return index of common new contact
    //CopyGamma     <<<BLOCKS(numP),THREADS>>>(
    //              CASTI1(temporaryB),
    //              CASTR3(old_gamma),
    //              CASTR3(data_container->device_gam_data),
    //              numP);
    //      //              for(int i=0; i<old_contact_pair.size(); i++){cout<<old_contact_pair[i]<<endl;}
    //      //              cout<<"------------------------"<<endl;
    //      //              for(int i=0; i<contact_pair.size(); i++){cout<<contact_pair[i]<<endl;}
    //      //              cout<<"------------------------"<<endl;
    //      //              for(int i=0; i<res1.size(); i++){cout<<res1[i]<<endl;}
    //      //              cout<<"------------------------"<<endl;
    //      //              for(int i=0; i<temporaryA.size(); i++){cout<<temporaryA[i]<<endl;}
    //      //              cout<<"------------------------"<<endl;
    //      //              for(int i=0; i<temporaryB.size(); i++){cout<<temporaryB[i]<<endl;}
    //      //
    //      //              exit(0);
    //  }
    //}
    //old_contact_pair = contact_pair;
}


