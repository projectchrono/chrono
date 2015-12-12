#include <algorithm>

#include "chrono_parallel/collision/ChCAABBGenerator.h"
using namespace chrono;
using namespace chrono::collision;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void ComputeAABBSphere(const real& radius, const real3& position, real3& minp, real3& maxp) {
  minp = position - radius;
  maxp = position + radius;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void ComputeAABBTriangle(const real3& A, const real3& B, const real3& C, real3& minp, real3& maxp) {
  minp.x = Min(A.x, Min(B.x, C.x));
  minp.y = Min(A.y, Min(B.y, C.y));
  minp.z = Min(A.z, Min(B.z, C.z));
  maxp.x = Max(A.x, Max(B.x, C.x));
  maxp.y = Max(A.y, Max(B.y, C.y));
  maxp.z = Max(A.z, Max(B.z, C.z));
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void ComputeAABBBox(const real3& dim,
                           const real3& lpositon,
                           const real3& positon,
                           const quaternion& rotation,
                           real3& minp,
                           real3& maxp) {
  real3 temp = AbsRotate(rotation, dim);
  real3 pos = Rotate(lpositon, rotation) + positon;
  minp = pos - temp;
  maxp = pos + temp;
}

static void ComputeAABBCone(const real3& dim,
                            const real3& lpositon,
                            const real3& positon,
                            const quaternion& rotation,
                            real3& minp,
                            real3& maxp) {
  real3 temp = AbsRotate(rotation, real3(dim.x, dim.y, dim.z / 2.0));
  real3 pos = Rotate(lpositon - real3(0, 0, dim.z / 2.0), rotation) + positon;
  minp = pos - temp;
  maxp = pos + temp;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void ComputeAABBConvex(const real3* convex_points,
                              const real3& B,
                              const real3& lpos,
                              const real3& pos,
                              const quaternion& rot,
                              real3& minp,
                              real3& maxp) {
  int start = B.y;
  int size = B.x;
  real3 point_0 = Rotate(convex_points[start] + lpos, rot) + pos;

  minp = maxp = point_0;
  for (int i = start; i < start + size; i++) {
    real3 p = Rotate(convex_points[i] + lpos, rot) + pos;
    if (minp.x > p.x) {
      minp.x = p.x;
    }
    if (minp.y > p.y) {
      minp.y = p.y;
    }
    if (minp.z > p.z) {
      minp.z = p.z;
    }
    if (maxp.x < p.x) {
      maxp.x = p.x;
    }
    if (maxp.y < p.y) {
      maxp.y = p.y;
    }
    if (maxp.z < p.z) {
      maxp.z = p.z;
    }
  }

  minp = minp - B.z;
  maxp = maxp + B.z;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ChCAABBGenerator::ChCAABBGenerator() {
  data_manager=0;
}

void ChCAABBGenerator::GenerateAABB() {
  const host_vector<shape_type>& obj_data_T = data_manager->host_data.typ_rigid;
  const host_vector<uint>& obj_data_ID = data_manager->host_data.id_rigid;
  const host_vector<real3>& obj_data_A = data_manager->host_data.ObA_rigid;
  const host_vector<real3>& obj_data_B = data_manager->host_data.ObB_rigid;
  const host_vector<real3>& obj_data_C = data_manager->host_data.ObC_rigid;
  const host_vector<quaternion>& obj_data_R = data_manager->host_data.ObR_rigid;
  const host_vector<real3>& convex_data = data_manager->host_data.convex_data;
  const host_vector<real3>& body_pos = data_manager->host_data.pos_rigid;
  const host_vector<quaternion>& body_rot = data_manager->host_data.rot_rigid;
  const uint num_rigid_shapes = data_manager->num_rigid_shapes;
  real collision_envelope = data_manager->settings.collision.collision_envelope;
  host_vector<real3>& aabb_min = data_manager->host_data.aabb_min;
  host_vector<real3>& aabb_max = data_manager->host_data.aabb_max;

  LOG(TRACE) << "AABB START";

  aabb_min.resize(num_rigid_shapes);
  aabb_max.resize(num_rigid_shapes);

#pragma omp parallel for
  for (int index = 0; index < num_rigid_shapes; index++) {
    shape_type type = obj_data_T[index];
    uint id = obj_data_ID[index];
    real3 A = obj_data_A[index];
    real3 B = obj_data_B[index];
    real3 C = obj_data_C[index];
    real3 position = body_pos[id];
    quaternion rotation = Mult(body_rot[id], obj_data_R[index]);
    real3 temp_min;
    real3 temp_max;

    if (type == SPHERE) {
      A = Rotate(A, body_rot[id]);
      ComputeAABBSphere(B.x + collision_envelope, A + position, temp_min, temp_max);
    } else if (type == TRIANGLEMESH) {
      A = Rotate(A, body_rot[id]) + position;
      B = Rotate(B, body_rot[id]) + position;
      C = Rotate(C, body_rot[id]) + position;
      ComputeAABBTriangle(A, B, C, temp_min, temp_max);
    } else if (type == ELLIPSOID || type == BOX || type == CYLINDER || type == CONE) {
      ComputeAABBBox(B + collision_envelope, A, position, rotation, temp_min, temp_max);
    } else if (type == ROUNDEDBOX || type == ROUNDEDCYL || type == ROUNDEDCONE) {
      ComputeAABBBox(B + C.x + collision_envelope, A, position, rotation, temp_min, temp_max);
    } else if (type == CAPSULE) {
      real3 B_ = real3(B.x, B.x + B.y, B.z) + collision_envelope;
      ComputeAABBBox(B_, A, position, rotation, temp_min, temp_max);
    } else if (type == CONVEX) {
      ComputeAABBConvex(convex_data.data(), B, A, position, rotation, temp_min, temp_max);
      temp_min -= collision_envelope;
      temp_max += collision_envelope;
    } else {
      continue;
    }

    aabb_min[index] = temp_min;
    aabb_max[index] = temp_max;
  }

  LOG(TRACE) << "AABB END";
}
