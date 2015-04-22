#include <algorithm>

#include "chrono_parallel/collision/ChCAABBGenerator.h"

using namespace chrono::collision;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void ComputeAABBSphere(const real& radius, const real3& position, real3& minp, real3& maxp) {
  minp = position - R3(radius);
  maxp = position + R3(radius);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void ComputeAABBTriangle(const real3& A, const real3& B, const real3& C, real3& minp, real3& maxp) {
  minp.x = std::min(A.x, std::min(B.x, C.x));
  minp.y = std::min(A.y, std::min(B.y, C.y));
  minp.z = std::min(A.z, std::min(B.z, C.z));
  maxp.x = std::max(A.x, std::max(B.x, C.x));
  maxp.y = std::max(A.y, std::max(B.y, C.y));
  maxp.z = std::max(A.z, std::max(B.z, C.z));
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void ComputeAABBBox(const real3& dim,
                           const real3& lpositon,
                           const real3& positon,
                           const real4& lrotation,
                           const real4& rotation,
                           real3& minp,
                           real3& maxp) {
  real4 q1 = mult(rotation, lrotation);
  M33 rotmat = AMat(q1);
  rotmat = AbsMat(rotmat);

  real3 temp = MatMult(rotmat, dim);

  real3 pos = quatRotate(lpositon, rotation) + positon;
  minp = pos - temp;
  maxp = pos + temp;

  // cout<<minp.x<<" "<<minp.y<<" "<<minp.z<<"  |  "<<maxp.x<<" "<<maxp.y<<" "<<maxp.z<<endl;
  //    real3 pos = quatRotate(lpositon, rotation) + positon; //new position
  //    real4 q1 = mult(rotation, lrotation); //full rotation
  //    real4 q = R4(q1.y, q1.z, q1.w, q1.x);
  //    real t[3] = { pos.x, pos.y, pos.z };
  //    real mina[3] = { -dim.x, -dim.y, -dim.z };
  //    real maxa[3] = { dim.x, dim.y, dim.z };
  //    real minb[3] = { 0, 0, 0 };
  //    real maxb[3] = { 0, 0, 0 };
  //    real m[3][3];
  //    real qx2 = q.x * q.x;
  //    real qy2 = q.y * q.y;
  //    real qz2 = q.z * q.z;
  //    m[0][0] = 1 - 2 * qy2 - 2 * qz2;
  //    m[1][0] = 2 * q.x * q.y + 2 * q.z * q.w;
  //    m[2][0] = 2 * q.x * q.z - 2 * q.y * q.w;
  //    m[0][1] = 2 * q.x * q.y - 2 * q.z * q.w;
  //    m[1][1] = 1 - 2 * qx2 - 2 * qz2;
  //    m[2][1] = 2 * q.y * q.z + 2 * q.x * q.w   ;
  //    m[0][2] = 2 * q.x * q.z + 2 * q.y * q.w;
  //    m[1][2] = 2 * q.y * q.z - 2 * q.x * q.w;
  //    m[2][2] = 1 - 2 * qx2 - 2 * qy2;
  //
  //    // For all three axes
  //    for (int i = 0; i < 3; i++) {
  //        // Start by adding in translation
  //        minb[i] = maxb[i] = t[i];
  //
  //        // Form extent by summing smaller and larger terms respectively
  //        for (int j = 0; j < 3; j++) {
  //            real e = m[i][j] * mina[j];
  //            real f = m[i][j] * maxa[j];
  //
  //            if (e < f) {
  //                minb[i] += e;
  //                maxb[i] += f;
  //            } else {
  //                minb[i] += f;
  //                maxb[i] += e;
  //            }
  //        }
  //    }

  //    minp = R3(minb[0], minb[1], minb[2]);
  //    maxp = R3(maxb[0], maxb[1], maxb[2]);
}

static void ComputeAABBCone(const real3& dim,
                            const real3& lpositon,
                            const real3& positon,
                            const real4& lrotation,
                            const real4& rotation,
                            real3& minp,
                            real3& maxp) {
  real4 q1 = mult(rotation, lrotation);
  M33 rotmat = AMat(q1);
  rotmat = AbsMat(rotmat);

  real3 temp = MatMult(rotmat, R3(dim.x, dim.y, dim.z / 2.0));

  real3 pos = quatRotate(lpositon - R3(0, 0, dim.z / 2.0), rotation) + positon;
  minp = pos - temp;
  maxp = pos + temp;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static void ComputeAABBConvex(const real3* convex_points,
                              const real3& B,
                              const real3& lpos,
                              const real3& pos,
                              const real4& rot,
                              real3& minp,
                              real3& maxp) {
  int start = B.y;
  int size = B.x;
  real3 point_0 = quatRotate(convex_points[start] + lpos, rot) + pos;

  minp = maxp = point_0;
  for (int i = start; i < start + size; i++) {
    real3 p = quatRotate(convex_points[i] + lpos, rot) + pos;
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

  minp = minp - R3(B.z);
  maxp = maxp + R3(B.z);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ChCAABBGenerator::ChCAABBGenerator() {
}

void ChCAABBGenerator::GenerateAABB() {
  const host_vector<shape_type>& obj_data_T = data_manager->host_data.typ_rigid;
  const host_vector<uint>& obj_data_ID = data_manager->host_data.id_rigid;
  const host_vector<real3>& obj_data_A = data_manager->host_data.ObA_rigid;
  const host_vector<real3>& obj_data_B = data_manager->host_data.ObB_rigid;
  const host_vector<real3>& obj_data_C = data_manager->host_data.ObC_rigid;
  const host_vector<real4>& obj_data_R = data_manager->host_data.ObR_rigid;
  const host_vector<real3>& convex_data = data_manager->host_data.convex_data;
  const host_vector<real3>& body_pos = data_manager->host_data.pos_rigid;
  const host_vector<real4>& body_rot = data_manager->host_data.rot_rigid;
  uint num_rigid_shapes = data_manager->num_rigid_shapes;

  real collision_envelope = data_manager->settings.collision.collision_envelope;
  host_vector<real3>& aabb_min_rigid = data_manager->host_data.aabb_min_rigid;
  host_vector<real3>& aabb_max_rigid = data_manager->host_data.aabb_max_rigid;

  LOG(TRACE) << "AABB START";

  aabb_min_rigid.resize(num_rigid_shapes);
  aabb_max_rigid.resize(num_rigid_shapes);

#pragma omp parallel for
  for (int index = 0; index < num_rigid_shapes; index++) {
    shape_type type = obj_data_T[index];
    uint id = obj_data_ID[index];
    real3 A = obj_data_A[index];
    real3 B = obj_data_B[index];
    real3 C = obj_data_C[index];
    real3 position = body_pos[id];
    real4 rotation = (mult(body_rot[id], obj_data_R[index]));
    real3 temp_min;
    real3 temp_max;

    if (type == SPHERE) {
      A = quatRotate(A, body_rot[id]);
      ComputeAABBSphere(B.x + collision_envelope, A + position, temp_min, temp_max);
    } else if (type == TRIANGLEMESH) {
      A = quatRotate(A, body_rot[id]) + position;
      B = quatRotate(B, body_rot[id]) + position;
      C = quatRotate(C, body_rot[id]) + position;
      ComputeAABBTriangle(A, B, C, temp_min, temp_max);
    } else if (type == ELLIPSOID || type == BOX || type == CYLINDER || type == CONE) {
      ComputeAABBBox(B + collision_envelope, A, position, obj_data_R[index], body_rot[id], temp_min, temp_max);
    } else if (type == ROUNDEDBOX || type == ROUNDEDCYL || type == ROUNDEDCONE) {
      ComputeAABBBox(B + C.x + collision_envelope, A, position, obj_data_R[index], body_rot[id], temp_min, temp_max);
    } else if (type == CAPSULE) {
      real3 B_ = R3(B.x, B.x + B.y, B.z) + collision_envelope;
      ComputeAABBBox(B_, A, position, obj_data_R[index], body_rot[id], temp_min, temp_max);
    } else if (type == CONVEX) {
      ComputeAABBConvex(convex_data.data(), B, A, position, rotation, temp_min, temp_max);
      temp_min -= collision_envelope;
      temp_max += collision_envelope;
    } else {
      continue;
    }

    aabb_min_rigid[index] = temp_min;
    aabb_max_rigid[index] = temp_max;
  }

  LOG(TRACE) << "AABB END";
}
