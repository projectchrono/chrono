// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: class for a parallel collision model
// =============================================================================

#include "chrono_parallel/collision/ChCCollisionModelParallel.h"
#include "physics/ChBody.h"
#include "physics/ChSystem.h"

namespace chrono {
namespace collision {

ChCollisionModelParallel::ChCollisionModelParallel()
: nObjects(0),
  family_group(1),
  family_mask(0x7FFF),
  inertia(ZERO_VECTOR),
  total_volume(0)
{
}

ChCollisionModelParallel::~ChCollisionModelParallel() {
   mData.clear();
}
int ChCollisionModelParallel::ClearModel() {
   if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
      GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
   }

   mData.clear();
   nObjects = 0;
   family_group = 1;
   family_mask = 0x7FFF;
   return 1;
}

int ChCollisionModelParallel::BuildModel() {
   this->GetBody()->SetInertiaXX(ChVector<>(inertia.x, inertia.y, inertia.z));

   if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
      GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);
   }

   return 1;
}
bool ChCollisionModelParallel::AddSphere(double radius,
                                         const ChVector<> &pos) {
   double mass = this->GetBody()->GetMass();

   real3 local_inertia = R3(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
   ChVector<> position = pos;

   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);

   model_type = SPHERE;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(radius, 0, 0);
   tData.C = R3(0, 0, 0);
   tData.R = R4(1, 0, 0, 0);
   tData.type = SPHERE;
   mData.push_back(tData);
   total_volume += 4.0 / 3.0 * CH_C_PI * pow(radius, 3.0);

   return true;
}
bool ChCollisionModelParallel::AddEllipsoid(double rx,
                                            double ry,
                                            double rz,
                                            const ChVector<> &pos,
                                            const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();

   real3 local_inertia = R3(1 / 5.0 * mass * (ry * ry + rz * rz), 1 / 5.0 * mass * (rx * rx + rz * rz), 1 / 5.0 * mass * (rx * rx + ry * ry));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);

   model_type = ELLIPSOID;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, ry, rz);
   tData.C = R3(0, 0, 0);
   ChMatrix33<> rotation = rot;
   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = ELLIPSOID;
   mData.push_back(tData);
   total_volume += 4.0 / 3.0 * CH_C_PI * rx * ry * rz;
   return true;
}
bool ChCollisionModelParallel::AddBox(double rx,
                                      double ry,
                                      double rz,
                                      const ChVector<> &pos,
                                      const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();

   real3 local_inertia = R3(1 / 12.0 * mass * (ry * ry + rz * rz), 1 / 12.0 * mass * (rx * rx + rz * rz), 1 / 12.0 * mass * (rx * rx + ry * ry));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);

   model_type = BOX;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, ry, rz);
   tData.C = R3(0, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = BOX;
   mData.push_back(tData);
   total_volume += rx * 2 * ry * 2 * rz * 2;
   return true;
}

bool ChCollisionModelParallel::AddRoundedBox(double rx,
                                             double ry,
                                             double rz,
                                             double sphere_r,
                                             const ChVector<> &pos,
                                             const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();

   real3 local_inertia = R3(1 / 12.0 * mass * (ry * ry + rz * rz), 1 / 12.0 * mass * (rx * rx + rz * rz), 1 / 12.0 * mass * (rx * rx + ry * ry));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);

   model_type = ROUNDEDBOX;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, ry, rz);
   tData.C = R3(sphere_r, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = ROUNDEDBOX;
   mData.push_back(tData);
   total_volume += rx * 2 * ry * 2 * rz * 2;
   return true;
}

bool ChCollisionModelParallel::AddTriangle(ChVector<> A,
                                           ChVector<> B,
                                           ChVector<> C,
                                           const ChVector<> &pos,
                                           const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();
   model_type = TRIANGLEMESH;
   nObjects++;
   bData tData;
   tData.A = R3(A.x + pos.x, A.y + pos.y, A.z + pos.z);
   tData.B = R3(B.x + pos.x, B.y + pos.y, B.z + pos.z);
   tData.C = R3(C.x + pos.x, C.y + pos.y, C.z + pos.z);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = TRIANGLEMESH;
   mData.push_back(tData);
   return true;
}
bool ChCollisionModelParallel::AddCylinder(double rx,
                                           double rz,
                                           double hy,
                                           const ChVector<> &pos,
                                           const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();

   real3 local_inertia = R3(1 / 12.0 * mass * (3 * rx * rx + hy * hy), 1 / 2.0 * mass * (rx * rz), 1 / 12.0 * mass * (3 * rz * rz + hy * hy));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);
   model_type = CYLINDER;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, hy, rz);
   tData.C = R3(0, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = CYLINDER;
   mData.push_back(tData);
   total_volume += CH_C_PI * rx * rz * hy * 2;
   return true;
}

bool ChCollisionModelParallel::AddRoundedCylinder(double rx,
                                                  double rz,
                                                  double hy,
                                                  double sphere_r,
                                                  const ChVector<> &pos,
                                                  const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();

   real3 local_inertia = R3(1 / 12.0 * mass * (3 * rx * rx + hy * hy), 1 / 2.0 * mass * (rx * rz), 1 / 12.0 * mass * (3 * rz * rz + hy * hy));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);
   model_type = ROUNDEDCYL;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, hy, rz);
   tData.C = R3(sphere_r, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = ROUNDEDCYL;
   mData.push_back(tData);
   total_volume += CH_C_PI * rx * rz * hy * 2;
   return true;
}

bool ChCollisionModelParallel::AddCone(double rx,
                                       double rz,
                                       double hy,
                                       const ChVector<> &pos,
                                       const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();
   real radius = rx;
   real height = hy;

   real3 local_inertia = R3((3.0f / 80.0f) * mass * (radius * radius + 4 * height * height), (3.0f / 10.0f) * mass * radius * radius,
                            (3.0f / 80.0f) * mass * (radius * radius + 4 * height * height));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);

   model_type = CONE;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, height, rz);
   tData.C = R3(0, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = CONE;
   mData.push_back(tData);
   total_volume += 1 / 3.0 * rx * hy * 2;
   return true;
}

bool ChCollisionModelParallel::AddRoundedCone(double rx,
                                              double rz,
                                              double hy,
                                              double sphere_r,
                                              const ChVector<> &pos,
                                              const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();
   real radius = rx;
   real height = hy;

   real3 local_inertia = R3((3.0f / 80.0f) * mass * (radius * radius + 4 * height * height), (3.0f / 10.0f) * mass * radius * radius,
                            (3.0f / 80.0f) * mass * (radius * radius + 4 * height * height));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);

   model_type = ROUNDEDCONE;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, height, rz);
   tData.C = R3(sphere_r, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = ROUNDEDCONE;
   mData.push_back(tData);
   total_volume += 1 / 3.0 * rx * hy * 2;
   return true;
}

bool ChCollisionModelParallel::AddCapsule(double radius,
                                          double hlen,
                                          const ChVector<> &pos,
                                          const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();

   //// TODO  For now just approximate inertia with that of a cylinder
   real hlen1 = radius + hlen;
   real3 local_inertia = R3(1 / 12.0 * mass * (3 * radius * radius + hlen1 * hlen1), 1 / 2.0 * mass * (radius * radius), 1 / 12.0 * mass * (3 * radius * radius + hlen1 * hlen1));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);

   model_type = CAPSULE;
   nObjects++;

   bData tData;
   ChQuaternion<> q = rot.Get_A_quaternion();

   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(radius, hlen, radius);
   tData.C = R3(0, 0, 0);
   tData.R = R4(q.e0, q.e1, q.e2, q.e3);
   tData.type = CAPSULE;

   mData.push_back(tData);

   total_volume += 2 * CH_C_PI * radius * radius * (hlen + 2 * radius / 3);

   return true;
}

bool ChCollisionModelParallel::AddConvexHull(std::vector<ChVector<double> > &pointlist,
                                             const ChVector<> &pos,
                                             const ChMatrix33<> &rot) {
	  inertia = R3(1);    // so that it gets initialized to something
	  model_type = CONVEX;
	  nObjects++;
	  bData tData;
	  tData.A = R3(pos.x, pos.y, pos.z);
	  tData.B = R3(pointlist.size(), local_convex_data.size(), 0);
	  tData.C = R3(0, 0, 0);
	  ChMatrix33<> rotation = rot;

	  tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
	  tData.type = CONVEX;
	  mData.push_back(tData);
	  total_volume += 0;

	  for (int i = 0; i < pointlist.size(); i++) {
	    local_convex_data.push_back(R3(pointlist[i].x, pointlist[i].y, pointlist[i].z));
	  }

	  return true;
}
bool ChCollisionModelParallel::AddBarrel(double Y_low,
                                         double Y_high,
                                         double R_vert,
                                         double R_hor,
                                         double R_offset,
                                         const ChVector<> &pos,
                                         const ChMatrix33<> &rot) {
   //NOT SUPPORTED
   return false;
}

/// Add a triangle mesh to this model
bool ChCollisionModelParallel::AddTriangleMesh(const geometry::ChTriangleMesh &trimesh,
                                               bool is_static,
                                               bool is_convex,
                                               const ChVector<> &pos,
                                               const ChMatrix33<> &rot) {
   model_type = TRIANGLEMESH;
   nObjects += trimesh.getNumTriangles();
   bData tData;
   for (int i = 0; i < trimesh.getNumTriangles(); i++) {
      geometry::ChTriangle temptri = trimesh.getTriangle(i);
      tData.A = R3(temptri.p1.x + pos.x, temptri.p1.y + pos.y, temptri.p1.z + pos.z);
      tData.B = R3(temptri.p2.x + pos.x, temptri.p2.y + pos.y, temptri.p2.z + pos.z);
      tData.C = R3(temptri.p3.x + pos.x, temptri.p3.y + pos.y, temptri.p3.z + pos.z);
      ChMatrix33<> rotation = rot;

      tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
      tData.type = TRIANGLEMESH;
      mData.push_back(tData);
   }

   return true;
}
bool ChCollisionModelParallel::AddCopyOfAnotherModel(ChCollisionModel *another) {
   //NOT SUPPORTED
   return false;
}
void ChCollisionModelParallel::GetAABB(ChVector<> &bbmin,
                                       ChVector<> &bbmax) const {
}

void ChCollisionModelParallel::SetFamily(int mfamily) {
  // Set family_group to a power of 2, with the set bit in position mfamily.
  assert(mfamily >= 0 && mfamily < 15);
  family_group = (1 << mfamily);
}

int ChCollisionModelParallel::GetFamily() {
  // Return the position of the single bit set in family_group.
  unsigned i = 1;
  int pos = 1;
  while (!(i & family_group)) {
    i << 1;
    pos++;
  }
  return pos;
}

void ChCollisionModelParallel::SetFamilyMaskNoCollisionWithFamily(int mfamily) {
  // Clear the family_mask bit in position mfamily.
  assert(mfamily >= 0 && mfamily < 15);
  family_mask &= ~(1 << mfamily);
}

void ChCollisionModelParallel::SetFamilyMaskDoCollisionWithFamily(int mfamily) {
  // Set the family_mask bit in position mfamily.
  assert(mfamily >= 0 && mfamily < 15);
  family_mask |= (1 << mfamily);
}

bool ChCollisionModelParallel::GetFamilyMaskDoesCollisionWithFamily(int mfamily) {
  // Return true if the family_mask bit in position mfamily is set.
  assert(mfamily >= 0 && mfamily < 15);
  return family_mask & (1 << mfamily);
}

void ChCollisionModelParallel::SetFamilyGroup(short group) {
  // In orer to properly encode a collision family, the value 'group' must be a
  // power of 2.
  assert(group > 0 && !(group & (group - 1)));
  family_group = group;
}

void ChCollisionModelParallel::SetFamilyMask(short mask) {
  // In order to properly encode a collision mask, the value 'mask' must not
  // exceed 0x7FFFF (i.e. 15 right bits all set)
  assert(mask >= 0 && mask <= 0x7FFF);
  family_mask = mask;
}

void ChCollisionModelParallel::SyncPosition() {
   ChBody *bpointer = GetBody();
   assert(bpointer);
   //assert(bpointer->GetSystem());
}

float ChCollisionModelParallel::getVolume() {
   return total_volume;
}

ChPhysicsItem *ChCollisionModelParallel::GetPhysicsItem() {
   return (ChPhysicsItem *) GetBody();
}

}     // END_OF_NAMESPACE____
}     // END_OF_NAMESPACE____
