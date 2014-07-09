//////////////////////////////////////////////////
//
//   ChCModelGPU.cpp
//
// ------------------------------------------------
//       Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCCollisionModelParallel.h"
#include "physics/ChBody.h"
#include "physics/ChSystem.h"

namespace chrono {
namespace collision {

ChCollisionModelParallel::ChCollisionModelParallel() {
   nObjects = 0;
   colFam = -1;
   noCollWith = -2;
   inertia = R3(0);
   total_volume = 0;
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
   colFam = -1;
   noCollWith = -2;
   return 1;
}

int ChCollisionModelParallel::BuildModel() {
   this->GetBody()->SetInertiaXX(ChVector<>(inertia.x, inertia.y, inertia.z));

   if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
      GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);
   }

   return 1;
}
bool ChCollisionModelParallel::AddSphere(
      double radius,
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
bool ChCollisionModelParallel::AddEllipsoid(
      double rx,
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
bool ChCollisionModelParallel::AddBox(
      double rx,
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
bool ChCollisionModelParallel::AddTriangle(
      ChVector<> A,
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
bool ChCollisionModelParallel::AddCylinder(
      double rx,
      double ry,
      double rz,
      const ChVector<> &pos,
      const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();

   real3 local_inertia = R3(1 / 12.0 * mass * (3 * rx * rx + ry * ry), 1 / 2.0 * mass * (rx * rx), 1 / 12.0 * mass * (3 * rx * rx + ry * ry));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);
   model_type = CYLINDER;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, ry, rz);
   tData.C = R3(0, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = CYLINDER;
   mData.push_back(tData);
   total_volume += CH_C_PI * rx * rx * ry * 2;
   return true;
}

bool ChCollisionModelParallel::AddRoundedCylinder(
      double rx,
      double ry,
      double sphere_r,
      const ChVector<> &pos,
      const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();

   real3 local_inertia = R3(1 / 12.0 * mass * (3 * rx * rx + ry * ry), 1 / 2.0 * mass * (rx * rx), 1 / 12.0 * mass * (3 * rx * rx + ry * ry));
   ChVector<> position = pos;
   inertia.x += local_inertia.x + mass * (position.Length2() - position.x * position.x);
   inertia.y += local_inertia.y + mass * (position.Length2() - position.y * position.y);
   inertia.z += local_inertia.z + mass * (position.Length2() - position.z * position.z);
   model_type = CYLINDER;
   nObjects++;
   bData tData;
   tData.A = R3(pos.x, pos.y, pos.z);
   tData.B = R3(rx, ry, sphere_r);
   tData.C = R3(0, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = CYLINDER;
   mData.push_back(tData);
   total_volume += CH_C_PI * rx * rx * ry * 2;
   return true;
}

bool ChCollisionModelParallel::AddCone(
      double rx,
      double ry,
      double rz,
      const ChVector<> &pos,
      const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();
   real radius = rx;
   real height = ry;

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
   tData.B = R3(radius, height, radius);
   tData.C = R3(0, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = CONE;
   mData.push_back(tData);
   total_volume += 1 / 3.0 * rx * ry * 2;
   return true;
}

bool ChCollisionModelParallel::AddRoundedCone(
      double rx,
      double ry,
      double sphere_r,
      const ChVector<> &pos,
      const ChMatrix33<> &rot) {
   double mass = this->GetBody()->GetMass();
   real radius = rx;
   real height = ry;
   real sphereradius = sphere_r;
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
   tData.B = R3(radius, height, sphereradius);
   tData.C = R3(0, 0, 0);
   ChMatrix33<> rotation = rot;

   tData.R = R4(rotation.Get_A_quaternion().e0, rotation.Get_A_quaternion().e1, rotation.Get_A_quaternion().e2, rotation.Get_A_quaternion().e3);
   tData.type = CONE;
   mData.push_back(tData);
   total_volume += 1 / 3.0 * rx * ry * 2;
   return true;
}

bool ChCollisionModelParallel::AddCapsule(
      double radius,
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

bool ChCollisionModelParallel::AddConvexHull(
      std::vector<ChVector<double> > &pointlist,
      const ChVector<> &pos,
      const ChMatrix33<> &rot) {
   //NOT SUPPORTED
   return false;
}
bool ChCollisionModelParallel::AddBarrel(
      double Y_low,
      double Y_high,
      double R_vert,
      double R_hor,
      double R_offset,
      const ChVector<> &pos,
      const ChMatrix33<> &rot) {
   //NOT SUPPORTED
   return false;
}

bool ChCollisionModelParallel::AddCone(
      double rad,
      double h) {
   return false;
}
/// Add a triangle mesh to this model
bool ChCollisionModelParallel::AddTriangleMesh(
      const geometry::ChTriangleMesh &trimesh,
      bool is_static,
      bool is_convex,
      const ChVector<> &pos,
      const ChMatrix33<> &rot) {
   model_type = TRIANGLEMESH;
   nObjects += trimesh.getNumTriangles();
   bData tData;
   for (int i = 0; i < trimesh.getNumTriangles(); i++) {
      ChTriangle temptri = trimesh.getTriangle(i);
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
bool ChCollisionModelParallel::AddCopyOfAnotherModel(
      ChCollisionModel *another) {
   //NOT SUPPORTED
   return false;
}
void ChCollisionModelParallel::GetAABB(
      ChVector<> &bbmin,
      ChVector<> &bbmax) const {
}

void ChCollisionModelParallel::SetFamily(
      int mfamily) {
   colFam = mfamily;
}

int ChCollisionModelParallel::GetFamily() {
   return colFam;
}

void ChCollisionModelParallel::SetFamilyMaskNoCollisionWithFamily(
      int mfamily) {
   noCollWith = mfamily;
}

void ChCollisionModelParallel::SetFamilyMaskDoCollisionWithFamily(
      int mfamily) {
   if (noCollWith == mfamily) {
      noCollWith = -1;
   }
}
bool ChCollisionModelParallel::GetFamilyMaskDoesCollisionWithFamily(
      int mfamily) {
   return (noCollWith != mfamily);
}

int ChCollisionModelParallel::GetNoCollFamily() {
   return noCollWith;
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

