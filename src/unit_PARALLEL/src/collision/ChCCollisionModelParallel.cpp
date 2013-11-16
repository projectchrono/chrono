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
bool ChCollisionModelParallel::AddSphere(double radius, const ChVector<> &pos) {
	double mass = this->GetBody()->GetMass();

	real3 local_inertia = R3(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
	inertia.x += local_inertia.x+mass*(pos.Length2()-pos.x*pos.x);
	inertia.y += local_inertia.y+mass*(pos.Length2()-pos.y*pos.y);
	inertia.z += local_inertia.z+mass*(pos.Length2()-pos.z*pos.z);

	model_type = SPHERE;
	nObjects++;
	bData tData;
	tData.A = R3(pos.x, pos.y, pos.z);
	tData.B = R3(radius, 0, 0);
	tData.C = R3(0, 0, 0);
	tData.R = R4(1, 0, 0, 0);
	tData.type = SPHERE;
	mData.push_back(tData);
	return true;
}
bool ChCollisionModelParallel::AddEllipsoid(double rx, double ry, double rz, const ChVector<> &pos, const ChMatrix33<> &rot) {
	double mass = this->GetBody()->GetMass();

	real3 local_inertia =R3(1 / 5.0 * mass * (ry * ry + rz * rz), 1 / 5.0 * mass * (rx * rx + rz * rz), 1 / 5.0 * mass * (rx * rx + ry * ry));
	inertia.x += local_inertia.x+mass*(pos.Length2()-pos.x*pos.x);
	inertia.y += local_inertia.y+mass*(pos.Length2()-pos.y*pos.y);
	inertia.z += local_inertia.z+mass*(pos.Length2()-pos.z*pos.z);


	model_type = ELLIPSOID;
	nObjects++;
	bData tData;
	tData.A = R3(pos.x, pos.y, pos.z);
	tData.B = R3(rx, ry, rz);
	tData.C = R3(0, 0, 0);
	tData.R = R4(rot.Get_A_quaternion().e0, rot.Get_A_quaternion().e1, rot.Get_A_quaternion().e2, rot.Get_A_quaternion().e3);
	tData.type = ELLIPSOID;
	mData.push_back(tData);
	return true;
}
bool ChCollisionModelParallel::AddBox(double rx, double ry, double rz, const ChVector<> &pos, const ChMatrix33<> &rot) {
	double mass = this->GetBody()->GetMass();

	real3 local_inertia =R3(1 / 12.0 * mass * (ry * ry + rz * rz), 1 / 12.0 * mass * (rx * rx + rz * rz), 1 / 12.0 * mass * (rx * rx + ry * ry));
	inertia.x += local_inertia.x+mass*(pos.Length2()-pos.x*pos.x);
	inertia.y += local_inertia.y+mass*(pos.Length2()-pos.y*pos.y);
	inertia.z += local_inertia.z+mass*(pos.Length2()-pos.z*pos.z);

	model_type = BOX;
	nObjects++;
	bData tData;
	tData.A = R3(pos.x, pos.y, pos.z);
	tData.B = R3(rx, ry, rz);
	tData.C = R3(0, 0, 0);
	tData.R = R4(rot.Get_A_quaternion().e0, rot.Get_A_quaternion().e1, rot.Get_A_quaternion().e2, rot.Get_A_quaternion().e3);
	tData.type = BOX;
	mData.push_back(tData);
	return true;
}
bool ChCollisionModelParallel::AddTriangle(ChVector<> A, ChVector<> B, ChVector<> C, const ChVector<> &pos, const ChMatrix33<> &rot) {
	double mass = this->GetBody()->GetMass();
	model_type = TRIANGLEMESH;
	nObjects++;
	bData tData;
	tData.A = R3(A.x + pos.x, A.y + pos.y, A.z + pos.z);
	tData.B = R3(B.x + pos.x, B.y + pos.y, B.z + pos.z);
	tData.C = R3(C.x + pos.x, C.y + pos.y, C.z + pos.z);
	tData.R = R4(rot.Get_A_quaternion().e0, rot.Get_A_quaternion().e1, rot.Get_A_quaternion().e2, rot.Get_A_quaternion().e3);
	tData.type = TRIANGLEMESH;
	mData.push_back(tData);
	return true;
}
bool ChCollisionModelParallel::AddCylinder(double rx, double ry, double rz, const ChVector<> &pos, const ChMatrix33<> &rot) {
	double mass = this->GetBody()->GetMass();

	real3 local_inertia =R3(1 / 12.0 * mass * (3 * rx * rx + ry * ry), 1 / 2.0 * mass * (rx * rx), 1 / 12.0 * mass * (3 * rx * rx + ry * ry));
	inertia.x += local_inertia.x+mass*(pos.Length2()-pos.x*pos.x);
	inertia.y += local_inertia.y+mass*(pos.Length2()-pos.y*pos.y);
	inertia.z += local_inertia.z+mass*(pos.Length2()-pos.z*pos.z);
	model_type = CYLINDER;
	nObjects++;
	bData tData;
	tData.A = R3(pos.x, pos.y, pos.z);
	tData.B = R3(rx, ry, rz);
	tData.C = R3(0, 0, 0);
	tData.R = R4(rot.Get_A_quaternion().e0, rot.Get_A_quaternion().e1, rot.Get_A_quaternion().e2, rot.Get_A_quaternion().e3);
	tData.type = CYLINDER;
	mData.push_back(tData);
	return true;
}

bool ChCollisionModelParallel::AddCone(double rx, double ry, double rz, const ChVector<> &pos, const ChMatrix33<> &rot) {
	double mass = this->GetBody()->GetMass();
	real radius = rx;
	real height = ry;

	real3 local_inertia =R3(
			(3.0f / 80.0f) * mass * (radius * radius + 4 * height * height),
			(3.0f / 10.0f) * mass * radius * radius,
			(3.0f / 80.0f) * mass * (radius * radius + 4 * height * height));
	inertia.x += local_inertia.x+mass*(pos.Length2()-pos.x*pos.x);
	inertia.y += local_inertia.y+mass*(pos.Length2()-pos.y*pos.y);
	inertia.z += local_inertia.z+mass*(pos.Length2()-pos.z*pos.z);

	model_type = CONE;
	nObjects++;
	bData tData;
	tData.A = R3(pos.x, pos.y, pos.z);
	tData.B = R3(radius, height, radius);
	tData.C = R3(0, 0, 0);
	tData.R = R4(rot.Get_A_quaternion().e0, rot.Get_A_quaternion().e1, rot.Get_A_quaternion().e2, rot.Get_A_quaternion().e3);
	tData.type = CONE;
	mData.push_back(tData);
	return true;
}

bool ChCollisionModelParallel::AddConvexHull(std::vector<ChVector<double> > &pointlist, const ChVector<> &pos, const ChMatrix33<> &rot) {
	//NOT SUPPORTED
	return false;
}
bool ChCollisionModelParallel::AddBarrel(double Y_low, double Y_high, double R_vert, double R_hor, double R_offset, const ChVector<> &pos, const ChMatrix33<> &rot) {
	//NOT SUPPORTED
	return false;
}
bool ChCollisionModelParallel::AddRectangle(double rx, double ry) {
	return false;
}
bool ChCollisionModelParallel::AddDisc(double rad) {
	return false;
}
bool ChCollisionModelParallel::AddEllipse(double rx, double ry) {
	return false;
}
bool ChCollisionModelParallel::AddCapsule(double len, double rad) {
	return false;
}
bool ChCollisionModelParallel::AddCone(double rad, double h) {
	return false;
}
/// Add a triangle mesh to this model
bool ChCollisionModelParallel::AddTriangleMesh(
                 const geometry::ChTriangleMesh &trimesh,
                 bool                            is_static,
                 bool                            is_convex,
                 const ChVector<>               &pos,
                 const ChMatrix33<>             &rot)
{
	model_type = TRIANGLEMESH;
	nObjects += trimesh.getNumTriangles();
	bData tData;
	for (int i = 0; i < trimesh.getNumTriangles(); i++) {
		ChTriangle temptri = trimesh.getTriangle(i);
		tData.A = R3(temptri.p1.x + pos.x, temptri.p1.y + pos.y, temptri.p1.z + pos.z);
		tData.B = R3(temptri.p2.x + pos.x, temptri.p2.y + pos.y, temptri.p2.z + pos.z);
		tData.C = R3(temptri.p3.x + pos.x, temptri.p3.y + pos.y, temptri.p3.z + pos.z);
		tData.R = R4(rot.Get_A_quaternion().e0, rot.Get_A_quaternion().e1, rot.Get_A_quaternion().e2, rot.Get_A_quaternion().e3);
		tData.type = TRIANGLEMESH;
		mData.push_back(tData);
	}

	return true;
}
bool ChCollisionModelParallel::AddCopyOfAnotherModel(ChCollisionModel *another) {
	//NOT SUPPORTED
	return false;
}
void ChCollisionModelParallel::GetAABB(ChVector<> &bbmin, ChVector<> &bbmax) const {
}

void ChCollisionModelParallel::SetFamily(int mfamily) {
	colFam = mfamily;
}

int ChCollisionModelParallel::GetFamily() {
	return colFam;
}

void ChCollisionModelParallel::SetFamilyMaskNoCollisionWithFamily(int mfamily) {
	noCollWith = mfamily;
}

void ChCollisionModelParallel::SetFamilyMaskDoCollisionWithFamily(int mfamily) {
	if (noCollWith == mfamily) {
		noCollWith = -1;
	}
}
bool ChCollisionModelParallel::GetFamilyMaskDoesCollisionWithFamily(int mfamily) {
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

ChPhysicsItem *ChCollisionModelParallel::GetPhysicsItem() {
	return (ChPhysicsItem *) GetBody();
}
}     // END_OF_NAMESPACE____
}     // END_OF_NAMESPACE____

