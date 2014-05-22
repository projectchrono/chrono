#include "creators.h"

namespace chrono {
namespace utils {

// -------------------------------------------------------------------------------
// CreateBoxContainerDEM
// CreateBoxContainerDVI
//
// Create a fixed body with contact and asset geometry representing a box with 5
// walls (no top).
// -------------------------------------------------------------------------------
void AddWall(ChBody*              body,
             const ChVector<>&    loc,
             const ChVector<>&    hdim)
{
	// Append to collision geometry
	body->GetCollisionModel()->AddBox(hdim.x, hdim.y, hdim.z, loc);

	// Append to assets
	ChSharedPtr<ChBoxShape> box_shape = ChSharedPtr<ChAsset>(new ChBoxShape);
	box_shape->Pos = loc;
	box_shape->Rot = ChQuaternion<>(1,0,0,0);
	box_shape->GetBoxGeometry().Size = hdim;

	body->GetAssets().push_back(box_shape);
}

void CreateBoxContainerDEM(ChSystem*                           system,
                           int                                 id,
                           ChSharedPtr<ChMaterialSurfaceDEM>&  mat,
                           const ChVector<>&                   hdim,
                           double                              hthick,
                           const ChVector<>&                   pos,
                           const ChQuaternion<>&               rot,
                           bool                                collide)
{
	// Infer system type.
	SystemType sysType = GetSystemType(system);
	assert(sysType == SEQUENTIAL_DEM || sysType == PARALLEL_DEM);

	// Create the body and set material
	ChBodyDEM* body;

	if (sysType == SEQUENTIAL_DEM)
		body = new ChBodyDEM();
	else
		body = new ChBodyDEM(new ChCollisionModelParallel);

	body->SetMaterialSurfaceDEM(mat);

	// Set body properties and geometry.
	body->SetIdentifier(id);
	body->SetMass(1);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetCollide(collide);
	body->SetBodyFixed(true);

	body->GetCollisionModel()->ClearModel();
	AddWall(body, ChVector<>(0, 0, -hthick), ChVector<>(hdim.x, hdim.y, hthick));
	AddWall(body, ChVector<>(-hdim.x-hthick, 0, hdim.z), ChVector<>(hthick, hdim.y, hdim.z));
	AddWall(body, ChVector<>( hdim.x+hthick, 0, hdim.z), ChVector<>(hthick, hdim.y, hdim.z));
	AddWall(body, ChVector<>(0, -hdim.y-hthick, hdim.z), ChVector<>(hdim.x, hthick, hdim.z));
	AddWall(body, ChVector<>(0,  hdim.y+hthick, hdim.z), ChVector<>(hdim.x, hthick, hdim.z));
	body->GetCollisionModel()->BuildModel();

	// Attach the body to the system.
	system->AddBody(ChSharedPtr<ChBodyDEM>(body));
}

void CreateBoxContainerDVI(ChSystem*                           system,
                           int                                 id,
                           ChSharedPtr<ChMaterialSurface>&     mat,
                           const ChVector<>&                   hdim,
                           double                              hthick,
                           const ChVector<>&                   pos,
                           const ChQuaternion<>&               rot,
                           bool                                collide)
{
	// Infer system type.
	SystemType sysType = GetSystemType(system);
	assert(sysType == SEQUENTIAL_DVI || sysType == PARALLEL_DVI);

	// Create the body and set material
	ChBody* body;

	if (sysType == SEQUENTIAL_DVI)
		body = new ChBody();
	else
		body = new ChBody(new ChCollisionModelParallel);

	body->SetMaterialSurface(mat);

	// Set body properties and geometry.
	body->SetIdentifier(id);
	body->SetMass(1);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetCollide(collide);
	body->SetBodyFixed(true);

	body->GetCollisionModel()->ClearModel();
	AddWall(body, ChVector<>(0, 0, -hthick), ChVector<>(hdim.x, hdim.y, hthick));
	AddWall(body, ChVector<>(-hdim.x-hthick, 0, hdim.z), ChVector<>(hthick, hdim.y, hdim.z));
	AddWall(body, ChVector<>( hdim.x+hthick, 0, hdim.z), ChVector<>(hthick, hdim.y, hdim.z));
	AddWall(body, ChVector<>(0, -hdim.y-hthick, hdim.z), ChVector<>(hdim.x, hthick, hdim.z));
	AddWall(body, ChVector<>(0,  hdim.y+hthick, hdim.z), ChVector<>(hdim.x, hthick, hdim.z));
	body->GetCollisionModel()->BuildModel();

	// Attach the body to the system.
	system->AddBody(ChSharedPtr<ChBody>(body));
}


}  // namespace utils
}  // namespace chrono