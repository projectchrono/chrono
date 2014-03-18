#ifndef CH_UTILS_CREATORS_H
#define CH_UTILS_CREATORS_H

#include <cmath>
#include <vector>
#include <string>

#include "core/ChSmartpointers.h"
#include "core/ChVector.h"
#include "core/ChQuaternion.h"

#include "physics/ChSystem.h"
#include "physics/ChSystemDEM.h"
#include "physics/ChBody.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChMaterialSurface.h"
#include "physics/ChMaterialSurfaceDEM.h"

#include "assets/ChSphereShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChBoxShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChConeShape.h"

#include "ChSystemParallel.h"

#include "utils/common.h"


namespace chrono {
namespace utils {


// -------------------------------------------------------------------------------
// AddSphereGeometry
// AddEllipsoidGeometry
// AddBoxGeometry
// AddCylinderGeometry
// AddConeGeometry
//
// Utility functions for adding contact and asset geometry shapes to a body
// -------------------------------------------------------------------------------
void AddSphereGeometry(ChBody*               body,
                       double                radius,
                       const ChVector<>&     pos = ChVector<>(0,0,0),
                       const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
	body->GetCollisionModel()->AddSphere(radius, pos);

	ChSharedPtr<ChSphereShape> sphere = ChSharedPtr<ChAsset>(new ChSphereShape);
	sphere->GetSphereGeometry().rad = radius;
	sphere->Pos = pos;
	sphere->Rot = rot;

	body->GetAssets().push_back(sphere);
}

void AddEllipsoidGeometry(ChBody*               body,
                          const ChVector<>&     size,
                          const ChVector<>&     pos = ChVector<>(0,0,0),
                          const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
	body->GetCollisionModel()->AddEllipsoid(size.x, size.y, size.z, pos, rot);

	ChSharedPtr<ChEllipsoidShape> ellipsoid = ChSharedPtr<ChAsset>(new ChEllipsoidShape);
	ellipsoid->GetEllipsoidGeometry().rad = size;
	ellipsoid->Pos = pos;
	ellipsoid->Rot = rot;

	body->GetAssets().push_back(ellipsoid);
}

void AddBoxGeometry(ChBody*               body,
                    const ChVector<>&     size,
                    const ChVector<>&     pos = ChVector<>(0,0,0),
                    const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
	body->GetCollisionModel()->AddBox(size.x, size.y, size.z, pos, rot);

	ChSharedPtr<ChBoxShape> box = ChSharedPtr<ChAsset>(new ChBoxShape);
	box->GetBoxGeometry().Size = size;
	box->Pos = pos;
	box->Rot = rot;

	body->GetAssets().push_back(box);
}

void AddCylinderGeometry(ChBody*               body,
                         double                radius,
                         double                height,
                         const ChVector<>&     pos = ChVector<>(0,0,0),
                         const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
	body->GetCollisionModel()->AddCylinder(radius, radius, height, pos, rot);

	ChSharedPtr<ChCylinderShape> cylinder = ChSharedPtr<ChAsset>(new ChCylinderShape);
	cylinder->GetCylinderGeometry().rad = radius;
	cylinder->GetCylinderGeometry().p1 = ChVector<>(0,  height / 2, 0);
	cylinder->GetCylinderGeometry().p2 = ChVector<>(0, -height / 2, 0);
	cylinder->Pos = pos;
	cylinder->Rot = rot;

	body->GetAssets().push_back(cylinder);
}

void AddConeGeometry(ChBody*               body,
                     double                radius,
                     double                height,
                     const ChVector<>&     pos = ChVector<>(0,0,0),
                     const ChQuaternion<>& rot = ChQuaternion<>(1,0,0,0))
{
	body->GetCollisionModel()->AddCone(radius, radius, height, pos, rot);

	ChSharedPtr<ChConeShape> cone = ChSharedPtr<ChAsset>(new ChConeShape);
	cone->GetConeGeometry().rad = ChVector<>(radius, height, radius);
	cone->Pos = pos;
	cone->Rot = rot;

	body->GetAssets().push_back(cone);
}


// -------------------------------------------------------------------------------
// CreateBoxContainer
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
                           const ChVector<>&                   pos = ChVector<>(0,0,0),
                           const ChQuaternion<>&               rot = ChQuaternion<>(1,0,0,0),
                           bool                                collide = true)
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
	ChSharedPtr<ChBodyDEM>  bodyPtr(body);

	if (sysType == SEQUENTIAL_DEM)
		system->AddBody(bodyPtr);
	else
		((ChSystemParallel*) system)->AddBody(bodyPtr);

}


} // end namespace utils
} // end namespace chrono


#endif
