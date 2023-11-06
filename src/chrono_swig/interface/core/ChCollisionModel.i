%{
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/collision/ChCollisionShapes.h"

using namespace chrono;
using namespace chrono::collision;
%}

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

%feature("director") chrono::collision::ChCollisionModel; // ?????
%feature("nodirector") chrono::collision::ChCollisionModel::GetPhysicsItem;

#endif             // --------------------------------------------------------------------- PYTHON

%shared_ptr(chrono::collision::ChCollisionShape)
%shared_ptr(chrono::collision::ChCollisionShapeArc2D)
%shared_ptr(chrono::collision::ChCollisionShapeBarrel)
%shared_ptr(chrono::collision::ChCollisionShapeBox)
%shared_ptr(chrono::collision::ChCollisionShapeCapsule)
%shared_ptr(chrono::collision::ChCollisionShapeCone)
%shared_ptr(chrono::collision::ChCollisionShapeConvexHull)
%shared_ptr(chrono::collision::ChCollisionShapeCylinder)
%shared_ptr(chrono::collision::ChCollisionShapeCylindricalShell)
%shared_ptr(chrono::collision::ChCollisionShapeEllipsoid)
%shared_ptr(chrono::collision::ChCollisionShapePath2D)
%shared_ptr(chrono::collision::ChCollisionShapePoint)
%shared_ptr(chrono::collision::ChCollisionShapeRoundedBox)
%shared_ptr(chrono::collision::ChCollisionShapeRoundedCylinder)
%shared_ptr(chrono::collision::ChCollisionShapeSegment2D)
%shared_ptr(chrono::collision::ChCollisionShapeSphere)
%shared_ptr(chrono::collision::ChCollisionShapeTriangle)
%shared_ptr(chrono::collision::ChCollisionShapeTriangleMesh)

// Parse the header file to generate wrappers
%include "../../../chrono/collision/ChCollisionModel.h"
%include "../../../chrono/collision/bullet/ChCollisionModelBullet.h"
%include "../../../chrono/collision/ChCollisionShapes.h"