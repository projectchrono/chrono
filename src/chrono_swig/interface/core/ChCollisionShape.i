#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::ChCollisionShape::GetType "public new"

#endif             // --------------------------------------------------------------------- CSHARP



%{
#include "chrono/collision/ChCollisionShape.h"
#include "chrono/collision/ChCollisionShapes.h"
#include "chrono/collision/ChCollisionShapeArc2D.h"
#include "chrono/collision/ChCollisionShapeBarrel.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/collision/ChCollisionShapeCapsule.h"
#include "chrono/collision/ChCollisionShapeCone.h"
#include "chrono/collision/ChCollisionShapeConvexHull.h"
#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/collision/ChCollisionShapeCylindricalShell.h"
#include "chrono/collision/ChCollisionShapeEllipsoid.h"
#include "chrono/collision/ChCollisionShapePath2D.h"
#include "chrono/collision/ChCollisionShapePoint.h"
#include "chrono/collision/ChCollisionShapeRoundedBox.h"
#include "chrono/collision/ChCollisionShapeRoundedCylinder.h"
#include "chrono/collision/ChCollisionShapeSegment2D.h"
#include "chrono/collision/ChCollisionShapeSphere.h"
#include "chrono/collision/ChCollisionShapeTriangle.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"

using namespace chrono;
%}

namespace chrono {
class ChContactMethod;
class ChContactMaterial;
class ChCollisionModel;
}

%include "std_pair.i"

%shared_ptr(chrono::ChCollisionShape)
%shared_ptr(chrono::ChCollisionShapeArc2D)
%shared_ptr(chrono::ChCollisionShapeBarrel)
%shared_ptr(chrono::ChCollisionShapeBox)
%shared_ptr(chrono::ChCollisionShapeCapsule)
%shared_ptr(chrono::ChCollisionShapeCone)
%shared_ptr(chrono::ChCollisionShapeConvexHull)
%shared_ptr(chrono::ChCollisionShapeCylinder)
%shared_ptr(chrono::ChCollisionShapeCylindricalShell)
%shared_ptr(chrono::ChCollisionShapeEllipsoid)
%shared_ptr(chrono::ChCollisionShapePath2D)
%shared_ptr(chrono::ChCollisionShapePoint)
%shared_ptr(chrono::ChCollisionShapeRoundedBox)
%shared_ptr(chrono::ChCollisionShapeRoundedCylinder)
%shared_ptr(chrono::ChCollisionShapeSegment2D)
%shared_ptr(chrono::ChCollisionShapeSphere)
%shared_ptr(chrono::ChCollisionShapeTriangle)
%shared_ptr(chrono::ChCollisionShapeTriangleMesh)

// Parse the header file to generate wrappers
%include "../../../chrono/collision/ChCollisionShape.h"
%include "../../../chrono/collision/ChCollisionShapes.h"
%include "../../../chrono/collision/ChCollisionShapeArc2D.h"
%include "../../../chrono/collision/ChCollisionShapeBarrel.h"
%include "../../../chrono/collision/ChCollisionShapeBox.h"
%include "../../../chrono/collision/ChCollisionShapeCapsule.h"
%include "../../../chrono/collision/ChCollisionShapeCone.h"
%include "../../../chrono/collision/ChCollisionShapeConvexHull.h"
%include "../../../chrono/collision/ChCollisionShapeCylinder.h"
%include "../../../chrono/collision/ChCollisionShapeCylindricalShell.h"
%include "../../../chrono/collision/ChCollisionShapeEllipsoid.h"
%include "../../../chrono/collision/ChCollisionShapePath2D.h"
%include "../../../chrono/collision/ChCollisionShapePoint.h"
%include "../../../chrono/collision/ChCollisionShapeRoundedBox.h"
%include "../../../chrono/collision/ChCollisionShapeRoundedCylinder.h"
%include "../../../chrono/collision/ChCollisionShapeSegment2D.h"
%include "../../../chrono/collision/ChCollisionShapeSphere.h"
%include "../../../chrono/collision/ChCollisionShapeTriangle.h"
%include "../../../chrono/collision/ChCollisionShapeTriangleMesh.h"


// Define CollisionShapeInstance as a pair of shared_ptr<ChCollisionShape> and ChFrame<>
%template(CollisionShapeInstance) std::pair<std::shared_ptr<chrono::ChCollisionShape>, chrono::ChFrame<double>>;

// Define a vector of CollisionShapeInstance
%template(vector_CollisionShapeInstance) std::vector<std::pair<std::shared_ptr<chrono::ChCollisionShape>, chrono::ChFrame<double>>>;

