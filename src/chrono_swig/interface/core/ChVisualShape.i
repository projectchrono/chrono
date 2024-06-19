%{
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualShapeBarrel.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeRoundedCylinder.h"
#include "chrono/assets/ChVisualShapeRoundedBox.h"
#include "chrono/assets/ChVisualShapePath.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/assets/ChVisualShapeSurface.h"
#include "chrono/assets/ChVisualShapeFEA.h"

using namespace chrono;
%}

namespace chrono {
class ChPhysicsItem;
namespace fea {
class ChMesh;
class ChNodeFEAxyz;
class ChNodeFEAxyzP;
class ChElementBase;
}}

%template(material_list) std::vector<std::shared_ptr<chrono::ChVisualMaterial>>;

%shared_ptr(chrono::ChVisualShape)
%shared_ptr(chrono::ChVisualShapeFEA)
%shared_ptr(chrono::ChVisualShapeModelFile)
%shared_ptr(chrono::ChVisualShapeTriangleMesh)
%shared_ptr(chrono::ChVisualShapeSphere)
%shared_ptr(chrono::ChVisualShapeEllipsoid)
%shared_ptr(chrono::ChVisualShapeBarrel)
%shared_ptr(chrono::ChVisualShapeBox)
%shared_ptr(chrono::ChVisualShapeCone)
%shared_ptr(chrono::ChVisualShapeCylinder)
%shared_ptr(chrono::ChVisualShapeCapsule)
%shared_ptr(chrono::ChVisualShapeRoundedCylinder)
%shared_ptr(chrono::ChVisualShapeRoundedBox)
%shared_ptr(chrono::ChVisualShapePath)
%shared_ptr(chrono::ChVisualShapeLine)
%shared_ptr(chrono::ChVisualShapePointPoint)
%shared_ptr(chrono::ChVisualShapeRotSpring)
%shared_ptr(chrono::ChVisualShapeSegment)
%shared_ptr(chrono::ChVisualShapeSpring)
%shared_ptr(chrono::ChVisualShapeSurface)





%include "../../../chrono/assets/ChVisualShape.h"
%include "../../../chrono/assets/ChVisualShapeModelFile.h"
%include "../../../chrono/assets/ChVisualShapeTriangleMesh.h"
%include "../../../chrono/assets/ChVisualShapeSphere.h"
%include "../../../chrono/assets/ChVisualShapeEllipsoid.h"
%include "../../../chrono/assets/ChVisualShapeBarrel.h"
%include "../../../chrono/assets/ChVisualShapeBox.h"
%include "../../../chrono/assets/ChVisualShapeCone.h"
%include "../../../chrono/assets/ChVisualShapeCylinder.h"
%include "../../../chrono/assets/ChVisualShapeCapsule.h"
%include "../../../chrono/assets/ChVisualShapeRoundedCylinder.h"
%include "../../../chrono/assets/ChVisualShapeRoundedBox.h"
%include "../../../chrono/assets/ChVisualShapePath.h"
%include "../../../chrono/assets/ChVisualShapeLine.h"
%include "../../../chrono/assets/ChVisualShapePointPoint.h"
%include "../../../chrono/assets/ChVisualShapeSurface.h"
%include "../../../chrono/assets/ChVisualShapeFEA.h"


