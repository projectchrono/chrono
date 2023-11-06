%{
#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChVisualShapes.h"

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
%shared_ptr(chrono::ChVisualShapeSurface)


%include "../../../chrono/assets/ChVisualShape.h"    
%include "../../../chrono/assets/ChVisualShapes.h"    

