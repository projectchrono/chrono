#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

%csmethodmodifiers chrono::ChGeometry::GetType "public virtual new"

#endif             // --------------------------------------------------------------------- CSHARP



%{

/* Includes the header in the wrapper code */
#include "chrono/geometry/ChGeometry.h"
#include "chrono/geometry/ChVolume.h"
#include "chrono/geometry/ChSurface.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/geometry/ChSphere.h"
#include "chrono/geometry/ChCylinder.h"
#include "chrono/geometry/ChCapsule.h"
#include "chrono/geometry/ChCone.h"
#include "chrono/geometry/ChEllipsoid.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChLineArc.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChLineNurbs.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLinePoly.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/geometry/ChLineBSpline.h"
#include "chrono/geometry/ChLineCam.h"
#include "chrono/geometry/ChSurface.h"
#include "chrono/geometry/ChSurfaceNurbs.h"
#include "chrono/geometry/ChVolume.h"
#include "chrono/geometry/ChTriangle.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/geometry/ChRoundedCylinder.h"
#include "chrono/geometry/ChRoundedBox.h"


using namespace chrono;

%}

%shared_ptr(chrono::ChAABB)

%shared_ptr(chrono::ChGeometry)
%shared_ptr(chrono::ChLine)
%shared_ptr(chrono::ChVolume)
%shared_ptr(chrono::ChSurface)
%shared_ptr(chrono::ChBox)
%shared_ptr(chrono::ChSphere)
%shared_ptr(chrono::ChCylinder)
%shared_ptr(chrono::ChCapsule)
%shared_ptr(chrono::ChCone)
%shared_ptr(chrono::ChEllipsoid)
%shared_ptr(chrono::ChLineArc)
%shared_ptr(chrono::ChLineSegment)
%shared_ptr(chrono::ChLineNurbs)
%shared_ptr(chrono::ChLinePath)
%shared_ptr(chrono::ChLinePoly)
%shared_ptr(chrono::ChLineBezier)
%shared_ptr(chrono::ChLineBSpline)
%shared_ptr(chrono::ChLineCam)
%shared_ptr(chrono::ChTriangle)
%shared_ptr(chrono::ChTriangleMesh)
%shared_ptr(chrono::ChTriangleMeshSoup)
%shared_ptr(chrono::ChTriangleMeshConnected)
%shared_ptr(chrono::ChSurface)
%shared_ptr(chrono::ChSurfaceNurbs)
%shared_ptr(chrono::ChRoundedCylinder)
%shared_ptr(chrono::ChRoundedBox)
 
//%feature("director") chrono::ChGeometry;
//%ignore chrono::ChGeometry::Clone;

%template(ChTriangleMeshConnected_list) std::vector<chrono::ChTriangleMeshConnected>;


/* Parse the header file(s) to generate wrappers */
%include "../../../chrono/geometry/ChGeometry.h"
%include "../../../chrono/geometry/ChLine.h"
%include "../../../chrono/geometry/ChVolume.h"
%include "../../../chrono/geometry/ChSurface.h"
%include "../../../chrono/geometry/ChBox.h"
%include "../../../chrono/geometry/ChSphere.h"
%include "../../../chrono/geometry/ChCylinder.h"
%include "../../../chrono/geometry/ChCapsule.h"
%include "../../../chrono/geometry/ChCone.h"
%include "../../../chrono/geometry/ChEllipsoid.h"
%include "../../../chrono/geometry/ChLineArc.h"
%include "../../../chrono/geometry/ChLineSegment.h"
%include "../../../chrono/geometry/ChLineNurbs.h"
%include "../../../chrono/geometry/ChLinePath.h"
%include "../../../chrono/geometry/ChLinePoly.h"
%include "../../../chrono/geometry/ChLineBezier.h"
%include "../../../chrono/geometry/ChLineBSpline.h"
%include "../../../chrono/geometry/ChLineCam.h"
%include "../../../chrono/geometry/ChTriangle.h"
%include "../../../chrono/geometry/ChTriangleMesh.h"
%include "../../../chrono/geometry/ChTriangleMeshSoup.h"
%include "../../../chrono/geometry/ChTriangleMeshConnected.h"
%include "../../../chrono/geometry/ChSurface.h"
%include "../../../chrono/geometry/ChSurfaceNurbs.h"
%include "../../../chrono/geometry/ChRoundedCylinder.h"
%include "../../../chrono/geometry/ChRoundedBox.h"

