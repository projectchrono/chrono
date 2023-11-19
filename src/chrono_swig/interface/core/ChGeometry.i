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
#include "chrono/geometry/ChLineBspline.h"
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
using namespace geometry;

%}

%shared_ptr(chrono::geometry::ChGeometry)
%shared_ptr(chrono::geometry::ChLine)
%shared_ptr(chrono::geometry::ChVolume)
%shared_ptr(chrono::geometry::ChSurface)
%shared_ptr(chrono::geometry::ChBox)
%shared_ptr(chrono::geometry::ChSphere)
%shared_ptr(chrono::geometry::ChCylinder)
%shared_ptr(chrono::geometry::ChCapsule)
%shared_ptr(chrono::geometry::ChCone)
%shared_ptr(chrono::geometry::ChEllipsoid)
%shared_ptr(chrono::geometry::ChLineArc)
%shared_ptr(chrono::geometry::ChLineSegment)
%shared_ptr(chrono::geometry::ChLineNurbs)
%shared_ptr(chrono::geometry::ChLinePath)
%shared_ptr(chrono::geometry::ChLinePoly)
%shared_ptr(chrono::geometry::ChLineBezier)
%shared_ptr(chrono::geometry::ChLineBspline)
%shared_ptr(chrono::geometry::ChLineCam)
%shared_ptr(chrono::geometry::ChTriangle)
%shared_ptr(chrono::geometry::ChTriangleMesh)
%shared_ptr(chrono::geometry::ChTriangleMeshSoup)
%shared_ptr(chrono::geometry::ChTriangleMeshConnected)
%shared_ptr(chrono::geometry::ChSurface)
%shared_ptr(chrono::geometry::ChSurfaceNurbs)
%shared_ptr(chrono::geometry::ChRoundedCylinder)
%shared_ptr(chrono::geometry::ChRoundedBox)
 
//%feature("director") chrono::geometry::ChGeometry;
//%ignore chrono::geometry::ChGeometry::Clone;

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
%include "../../../chrono/geometry/ChLineBspline.h"
%include "../../../chrono/geometry/ChLineCam.h"
%include "../../../chrono/geometry/ChTriangle.h"
%include "../../../chrono/geometry/ChTriangleMesh.h"
%include "../../../chrono/geometry/ChTriangleMeshSoup.h"
%include "../../../chrono/geometry/ChTriangleMeshConnected.h"
%include "../../../chrono/geometry/ChSurface.h"
%include "../../../chrono/geometry/ChSurfaceNurbs.h"
%include "../../../chrono/geometry/ChRoundedCylinder.h"
%include "../../../chrono/geometry/ChRoundedBox.h"

