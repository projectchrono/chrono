%{

/* Includes the header in the wrapper code */
#include "chrono/geometry/ChGeometry.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/geometry/ChSphere.h"
#include "chrono/geometry/ChCylinder.h"
#include "chrono/geometry/ChTriangle.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

using namespace chrono;
using namespace geometry;

%}
 
/* Parse the header file(s) to generate wrappers */
%include "../chrono/geometry/ChGeometry.h"
%include "../chrono/geometry/ChBox.h"
%include "../chrono/geometry/ChSphere.h"
%include "../chrono/geometry/ChCylinder.h"
%include "../chrono/geometry/ChTriangle.h"
%include "../chrono/geometry/ChTriangleMesh.h"
%include "../chrono/geometry/ChTriangleMeshSoup.h"
%include "../chrono/geometry/ChTriangleMeshConnected.h"


