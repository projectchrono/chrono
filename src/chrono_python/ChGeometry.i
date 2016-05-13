%{

/* Includes the header in the wrapper code */
#include "geometry/ChBox.h"
#include "geometry/ChSphere.h"
#include "geometry/ChCylinder.h"
#include "geometry/ChTriangle.h"
#include "geometry/ChTriangleMesh.h"
#include "geometry/ChTriangleMeshSoup.h"
#include "geometry/ChTriangleMeshConnected.h"

using namespace chrono;
using namespace geometry;

%}
 
/* Parse the header file(s) to generate wrappers */
%include "../chrono/geometry/ChBox.h"
%include "../chrono/geometry/ChSphere.h"
#include "../chrono/geometry/ChCylinder.h"
%include "../chrono/geometry/ChTriangle.h"
%include "../chrono/geometry/ChTriangleMesh.h"
%include "../chrono/geometry/ChTriangleMeshSoup.h"
%include "../chrono/geometry/ChTriangleMeshConnected.h"


