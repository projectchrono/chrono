%{

/* Includes the header in the wrapper code */
#include "geometry/ChCBox.h"
#include "geometry/ChCSphere.h"
#include "geometry/ChCCylinder.h"
#include "geometry/ChCTriangle.h"
#include "geometry/ChCTriangleMesh.h"
#include "geometry/ChCTriangleMeshSoup.h"
#include "geometry/ChCTriangleMeshConnected.h"

using namespace chrono;
using namespace geometry;

%}
 
/* Parse the header file(s) to generate wrappers */
%include "../chrono/geometry/ChCBox.h"
%include "../chrono/geometry/ChCSphere.h"
#include "../chrono/geometry/ChCCylinder.h"
%include "../chrono/geometry/ChCTriangle.h"
%include "../chrono/geometry/ChCTriangleMesh.h"
%include "../chrono/geometry/ChCTriangleMeshSoup.h"
%include "../chrono/geometry/ChCTriangleMeshConnected.h"


