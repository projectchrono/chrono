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

%}
 
/* Parse the header file(s) to generate wrappers */
%include "../geometry/ChCBox.h"
%include "../geometry/ChCSphere.h"
#include "../geometry/ChCCylinder.h"
%include "../geometry/ChCTriangle.h"
%include "../geometry/ChCTriangleMesh.h"
%include "../geometry/ChCTriangleMeshSoup.h"
%include "../geometry/ChCTriangleMeshConnected.h"


