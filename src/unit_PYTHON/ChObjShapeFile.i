%{

/* Includes the header in the wrapper code */
#include "assets/ChObjShapeFile.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../assets/ChObjShapeFile.h"    

// Define also the shared pointer chrono::ChShared<ChObjShapeFile> 
// (renamed as 'ChObjShapeFileShared' in python)

%DefChSharedPtr(ChObjShapeFileShared, ChObjShapeFile)

