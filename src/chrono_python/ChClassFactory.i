%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChClassFactory.h"

using namespace chrono;

%}

// Trick to disable a macro that stops SWIG
#define CH_CREATE_MEMBER_DETECTOR(ArchiveINconstructor)

%ignore chrono::ChClassRegistrationBase;

/* Parse the header file to generate wrappers */
 %include "../chrono/core/ChClassFactory.h"    


