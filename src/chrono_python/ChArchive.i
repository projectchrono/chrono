%{

/* Includes the header in the wrapper code */
#include "chrono/serialization/ChArchive.h"

using namespace chrono;

%}

// Trick to disable a macro that stops SWIG
#define CH_CREATE_MEMBER_DETECTOR(ArchiveOUTconstructor) 
#define CH_CREATE_MEMBER_DETECTOR(ArchiveOUT) 
#define CH_CREATE_MEMBER_DETECTOR(ArchiveIN) 



/* Parse the header file to generate wrappers */
 %include "../chrono/serialization/ChArchive.h"    


