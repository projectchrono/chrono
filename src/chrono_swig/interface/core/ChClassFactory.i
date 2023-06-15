%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChClassFactory.h"

using namespace chrono;

%}

// Trick to disable a macro that stops SWIG
#define CH_CREATE_MEMBER_DETECTOR(ArchiveInConstructor)
#define CH_CREATE_MEMBER_DETECTOR(ArchiveOutConstructor)
#define CH_CREATE_MEMBER_DETECTOR(ArchiveOut)
#define CH_CREATE_MEMBER_DETECTOR(ArchiveIn)
#define CH_CREATE_MEMBER_DETECTOR(ArchiveContainerName)

%ignore CH_UPCASTING;
%ignore CH_UPCASTING_SANITIZED;
%ignore chrono::ChClassRegistrationBase;
%ignore chrono::ChCastingMap;

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChClassFactory.h"


