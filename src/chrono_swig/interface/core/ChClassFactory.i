%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChClassFactory.h"

using namespace chrono;

%}

// Trick to disable a macro that stops SWIG
#define CH_CREATE_MEMBER_DETECTOR(ArchiveINconstructor)
#define CH_CREATE_MEMBER_DETECTOR(ArchiveOUTconstructor)
#define CH_CREATE_MEMBER_DETECTOR(ArchiveOUT)
#define CH_CREATE_MEMBER_DETECTOR(ArchiveIN)
#define CH_CREATE_MEMBER_DETECTOR(ArchiveContainerName)

%ignore CH_CASTING_PARENT;
%ignore CH_CASTING_PARENT_SANITIZED;
%ignore chrono::ChClassRegistrationBase;
%ignore chrono::ChCastingMap;

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChClassFactory.h"


