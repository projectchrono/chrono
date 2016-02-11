%{

/* Includes the header in the wrapper code */
#include "physics/ChIndexedParticles.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChIndexedParticles)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChIndexedParticles.h"    

