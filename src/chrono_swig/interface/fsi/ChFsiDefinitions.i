%{

/* Includes the header in the wrapper code */
#include "chrono_fsi/ChFsiDefinitions.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::fsi;

%}

%shared_ptr(chrono::fsi::FsiBodyState)
%shared_ptr(chrono::fsi::FsiBodyForce)
%shared_ptr(chrono::fsi::FsiMeshState)
%shared_ptr(chrono::fsi::FsiMeshForce)
%shared_ptr(chrono::fsi::FsiBody)
%shared_ptr(chrono::fsi::FsiMesh1D)
%shared_ptr(chrono::fsi::FsiMesh2D)

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/ChFsiDefinitions.h"    
