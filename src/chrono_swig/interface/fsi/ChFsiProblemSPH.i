%{

/* Includes the header in the wrapper code */
#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::fsi;

%}

// Those classes are used in vehicle, so the shared_ptr must be declared here to make it shared with vehicle module
%shared_ptr(chrono::fsi::sph::ChFsiProblemSPH)
%shared_ptr(chrono::fsi::sph::ChFsiProblemCartesian)
%shared_ptr(chrono::fsi::sph::ChFsiProblemWavetank)
%shared_ptr(chrono::fsi::sph::ChFsiProblemCylindrical)

%shared_ptr(chrono::fsi::sph::ChFsiProblemSPH::ParticlePropertiesCallback)
%shared_ptr(chrono::fsi::sph::DepthPressurePropertiesCallback)

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/sph/ChFsiProblemSPH.h"    


%DefSharedPtrDynamicCast(chrono::fsi::sph, ChFsiProblemCartesian, ChFsiProblemSPH)
