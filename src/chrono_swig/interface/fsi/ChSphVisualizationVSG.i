%{

/* Includes the header in the wrapper code */
#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"

using namespace chrono::fsi::sph;
using namespace chrono::vsg3d;

%}

%shared_ptr(chrono::fsi::sph::ChSphVisualizationVSG::MarkerVisibilityCallback)
%shared_ptr(chrono::fsi::sph::MarkerPlanesVisibilityCallback)
%shared_ptr(chrono::fsi::sph::MarkerPlanesVisibilityCallback::Plane)

%rename(MarkerPlanesVisibilityCallback_Plane) chrono::fsi::sph::MarkerPlanesVisibilityCallback::Plane;
%template(vector_MarkerPlanesVisibilityCallback_Plane) std::vector<chrono::fsi::sph::MarkerPlanesVisibilityCallback::Plane>;

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"    
