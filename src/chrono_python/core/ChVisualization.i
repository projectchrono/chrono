%{

/* Includes the header in the wrapper code */
#include "chrono/assets/ChVisualization.h"

using namespace chrono;

%}
%template(material_list) std::vector<std::shared_ptr<chrono::ChVisualMaterial>>;

%shared_ptr(chrono::ChVisualization)

/* Parse the header file to generate wrappers */
%include "../../chrono/assets/ChVisualization.h"    



