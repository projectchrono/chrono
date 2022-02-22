%{

/* Includes the header in the wrapper code */
#include "chrono/assets/ChVisualShape.h"

using namespace chrono;

%}
%template(material_list) std::vector<std::shared_ptr<chrono::ChVisualMaterial>>;

%shared_ptr(chrono::ChVisualShape)

/* Parse the header file to generate wrappers */
%include "../../../chrono/assets/ChVisualShape.h"    



