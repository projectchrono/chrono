%{
#include <cstddef>
/* Includes the header in the wrapper code */
#include "chrono/core/ChVector.h"
#include <Eigen/Core>
%}

%import "ChMatrix.i"

// Hack to avoid problems with .x() .y() .z() that work with references. 
// This is not straightforward in SWIG. So access them as .x .y .z attributes 
// using the following workaround (NOTE! must happen before calling %include)
%include <attribute.i>
%attributeref(chrono::ChVector<double>, double, x);
%attributeref(chrono::ChVector<double>, double, y);
%attributeref(chrono::ChVector<double>, double, z);
%attributeref(chrono::ChVector<float>, float, x);
%attributeref(chrono::ChVector<float>, float, y);
%attributeref(chrono::ChVector<float>, float, z);
%attributeref(chrono::ChVector<int>, int, x);
%attributeref(chrono::ChVector<int>, int, y);
%attributeref(chrono::ChVector<int>, int, z);

%ignore chrono::ChVector::eigen;

/* Parse the header file to generate wrappers */
%include "../../chrono/core/ChVector.h"  


%template(ChVectorD) chrono::ChVector<double>; 
%template(ChVectorF) chrono::ChVector<float>; 
%template(ChVectorI) chrono::ChVector<int>;

// This is needed because a std::vector<ChVector<double> or std::vector<ChVector<int>>
// might be used  somewhere, and we want to use them in C#.
%template(vector_ChVectorD) std::vector< chrono::ChVector<double> >;
%template(vector_ChVectorF) std::vector< chrono::ChVector<float> >;
%template(vector_ChVectorI) std::vector< chrono::ChVector<int> >;
