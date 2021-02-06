%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChQuaternion.h"
#include <Eigen/Core>
%}
 
 %import "ChMatrix.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi  


// Hack to avoid problems with .e0() .e1() .e2() .e3 that work with references. 
// This is not straightforward in SWIG. So access them as .e0 .e1 .e2 .e3 attributes 
// using the following workaround (NOTE! must happen before calling %include)
%include <attribute.i>
%attributeref(chrono::ChQuaternion<double>, double, e0);
%attributeref(chrono::ChQuaternion<double>, double, e1);
%attributeref(chrono::ChQuaternion<double>, double, e2);
%attributeref(chrono::ChQuaternion<double>, double, e3);
%attributeref(chrono::ChQuaternion<float>, float, e0);
%attributeref(chrono::ChQuaternion<float>, float, e1);
%attributeref(chrono::ChQuaternion<float>, float, e2);
%attributeref(chrono::ChQuaternion<float>, float, e3);

%import "ChMatrix.i"

%ignore chrono::ChQuaternion::eigen;

/* Parse the header file to generate wrappers */
%include "../../chrono/core/ChQuaternion.h"  


%template(ChQuaternionD) chrono::ChQuaternion<double>; 
%template(ChQuaternionF) chrono::ChQuaternion<float>; 
