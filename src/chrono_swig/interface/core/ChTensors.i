%{
#include <cstddef>
#include "chrono/core/ChTensors.h"
%}

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// Hack to avoid problems with .x() .y() .z() that work with references. 
// This is not straightforward in SWIG. So access them as .x .y .z attributes 
// using the following workaround (NOTE! must happen before calling %include)
/*%include <attribute.i>
%attributeref(chrono::ChVector3<double>, double, x);
%attributeref(chrono::ChVector3<double>, double, y);
%attributeref(chrono::ChVector3<double>, double, z);
%attributeref(chrono::ChVector3<float>, float, x);
%attributeref(chrono::ChVector3<float>, float, y);
%attributeref(chrono::ChVector3<float>, float, z);*/

#endif             // --------------------------------------------------------------------- CSHARP

// The underlying XX() method is ignored, then we rename our custom getXX as XX. 
// This way XX is substituted. We do the same for all the tensor elements
%ignore chrono::ChVoigtTensor::XX;
%rename(XX) chrono::ChVoigtTensor::getXX;
%ignore chrono::ChVoigtTensor::YY;
%rename(YY) chrono::ChVoigtTensor::getYY;
%ignore chrono::ChVoigtTensor::ZZ;
%rename(ZZ) chrono::ChVoigtTensor::getZZ;
%ignore chrono::ChVoigtTensor::XY;
%rename(XY) chrono::ChVoigtTensor::getXY;
%ignore chrono::ChVoigtTensor::XZ;
%rename(XZ) chrono::ChVoigtTensor::getXZ;
%ignore chrono::ChVoigtTensor::YZ;
%rename(YZ) chrono::ChVoigtTensor::getYZ;

// Parse the header file to generate wrappers
%include "../../../chrono/core/ChTensors.h"

// Tensors are templated by type
%template(ChStressTensorD) chrono::ChStressTensor<double>;
%template(ChStrainEngTensorD) chrono::ChStrainTensor<double>;

%extend chrono::ChVoigtTensor<double>{
		public:

			double const getXX() {
				double value = (*$self)(0);
				return value;
				};
			double const getYY() {
				double value = (*$self)(1);
				return value;
				};
			double const getZZ() {
				double value = (*$self)(2);
				return value;
				};
			double const getXY() {
				double value = (*$self)(3);
				return value;
				};
			double const getXZ() {
				double value = (*$self)(4);
				return value;
				};
			double const getYZ() {
				double value = (*$self)(5);
				return value;
				};
		};


