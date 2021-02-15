%{
#include <cstddef>
/* Includes the header in the wrapper code */
#include "chrono/core/ChTensors.h"

%}

// The underlaying XX() method is ignored, then we rename our custom getXX as XX. 
// This way XX is substituted. We do the same for all the tensor elements
%ignore chrono::ChVoightTensor::XX;
%rename(XX) chrono::ChVoightTensor::getXX;
%ignore chrono::ChVoightTensor::YY;
%rename(YY) chrono::ChVoightTensor::getYY;
%ignore chrono::ChVoightTensor::ZZ;
%rename(ZZ) chrono::ChVoightTensor::getZZ;
%ignore chrono::ChVoightTensor::XY;
%rename(XY) chrono::ChVoightTensor::getXY;
%ignore chrono::ChVoightTensor::XZ;
%rename(XZ) chrono::ChVoightTensor::getXZ;
%ignore chrono::ChVoightTensor::YZ;
%rename(YZ) chrono::ChVoightTensor::getYZ;

/* Parse the header file to generate wrappers */
%include "../../chrono/core/ChTensors.h"

// Tensors are templated by type
%template(ChVoightTensorD) chrono::ChVoightTensor<double>;
%template(ChStressTensorD) chrono::ChStressTensor<double>;
%template(ChStrainTensorD) chrono::ChStrainTensor<double>;


%extend chrono::ChVoightTensor<double>{
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


