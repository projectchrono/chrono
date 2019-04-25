%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChVectorDynamic.h"

using namespace chrono;

%}
 
 %import"ChMatrix.i"

/* Parse the header file to generate wrappers */
%include "../chrono/core/ChVectorDynamic.h"


%template(ChVectorDynamicD) chrono::ChVectorDynamic<double>;


%extend chrono::ChVectorDynamic<double>{
		public:
					// these functions are also argument-templated, so we need to specify the types
					// ***SWIG template mechanism does not work here for operator() ***
			//%template(operator+) operator+<double>;
			//%template(operator-) operator-<double>;
			//%template(operator*) operator*<double>;
			ChVectorDynamic<double> operator+(const ChMatrix<double>& matbis) 
						{ return $self->operator+(matbis);};
			ChVectorDynamic<double> operator-(const ChMatrix<double>& matbis) 
						{ return $self->operator-(matbis);};
		};


