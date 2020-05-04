%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"
#include <Eigen/Core>
#include <Eigen/Dense>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Macros.h"

using namespace chrono;

%}

%import "ChQuaternion.i"


%include "../../chrono/core/ChMatrix33.h"


%template(ChMatrix33D) chrono::ChMatrix33<double>; 


%extend chrono::ChMatrix33<double>{
		public:
			double getitem(int i, int j) {
				return (*$self)(i, j);
				}

			void setitem(int i, int j, double v) {
				(*$self)(i, j) = v;
				}
			const int GetRows() {
				const int r = $self->rows();
				return r;
				}
			const int GetColumns() {
				const int c = $self->cols();
				return c;
				}			

		/*
					// these functions are also argument-templated, so we need to specify the types
					// ***SWIG template mechanism does not work here for operator() ***
			//%template(operator+) operator+<double>;
			//%template(operator-) operator-<double>;
			//%template(operator*) operator*<double>;
			ChMatrix33<double> operator+(const ChMatrix<double>& matbis) 
						{ return $self->operator+(matbis);};
			ChMatrix33<double> operator-(const ChMatrix<double>& matbis) 
						{ return $self->operator-(matbis);};
			ChMatrix33<double> operator*(const ChMatrix<double>& matbis) 
						{ return $self->operator*(matbis);};

			ChMatrix33<double>(const ChQuaternion<double>& mq){ 
						ChMatrix33<double>* newX = new ChMatrix33<double>();
						newX->Set_A_quaternion(mq);
						return newX;};
			
			//%template(ChMatrix33) ChMatrix33<double>;
			%template(Matr_x_Vect) Matr_x_Vect<double>;
			%template(MatrT_x_Vect) MatrT_x_Vect<double>;
			%template(FastInvert) FastInvert<double>;
			%template(Set_A_quaternion) Set_A_quaternion<double>;
			%template(Set_X_matrix) Set_X_matrix<double>;
			%template(Set_A_axis) Set_A_axis<double>;
			%template(Set_A_Eulero) Set_A_Eulero<double>;
			%template(Set_A_Cardano) Set_A_Cardano<double>;
			%template(Set_A_Hpb) Set_A_Hpb<double>;
			%template(Set_A_Rxyz) Set_A_Rxyz<double>;
			%template(Set_A_Rodriguez) Set_A_Rodriguez<double>;
			*/
		};
