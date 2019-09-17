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


%include "../chrono/core/ChMatrix33.h"


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
//
// ADD PYTHON CODE
//

%pythoncode %{

def __matr33_setitem(self,index,vals):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    self.setitem(index[0],index[1],vals)

def __matr33_getitem(self,index):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    return self.getitem(index[0],index[1])

setattr(ChMatrix33D, "__getitem__", __matr33_getitem)
setattr(ChMatrix33D, "__setitem__", __matr33_setitem)

def SetMatr(self, l_in):
    if len(l_in)>3 or len(l_in[0])>3:
	    raise NameError('Wrong Input List. Must be 3x3')
    for i, li in enumerate(l_in):
        for j, lij in enumerate(li):
          self[i,j] = lij 

def GetMatr(self, ):
    l_out = []
    for i in range(3):
        irow = []
        for j in range(3):
          irow.append(self[i,j])
        l_out.append(irow)
    return l_out

setattr(ChMatrix33D, "SetMatr", SetMatr)
setattr(ChMatrix33D, "GetMatr", GetMatr)

%}