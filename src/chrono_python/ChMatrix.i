%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChMatrixNM.h"
#include "chrono/core/ChMatrix33.h"

using namespace chrono;

%}
 

/* Parse the header file to generate wrappers */
%include "../chrono/core/ChMatrix.h"    
%include "../chrono/core/ChMatrixDynamic.h"
%include "../chrono/core/ChMatrixNM.h"
%include "../chrono/core/ChMatrix33.h"

//%feature("notabstract") chrono::ChMatrix;

%template(ChMatrixD) chrono::ChMatrix<double>;
%template(ChMatrixNMD) chrono::ChMatrixNM<double>; 
%template(ChMatrix33D) chrono::ChMatrix33<double>; 
%template(ChMatrixDynamicD) chrono::ChMatrixDynamic<double>;

%extend chrono::ChMatrix<double>{
		public:
					// Add function to support python 'print(...)'
			char *__str__() 
						{
							static char temp[2000];
							sprintf(temp, "Matrix, size= %d x %d \n", $self->GetRows(), $self->GetColumns());
							for (int row=0; row < ChMin(6,$self->GetRows()); row++)
							{
								for (int col=0; col < ChMin(6,$self->GetColumns()); col++)
								{
									sprintf(temp, "%s%g  ", temp, $self->GetElement(row,col));
								}
								if ($self->GetColumns() >6) sprintf(temp,"%s ...", temp);
								sprintf(temp,"%s\n", temp);
							}
							if ($self->GetRows() >6) sprintf(temp,"%s ...\n", temp);
							return &temp[0];
						}
					// these functions are also argument-templated, so we need to specify the types
			%template(CopyFromMatrix) CopyFromMatrix<double>;
			%template(CopyFromMatrixT) CopyFromMatrixT<double>;
			%template(MatrAdd) MatrAdd<double,double>;
			%template(MatrSub) MatrSub<double,double>;
			%template(MatrInc) MatrInc<double>;
			%template(MatrDec) MatrDec<double>;
			%template(MatrMultiply) MatrMultiply<double,double>;
			%template(MatrTMultiply) MatrTMultiply<double,double>;
			%template(MatrMultiplyT) MatrMultiplyT<double,double>;
			//%template(MatrDot) MatrDot<double,double>; // static fn, does not work here..
			%template(Matr34_x_Quat) Matr34_x_Quat<double>;
			%template(Matr34T_x_Vect) Matr34T_x_Vect<double>;
			%template(Matr44_x_Quat) Matr44_x_Quat<double>;
			%template(PasteMatrix) PasteMatrix<double>;
			%template(PasteSumMatrix) PasteSumMatrix<double>;
			%template(PasteTranspMatrix) PasteTranspMatrix<double>;
			%template(PasteSumTranspMatrix) PasteSumTranspMatrix<double>;
			%template(PasteClippedMatrix) PasteClippedMatrix<double>;
			%template(PasteSumClippedMatrix) PasteSumClippedMatrix<double>;
			%template(PasteVector) PasteVector<double>;
			%template(PasteSumVector) PasteSumVector<double>;
			%template(PasteSubVector) PasteSubVector<double>;
			%template(PasteQuaternion) PasteQuaternion<double>;
			%template(PasteSumQuaternion) PasteSumQuaternion<double>;
			%template(PasteCoordsys) PasteCoordsys<double>;
			%template(Set_Xq_matrix) Set_Xq_matrix<double>;
		};

%extend chrono::ChMatrixDynamic<double>{
		public:
					// these functions are also argument-templated, so we need to specify the types
					// ***SWIG template mechanism does not work here for operator() ***
			//%template(operator+) operator+<double>;
			//%template(operator-) operator-<double>;
			//%template(operator*) operator*<double>;
			ChMatrixDynamic<double> operator+(const ChMatrix<double>& matbis) 
						{ return $self->operator+(matbis);};
			ChMatrixDynamic<double> operator-(const ChMatrix<double>& matbis) 
						{ return $self->operator-(matbis);};
			ChMatrixDynamic<double> operator*(const ChMatrix<double>& matbis) 
						{ return $self->operator*(matbis);};

		};

%extend chrono::ChMatrix33<double>{
		public:
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
		};

//
// ADD PYTHON CODE
//

%pythoncode %{

def __matr_setitem(self,index,vals):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    self.SetElement(index[0],index[1],vals)

def __matr_getitem(self,index):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    return self.GetElement(index[0],index[1])

setattr(ChMatrixD, "__getitem__", __matr_getitem)
setattr(ChMatrixD, "__setitem__", __matr_setitem)

%}