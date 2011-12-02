%{

/* Includes the header in the wrapper code */
#include "core/ChMatrix.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi  

/* Parse the header file to generate wrappers */
%include "../core/ChMatrix.h"    

//%feature("notabstract") chrono::ChMatrix;

%ignore chrono::Chrono_to_Marray(ChMatrix33<>& ma, double marr[3][3]);
%ignore chrono::Chrono_from_Marray(ChMatrix33<>& ma, double marr[3][3]);


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
			
			%template(CopyFromMatrix) CopyFromMatrix<double>;
			%template(CopyFromMatrixT) CopyFromMatrixT<double>;
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