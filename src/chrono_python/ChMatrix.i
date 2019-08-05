%{

/* Includes the header in the wrapper code */
#include <math.h>  
#include "chrono/core/ChMatrix.h"
#include <Eigen/Core>
#include <Eigen/Dense>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Macros.h"

using namespace chrono;

%}

%typemap(in) (double* p, int len) %{
    if(!PyLong_Check($input))
        SWIG_exception(SWIG_TypeError, "expected integer");
    $2 = PyLong_AsUnsignedLong($input);
    $1 = (double*)malloc($2);
%}

%typemap(freearg) (double* p, int len) %{
    free($1);
%}

%typemap(argout) (double* p, int len) {
    PyObject* list = PyList_New($2);
    int i;
    for(i = 0; i < $2; ++i)
        PyList_SET_ITEM(list, i, PyFloat_FromDouble($1[i]));
    $result = SWIG_Python_AppendOutput($result, list);
}

template <typename Real = double>
class chrono::ChMatrixDynamic : public Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> {
	public:
		ChMatrixDynamic() : Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>() {}
		ChMatrixDynamic(int r, int c) {
			Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>();
			this->resize(r,c);
			}
		};

%template(ChMatrixDynamicD) chrono::ChMatrixDynamic<double>;

template <typename T = double>
class chrono::ChVectorDynamic : public Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor> {
	public:
		ChVectorDynamic() : Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor>() {}
		ChVectorDynamic(int r) {
			Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor>();
			this->resize(r);
			}
		};

%template(ChVectorDynamicD) chrono::ChVectorDynamic<double>;

%extend chrono::ChVectorDynamic<double>{
		public:
			double __getitem__(int i) {
				return (*$self)(i,1);
				}
			void __setitem__(int i, double v) {
				(*$self)(i, 1) = v;
				}

			const int Size() {
				const int r = $self->rows();
				return r;
				}
			void GetVectorData(double* p, int len) {
				for (int i = 0; i < len; i++){
					p[i] =  (double)(*$self)(i, 1);
						}
				}
			void SetVectorData(double * p, int len){
				for (int i = 0; i < len; i++){
					(double)(*$self)(i, 1) = p[i];
						}
				}
		};

%extend chrono::ChMatrixDynamic<double>{
		public:
					// these functions are also argument-templated, so we need to specify the types
					// ***SWIG template mechanism does not work here for operator() ***
			//%template(operator+) operator+<double>;
			//%template(operator-) operator-<double>;
			//%template(operator*) operator*<double>;
			/*ChMatrixDynamic<double> operator+(const ChMatrix<double>& matbis) 
						{ return $self->operator+(matbis);};
			ChMatrixDynamic<double> operator-(const ChMatrix<double>& matbis) 
						{ return $self->operator-(matbis);};
			ChMatrixDynamic<double> operator*(const ChMatrix<double>& matbis) 
						{ return $self->operator*(matbis);};*/

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

			void GetMatrixData(double* p, int len) {
				int r = $self->rows();
				int c = $self->cols();
				//double matr[len];
				//double* matr = $self->data();
				for (int i = 0; i < len; i++){
					int ri = floor (i/c);
					int ci = i - c*ri;
					p[i] =  (double)(*$self)(ri, ci);
					
				}
			}
		};

//
// ADD PYTHON CODE
//

%pythoncode %{

def reshape(seq, rows, cols):
    return [list(u) for u in zip(*[iter(seq)] * cols)]

def GetMatr(self):
    cls = self.GetColumns()
    rws = self.GetRows()
    len = cls*rws
    lst = self.GetMatrixData(len)
    rs_list = reshape(lst, rws, cls)
    return rs_list

setattr(ChMatrixDynamicD, "GetMatr", GetMatr)

def GetVect(self):
    len = self.Size()
    lst = self.GetVectorData(len)
    rs_list = reshape(lst, len, 1)
    return rs_list

setattr(ChVectorDynamicD, "GetVect", GetVect)

%}
%ignore chrono::ChMatrixDynamic;
%include "../chrono/core/ChMatrix.h"