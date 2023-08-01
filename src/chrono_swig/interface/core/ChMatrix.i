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

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

/*
typemaps redefine inputs and outputs of the methods that have the same arguments NAME and TYPE such as  (double* p, int len)

typemap in :
according to the Python input of the function ($input) feeds the arguments ($1, $2...) ti the C++ function

freearg:
delete / free whatever is new / malloc in typemap(in)

typemap argout
defines the Python function return ($result) from the C++ args ($1, $2...)
*/

%typemap(in) (double* p, int len) %{
    if(!PyLong_Check($input))
        SWIG_exception(SWIG_TypeError, "expected integer");
    $2 = PyLong_AsUnsignedLong($input);
    $1 = new double[$2];
%}

%typemap(freearg) (double* p, int len) %{
    delete($1);
%}

%typemap(argout) (double* p, int len) {
    PyObject* list = PyList_New($2);
    int i;
    for(i = 0; i < $2; ++i)
        PyList_SET_ITEM(list, i, PyFloat_FromDouble($1[i]));
    $result = SWIG_Python_AppendOutput($result, list);
}

%typemap(in) (int numel, double* q) %{
    if (PyList_Check($input)) {
        $1 = PyList_Size($input);
        $2 = new double[$1];
        for (int i = 0; i < $1; i++) {
            PyObject *o = PyList_GetItem($input, i);
            double tmp = PyFloat_AsDouble(o);
            if(PyErr_Occurred())
                SWIG_fail;
            $2[i] = PyFloat_AsDouble(o);
        }
    } else {
        PyErr_SetString(PyExc_TypeError, "not a list");
        return NULL;
    }
%}


%typemap(freearg) (int numel, double* q) %{
    delete($2);
%}


%typemap(in) (int* p, int len) %{
    if(!PyLong_Check($input))
        SWIG_exception(SWIG_TypeError, "expected integer");
    $2 = PyLong_AsUnsignedLong($input);
    $1 = new int[$2];
%}

%typemap(freearg) (int* p, int len) %{
    delete($1);
%}

%typemap(argout) (int* p, int len) {
    PyObject* list = PyList_New($2);
    int i;
    for(i = 0; i < $2; ++i)
        PyList_SET_ITEM(list, i, PyLong_FromLong($1[i]));
    $result = SWIG_Python_AppendOutput($result, list);
}


%typemap(in) (int numel, int* q) %{
    if (PyList_Check($input)) {
        $1 = PyList_Size($input);
        $2 = new int[$1];
        for (int i = 0; i < $1; i++) {
            PyObject *o = PyList_GetItem($input, i);
            double tmp = PyLong_AsLong(o);
            if(PyErr_Occurred())
                SWIG_fail;
            $2[i] = PyLong_AsLong(o);
        }
    } else {
        PyErr_SetString(PyExc_TypeError, "not a list");
        return NULL;
    }
%}

%typemap(freearg) (int numel, int* q) %{
    delete($2);
%}


%typemap(in) (double *mat, int ros, int col) %{
    if (PyList_Check($input)) {
        $2 = PyList_Size($input);
        $3 = PyList_Size(PyList_GetItem($input, 0));
		$1 = new double[$2 * $3];
        for (int i = 0; i < $2; i++) {
            for (int j = 0; j < $3; j++) {
                PyObject *o = PyList_GetItem(PyList_GetItem($input, i), j);
				// TODO: check what's this line for
                double tmp = PyFloat_AsDouble(o);
                if(PyErr_Occurred())
                    SWIG_fail;
                $1[i * $3 + j] = PyFloat_AsDouble(o);
            }
        }
    } else {
        PyErr_SetString(PyExc_TypeError, "not a list");
        return NULL;
    }
%}

%typemap(freearg) (double *mat, int ros, int col) %{
    delete($1);
%}

#endif             // --------------------------------------------------------------------- PYTHON

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
			void SetVect(int numel, double* q){
				($self)->resize(numel);
				for (int i = 0; i < numel; i++){
					(*$self)(i, 1) = q[i];
						}
				}
		};
		
%template(ChVectorDynamicI) chrono::ChVectorDynamic<int>;

%extend chrono::ChVectorDynamic<int>{
		public:
			int __getitem__(int i) {
				return (*$self)(i,1);
				}
			void __setitem__(int i, int v) {
				(*$self)(i, 1) = v;
				}

			const int Size() {
				const int r = $self->rows();
				return r;
				}
			void GetVectorData(int* p, int len) {
				for (int i = 0; i < len; i++){
					p[i] =  (int)(*$self)(i, 1);
						}
				}
			void SetVect(int numel, int* q){
				($self)->resize(numel);
				for (int i = 0; i < numel; i++){
					(*$self)(i, 1) = q[i];
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

			void SetMatr(double *mat, int ros, int col) {
				($self)->resize(ros, col);
				for (int i = 0; i < ros; i++){
					for (int j = 0; j < col; j++){
						(*$self)(i, j) = mat[i*col + j];
					}
				}

			}
		};

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

// ADD PYTHON CODE

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

def __matr_setitem(self,index,vals):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    self.setitem(index[0],index[1],vals)

def __matr_getitem(self,index):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    return self.getitem(index[0],index[1])

setattr(ChMatrixDynamicD, "__getitem__", __matr_getitem)
setattr(ChMatrixDynamicD, "__setitem__", __matr_setitem)

%}

#endif             // --------------------------------------------------------------------- PYTHON

%ignore chrono::ChMatrixDynamic;
%include "../../../chrono/core/ChMatrix.h"
