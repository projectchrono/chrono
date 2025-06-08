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

// version handling
#if SWIG_VERSION >= 0x040300
  // SWIG 4.3.0+ use a new three argument form with $isvoid flag
  %typemap(argout) (double* p, int len) {
    PyObject* list = PyList_New($2);
    for (int i = 0; i < $2; ++i) {
        PyList_SET_ITEM(list, i, PyFloat_FromDouble($1[i]));
    }
    $result = SWIG_Python_AppendOutput($result, list, $isvoid);
  }
#else
  // SWIG < 4.3.0 fall back to the older AppendOutput form
  %typemap(argout) (double* p, int len) {
    PyObject* list = PyList_New($2);
    for (int i = 0; i < $2; ++i) {
        PyList_SET_ITEM(list, i, PyFloat_FromDouble($1[i]));
    }
    $result = SWIG_Python_AppendOutput($result, list);
  }
#endif

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

#if SWIG_VERSION >= 0x040300
  // SWIG 4.3.0+ use a new three argument form with $isvoid flag
  %typemap(argout) (int* p, int len) {
    PyObject* list = PyList_New($2);
    for (int i = 0; i < $2; ++i) {
      PyList_SET_ITEM(list, i, PyLong_FromLong($1[i]));
    }
    $result = SWIG_Python_AppendOutput($result, list, $isvoid);
  }
#else
  // SWIG < 4.3.0 fall back to the older AppendOutput form
  %typemap(argout) (int* p, int len) {
    PyObject* list = PyList_New($2);
    for (int i = 0; i < $2; ++i) {
      PyList_SET_ITEM(list, i, PyLong_FromLong($1[i]));
    }
    $result = SWIG_Python_AppendOutput($result, list);
  }
#endif



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


%typemap(in) (double* mat, int rows, int cols) %{
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

%typemap(freearg) (double* mat, int rows, int cols) %{
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

%template(ChMatrixDynamicd) chrono::ChMatrixDynamic<double>;

template <typename T = double, int N, int M>
class chrono::ChMatrixNM : public Eigen::Matrix<T, M, N, Eigen::RowMajor> {
	public:
		ChMatrixNM() : Eigen::Matrix<T, M, N, Eigen::RowMajor>() {}

};

template <typename T = double>
class chrono::ChVectorDynamic : public Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor> {
    public:
        ChVectorDynamic() : Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor>() {}
        ChVectorDynamic(int r) {
            Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor>();
            this->resize(r);
        }
};

%template(ChVectorDynamicd) chrono::ChVectorDynamic<double>;

%extend chrono::ChVectorDynamic<double> {
    public:
        double GetItem(int i) {
            return (*$self)(i);
        }

        void SetItem(int i, double v) {
            (*$self)(i) = v;
        }

        const int Size() {
            const int r = $self->rows();
            return r;
        }

        void GetVectorData(double* p, int len) {
            for (int i = 0; i < len; i++){
                p[i] =  (double)(*$self)(i);
            }
        }

        void SetVect(int numel, double* q){
            ($self)->resize(numel);
            for (int i = 0; i < numel; i++){
                (*$self)(i) = q[i];
            }
        }

        void SetZero() {
            for (int i = 0; i < $self->rows(); i++)
                (*$self)(i) = 0;
        }
};

%template(ChVectorDynamicI) chrono::ChVectorDynamic<int>;

%extend chrono::ChVectorDynamic<int> {
    public:
        int SetItem(int i) {
            return (*$self)(i);
        }

        void GetItem(int i, int v) {
            (*$self)(i) = v;
        }

        const int Size() {
            const int r = $self->rows();
            return r;
        }

        void GetVectorData(int* p, int len) {
            for (int i = 0; i < len; i++){
                p[i] =  (int)(*$self)(i);
            }
        }

        void SetVect(int numel, int* q){
            ($self)->resize(numel);
            for (int i = 0; i < numel; i++){
                (*$self)(i) = q[i];
            }
        }

        void SetZero() {
            for (int i = 0; i < $self->rows(); i++)
                (*$self)(i) = 0;
        }
};


%extend chrono::ChMatrixDynamic<double> {
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

        double GetItem(int i, int j) {
            return (*$self)(i, j);
        }

        void SetItem(int i, int j, double v) {
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
            int c = $self->cols();
            //double matr[len];
            //double* matr = $self->data();
            for (int i = 0; i < len; i++){
                int ri = floor (i/c);
                int ci = i - c*ri;
                p[i] =  (double)(*$self)(ri, ci);
            }
        }

        void SetMatr(double* mat, int rows, int cols) {
            ($self)->resize(rows, cols);
            for (int i = 0; i < rows; i++){
                for (int j = 0; j < cols; j++){
                    (*$self)(i, j) = mat[i*cols + j];
                }
            }
        }

        void SetZero() {
            for (int i = 0; i < $self->rows(); i++) {
              for (int j = 0; j < $self->cols(); j++)
                (*$self)(i,j) = 0;
            }
        }
};


// Explicitly instruct SWIG to know about the type alias. This is to ensure the '&' operator is properly mapped for ChMatrix66d& uses.
namespace chrono {
    template <typename T, int N, int M>
    class ChMatrixNM;
    typedef ChMatrixNM<double, 6, 6> ChMatrix66d;
}

%template(ChMatrix66d) chrono::ChMatrixNM<double, 6, 6>;

%extend chrono::ChMatrixNM<double, 6, 6> {
    public:
        double GetItem(int i, int j) {
            return (*$self)(i, j);
        }

        void SetItem(int i, int j, double v) {
            (*$self)(i, j) = v;
        }

        const int GetRows() {
            return 6;
        }

        const int GetColumns() {
            return 6;
        }

        void GetMatrixData(double* p, int len) {
            int c = $self->cols();
            for (int i = 0; i < len; i++){
                int ri = floor (i/c);
                int ci = i - c*ri;
                p[i] =  (double)(*$self)(ri, ci);
            }
        }

        void SetMatr(double* mat, int rows, int cols) {
            for (int i = 0; i < 6; i++){
                for (int j = 0; j < 6; j++){
                    (*$self)(i, j) = mat[i*cols + j];
                }
            }
        }

        void SetZero() {
            for (int i = 0; i < 6; i++) {
              for (int j = 0; j < 6; j++)
                (*$self)(i,j) = 0;
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

setattr(ChMatrixDynamicd, "GetMatr", GetMatr)

def GetVect(self):
    len = self.Size()
    lst = self.GetVectorData(len)
    rs_list = reshape(lst, len, 1)
    return rs_list

setattr(ChVectorDynamicd, "GetVect", GetVect)

def __matr_setitem(self,index,vals):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    self.SetItem(index[0],index[1],vals)

def __matr_getitem(self,index):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    return self.GetItem(index[0],index[1])

setattr(ChMatrixDynamicd, "__getitem__", __matr_getitem)
setattr(ChMatrixDynamicd, "__setitem__", __matr_setitem)

%}

#endif             // --------------------------------------------------------------------- PYTHON

%ignore chrono::ChMatrixDynamic;
%include "../../../chrono/core/ChMatrix.h"
