%{

/* Includes the header in the wrapper code */
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

using namespace chrono::fsi::sph;

#ifdef CHRONO_PYTHON_NUMPY
#include <numpy/arrayobject.h>
#endif

%}

#ifdef CHRONO_PYTHON_NUMPY

%{
    PyObject* Real3ArrayToNumpy(const std::vector<Real3>& vec) {
        npy_intp dims[2];
        dims[0] = (npy_intp)vec.size();
        dims[1] = 3;

        PyObject* arr = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
        if (!arr) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to allocate NumPy array");
            return NULL;
        }

        double* data = static_cast<double*>(PyArray_DATA(reinterpret_cast<PyArrayObject*>(arr)));
        for (size_t i = 0; i < vec.size(); ++i) {
            data[3*i + 0] = vec[i].x;
            data[3*i + 1] = vec[i].y;
            data[3*i + 2] = vec[i].z;
        }
        return arr;
    }
%}

%extend chrono::fsi::sph::ChFsiFluidSystemSPH {

    PyObject* GetParticlePositionsNumpy() {
        return Real3ArrayToNumpy(self->GetPositions());
    }

    PyObject* GetParticleVelocitiesNumpy() {
        return Real3ArrayToNumpy(self->GetVelocities());
    }

    PyObject* GetParticleAccelerationsNumpy() {
        return Real3ArrayToNumpy(self->GetAccelerations());
    }

    PyObject* GetParticleForcesNumpy() {
        return Real3ArrayToNumpy(self->GetForces());
    }

    PyObject* GetParticleFluidPropertiesNumpy() {
        return Real3ArrayToNumpy(self->GetProperties());
    }
}
#endif

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/sph/ChFsiFluidSystemSPH.h"    

