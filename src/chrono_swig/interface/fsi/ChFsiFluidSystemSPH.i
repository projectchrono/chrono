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

    PyObject* GetParticlePositionsNumpy(chrono::fsi::sph::MarkerGroup group = chrono::fsi::sph::MarkerGroup::ALL) {
        return Real3ArrayToNumpy(self->GetPositions(group));
    }

    PyObject* GetParticleVelocitiesNumpy(chrono::fsi::sph::MarkerGroup group = chrono::fsi::sph::MarkerGroup::ALL) {
        return Real3ArrayToNumpy(self->GetVelocities(group));
    }

    PyObject* GetParticleAccelerationsNumpy(chrono::fsi::sph::MarkerGroup group = chrono::fsi::sph::MarkerGroup::ALL) {
        return Real3ArrayToNumpy(self->GetAccelerations(group));
    }

    PyObject* GetParticleForcesNumpy(chrono::fsi::sph::MarkerGroup group = chrono::fsi::sph::MarkerGroup::ALL) {
        return Real3ArrayToNumpy(self->GetForces(group));
    }

    PyObject* GetParticleFluidPropertiesNumpy(chrono::fsi::sph::MarkerGroup group = chrono::fsi::sph::MarkerGroup::ALL) {
        return Real3ArrayToNumpy(self->GetProperties(group));
    }
}

#endif

%shared_ptr(chrono::fsi::sph::ChFsiFluidSystemSPH::ParticlePropertiesCallback)
%shared_ptr(chrono::fsi::sph::DepthPressurePropertiesCallback)

%feature("director") chrono::fsi::sph::ChFsiFluidSystemSPH::ParticlePropertiesCallback;

/* Parse the header file to generate wrappers */
%include "../../../chrono_fsi/sph/ChFsiFluidSystemSPH.h"    
