#include "../ChApiGranular.h"

#pragma once


namespace chrono {

    class CH_GRANULAR_API ChGRN_host_SphContainer {
    private:
        size_t nSpheres;                  ///< Number of sph
        float* pGRN_xyzSpheres;
    public:
        ChGRN_host_SphContainer() : nSpheres(0), pGRN_xyzSpheres(nullptr) {}
        ~ChGRN_host_SphContainer() {
            if (pGRN_xyzSpheres != nullptr)
                delete[] pGRN_xyzSpheres;
        }

        void allocate_xyzSphereSpace(const size_t& nS) {
            nSpheres = nS;
            pGRN_xyzSpheres = new float[nS];
        }

        inline size_t sphereCount() const { return nSpheres; }
        inline float* pXYZsphereLocation() const { return pGRN_xyzSpheres; }

    } ;

    class ChGRN_device_SphContainer {
    public:
        //
        float* pGRN_device_xyzSpheres;
    };

}