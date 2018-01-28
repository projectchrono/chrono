// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut
// =============================================================================

#include "../ChApiGranular.h"

#pragma once


namespace chrono {

    class CH_GRANULAR_API ChGRN_DE_Container {
    protected:
        size_t nDE;                  ///< Number of discrete elements
        float* pGRN_xyz_DE;
        float* pGRN_xyzDOT_DE;

        float slide_mu_kin;
        float slide_mu_dyn;

    public:
        ChGRN_DE_Container() : nSpheres(0), pGRN_xyz_DE(nullptr), pGRN_xyzDOT_DE(nullptr) {}
        ~ChGRN_DE_Container() {
            if (pGRN_xyz_DE != nullptr)
                delete[] pGRN_xyz_DE;
            if (pGRN_xyzDOT_DE != nullptr)
                delete[] pGRN_xyzDOT_DE;
        }

        void setup(const size_t& nS) {
            nSpheres = nS;
            pGRN_xyz_DE = new float[nS * 3 * sizeof(float)];
            pGRN_xyzDOT_DE = new float[nS * 3 * sizeof(float)];
        }

        inline size_t sphereCount() const { return nSpheres; }
        inline float* pXYZsphereLocation() const { return pGRN_xyz_DE; }
        inline float* pXYZsphereVelocity() const { return pGRN_xyzDOT_DE; }

    };

    class CH_GRANULAR_API ChGRN_DE_MONOSPH_IN_BOX : public ChGRN_DE_Container {
    private:
        //!< Reference Frame of the box
        float sphere_radius;
        //!< XYZ location of the center of the box
        //!< Euler params for orientation of the box

    public:
        ChGRN_DE_MONOSPH_IN_BOX(float radiusSPH = 1.f, unsigned int countSPHs = 0) : ChGRN_DE_Container(){
            this->setup(countSPHs);
        }

        ~ChGRN_DE_MONOSPH_IN_BOX(){}
    };

}