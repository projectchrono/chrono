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

/**
* Discrete Elment info 
*
* Observations:
*   - The units are not specified; they are user units. Additionally, internally, Chrono::Grnular adimensiolizes evertyhing using element characteristic size, etc.
*
*/
namespace chrono {

    enum GRN_TIME_STEPPING { AUTO, USER_SET };

    class CH_GRANULAR_API ChGRN_DE_Container {
    protected:
        size_t nDEs;                  ///< Number of discrete elements
        float* pGRN_xyz_DE;
        float* pGRN_xyzDOT_DE;

        float slide_mu_kin;
        float slide_mu_dyn;
        GRN_TIME_STEPPING time_stepping;

    public:

        ChGRN_DE_Container() : time_stepping(GRN_TIME_STEPPING::AUTO), nDEs(0), pGRN_xyz_DE(nullptr), pGRN_xyzDOT_DE(nullptr) {}
        ~ChGRN_DE_Container() {
            if (pGRN_xyz_DE != nullptr)
                delete[] pGRN_xyz_DE;
            if (pGRN_xyzDOT_DE != nullptr)
                delete[] pGRN_xyzDOT_DE;
        }

        void setup(const size_t& nElems) {
            nDEs = nElems;
            pGRN_xyz_DE = new float[nDEs * 3 * sizeof(float)];
            pGRN_xyzDOT_DE = new float[nDEs * 3 * sizeof(float)];
        }

        inline size_t elementCount() const { return nDEs; }
        inline float* pXYZsphereLocation() const { return pGRN_xyz_DE; }
        inline float* pXYZsphereVelocity() const { return pGRN_xyzDOT_DE; }

    };


    /**
    * ChGRN_DE_MONODISP_SPH_IN_BOX: Mono-disperse setup, one radius for all spheres
    */
    class CH_GRANULAR_API ChGRN_DE_MONODISP_SPH_IN_BOX_SMC: public ChGRN_DE_Container {
    protected:
        //!< Reference Frame of the box
        float sphere_radius;
        //!< XYZ location of the center of the box
        //!< Euler params for orientation of the box
        float modulusYoung_SPH2SPH;
        float modulusYoung_SPH2WALL;

        unsigned int PRECISION_FACTOR_SPACE; //!< Everthing is measured as multiples of sphere_radius/PRECISION_FACTOR_SPACE. Ex.: the radius of the sphere is 1000 units
        unsigned int PRECISION_FACTOR_TIME;  //!< Any time quanity is measured as a multiple of 

    public:
        ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(float radiusSPH = 1.f, unsigned int countSPHs = 0) : ChGRN_DE_Container(){
            this->setup(countSPHs);
        }

        ~ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(){}

        virtual void settle(float t_end) = 0;
        void setBOXdims(float xDIM, float yDIM, float zDIM);
        inline void YoungModulus_SPH2SPH (float someValue) { modulusYoung_SPH2SPH  = someValue; }
        inline void YoungModulus_SPH2WALL(float someValue) { modulusYoung_SPH2WALL = someValue; }

    };

    /**
    * ChGRN_DE_MONODISP_SPH_IN_BOX_NOFRIC_SMC: Mono-disperse setup, one radius for all spheres. There is no friction,
    * which means that there is no need to keep data that stores history for contacts
    */
    class CH_GRANULAR_API ChGRN_DE_MONODISP_SPH_IN_BOX_NOFRIC_SMC : public ChGRN_DE_MONODISP_SPH_IN_BOX_SMC {
    protected:

    public:
        ChGRN_DE_MONODISP_SPH_IN_BOX_NOFRIC_SMC(float radiusSPH = 1.f, unsigned int countSPHs = 0) : ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(radiusSPH, countSPHs){}

        ~ChGRN_DE_MONODISP_SPH_IN_BOX_NOFRIC_SMC(){}

        virtual void settle(float t_end);
    };

}