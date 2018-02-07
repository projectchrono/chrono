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
#pragma once

#include <cstddef>
#include "../ChApiGranular.h"

/**
 * Discrete Elment info
 *
 * Observations:
 *   - The units are not specified; they are user units. Additionally, internally, Chrono::Grnular adimensiolizes
 * evertyhing using element characteristic size, etc.
 *
 */
namespace chrono {

    enum GRN_TIME_STEPPING { AUTO, USER_SET };

    class CH_GRANULAR_API ChGRN_DE_Container {
    protected:
        size_t nDEs;  ///< Number of discrete elements
        float* p_h_xyz_DE;
        float* p_h_xyzDOT_DE;

        /// Device pointers
        unsigned int* p_d_CM_X;
        unsigned int* p_d_CM_Y;
        unsigned int* p_d_CM_Z;
        unsigned int* p_d_CM_XDOT;
        unsigned int* p_d_CM_YDOT;
        unsigned int* p_d_CM_ZDOT;

        unsigned int* p_device_SD_NumOf_DEs_Touching;  //!< Entry "i" says how many spheres touch SD i
        unsigned int* p_device_DEs_in_SD_composite;    //!< Array containing the IDs of the spheres stored in the SDs associated with the box

        unsigned int nSDs;


        GRN_TIME_STEPPING time_stepping;

        /// Partition the big domain (BD) and sets the number of SDs that BD is split in. 
        /// This is protected since the user has very little insights in how to split the BD.
        /// This is pure virtual since each problem will have a specific way of splitting BD based on shape of BD and DEs
        virtual void partition_BD() = 0;
        virtual void adimensionlize() = 0;

    public:
        ChGRN_DE_Container()
            : time_stepping(GRN_TIME_STEPPING::AUTO), nDEs(0), p_h_xyz_DE(nullptr), p_h_xyzDOT_DE(nullptr),
            p_d_CM_X(nullptr), p_d_CM_Y(nullptr), p_d_CM_Z(nullptr), p_d_CM_XDOT(nullptr), p_d_CM_YDOT(nullptr), p_d_CM_ZDOT(nullptr) {}

        ~ChGRN_DE_Container();

        void setup(const size_t& nElems) {
            if (nElems) {
                nDEs = nElems;
                p_h_xyz_DE = new float[nDEs * 3 * sizeof(float)];
                p_h_xyzDOT_DE = new float[nDEs * 3 * sizeof(float)];
            }
        }

        inline size_t elementCount() const { return nDEs; }
        inline float* pXYZsphereLocation() const { return p_h_xyz_DE; }
        inline float* pXYZsphereVelocity() const { return p_h_xyzDOT_DE; }

        inline unsigned int get_SD_count() const { return nSDs; }
    };

    /**
     * ChGRN_DE_MONODISP_SPH_IN_BOX: Mono-disperse setup, one radius for all spheres
     */
    class CH_GRANULAR_API ChGRN_DE_MONODISP_SPH_IN_BOX_SMC : public ChGRN_DE_Container {
    protected:        
        float sphere_radius;    /// User defined radius of the sphere 
        float sphere_density;   /// User defined density of the sphere

        float modulusYoung_SPH2SPH;
        float modulusYoung_SPH2WALL;

        float box_L; //!< length of physical box; will define the local X axis located at the CM of the box (left to right)
        float box_D; //!< depth of physical box; will define the local Y axis located at the CM of the box (into screen)
        float box_H; //!< height of physical box; will define the local Z axis located at the CM of the box (pointing up)

        double SPACE_UNIT;  //!< Everthing is measured as a multiple of SPACE_UNIT
        double TIME_UNIT;   //!< Any time quanity is measured as a positive multiple of TIME_UNIT
        double MASS_UNIT;   //!< Any mass quanity is measured as a positive multiple of MASS_UNIT. NOTE: The MASS_UNIT is equal the the mass of a sphere

        unsigned int monoDisperseSphRadius_AD; //!< The AD-ed value of the sphere radius

        unsigned int SD_L_AD;  //!< The AD-ed value of an SD in the L direction
        unsigned int SD_D_AD;  //!< The AD-ed value of an SD in the D direction
        unsigned int SD_H_AD;  //!< The AD-ed value of an SD in the H direction

        void partition_BD();
        void adimensionlize();


    public:
        ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(float radiusSPH, unsigned int countSPHs) : ChGRN_DE_Container() {
            this->setup(countSPHs);
            sphere_radius = radiusSPH;
        }

        ~ChGRN_DE_MONODISP_SPH_IN_BOX_SMC() {}

        virtual void settle(float t_end) = 0;
        virtual void setup_simulation() = 0;
        void setBOXdims(float L_DIM, float D_DIM, float H_DIM) { box_L = L_DIM; box_D = D_DIM; box_H = H_DIM; }
        inline void YoungModulus_SPH2SPH(float someValue) { modulusYoung_SPH2SPH = someValue; }
        inline void YoungModulus_SPH2WALL(float someValue) { modulusYoung_SPH2WALL = someValue; }
        inline void setDensity(float someValue) { sphere_density = someValue; }

        inline size_t  nSpheres() { return nDEs; }
    };

    /**
     * ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC: Mono-disperse setup, one radius for all spheres. There is no friction,
     * which means that there is no need to keep data that stores history for contacts
     */
    class CH_GRANULAR_API ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC : public ChGRN_DE_MONODISP_SPH_IN_BOX_SMC {
    protected:

    public:
        ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC(float radiusSPH, unsigned int countSPHs): ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(radiusSPH, countSPHs) {
            setup_simulation();
        }

        ~ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC() {}

        virtual void setup_simulation(); //!< set up data structures and carry out pre-processing tasks
        virtual void settle(float t_end);
    };

}  // namespace chrono