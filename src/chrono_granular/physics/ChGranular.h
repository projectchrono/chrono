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
    float* pGRN_xyz_DE;
    float* pGRN_xyzDOT_DE;

    float* p_device_GRN_xyz_DE;
    float* p_device_GRN_xyzDOT_DE;

    GRN_TIME_STEPPING time_stepping;

  public:
    ChGRN_DE_Container()
        : time_stepping(GRN_TIME_STEPPING::AUTO), nDEs(0), pGRN_xyz_DE(nullptr), pGRN_xyzDOT_DE(nullptr), p_device_GRN_xyz_DE(nullptr), p_device_GRN_xyzDOT_DE(nullptr) {}

    ~ChGRN_DE_Container();

    void setup(const size_t& nElems) {
        if (nElems) {
            nDEs = nElems;
            pGRN_xyz_DE = new float[nDEs * 3 * sizeof(float)];
            pGRN_xyzDOT_DE = new float[nDEs * 3 * sizeof(float)];
        }
    }

    inline size_t elementCount() const { return nDEs; }
    inline float* pXYZsphereLocation() const { return pGRN_xyz_DE; }
    inline float* pXYZsphereVelocity() const { return pGRN_xyzDOT_DE; }
};

/**
 * ChGRN_DE_MONODISP_SPH_IN_BOX: Mono-disperse setup, one radius for all spheres
 */
class CH_GRANULAR_API ChGRN_DE_MONODISP_SPH_IN_BOX_SMC : public ChGRN_DE_Container {
  protected:
    //!< Reference Frame of the box
    float sphere_radius;
    //!< XYZ location of the center of the box
    //!< Euler params for orientation of the box
    float modulusYoung_SPH2SPH;
    float modulusYoung_SPH2WALL;

    unsigned int PRECISION_FACTOR_SPACE;  //!< Everthing is measured as multiples of
                                          //!< sphere_radius/PRECISION_FACTOR_SPACE. Ex.: the radius of the sphere is
                                          //!< 1000 units
    unsigned int PRECISION_FACTOR_TIME;   //!< Any time quanity is measured as a multiple of

    unsigned int* p_device_SD_countsOfSheresTouching;  //!< Entry "i" says how many spheres touch SD i
    unsigned int* p_device_spheres_in_SD_composite;    //!< Array containing the IDs of the spheres stored in the SDs associated with the box

  public:
    ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(float radiusSPH, unsigned int countSPHs) : ChGRN_DE_Container() {
        this->setup(countSPHs);
        sphere_radius = radiusSPH;
    }

    ~ChGRN_DE_MONODISP_SPH_IN_BOX_SMC() {}

    virtual void settle(float t_end) = 0;
    virtual void setup_simulation() = 0;
    void setBOXdims(float xDIM, float yDIM, float zDIM) {}  /// Set unimplemented for now
    inline void YoungModulus_SPH2SPH(float someValue) { modulusYoung_SPH2SPH = someValue; }
    inline void YoungModulus_SPH2WALL(float someValue) { modulusYoung_SPH2WALL = someValue; }

    inline size_t  nSpheres() { return nDEs; }
};

/**
 * ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC: Mono-disperse setup, one radius for all spheres. There is no friction,
 * which means that there is no need to keep data that stores history for contacts
 */
class CH_GRANULAR_API ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC : public ChGRN_DE_MONODISP_SPH_IN_BOX_SMC {
  protected:
  public:
    ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC(float radiusSPH, unsigned int countSPHs)
        : ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(radiusSPH, countSPHs) {}

    ~ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC() {}

    virtual void setup_simulation(); //!< set up data structures and carry out pre-processing tasks
    virtual void settle(float t_end);
};

}  // namespace chrono