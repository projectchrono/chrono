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
#include <string>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>
#include "../ChApiGranular.h"
#include "chrono_granular/ChGranularDefines.h"
#include "cudalloc.hpp"

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
    double LENGTH_UNIT;  //!< Any length expressed in SU is a multiple of LENGTH_UNIT
    double TIME_UNIT;    //!< Any time quanity in SU is measured as a positive multiple of TIME_UNIT
    double MASS_UNIT;    //!< Any mass quanity is measured as a positive multiple of MASS_UNIT. NOTE: The MASS_UNIT is
                         //!< equal the the mass of a sphere

    unsigned int nDEs;  ///< Number of discrete elements
    unsigned int nSDs;  ///< Number of subdomains that the BD is split in

    // Use CUDA allocator written by Colin, could hit system performance if there's not a lot of RAM
    // Makes somewhat faster memcpys
    std::vector<signed int, cudallocator<signed int>> h_X_DE;
    std::vector<signed int, cudallocator<signed int>> h_Y_DE;
    std::vector<signed int, cudallocator<signed int>> h_Z_DE;
    std::vector<signed int, cudallocator<signed int>> h_XDOT_DE;
    std::vector<signed int, cudallocator<signed int>> h_YDOT_DE;
    std::vector<signed int, cudallocator<signed int>> h_ZDOT_DE;

    /// Device pointers
    int* p_d_CM_X;
    int* p_d_CM_Y;
    int* p_d_CM_Z;
    int* p_d_CM_XDOT;
    int* p_d_CM_YDOT;
    int* p_d_CM_ZDOT;

    float X_accGrav;  //!< X component of the gravitational acceleration
    float Y_accGrav;  //!< Y component of the gravitational acceleration
    float Z_accGrav;  //!< Z component of the gravitational acceleration

    float
        gravAcc_X_factor_SU;  //!< \f$Psi_L/(Psi_T^2 Psi_h) \times (g_X/g)\f$, where g is the gravitational acceleration
    float
        gravAcc_Y_factor_SU;  //!< \f$Psi_L/(Psi_T^2 Psi_h) \times (g_Y/g)\f$, where g is the gravitational acceleration
    float
        gravAcc_Z_factor_SU;  //!< \f$Psi_L/(Psi_T^2 Psi_h) \times (g_Z/g)\f$, where g is the gravitational acceleration

    unsigned int* p_device_SD_NumOf_DEs_Touching;  //!< Entry "i" says how many spheres touch SD i
    unsigned int* p_device_DEs_in_SD_composite;    //!< Array containing the IDs of the spheres stored in the SDs
                                                   //!< associated with the box

    GRN_TIME_STEPPING time_stepping;  //!< Indicates what type of time stepping the simulation employs.

    /// Partition the big domain (BD) and sets the number of SDs that BD is split in.
    /// This is pure virtual since each problem will have a specific way of splitting BD based on shape of BD and DEs
    virtual void partition_BD() = 0;
    virtual void copyCONSTdata_to_device() = 0;
    virtual void setup_simulation() = 0;
    virtual void cleanup_simulation() = 0;
    virtual void switch_to_SimUnits() = 0;

  public:
    ChGRN_DE_Container()
        : time_stepping(GRN_TIME_STEPPING::AUTO),
          nDEs(0),
          p_d_CM_X(nullptr),
          p_d_CM_Y(nullptr),
          p_d_CM_Z(nullptr),
          p_d_CM_XDOT(nullptr),
          p_d_CM_YDOT(nullptr),
          p_d_CM_ZDOT(nullptr),
          p_device_SD_NumOf_DEs_Touching(nullptr),
          p_device_DEs_in_SD_composite(nullptr) {}

    ~ChGRN_DE_Container() {}

    inline unsigned int elementCount() const { return nDEs; }
    inline unsigned int get_SD_count() const { return nSDs; }
    void set_gravitational_acceleration(float xVal, float yVal, float zVal) {
        X_accGrav = xVal;
        Y_accGrav = yVal;
        Z_accGrav = zVal;
    }

    virtual void generate_DEs() = 0;
};

/**
 * ChGRN_DE_MONODISP_SPH_IN_BOX: Mono-disperse setup, one radius for all spheres
 */
class CH_GRANULAR_API ChGRN_DE_MONODISP_SPH_IN_BOX_SMC : public ChGRN_DE_Container {
  protected:
    float sphere_radius;   /// User defined radius of the sphere
    float sphere_density;  /// User defined density of the sphere

    double modulusYoung_SPH2SPH;
    double modulusYoung_SPH2WALL;
    double K_stiffness;

    float box_L;  //!< length of physical box; will define the local X axis located at the CM of the box (left to right)
    float box_D;  //!< depth of physical box; will define the local Y axis located at the CM of the box (into screen)
    float box_H;  //!< height of physical box; will define the local Z axis located at the CM of the box (pointing up)

    unsigned int psi_T_Factor;
    unsigned int psi_h_Factor;
    unsigned int psi_L_Factor;

    unsigned int monoDisperseSphRadius_SU;  //!< Size of the sphere radius, in Simulation Units
    double reciprocal_sphDiam_SU;           //!< The inverse of the sphere diameter, in Simulation Units

    unsigned int SD_L_SU;  //!< Size of the SD in the L direction (expressed in Simulation Units)
    unsigned int SD_D_SU;  //!< Size of the SD in the L direction (expressed in Simulation Units)
    unsigned int SD_H_SU;  //!< Size of the SD in the L direction (expressed in Simulation Units)

    unsigned int nSDs_L_SU;  //!< Number of SDs along the L dimension of the box
    unsigned int nSDs_D_SU;  //!< Number of SDs along the D dimension of the box
    unsigned int nSDs_H_SU;  //!< Number of SDs along the H dimension of the box

    void partition_BD();

    void switch_to_SimUnits();

  public:
    ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(float radiusSPH, float density) : ChGRN_DE_Container() {
        sphere_radius = radiusSPH;
        sphere_density = density;

        psi_T_Factor = PSI_T;
        psi_h_Factor = PSI_h;
        psi_L_Factor = PSI_L;
    }

    ~ChGRN_DE_MONODISP_SPH_IN_BOX_SMC() {}

    virtual void settle(float t_end) = 0;
    void setBOXdims(float L_DIM, float D_DIM, float H_DIM) {
        box_L = L_DIM;
        box_D = D_DIM;
        box_H = H_DIM;
    }
    inline void YoungModulus_SPH2SPH(double someValue) { modulusYoung_SPH2SPH = someValue; }
    inline void YoungModulus_SPH2WALL(double someValue) { modulusYoung_SPH2WALL = someValue; }

    inline size_t nSpheres() { return nDEs; }
};

/**
 * ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC: Mono-disperse setup, one radius for all spheres. There is no friction,
 * which means that there is no need to keep data that stores history for contacts
 */
class CH_GRANULAR_API ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC : public ChGRN_DE_MONODISP_SPH_IN_BOX_SMC {
  protected:
    virtual void copyCONSTdata_to_device();

    virtual void cleanup_simulation();

  public:
    ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC(float radiusSPH, float density)
        : ChGRN_DE_MONODISP_SPH_IN_BOX_SMC(radiusSPH, density) {}

    ~ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC() {}

    virtual void setup_simulation();  //!< set up data structures and carry out pre-processing tasks
    virtual void settle(float t_end);
    /// Copy back the sd device data and save it to a file for error checking on the priming kernel
    void checkSDCounts(std::string, bool, bool);
    void writeFile(std::string, unsigned int*);
    void copyDataBackToHost();
    virtual void generate_DEs();
};

}  // namespace chrono