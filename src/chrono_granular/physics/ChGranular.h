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
// Authors: Dan Negrut, Nic Olsen
// =============================================================================
/*! \file */

#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <cmath>
#include "../ChApiGranular.h"
#include "chrono/core/ChVector.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranularBoundaryConditions.h"
#include "chrono_granular/utils/ChGranularUtilities.h"
#include "cudalloc.hpp"

/**
 * Discrete Elment info
 *
 * Observations:
 *   - The units are not specified; they are user units. Additionally, internally, Chrono::Granular redimensiolizes
 * evertyhing using element characteristic size, etc.
 *
 */
namespace chrono {
namespace granular {

/// hold pointers
struct sphereDataStruct {
  public:
    /// Store positions and velocities in unified memory
    int* pos_X;
    int* pos_Y;
    int* pos_Z;
    float* pos_X_dt;
    float* pos_Y_dt;
    float* pos_Z_dt;

    /// store angular velocity (axis and magnitude in one vector)
    float* omega_X;
    float* omega_Y;
    float* omega_Z;

    float* sphere_force_X;
    float* sphere_force_Y;
    float* sphere_force_Z;

    /// store angular acceleration (axis and magnitude in one vector)
    float* sphere_torque_X;
    float* sphere_torque_Y;
    float* sphere_torque_Z;

    /// used for chung integrator
    float* sphere_force_X_old;
    float* sphere_force_Y_old;
    float* sphere_force_Z_old;

    /// sphere broadphase data
    unsigned int* SD_NumOf_DEs_Touching;
    unsigned int* DEs_in_SD_composite;
};

// How are we writing?
enum GRAN_OUTPUT_MODE { CSV, BINARY, NONE };
// How are we stepping through time?
enum GRAN_TIME_STEPPING { AUTO, FIXED };
/// How are we integrating w.r.t. time
enum GRAN_TIME_INTEGRATOR { FORWARD_EULER, CHUNG };

enum GRAN_FRICTION_MODE { FRICTIONLESS, SINGLE_STEP, MULTI_STEP };

enum GRAN_CONTACT_MODEL { HOOKE, HERTZ };

/// Parameters needed for sphere-based granular dynamics
struct GranParamsHolder {
    // Timestep in SU
    float alpha_h_bar;

    GRAN_FRICTION_MODE friction_mode;
    GRAN_TIME_INTEGRATOR integrator_type;
    GRAN_CONTACT_MODEL contact_model;

    // Use user-defined quantities for coefficients
    float Gamma_n_s2s_SU;  //!< sphere-to-sphere contact damping coefficient, expressed in SU
    float Gamma_n_s2w_SU;  //!< sphere-to-sphere contact damping coefficient, expressed in SU
    float Gamma_t_s2s_SU;
    float Gamma_t_s2w_SU;

    float Kn_s2s_SU;  //!< normal stiffness coefficient, expressed in SU: sphere-to-sphere
    float Kn_s2w_SU;  //!< normal stiffness coefficient, expressed in SU: sphere-to-wall
    float K_t_s2s_SU;
    float K_t_s2w_SU;

    unsigned int sphereRadius_SU;  //!< Radius of the sphere, expressed in SU
    unsigned int SD_size_X_SU;     //!< X-dimension of the SD box, expressed in SU
    unsigned int SD_size_Y_SU;     //!< Y-dimension of the SD box, expressed in SU
    unsigned int SD_size_Z_SU;     //!< Z-dimension of the SD box, expressed in SU
    unsigned int nSDs_X;           //!< X-dimension of the BD box in multiples of subdomains, expressed in SU
    unsigned int nSDs_Y;           //!< Y-dimension of the BD box in multiples of subdomains, expressed in SU
    unsigned int nSDs_Z;           //!< Z-dimension of the BD box in multiples of subdomains, expressed in SU
    float gravAcc_X_SU;            //!< Device counterpart of the constant gravity_X_SU
    float gravAcc_Y_SU;            //!< Device counterpart of the constant gravity_Y_SU
    float gravAcc_Z_SU;            //!< Device counterpart of the constant gravity_Z_SU
    float gravMag_SU;

    // Changed by updateBDPosition() at every timestep
    int BD_frame_X;  //!< The bottom-left corner xPos of the BD, allows boxes not centered at origin
    int BD_frame_Y;  //!< The bottom-left corner yPos of the BD, allows boxes not centered at origin
    int BD_frame_Z;  //!< The bottom-left corner zPos of the BD, allows boxes not centered at origin
    float BD_frame_X_dot;
    float BD_frame_Y_dot;
    float BD_frame_Z_dot;

    unsigned int psi_T;
    unsigned int psi_h;
    unsigned int psi_L;

    /// Ratio of cohesion force to gravity
    float cohesion_ratio;

    double LENGTH_UNIT;  //!< 1 / C_L. Any length expressed in SU is a multiple of LENGTH_UNIT
    double TIME_UNIT;    //!< 1 / C_T. Any time quanity in SU is measured as a positive multiple of TIME_UNIT
    double MASS_UNIT;    //!< 1 / C_M. Any mass quanity is measured as a positive multiple of MASS_UNIT.
};
}  // namespace granular
}  // namespace chrono

// Do two things: make the naming nicer and require a const pointer everywhere
typedef const chrono::granular::GranParamsHolder* ParamsPtr;
namespace chrono {
namespace granular {
class CH_GRANULAR_API ChSystemGranular_MonodisperseSMC {
  public:
    ChSystemGranular_MonodisperseSMC() = delete;  // TODO do we want this one around?
    ChSystemGranular_MonodisperseSMC(float radiusSPH, float density);
    virtual ~ChSystemGranular_MonodisperseSMC();

    unsigned int elementCount() const { return nDEs; }
    unsigned int get_SD_count() const { return nSDs; }
    void set_gravitational_acceleration(float xVal, float yVal, float zVal) {
        X_accGrav = xVal;
        Y_accGrav = yVal;
        Z_accGrav = zVal;
    }

    // Get the max Young Modulus

    virtual void generate_DEs();
    virtual void generate_DEs_FillBounds();
    virtual void generate_DEs_positions();

    /// Set the BD to be fixed or not, if fixed it will ignore any given position functions
    void set_BD_Fixed(bool fixed) { BD_is_fixed = fixed; }

    /// Set bounds to fill on the big box, goes xyz min, xyz max as floats from -1 to 1
    /// Passing xmin = -1, xmax = 1 means fill the box in xdir
    // TODO comment this more
    void setFillBounds(float xmin, float ymin, float zmin, float xmax, float ymax, float zmax);

    void setParticlePositions(std::vector<ChVector<float>>& points);

    /// Prescribe the motion of the BD, allows wavetank-style simulations
    /// NOTE that this is the center of the container
    void setBDPositionFunction(std::function<double(double)> fx,
                               std::function<double(double)> fy,
                               std::function<double(double)> fz) {
        BDPositionFunctionX = fx;
        BDPositionFunctionY = fy;
        BDPositionFunctionZ = fz;
    }

    void setBOXdims(float L_DIM, float D_DIM, float H_DIM) {
        box_size_X = L_DIM;
        box_size_Y = D_DIM;
        box_size_Z = H_DIM;
    }

    void setPsiFactors(unsigned int psi_T_new, unsigned int psi_h_new, unsigned int psi_L_new) {
        gran_params->psi_T = psi_T_new;
        gran_params->psi_h = psi_h_new;
        gran_params->psi_L = psi_L_new;
    }

    size_t nSpheres() { return nDEs; }

    /// Create an axis-aligned box BC
    void Create_BC_AABox(float hdims[3], float center[3], bool outward_normal);

    /// Create an axis-aligned sphere BC
    void Create_BC_Sphere(float center[3], float radius, bool outward_normal);

    /// Create an z-axis aligned cone
    void Create_BC_Cone(float cone_tip[3], float slope, float hmax, float hmin, bool outward_normal);

    /// Set the output mode of the simulation
    void setOutputMode(GRAN_OUTPUT_MODE mode) { file_write_mode = mode; }
    /// Set the simulation's output directory, files are output as step%06d, where the number is replaced by the current
    /// render frame. This directory is assumed to be created by the user, either manually or in the driver file.
    void setOutputDirectory(std::string dir) { output_directory = dir; }

    void setVerbose(bool is_verbose) { verbose_runtime = is_verbose; }

    /// allows the user to request a step size, will find the closest SU size to it
    void set_max_adaptive_stepSize(float size_UU) { max_adaptive_step_UU = size_UU; }
    void set_fixed_stepSize(float size_UU) { fixed_step_UU = size_UU; }
    void set_timeStepping(GRAN_TIME_STEPPING new_stepping) { time_stepping = new_stepping; }
    void set_timeIntegrator(GRAN_TIME_INTEGRATOR new_integrator) { time_integrator = new_integrator; }
    void set_contactModel(GRAN_CONTACT_MODEL new_contact_model) { contact_model = new_contact_model; }

    /// get the max z position of the spheres, this allows us to do easier cosimulation
    double get_max_z() const;

    virtual void setup_simulation();  ///!< set up data structures and carry out pre-processing tasks

    /// advance simulation by duration seconds in user units, return actual duration elapsed
    /// Requires initialize() to have been called
    virtual double advance_simulation(float duration);
    virtual double get_max_K();

    /// Initialize simulation so that it can be advanced
    virtual void initialize();

    void set_friction_mode(GRAN_FRICTION_MODE new_mode) { fric_mode = new_mode; }

    void set_K_n_SPH2SPH(double someValue) { K_n_s2s_UU = someValue; }
    void set_K_n_SPH2WALL(double someValue) { K_n_s2w_UU = someValue; }

    void set_Gamma_n_SPH2SPH(double someValue) { Gamma_n_s2s_UU = someValue; }
    void set_Gamma_n_SPH2WALL(double someValue) { Gamma_n_s2w_UU = someValue; }

    void set_K_t_SPH2SPH(double someValue) { K_t_s2s_UU = someValue; }
    void set_Gamma_t_SPH2SPH(double someValue) { Gamma_t_s2s_UU = someValue; }

    void set_K_t_SPH2WALL(double someValue) { K_t_s2w_UU = someValue; }
    void set_Gamma_t_SPH2WALL(double someValue) { Gamma_t_s2w_UU = someValue; }

    /// Set the ratio of cohesion to gravity for monodisperse spheres
    void set_Cohesion_ratio(float someValue) { cohesion_over_gravity = someValue; }

    void runInitialSpherePriming();

    /// Copy back the sd device data and save it to a file for error checking on the priming kernel
    void checkSDCounts(std::string ofile, bool write_out, bool verbose);
    void writeFile(std::string ofile, unsigned int* deCounts);
    void writeFileUU(std::string ofile);
    void updateBDPosition(const float stepSize_SU);

  protected:
    /// Holds the friction mode for the system
    GRAN_FRICTION_MODE fric_mode;
    /// holds the sphere and BD-related params in unified memory
    GranParamsHolder* gran_params;

    /// Allows the code to be very verbose for debug
    bool verbose_runtime = false;
    /// How to write the output files? Default is CSV
    GRAN_OUTPUT_MODE file_write_mode = CSV;
    /// Directory to write to, this code assumes it already exists
    std::string output_directory;

    /// Number of discrete elements
    unsigned int nDEs;
    /// Number of subdomains that the BD is split in
    unsigned int nSDs;

    // Use CUDA allocator written by Colin, could hit system performance if there's not a lot of RAM
    // Makes somewhat faster memcpys
    /// Store positions and velocities in unified memory
    std::vector<int, cudallocator<int>> pos_X;
    std::vector<int, cudallocator<int>> pos_Y;
    std::vector<int, cudallocator<int>> pos_Z;
    std::vector<float, cudallocator<float>> pos_X_dt;
    std::vector<float, cudallocator<float>> pos_Y_dt;
    std::vector<float, cudallocator<float>> pos_Z_dt;

    /// store angular velocity (axis and magnitude in one vector)
    std::vector<float, cudallocator<float>> omega_X;
    std::vector<float, cudallocator<float>> omega_Y;
    std::vector<float, cudallocator<float>> omega_Z;

    std::vector<float, cudallocator<float>> sphere_force_X;
    std::vector<float, cudallocator<float>> sphere_force_Y;
    std::vector<float, cudallocator<float>> sphere_force_Z;

    /// store angular acceleration (axis and magnitude in one vector)
    std::vector<float, cudallocator<float>> sphere_torque_X;
    std::vector<float, cudallocator<float>> sphere_torque_Y;
    std::vector<float, cudallocator<float>> sphere_torque_Z;

    /// used for chung integrator
    std::vector<float, cudallocator<float>> sphere_force_X_old;
    std::vector<float, cudallocator<float>> sphere_force_Y_old;
    std::vector<float, cudallocator<float>> sphere_force_Z_old;

    /// gravity in user units
    float X_accGrav;
    float Y_accGrav;
    float Z_accGrav;

    ///  gravity in sim units
    float gravity_X_SU;
    float gravity_Y_SU;
    float gravity_Z_SU;

    /// User provided maximum timestep in UU, used in adaptive timestepping
    float max_adaptive_step_UU = 1e-3;
    /// User provided fixed timestep in UU, used in USER_SET timestepping
    float fixed_step_UU = 1e-4;
    /// Step size in SU, user can request a larger one but default is 1
    float stepSize_SU;

    /// Entry "i" says how many spheres touch SD i
    std::vector<unsigned int, cudallocator<unsigned int>> SD_NumOf_DEs_Touching;

    /// Array containing the IDs of the spheres stored in the SDs associated with the box
    std::vector<unsigned int, cudallocator<unsigned int>> DEs_in_SD_composite;

    GRAN_TIME_STEPPING time_stepping;      //!< Indicates what type of time stepping the simulation employs.
    GRAN_TIME_INTEGRATOR time_integrator;  //!< Indicates what type of time integrator the simulation employs.

    GRAN_CONTACT_MODEL contact_model;  //!< DEM local contact force model

    /// Partition the big domain (BD) and sets the number of SDs that BD is split in.
    void partition_BD();

    /// Copy constant parameter data to device
    void copy_const_data_to_device();
    /// Copy (potentially updated) BD frame to device
    void copyBD_Frame_to_device();

    /// Reset binning and broadphase info
    void resetBroadphaseInformation();
    /// Reset sphere-sphere forces
    void resetSphereForces();

    /// collect all the sphere data into a struct
    sphereDataStruct packSphereDataPointers();

    /// Store the prescribed position function for the BD, used for moving frames
    // Default is at rest
    std::function<double(double)> BDPositionFunctionX = [](double a) { return 0; };
    std::function<double(double)> BDPositionFunctionY = [](double a) { return 0; };
    std::function<double(double)> BDPositionFunctionZ = [](double a) { return 0; };

    /// Total time elapsed since beginning of simulation
    float elapsedSimTime;
    /// Max velocity of all particles in system
    float get_max_vel();

    // just a handy helper function
    template <typename T1, typename T2>
    T1 convertToPosSU(T2 val) {
        return val / gran_params->LENGTH_UNIT;
    }

    /// convert all BCs from UU to SU
    void convertBCUnits();

    /// Convert unit parameters from UU to SU
    virtual void switch_to_SimUnits();

    /// sort data based on owner SD
    virtual void defragment_data();

    const float new_step_freq = .01;
    virtual void determine_new_stepSize_SU();

    /// size of the normal stiffness (SU) for sphere-to-sphere contact
    double K_n_s2s_UU;
    float K_n_s2s_SU;

    /// size of the normal stiffness (SU) for sphere-to-wall contact
    double K_n_s2w_UU;
    float K_n_s2w_SU;

    /// normal damping for sphere-to-sphere
    double Gamma_n_s2s_UU;
    float Gamma_n_s2s_SU;

    /// normal damping for sphere-to-wall
    double Gamma_n_s2w_UU;
    float Gamma_n_s2w_SU;

    /// tangential stiffness for sphere-to-sphere
    double K_t_s2s_UU;
    float K_t_s2s_SU;

    /// tangential stiffness for sphere-to-wall
    double K_t_s2w_UU;
    float K_t_s2w_SU;

    /// tangential damping for sphere-to-sphere
    double Gamma_t_s2s_UU;
    float Gamma_t_s2s_SU;

    /// tangential damping for sphere-to-wall
    double Gamma_t_s2w_UU;
    float Gamma_t_s2w_SU;

    /// Store the ratio of the acceleration due to cohesion vs the acceleration due to gravity, makes simple API
    float cohesion_over_gravity;

    /// List of generalized BCs that constrain sphere motion
    std::vector<BC_type, cudallocator<BC_type>> BC_type_list;
    std::vector<BC_params_t<int, int3>, cudallocator<BC_params_t<int, int3>>> BC_params_list_SU;
    std::vector<BC_params_t<float, float3>, cudallocator<BC_params_t<float, float3>>> BC_params_list_UU;

    /// amount to fill box, as proportions of half-length
    /// Default is full box
    float boxFillXmin = -1.f;
    float boxFillYmin = -1.f;
    float boxFillZmin = -1.f;
    float boxFillXmax = 1.f;
    float boxFillYmax = 1.f;
    float boxFillZmax = 1.f;

    float sphere_radius;   /// User defined radius of the sphere
    float sphere_density;  /// User defined density of the sphere

    float box_size_X;  //!< length of physical box; will define the local X axis located at the CM of the box (left to
                       //!< right)
    float
        box_size_Y;  //!< depth of physical box; will define the local Y axis located at the CM of the box (into screen)
    float box_size_Z;  //!< height of physical box; will define the local Z axis located at the CM of the box (pointing
                       //!< up)

    unsigned int sphereRadius_SU;  //!< Size of the sphere radius, in Simulation Units

    unsigned int SD_size_X_SU;  //!< Size of the SD in the L direction (expressed in Simulation Units)
    unsigned int SD_size_Y_SU;  //!< Size of the SD in the D direction (expressed in Simulation Units)
    unsigned int SD_size_Z_SU;  //!< Size of the SD in the H direction (expressed in Simulation Units)

    unsigned int nSDs_X;  //!< Number of SDs along the L dimension of the box
    unsigned int nSDs_Y;  //!< Number of SDs along the D dimension of the box
    unsigned int nSDs_Z;  //!< Number of SDs along the H dimension of the box

    std::vector<ChVector<float>> h_points;

    /// The position of the BD in the global frame, allows us to have a moving BD or BD not at origin, etc.
    int BD_frame_X;
    int BD_frame_Y;
    int BD_frame_Z;

    /// The velocity of the BD in the global frame, allows us to have a moving BD or BD not at origin, etc.
    float BD_frame_X_dot;
    float BD_frame_Y_dot;
    float BD_frame_Z_dot;

    /// Allow the user to set the BD to be fixed, ignoring any given position functions
    bool BD_is_fixed = true;
};

}  // namespace granular
}  // namespace chrono
