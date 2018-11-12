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
#include "chrono_granular/ChApiGranular.h"
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
    float* sphere_Omega_X;  // Could these just be a single float3 array?
    float* sphere_Omega_Y;
    float* sphere_Omega_Z;

    float* sphere_force_X;
    float* sphere_force_Y;
    float* sphere_force_Z;

    /// store angular acceleration (axis and magnitude in one vector)
    float* sphere_ang_acc_X;
    float* sphere_ang_acc_Y;
    float* sphere_ang_acc_Z;

    /// used for chung integrator
    float* sphere_force_X_old;
    float* sphere_force_Y_old;
    float* sphere_force_Z_old;
    float* sphere_ang_acc_X_old;
    float* sphere_ang_acc_Y_old;
    float* sphere_ang_acc_Z_old;

    /// sphere broadphase data
    unsigned int* SD_NumOf_DEs_Touching;
    unsigned int* DEs_in_SD_composite;
};

// How are we writing?
enum GRAN_OUTPUT_MODE { CSV, BINARY, NONE };
// How are we stepping through time?
enum GRAN_TIME_STEPPING { ADAPTIVE, FIXED };
/// How are we integrating w.r.t. time
enum GRAN_TIME_INTEGRATOR { FORWARD_EULER, CHUNG };

enum GRAN_FRICTION_MODE { FRICTIONLESS, SINGLE_STEP, MULTI_STEP };

enum GRAN_CONTACT_MODEL { HOOKE, HERTZ };

/// Parameters needed for sphere-based granular dynamics
struct ChGranParams {
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

    float K_n_s2s_SU;  //!< normal stiffness coefficient, expressed in SU: sphere-to-sphere
    float K_n_s2w_SU;  //!< normal stiffness coefficient, expressed in SU: sphere-to-wall
    float K_t_s2s_SU;
    float K_t_s2w_SU;

    /// Radius of the sphere, expressed in SU
    unsigned int sphereRadius_SU;
    /// Moment of inertia of a sphere, normalized by the radius
    float sphereInertia_by_r;

    unsigned int SD_size_X_SU;  //!< X-dimension of the SD box, expressed in SU
    unsigned int SD_size_Y_SU;  //!< Y-dimension of the SD box, expressed in SU
    unsigned int SD_size_Z_SU;  //!< Z-dimension of the SD box, expressed in SU
    unsigned int nSDs_X;        //!< X-dimension of the BD box in multiples of subdomains, expressed in SU
    unsigned int nSDs_Y;        //!< Y-dimension of the BD box in multiples of subdomains, expressed in SU
    unsigned int nSDs_Z;        //!< Z-dimension of the BD box in multiples of subdomains, expressed in SU

    // These are the max X, Y, Z dimensions in the BD frame
    int64_t max_x_pos_unsigned;  // ((int64_t)gran_params->SD_size_X_SU * gran_params->nSDs_X)
    int64_t max_y_pos_unsigned;  // ((int64_t)gran_params->SD_size_Y_SU * gran_params->nSDs_Y)
    int64_t max_z_pos_unsigned;  //((int64_t)gran_params->SD_size_Z_SU * gran_params->nSDs_Z)

    float gravAcc_X_SU;  //!< Device counterpart of the constant gravity_X_SU
    float gravAcc_Y_SU;  //!< Device counterpart of the constant gravity_Y_SU
    float gravAcc_Z_SU;  //!< Device counterpart of the constant gravity_Z_SU
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

    /// Ratio of cohesion acceleration to gravity
    float cohesion_ratio;

    /// Ratio of adhesion acceleration to gravity
    float adhesion_ratio_s2w;

    double LENGTH_UNIT;  //!< 1 / C_L. Any length expressed in SU is a multiple of LENGTH_UNIT
    double TIME_UNIT;    //!< 1 / C_T. Any time quanity in SU is measured as a positive multiple of TIME_UNIT
    double MASS_UNIT;    //!< 1 / C_M. Any mass quanity is measured as a positive multiple of MASS_UNIT.
};
}  // namespace granular
}  // namespace chrono

// Do two things: make the naming nicer and require a const pointer everywhere
typedef const chrono::granular::ChGranParams* GranParamsPtr;
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

    virtual void generateDEs();

    size_t getNumSpheres() { return nDEs; }

    /// Create an axis-aligned box BC
    size_t Create_BC_AABox(float hdims[3], float center[3], bool outward_normal);

    /// Create an axis-aligned sphere BC
    size_t Create_BC_Sphere(float center[3], float radius, bool outward_normal);

    /// Create an z-axis aligned cone
    size_t Create_BC_Cone_Z(float cone_tip[3], float slope, float hmax, float hmin, bool outward_normal);

    /// Create an z-axis aligned cone
    size_t Create_BC_Plane(float plane_pos[3], float plane_normal[3]);

    bool disable_BC_by_ID(size_t BC_id) {
        size_t max_id = BC_params_list_UU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to disable invalid BC ID %lu\n", BC_id);
            return false;
        }
        BC_params_list_UU.at(BC_id).active = false;
        BC_params_list_SU.at(BC_id).active = false;
        return true;
    }

    bool enable_BC_by_ID(size_t BC_id) {
        size_t max_id = BC_params_list_UU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to disable invalid BC ID %lu\n", BC_id);
            return false;
        }
        BC_params_list_UU.at(BC_id).active = true;
        BC_params_list_SU.at(BC_id).active = true;
        return true;
    }

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

    /// set up data structures and carry out pre-processing tasks
    virtual void setupSimulation();

    /// advance simulation by duration seconds in user units, return actual duration elapsed
    /// Requires initialize() to have been called
    virtual double advance_simulation(float duration);

    /// Get the maximum stiffness term in the system
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

    /// Set the ratio of adhesion to gravity for sphere to wall
    void set_Adhesion_ratio_S2W(float someValue) { adhesion_s2w_over_gravity = someValue; }

    /// Set the BD to be fixed or not, if fixed it will ignore any given position functions
    void set_BD_Fixed(bool fixed) { BD_is_fixed = fixed; }

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

    /// Copy back the sd device data and save it to a file for error checking on the priming kernel
    void checkSDCounts(std::string ofile, bool write_out, bool verbose);
    void writeFile(std::string ofile, unsigned int* deCounts);
    void writeFileUU(std::string ofile);
    void updateBDPosition(const float stepSize_SU);

  protected:
    /// Holds the friction mode for the system
    GRAN_FRICTION_MODE fric_mode;
    /// holds the sphere and BD-related params in unified memory
    ChGranParams* gran_params;

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
    std::vector<float, cudallocator<float>> sphere_Omega_X;
    std::vector<float, cudallocator<float>> sphere_Omega_Y;
    std::vector<float, cudallocator<float>> sphere_Omega_Z;

    std::vector<float, cudallocator<float>> sphere_force_X;
    std::vector<float, cudallocator<float>> sphere_force_Y;
    std::vector<float, cudallocator<float>> sphere_force_Z;

    /// store angular acceleration (axis and magnitude in one vector)
    std::vector<float, cudallocator<float>> sphere_ang_acc_X;
    std::vector<float, cudallocator<float>> sphere_ang_acc_Y;
    std::vector<float, cudallocator<float>> sphere_ang_acc_Z;

    /// used for chung integrator
    std::vector<float, cudallocator<float>> sphere_force_X_old;
    std::vector<float, cudallocator<float>> sphere_force_Y_old;
    std::vector<float, cudallocator<float>> sphere_force_Z_old;
    std::vector<float, cudallocator<float>> sphere_ang_acc_X_old;
    std::vector<float, cudallocator<float>> sphere_ang_acc_Y_old;
    std::vector<float, cudallocator<float>> sphere_ang_acc_Z_old;

    /// gravity in user units
    float X_accGrav;
    float Y_accGrav;
    float Z_accGrav;

    ///  gravity in sim units
    float gravity_X_SU;
    float gravity_Y_SU;
    float gravity_Z_SU;

    /// Entry "i" says how many spheres touch SD i
    std::vector<unsigned int, cudallocator<unsigned int>> SD_NumOf_DEs_Touching;

    /// Array containing the IDs of the spheres stored in the SDs associated with the box
    std::vector<unsigned int, cudallocator<unsigned int>> DEs_in_SD_composite;

    /// User provided maximum timestep in UU, used in adaptive timestepping
    float max_adaptive_step_UU = 1e-3;
    /// User provided fixed timestep in UU, used in FIXED timestepping
    float fixed_step_UU = 1e-4;
    /// Step size in SU, user can request a larger one but default is 1
    float stepSize_SU;

    /// The type of time stepping (fixed or adaptive) used in the simulation
    GRAN_TIME_STEPPING time_stepping;

    /// The type of time integrator used in the simulation
    GRAN_TIME_INTEGRATOR time_integrator;
    GRAN_CONTACT_MODEL contact_model;  //!< DEM local contact force model

    /// Partitions the big domain (BD) and sets the number of SDs that BD is split in.
    void partitionBD();

    /// Copy constant parameter data to device
    void copyConstSphereDataToDevice();
    /// Copy (potentially updated) BD frame to device
    void copyBDFrameToDevice();

    /// Reset binning and broadphase info
    void resetBroadphaseInformation();
    /// Reset sphere-sphere forces
    void resetSphereForces();

    /// collect all the sphere data into a given struct
    void packSphereDataPointers(sphereDataStruct& packed);
    /// Run the first sphere broadphase pass to get things started
    void runInitialSpherePriming();
    // just a handy helper function
    template <typename T1, typename T2>
    T1 convertToPosSU(T2 val) {
        return val / gran_params->LENGTH_UNIT;
    }

    /// convert all BCs from UU to SU
    void convertBCUnits();

    /// Convert unit parameters from UU to SU
    virtual void switchToSimUnits();

    /// sort data based on owner SD
    virtual void defragment_data();

    const float new_step_freq = .01;
    virtual void determineNewStepSize_SU();

    /// Store the prescribed position function for the BD, used for moving frames
    // Default is at rest
    std::function<double(double)> BDPositionFunctionX = [](double a) { return 0; };
    std::function<double(double)> BDPositionFunctionY = [](double a) { return 0; };
    std::function<double(double)> BDPositionFunctionZ = [](double a) { return 0; };

    /// Total time elapsed since beginning of simulation
    float elapsedSimTime;
    /// Max velocity of all particles in system
    float get_max_vel();

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

    /// Store the ratio of the acceleration due to adhesion vs the acceleration due to gravity
    float adhesion_s2w_over_gravity;

    /// List of generalized BCs that constrain sphere motion
    std::vector<BC_type, cudallocator<BC_type>> BC_type_list;
    std::vector<BC_params_t<int, int3>, cudallocator<BC_params_t<int, int3>>> BC_params_list_SU;
    std::vector<BC_params_t<float, float3>, cudallocator<BC_params_t<float, float3>>> BC_params_list_UU;

    /// User defined radius of the sphere
    float sphere_radius;
    /// User defined density of the sphere
    float sphere_density;

    /// length of physical box; defines the global X axis located at the CM of the box
    float box_size_X;
    /// depth of physical box; defines the global Y axis located at the CM of the box
    float box_size_Y;
    /// height of physical box; defines the global Z axis located at the CM of the box
    float box_size_Z;

    /// Size of the sphere radius in Simulation Units
    unsigned int sphereRadius_SU;

    /// Size of the SD in the X direction (expressed in Simulation Units)
    unsigned int SD_size_X_SU;
    /// Size of the SD in the Y direction (expressed in Simulation Units)
    unsigned int SD_size_Y_SU;
    /// Size of the SD in the Z direction (expressed in Simulation Units)
    unsigned int SD_size_Z_SU;

    /// Number of SDs along the X dimension of the box
    unsigned int nSDs_X;
    /// Number of SDs along the Y dimension of the box
    unsigned int nSDs_Y;
    /// Number of SDs along the Z dimension of the box
    unsigned int nSDs_Z;

    /// User-provided sphere positions in UU
    std::vector<ChVector<float>> user_sphere_positions;

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
