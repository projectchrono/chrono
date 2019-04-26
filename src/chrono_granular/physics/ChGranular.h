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
#include "chrono/core/ChMathematics.h"
#include "cudalloc.hpp"

typedef unsigned char not_stupid_bool;

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

// use to compute position as a function of time
typedef std::function<double3(float)> GranPositionFunction;

// position function representing no motion or offset
const GranPositionFunction GranPosFunction_default = [](float t) { return make_double3(0, 0, 0); };

/// hold pointers
struct ChGranSphereData {
  public:
    /// Store positions and velocities in unified memory
    int* sphere_local_pos_X;
    int* sphere_local_pos_Y;
    int* sphere_local_pos_Z;
    float* pos_X_dt;
    float* pos_Y_dt;
    float* pos_Z_dt;

    /// store angular velocity (axis and magnitude in one vector)
    float* sphere_Omega_X;  // Could these just be a single float3 array?
    float* sphere_Omega_Y;
    float* sphere_Omega_Z;

    float* sphere_acc_X;
    float* sphere_acc_Y;
    float* sphere_acc_Z;

    /// store angular acceleration (axis and magnitude in one vector)
    float* sphere_ang_acc_X;
    float* sphere_ang_acc_Y;
    float* sphere_ang_acc_Z;

    /// used for multistep integrators
    float* sphere_acc_X_old;
    float* sphere_acc_Y_old;
    float* sphere_acc_Z_old;
    float* sphere_ang_acc_X_old;
    float* sphere_ang_acc_Y_old;
    float* sphere_ang_acc_Z_old;

    /// set of data for sphere contact pairs
    unsigned int* contact_partners_map;
    not_stupid_bool* contact_active_map;
    float3* contact_history_map;

    /// number of DEs touching each SD
    unsigned int* SD_NumSpheresTouching;
    /// offset of each SD in the big composite array
    unsigned int* SD_SphereCompositeOffsets;
    /// big composite array of sphere-SD membership
    unsigned int* spheres_in_SD_composite;

    /// list of owner SDs for each sphere
    unsigned int* sphere_owner_SDs;
};

// How are we writing?
enum GRAN_OUTPUT_MODE { CSV, BINARY, NONE };
// How are we stepping through time?
enum GRAN_TIME_STEPPING { ADAPTIVE, FIXED };
/// How are we integrating w.r.t. time
enum GRAN_TIME_INTEGRATOR { FORWARD_EULER, CHUNG, CENTERED_DIFFERENCE, EXTENDED_TAYLOR };

enum GRAN_FRICTION_MODE { FRICTIONLESS, SINGLE_STEP, MULTI_STEP };

enum GRAN_ROLLING_MODE { NO_RESISTANCE, CONSTANT_TORQUE, VISCOUS, ELASTIC_PLASTIC };

enum GRAN_INPUT_MODE { USER, CHECKPOINT_POSITION, CHECKPOINT_FULL };

/// Parameters needed for sphere-based granular dynamics
struct ChGranParams {
    // Timestep in SU
    float stepSize_SU;

    // settings on friction, integrator, and force model
    GRAN_FRICTION_MODE friction_mode;
    GRAN_ROLLING_MODE rolling_mode;
    GRAN_TIME_INTEGRATOR time_integrator;

    // ratio of normal force to peak tangent force, also arctan(theta) where theta is the friction angle
    float static_friction_coeff;

    // Coefficient of rolling resistance
    float rolling_coeff_SU;

    // Use user-defined quantities for coefficients
    // sphere-to-sphere contact damping coefficient, expressed in SU
    float Gamma_n_s2s_SU;
    // sphere-to-wall contact damping coefficient, expressed in SU
    float Gamma_n_s2w_SU;
    float Gamma_t_s2s_SU;
    float Gamma_t_s2w_SU;

    // normal stiffness coefficient, expressed in SU: sphere-to-sphere
    float K_n_s2s_SU;
    // normal stiffness coefficient, expressed in SU: sphere-to-wall
    float K_n_s2w_SU;
    float K_t_s2s_SU;
    float K_t_s2w_SU;

    /// Radius of the sphere, expressed in SU
    unsigned int sphereRadius_SU;
    /// Moment of inertia of a sphere, normalized by the radius
    float sphereInertia_by_r;

    /// X-dimension of the SD box, expressed in SU
    unsigned int SD_size_X_SU;
    /// Y-dimension of the SD box, expressed in SU
    unsigned int SD_size_Y_SU;
    /// Z-dimension of the SD box, expressed in SU
    unsigned int SD_size_Z_SU;

    /// total number of spheres in system, used for BC multistep friction
    unsigned int nSpheres;

    /// Total number of subdomains
    unsigned int nSDs;
    /// X-dimension of the BD box in multiples of subdomains, expressed in SU
    unsigned int nSDs_X;
    /// Y-dimension of the BD box in multiples of subdomains, expressed in SU
    unsigned int nSDs_Y;
    /// Z-dimension of the BD box in multiples of subdomains, expressed in SU
    unsigned int nSDs_Z;

    /// These are the max X, Y, Z dimensions in the BD frame
    int64_t max_x_pos_unsigned;  // ((int64_t)SD_size_X_SU * nSDs_X)
    int64_t max_y_pos_unsigned;  // ((int64_t)SD_size_Y_SU * nSDs_Y)
    int64_t max_z_pos_unsigned;  // ((int64_t)SD_size_Z_SU * nSDs_Z)

    float gravAcc_X_SU;  //!< Device counterpart of the constant gravity_X_SU
    float gravAcc_Y_SU;  //!< Device counterpart of the constant gravity_Y_SU
    float gravAcc_Z_SU;  //!< Device counterpart of the constant gravity_Z_SU

    /// The bottom-left corner xPos of the BD
    int64_t BD_frame_X;
    /// The bottom-left corner yPos of the BD
    int64_t BD_frame_Y;
    /// The bottom-left corner zPos of the BD
    int64_t BD_frame_Z;

    /// The offset of the BD from its original frame, used to allow the SD definitions to move
    int64_t BD_offset_X;
    /// The offset of the BD from its original frame, used to allow the SD definitions to move
    int64_t BD_offset_Y;
    /// The offset of the BD from its original frame, used to allow the SD definitions to move
    int64_t BD_offset_Z;

    /// Acceleration of cohesion
    float cohesionAcc_s2s;

    /// Accleration of adhesion
    float adhesionAcc_s2w;

    double LENGTH_UNIT;  //!< 1 / C_L. Any length expressed in SU is a multiple of LENGTH_UNIT
    double TIME_UNIT;    //!< 1 / C_T. Any time quantity in SU is measured as a positive multiple of TIME_UNIT
    double MASS_UNIT;    //!< 1 / C_M. Any mass quantity is measured as a positive multiple of MASS_UNIT.

    /// this is to make clear that the underlying assumption is unit SU mass
    constexpr static float sphere_mass_SU = 1.f;

    float max_safe_vel = (float)UINT_MAX;
};
}  // namespace granular
}  // namespace chrono

// Do two things: make the naming nicer and require a const pointer everywhere
typedef const chrono::granular::ChGranParams* GranParamsPtr;
typedef const chrono::granular::ChGranSphereData* GranSphereDataPtr;
namespace chrono {
namespace granular {
class CH_GRANULAR_API ChSystemGranular_MonodisperseSMC {
  public:
    // we do not want the system to be default-constructible
    ChSystemGranular_MonodisperseSMC() = delete;
    ChSystemGranular_MonodisperseSMC(float radiusSPH, float density, float3 boxDims);
    virtual ~ChSystemGranular_MonodisperseSMC();

    unsigned int get_SD_count() const { return nSDs; }
    void set_gravitational_acceleration(float xVal, float yVal, float zVal) {
        X_accGrav = xVal;
        Y_accGrav = yVal;
        Z_accGrav = zVal;
    }
    // /// write sphere data to file for later loading
    // void writeCheckpoint(std::string ofile) const;
    //
    // /// load sphere data from a file
    // void loadCheckpoint(std::string infile);

    size_t getNumSpheres() const { return nSpheres; }

    size_t estimateMemUsage() const;

    /// Create an axis-aligned box BC
    // size_t Create_BC_AABox(float hdims[3], float center[3], bool outward_normal);

    /// Create an axis-aligned sphere BC
    size_t Create_BC_Sphere(float center[3], float radius, bool outward_normal, bool track_forces);

    /// Create an z-axis aligned cone
    size_t Create_BC_Cone_Z(float cone_tip[3],
                            float slope,
                            float hmax,
                            float hmin,
                            bool outward_normal,
                            bool track_forces);

    /// Create an z-axis aligned cone
    size_t Create_BC_Plane(float plane_pos[3], float plane_normal[3], bool track_forces);

    /// Create an z-axis aligned cylinder
    size_t Create_BC_Cyl_Z(float center[3], float radius, bool outward_normal, bool track_forces);

    /// Create big domain walls out of planes
    void createWallBCs();

    /// disable a BC by its ID, returns false if the BC does not exist
    bool disable_BC_by_ID(size_t BC_id) {
        size_t max_id = BC_params_list_SU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to disable invalid BC ID %lu\n", BC_id);
            return false;
        }

        if (BC_id <= NUM_RESERVED_BC_IDS - 1) {
            printf("ERROR: Trying to modify reserved BC ID %lu\n", BC_id);
            return false;
        }
        BC_params_list_UU.at(BC_id).active = false;
        BC_params_list_SU.at(BC_id).active = false;
        return true;
    }

    /// enable a BC by its ID, returns false if the BC does not exist
    bool enable_BC_by_ID(size_t BC_id) {
        size_t max_id = BC_params_list_SU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to enable invalid BC ID %lu\n", BC_id);
            return false;
        }
        if (BC_id <= NUM_RESERVED_BC_IDS - 1) {
            printf("ERROR: Trying to modify reserved BC ID %lu\n", BC_id);
            return false;
        }
        BC_params_list_UU.at(BC_id).active = true;
        BC_params_list_SU.at(BC_id).active = true;
        return true;
    }

    /// enable a BC by its ID, returns false if the BC does not exist
    bool set_BC_offset_function(size_t BC_id, const GranPositionFunction& offset_function) {
        size_t max_id = BC_params_list_SU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to set offset function for invalid BC ID %lu\n", BC_id);
            return false;
        }
        if (BC_id <= NUM_RESERVED_BC_IDS - 1) {
            printf("ERROR: Trying to modify reserved BC ID %lu\n", BC_id);
            return false;
        }
        BC_offset_function_list.at(BC_id) = offset_function;
        BC_params_list_UU.at(BC_id).fixed = false;
        BC_params_list_SU.at(BC_id).fixed = false;
        return true;
    }

    /// get the reaction forces on a BC by ID, returns false if the forces are invalid (bad BC ID)
    bool getBCReactionForces(size_t BC_id, float forces[3]) const {
        size_t max_id = BC_params_list_SU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to get forces for invalid BC ID %lu\n", BC_id);
            return false;
        }
        if (BC_id <= NUM_RESERVED_BC_IDS - 1) {
            printf("ERROR: Trying to modify reserved BC ID %lu\n", BC_id);
            return false;
        }
        if (BC_params_list_SU.at(BC_id).track_forces == false) {
            printf("ERROR: Trying to get forces for non-force-tracking BC ID %lu\n", BC_id);
            return false;
        }
        if (BC_params_list_SU.at(BC_id).active == false) {
            printf("ERROR: Trying to get forces for inactive BC ID %lu\n", BC_id);
            return false;
        }
        float3 reaction_forces = BC_params_list_SU.at(BC_id).reaction_forces;

        // conversion from SU to UU force
        forces[0] = reaction_forces.x * FORCE_SU2UU;
        forces[1] = reaction_forces.y * FORCE_SU2UU;
        forces[2] = reaction_forces.z * FORCE_SU2UU;
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
    void set_timeIntegrator(GRAN_TIME_INTEGRATOR new_integrator) {
        gran_params->time_integrator = new_integrator;
        time_integrator = new_integrator;
    }

    void set_friction_mode(GRAN_FRICTION_MODE new_mode) {
        gran_params->friction_mode = new_mode;
        friction_mode = new_mode;
    }

    void set_rolling_mode(GRAN_ROLLING_MODE new_mode) {
        gran_params->rolling_mode = new_mode;
        rolling_mode = new_mode;
    }

    /// get the max z position of the spheres, this allows us to do easier cosimulation
    double get_max_z() const;

    /// advance simulation by duration seconds in user units, return actual duration elapsed
    /// Requires initialize() to have been called
    virtual double advance_simulation(float duration);

    /// Initialize simulation so that it can be advanced
    virtual void initialize();

    void set_static_friction_coeff(float mu) { gran_params->static_friction_coeff = mu; }
    void set_rolling_coeff(float mu) { rolling_coeff_UU = mu; }

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

    void setParticlePositions(const std::vector<ChVector<float>>& points);

    GranPositionFunction BDOffsetFunction;

    /// Prescribe the motion of the BD, allows wavetank-style simulations
    void setBDWallsMotionFunction(const GranPositionFunction& pos_fn) {
        BDOffsetFunction = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_X_BOT) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_X_TOP) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_Y_BOT) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_Y_TOP) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_Z_BOT) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_Z_TOP) = pos_fn;
    }

    void setPsiFactors(unsigned int psi_T_new, unsigned int psi_h_new, unsigned int psi_L_new) {
        psi_T = psi_T_new;
        psi_h = psi_h_new;
        psi_L = psi_L_new;
    }

    /// Copy back the sd device data and save it to a file for error checking on the priming kernel
    void checkSDCounts(std::string ofile, bool write_out, bool verbose) const;
    void writeFile(std::string ofile) const;

    void setMaxSafeVelocity_SU(float max_vel) { gran_params->max_safe_vel = max_vel; }

  protected:
    // Conversion factors from SU to UU
    /// 1 / C_L. Any length expressed in SU is a multiple of SU2UU
    double LENGTH_SU2UU;
    /// 1 / C_T. Any time quantity in SU is measured as a positive multiple of TIME_SU2UU
    double TIME_SU2UU;
    /// 1 / C_M. Any mass quantity is measured as a positive multiple of MASS_SU2UU.
    double MASS_SU2UU;
    /// 1 / C_F. Any force quantity is measured as a multiple of FORCE_SU2UU.
    double FORCE_SU2UU;
    /// 1 / C_tau. Any torque quantity is measured as a multiple of TORQUE_SU2UU.
    double TORQUE_SU2UU;
    /// 1 / C_v. Any velocity quantity is measured as a multiple of VEL_SU2UU.
    double VEL_SU2UU;

    unsigned int psi_T;
    unsigned int psi_h;
    unsigned int psi_L;

    /// Wrap the device helper function
    int3 getSDTripletFromID(unsigned int SD_ID) const;

    /// Create a helper to do sphere initialization
    void initializeSpheres();

    void defragment_initial_positions();

    /// Setup sphere data, initialize local coords
    void setupSphereDataStructures();

    // /// whether we are using user-provided or checkpointed spheres
    // bool load_checkpoint;
    //
    // /// what checkpoint file to use, if any
    // std::string checkpoint_file;

    /// holds the sphere and BD-related params in unified memory
    ChGranParams* gran_params;
    ChGranSphereData* sphere_data;

    /// Allows the code to be very verbose for debug
    bool verbose_runtime;
    /// How to write the output files? Default is CSV
    GRAN_OUTPUT_MODE file_write_mode;
    /// Directory to write to, this code assumes it already exists
    std::string output_directory;

    /// Number of discrete elements
    unsigned int nSpheres;
    /// Number of subdomains
    unsigned int nSDs;

    // Use CUDA allocator written by Colin, could hit system performance if there's not a lot of RAM
    // Makes somewhat faster memcpys
    /// Store positions and velocities in unified memory
    std::vector<int, cudallocator<int>> sphere_local_pos_X;
    std::vector<int, cudallocator<int>> sphere_local_pos_Y;
    std::vector<int, cudallocator<int>> sphere_local_pos_Z;
    std::vector<float, cudallocator<float>> pos_X_dt;
    std::vector<float, cudallocator<float>> pos_Y_dt;
    std::vector<float, cudallocator<float>> pos_Z_dt;

    /// store angular velocity (axis and magnitude in one vector)
    std::vector<float, cudallocator<float>> sphere_Omega_X;
    std::vector<float, cudallocator<float>> sphere_Omega_Y;
    std::vector<float, cudallocator<float>> sphere_Omega_Z;

    std::vector<float, cudallocator<float>> sphere_acc_X;
    std::vector<float, cudallocator<float>> sphere_acc_Y;
    std::vector<float, cudallocator<float>> sphere_acc_Z;

    /// store angular acceleration (axis and magnitude in one vector)
    std::vector<float, cudallocator<float>> sphere_ang_acc_X;
    std::vector<float, cudallocator<float>> sphere_ang_acc_Y;
    std::vector<float, cudallocator<float>> sphere_ang_acc_Z;

    /// used for chung integrator
    std::vector<float, cudallocator<float>> sphere_acc_X_old;
    std::vector<float, cudallocator<float>> sphere_acc_Y_old;
    std::vector<float, cudallocator<float>> sphere_acc_Z_old;
    std::vector<float, cudallocator<float>> sphere_ang_acc_X_old;
    std::vector<float, cudallocator<float>> sphere_ang_acc_Y_old;
    std::vector<float, cudallocator<float>> sphere_ang_acc_Z_old;

    std::vector<unsigned int, cudallocator<unsigned int>> contact_partners_map;
    std::vector<not_stupid_bool, cudallocator<not_stupid_bool>> contact_active_map;
    std::vector<float3, cudallocator<float3>> contact_history_map;

    /// gravity in user units
    float X_accGrav;
    float Y_accGrav;
    float Z_accGrav;

    /// Entry "i" says how many spheres touch SD i
    std::vector<unsigned int, cudallocator<unsigned int>> SD_NumSpheresTouching;
    std::vector<unsigned int, cudallocator<unsigned int>> SD_SphereCompositeOffsets;

    /// Array containing the IDs of the spheres stored in the SDs associated with the box
    std::vector<unsigned int, cudallocator<unsigned int>> spheres_in_SD_composite;

    /// list of owner SDs for each sphere
    std::vector<unsigned int, cudallocator<unsigned int>> sphere_owner_SDs;

    /// User provided maximum timestep in UU, used in adaptive timestepping
    float max_adaptive_step_UU = 1e-3;
    /// User provided fixed timestep in UU, used in FIXED timestepping
    float fixed_step_UU = 1e-4;
    /// Step size in SU, user can request a larger one but default is 1
    float stepSize_SU;

    /// The type of time stepping (fixed or adaptive) used in the simulation
    GRAN_TIME_STEPPING time_stepping;

    GRAN_FRICTION_MODE friction_mode;
    GRAN_ROLLING_MODE rolling_mode;
    GRAN_TIME_INTEGRATOR time_integrator;

    /// Partitions the big domain (BD) and sets the number of SDs that BD is split in.
    void partitionBD();

    /// Copy constant parameter data to device
    void copyConstSphereDataToDevice();

    /// Reset binning and broadphase info
    void resetBroadphaseInformation();
    /// Reset sphere accelerations
    void resetSphereAccelerations();

    /// Reset sphere-wall forces
    void resetBCForces();

    /// collect all the sphere data into the member struct
    void packSphereDataPointers();

    /// Run the first sphere broadphase pass to get things started
    void runSphereBroadphase();
    // just a handy helper function
    template <typename T1, typename T2>
    T1 convertToPosSU(T2 val) {
        return val / gran_params->LENGTH_UNIT;
    }

    /// convert all BCs from UU to SU
    void convertBCUnits();

    /// Max velocity of all particles in system
    float get_max_vel() const;

    /// Get the maximum stiffness term in the system
    virtual double get_max_K() const;

    /// Convert unit parameters from UU to SU
    virtual void switchToSimUnits();

    /// sort data based on owner SD
    virtual void defragment_data();

    const float new_step_freq = .01;
    virtual void determineNewStepSize_SU();

    /// set the position of a BC and account for the offset
    void setBCOffset(const BC_type&,
                     const BC_params_t<float, float3>& params_UU,
                     BC_params_t<int64_t, int64_t3>& params_SU,
                     double3 offset_UU);

    /// update positions of each BC using prescribed functions
    void updateBCPositions();

    /// Total time elapsed since beginning of simulation
    float elapsedSimTime;

    /// size of the normal stiffness (UU) for sphere-to-sphere contact
    double K_n_s2s_UU;

    /// size of the normal stiffness (UU) for sphere-to-wall contact
    double K_n_s2w_UU;

    /// normal damping for sphere-to-sphere
    double Gamma_n_s2s_UU;

    /// normal damping for sphere-to-wall
    double Gamma_n_s2w_UU;

    /// tangential stiffness for sphere-to-sphere
    double K_t_s2s_UU;

    /// tangential stiffness for sphere-to-wall
    double K_t_s2w_UU;

    /// tangential damping for sphere-to-sphere
    double Gamma_t_s2s_UU;

    /// tangential damping for sphere-to-wall
    double Gamma_t_s2w_UU;

    /// rolling friction coefficient
    double rolling_coeff_UU;

    /// Store the ratio of the acceleration due to cohesion vs the acceleration due to gravity, makes simple API
    float cohesion_over_gravity;

    /// Store the ratio of the acceleration due to adhesion vs the acceleration due to gravity
    float adhesion_s2w_over_gravity;

    /// The reference point for UU to SU local coordinates
    int64_t3 BD_rest_frame_SU;

    /// List of generalized BCs that constrain sphere motion
    std::vector<BC_type, cudallocator<BC_type>> BC_type_list;
    std::vector<BC_params_t<int64_t, int64_t3>, cudallocator<BC_params_t<int64_t, int64_t3>>> BC_params_list_SU;
    std::vector<BC_params_t<float, float3>, cudallocator<BC_params_t<float, float3>>> BC_params_list_UU;
    std::vector<GranPositionFunction> BC_offset_function_list;

    /// User defined radius of the sphere
    const float sphere_radius_UU;
    /// User defined density of the sphere
    const float sphere_density_UU;

    /// length of physical box; defines the global X axis located at the CM of the box
    const float box_size_X;
    /// depth of physical box; defines the global Y axis located at the CM of the box
    const float box_size_Y;
    /// height of physical box; defines the global Z axis located at the CM of the box
    const float box_size_Z;

    /// User-provided sphere positions in UU
    std::vector<ChVector<float>> user_sphere_positions;

    /// Allow the user to set the BD to be fixed, ignoring any given position functions
    bool BD_is_fixed = true;
};

}  // namespace granular
}  // namespace chrono
