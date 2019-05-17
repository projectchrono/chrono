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
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>
#include <algorithm>
// make windows behave with math
#define _USE_MATH_DEFINES
#include <cmath>
#include "chrono_granular/ChApiGranular.h"
#include "chrono/core/ChVector.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranularBoundaryConditions.h"
#include "chrono/core/ChMathematics.h"
#include "cudalloc.hpp"

typedef unsigned char not_stupid_bool;

namespace chrono {
namespace granular {

/// @addtogroup granular_physics
/// @{

/// Used to compute position as a function of time
typedef std::function<double3(float)> GranPositionFunction;

/// Position function representing no motion or offset as a funtion of time
const GranPositionFunction GranPosFunction_default = [](float t) { return make_double3(0, 0, 0); };

/// Hold pointers to kinematic quantities of the granular system
struct ChGranSphereData {
  public:
    /// X position relative to owner subdomain in unified memory
    int* sphere_local_pos_X;
    /// Y position relative to owner subdomain in unified memory
    int* sphere_local_pos_Y;
    /// Z position relative to owner subdomain in unified memory
    int* sphere_local_pos_Z;

    /// X velocity in unified memory
    float* pos_X_dt;
    /// Y velocity in unified memory
    float* pos_Y_dt;
    /// Z velocity in unified memory
    float* pos_Z_dt;

    /// X angular velocity in unified memory
    float* sphere_Omega_X;
    /// Y angular velocity in unified memory
    float* sphere_Omega_Y;
    /// Z angular velocity in unified memory
    float* sphere_Omega_Z;

    /// X angular acceleration in unified memory
    float* sphere_acc_X;
    /// Y angular acceleration in unified memory
    float* sphere_acc_Y;
    /// Z angular acceleration in unified memory
    float* sphere_acc_Z;

    /// X angular acceleration in unified memory
    float* sphere_ang_acc_X;
    /// Y angular acceleration in unified memory
    float* sphere_ang_acc_Y;
    /// Z angular acceleration in unified memory
    float* sphere_ang_acc_Z;

    /// Previous step X acceleration for multistep integrators
    float* sphere_acc_X_old;
    /// Previous step Y acceleration for multistep integrators
    float* sphere_acc_Y_old;
    /// Previous step Z acceleration for multistep integrators
    float* sphere_acc_Z_old;

    /// Previous step X angular acceleration for multistep integrators
    float* sphere_ang_acc_X_old;
    /// Previous step Y angular acceleration for multistep integrators
    float* sphere_ang_acc_Y_old;
    /// Previous step Z angular acceleration for multistep integrators
    float* sphere_ang_acc_Z_old;

    /// Set of data for sphere contact pairs
    unsigned int* contact_partners_map;
    not_stupid_bool* contact_active_map;
    float3* contact_history_map;

    /// Number of particles touching each subdomain
    unsigned int* SD_NumSpheresTouching;
    /// Offset of each subdomain in the big composite array
    unsigned int* SD_SphereCompositeOffsets;
    /// Big composite array of sphere-subdomain membership
    unsigned int* spheres_in_SD_composite;

    /// List of owner subdomains for each sphere
    unsigned int* sphere_owner_SDs;
};

/// Output mode of system
enum GRAN_OUTPUT_MODE { CSV, BINARY, NONE };
/// Allows adaptive time stepping -- CAREFUL, NOT WELL TESTED
enum GRAN_TIME_STEPPING { ADAPTIVE, FIXED };
/// How are we integrating w.r.t. time
enum GRAN_TIME_INTEGRATOR { FORWARD_EULER, CHUNG, CENTERED_DIFFERENCE, EXTENDED_TAYLOR };

enum GRAN_FRICTION_MODE { FRICTIONLESS, SINGLE_STEP, MULTI_STEP };
/// Rolling resistance models -- ELASTIC_PLASTIC not implemented yet
enum GRAN_ROLLING_MODE { NO_RESISTANCE, CONSTANT_TORQUE, VISCOUS, ELASTIC_PLASTIC };

enum GRAN_INPUT_MODE { USER, CHECKPOINT_POSITION, CHECKPOINT_FULL };

/// Parameters needed for sphere-based granular dynamics
struct ChGranParams {
    /// Timestep in SU
    float stepSize_SU;

    // settings on friction, integrator, and force model
    GRAN_FRICTION_MODE friction_mode;

    GRAN_ROLLING_MODE rolling_mode;

    GRAN_TIME_INTEGRATOR time_integrator;

    /// Ratio of normal force to peak tangent force, also arctan(theta) where theta is the friction angle
    /// sphere-to-sphere
    float static_friction_coeff_s2s;
    /// Ratio of normal force to peak tangent force, also arctan(theta) where theta is the friction angle
    /// sphere-to-wall
    float static_friction_coeff_s2w;

    /// Coefficient of rolling resistance sphere-to-sphere
    float rolling_coeff_s2s_SU;
    /// Coefficient of rolling resistance sphere-to-wall
    float rolling_coeff_s2w_SU;

    /// sphere-to-sphere normal contact damping coefficient, expressed in SU
    float Gamma_n_s2s_SU;
    /// sphere-to-wall normal contact damping coefficient, expressed in SU
    float Gamma_n_s2w_SU;
    /// sphere-to-sphere tangent contact damping coefficient, expressed in SU
    float Gamma_t_s2s_SU;
    /// sphere-to-wall tangent contact damping coefficient, expressed in SU
    float Gamma_t_s2w_SU;

    /// sphere-to-sphere normal contact stiffness, expressed in SU
    float K_n_s2s_SU;
    /// sphere-to-wall normal contact stiffness, expressed in SU
    float K_n_s2w_SU;
    /// sphere-to-sphere tangent contact stiffness, expressed in SU
    float K_t_s2s_SU;
    /// sphere-to-wall tangent contact stiffness, expressed in SU
    float K_t_s2w_SU;

    /// Radius of the sphere, expressed in SU
    unsigned int sphereRadius_SU;
    /// Moment of inertia of a sphere, normalized by the radius, expressed in SU
    float sphereInertia_by_r;

    /// X-dimension of each subdomain box, expressed in SU
    unsigned int SD_size_X_SU;
    /// Y-dimension of each subdomain box, expressed in SU
    unsigned int SD_size_Y_SU;
    /// Z-dimension of each subdomain box, expressed in SU
    unsigned int SD_size_Z_SU;

    /// Total number of spheres in system, used for boundary condition multistep friction
    unsigned int nSpheres;

    /// Total number of subdomains
    unsigned int nSDs;

    /// X-dimension of the big domain box in multiples of subdomains
    unsigned int nSDs_X;
    /// Y-dimension of the big domain box in multiples of subdomains
    unsigned int nSDs_Y;
    /// Z-dimension of the big domain box in multiples of subdomains
    unsigned int nSDs_Z;

    /// Maximum X dimension in the big domain frame
    int64_t max_x_pos_unsigned;  // ((int64_t)SD_size_X_SU * nSDs_X)
    /// Maximum Y dimension in the big domain frame
    int64_t max_y_pos_unsigned;  // ((int64_t)SD_size_Y_SU * nSDs_Y)
    /// Maximum Z dimension in the big domain frame
    int64_t max_z_pos_unsigned;  // ((int64_t)SD_size_Z_SU * nSDs_Z)

    /// X gravity in SU
    float gravAcc_X_SU;
    /// Y gravity in SU
    float gravAcc_Y_SU;
    /// Z gravity in SU
    float gravAcc_Z_SU;

    /// The bottom-left corner xPos of the big domain
    int64_t BD_frame_X;
    /// The bottom-left corner yPos of the big domain
    int64_t BD_frame_Y;
    /// The bottom-left corner zPos of the big domain
    int64_t BD_frame_Z;

    /// The offset of the big domain from its original frame, used to allow the subdomain definitions to move
    int64_t BD_offset_X;
    /// The offset of the big domain from its original frame, used to allow the subdomain definitions to move
    int64_t BD_offset_Y;
    /// The offset of the big domain from its original frame, used to allow the subdomain definitions to move
    int64_t BD_offset_Z;

    /// Constant acceleration of sphere-to-sphere cohesion
    float cohesionAcc_s2s;

    /// Accleration of adhesion
    float adhesionAcc_s2w;

    double LENGTH_UNIT;  //!< 1 / C_L. Any length expressed in SU is a multiple of LENGTH_UNIT
    double TIME_UNIT;    //!< 1 / C_T. Any time quantity in SU is measured as a positive multiple of TIME_UNIT
    double MASS_UNIT;    //!< 1 / C_M. Any mass quantity is measured as a positive multiple of MASS_UNIT.

    /// this is to make clear that the underlying assumption is unit SU mass
    constexpr static float sphere_mass_SU = 1.f;

    float max_safe_vel = (float)UINT_MAX;

    /// @} granular_physics
};
}  // namespace granular
}  // namespace chrono

// Do two things: make the naming nicer and require a const pointer everywhere
typedef const chrono::granular::ChGranParams* GranParamsPtr;
typedef const chrono::granular::ChGranSphereData* GranSphereDataPtr;
namespace chrono {
namespace granular {

/// @addtogroup granular_physics
/// @{

/**
 * \brief Main Chrono::Granular system class used to control and dispatch the GPU
 * sphere-only solver.
 */
class CH_GRANULAR_API ChSystemGranular_MonodisperseSMC {
  public:
    /// The system is not default-constructible
    ChSystemGranular_MonodisperseSMC() = delete;
    ChSystemGranular_MonodisperseSMC(float radiusSPH, float density, float3 boxDims);
    virtual ~ChSystemGranular_MonodisperseSMC();

    /// Return number of subdomains in the big domain
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

    /// Return total number of spheres in the system
    size_t getNumSpheres() const { return nSpheres; }

    /// Roughly estimates the total amount of memory used by the system
    size_t estimateMemUsage() const;

    /// Create an axis-aligned box BC
    // size_t Create_BC_AABox(float hdims[3], float center[3], bool outward_normal);

    /// Create an axis-aligned sphere boundary condition
    size_t Create_BC_Sphere(float center[3], float radius, bool outward_normal, bool track_forces);

    /// Create an z-axis aligned cone boundary condition
    size_t Create_BC_Cone_Z(float cone_tip[3],
                            float slope,
                            float hmax,
                            float hmin,
                            bool outward_normal,
                            bool track_forces);

    /// Create an z-axis aligned cone boundary condition
    size_t Create_BC_Plane(float plane_pos[3], float plane_normal[3], bool track_forces);

    /// Create an z-axis aligned cylinder boundary condition
    size_t Create_BC_Cyl_Z(float center[3], float radius, bool outward_normal, bool track_forces);

    /// Create big domain walls out of plane boundary conditions
    void createWallBCs();

    /// Disable a boundary condition by its ID, returns false if the BC does not exist
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

    /// Enable a boundary condition by its ID, returns false if the BC does not exist
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

    /// Enable a boundary condition by its ID, returns false if the BC does not exist
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

    /// Get the reaction forces on a boundary by ID, returns false if the forces are invalid (bad BC ID)
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

    /// Allows the user to request a step size, will find the closest SU size to it
    void set_max_adaptive_stepSize(float size_UU) { max_adaptive_step_UU = size_UU; }
    /// Set timestep size for the fixed stepsize mode
    void set_fixed_stepSize(float size_UU) { fixed_step_UU = size_UU; }
    /// Set the timestepper to be either fixed or adaptive
    void set_timeStepping(GRAN_TIME_STEPPING new_stepping) { time_stepping = new_stepping; }
    /// Set the time integration scheme for the system
    void set_timeIntegrator(GRAN_TIME_INTEGRATOR new_integrator) {
        gran_params->time_integrator = new_integrator;
        time_integrator = new_integrator;
    }

    /// Set friction formulation
    void set_friction_mode(GRAN_FRICTION_MODE new_mode) {
        gran_params->friction_mode = new_mode;
        this->friction_mode = new_mode;
    }

    /// Set rolling resistence formulation
    void set_rolling_mode(GRAN_ROLLING_MODE new_mode) { gran_params->rolling_mode = new_mode; }

    /// Get the max z position of the spheres, allows easier co-simulation
    double get_max_z() const;

    /// Advance simulation by duration seconds in user units, return actual duration elapsed
    /// Requires initialize() to have been called
    virtual double advance_simulation(float duration);

    /// Initialize simulation so that it can be advanced
    /// Must be called before advance_simulation and after simulation parameters are set
    virtual void initialize();

    /// Set sphere-to-sphere static friction coefficient
    void set_static_friction_coeff_SPH2SPH(float mu) { gran_params->static_friction_coeff_s2s = mu; }
    /// Set sphere-to-wall static friction coefficient
    void set_static_friction_coeff_SPH2WALL(float mu) { gran_params->static_friction_coeff_s2w = mu; }
    /// Set sphere-to-sphere rolling friction coefficient - units and use vary by rolling friction mode
    void set_rolling_coeff_SPH2SPH(float mu) { rolling_coeff_s2s_UU = mu; }
    /// Set sphere-to-wall rolling friction coefficient - units and use vary by rolling friction mode
    void set_rolling_coeff_SPH2WALL(float mu) { rolling_coeff_s2w_UU = mu; }

    /// Set sphere-to-sphere normal contact stiffness
    void set_K_n_SPH2SPH(double someValue) { K_n_s2s_UU = someValue; }
    /// Set sphere-to-wall normal contact stiffness
    void set_K_n_SPH2WALL(double someValue) { K_n_s2w_UU = someValue; }

    /// Set sphere-to-sphere normal damping coefficient
    void set_Gamma_n_SPH2SPH(double someValue) { Gamma_n_s2s_UU = someValue; }
    /// Set sphere-to-wall normal damping coefficient
    void set_Gamma_n_SPH2WALL(double someValue) { Gamma_n_s2w_UU = someValue; }

    /// Set sphere-to-sphere tangent contact stiffness
    void set_K_t_SPH2SPH(double someValue) { K_t_s2s_UU = someValue; }
    /// Set sphere-to-sphere tangent damping coefficient
    void set_Gamma_t_SPH2SPH(double someValue) { Gamma_t_s2s_UU = someValue; }

    /// Set sphere-to-wall tangent contact stiffness
    void set_K_t_SPH2WALL(double someValue) { K_t_s2w_UU = someValue; }
    /// Set sphere-to-wall tangent damping coefficient
    void set_Gamma_t_SPH2WALL(double someValue) { Gamma_t_s2w_UU = someValue; }

    /// Set the ratio of cohesion to gravity for monodisperse spheres
    /// Assumes a constant cohesion model
    void set_Cohesion_ratio(float someValue) { cohesion_over_gravity = someValue; }

    /// Set the ratio of adhesion to gravity for sphere to wall
    /// Assumes a constant cohesion model
    void set_Adhesion_ratio_S2W(float someValue) { adhesion_s2w_over_gravity = someValue; }

    /// Set the big domain to be fixed or not, if fixed it will ignore any given position functions
    void set_BD_Fixed(bool fixed) { BD_is_fixed = fixed; }

    /// Set initial particle positions
    /// Must be called only once and only before initialize
    void setParticlePositions(const std::vector<ChVector<float>>& points);

    GranPositionFunction BDOffsetFunction;

    /// Prescribe the motion of the big domain, allows wavetank-style simulations
    void setBDWallsMotionFunction(const GranPositionFunction& pos_fn) {
        BDOffsetFunction = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_X_BOT) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_X_TOP) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_Y_BOT) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_Y_TOP) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_Z_BOT) = pos_fn;
        BC_offset_function_list.at(BD_WALL_ID_Z_TOP) = pos_fn;
    }

    /// Set tuning psi factors for tuning the non-dimensionalization
    void setPsiFactors(unsigned int psi_T_new, unsigned int psi_h_new, unsigned int psi_L_new) {
        psi_T = psi_T_new;
        psi_h = psi_h_new;
        psi_L = psi_L_new;
    }

    /// Copy back the subdomain device data and save it to a file for error checking on the priming kernel
    void checkSDCounts(std::string ofile, bool write_out, bool verbose) const;
    /// Writes out particle positions according to the system output mode
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

    /// Tuning for non-dimensionalization
    unsigned int psi_T;
    unsigned int psi_h;
    unsigned int psi_L;

    /// Wrap the device helper function
    int3 getSDTripletFromID(unsigned int SD_ID) const;

    /// Create a helper to do sphere initialization
    void initializeSpheres();

    /// Sorts particle positions spatially in order to improve memory efficiency
    void defragment_initial_positions();

    /// Setup sphere data, initialize local coords
    void setupSphereDataStructures();

    // /// whether we are using user-provided or checkpointed spheres
    // bool load_checkpoint;
    //
    // /// what checkpoint file to use, if any
    // std::string checkpoint_file;

    /// Holds the sphere and big-domain-related params in unified memory
    ChGranParams* gran_params;
    /// Holds system degrees of freedom
    ChGranSphereData* sphere_data;

    /// Allows the code to be very verbose for debugging
    bool verbose_runtime;

    /// How to write the output files?
    /// Default is CSV
    GRAN_OUTPUT_MODE file_write_mode;
    /// Directory to write to, this code assumes it already exists
    std::string output_directory;

    /// Number of discrete elements
    unsigned int nSpheres;
    /// Number of subdomains
    unsigned int nSDs;

    // Use CUDA allocator written by Colin Vanden Heuvel
    // Could hit system performance if there's not a lot of RAM
    // Makes somewhat faster memcpys
    /// Store X positions relative to owner subdomain in unified memory
    std::vector<int, cudallocator<int>> sphere_local_pos_X;
    /// Store Y positions relative to owner subdomain in unified memory
    std::vector<int, cudallocator<int>> sphere_local_pos_Y;
    /// Store Z positions relative to owner subdomain in unified memory
    std::vector<int, cudallocator<int>> sphere_local_pos_Z;
    /// Store X velocity in unified memory
    std::vector<float, cudallocator<float>> pos_X_dt;
    /// Store Y velocity in unified memory
    std::vector<float, cudallocator<float>> pos_Y_dt;
    /// Store Z velocity in unified memory
    std::vector<float, cudallocator<float>> pos_Z_dt;

    /// Store X angular velocity (axis and magnitude in one vector) in unified memory
    std::vector<float, cudallocator<float>> sphere_Omega_X;
    /// Store Y angular velocity (axis and magnitude in one vector) in unified memory
    std::vector<float, cudallocator<float>> sphere_Omega_Y;
    /// Store Z angular velocity (axis and magnitude in one vector) in unified memory
    std::vector<float, cudallocator<float>> sphere_Omega_Z;

    /// Store X acceleration in unified memory
    std::vector<float, cudallocator<float>> sphere_acc_X;
    /// Store Y acceleration in unified memory
    std::vector<float, cudallocator<float>> sphere_acc_Y;
    /// Store Z acceleration in unified memory
    std::vector<float, cudallocator<float>> sphere_acc_Z;

    /// Store X angular acceleration (axis and magnitude in one vector) in unified memory
    std::vector<float, cudallocator<float>> sphere_ang_acc_X;
    /// Store Y angular acceleration (axis and magnitude in one vector) in unified memory
    std::vector<float, cudallocator<float>> sphere_ang_acc_Y;
    /// Store Z angular acceleration (axis and magnitude in one vector) in unified memory
    std::vector<float, cudallocator<float>> sphere_ang_acc_Z;

    /// X acceleration history used for the Chung integrator in unified memory
    std::vector<float, cudallocator<float>> sphere_acc_X_old;
    /// Y acceleration history used for the Chung integrator in unified memory
    std::vector<float, cudallocator<float>> sphere_acc_Y_old;
    /// Z acceleration history used for the Chung integrator in unified memory
    std::vector<float, cudallocator<float>> sphere_acc_Z_old;
    /// X angular acceleration history used for the Chung integrator in unified memory
    std::vector<float, cudallocator<float>> sphere_ang_acc_X_old;
    /// Y angular acceleration history used for the Chung integrator in unified memory
    std::vector<float, cudallocator<float>> sphere_ang_acc_Y_old;
    /// Z angular acceleration history used for the Chung integrator in unified memory
    std::vector<float, cudallocator<float>> sphere_ang_acc_Z_old;

    std::vector<unsigned int, cudallocator<unsigned int>> contact_partners_map;
    std::vector<not_stupid_bool, cudallocator<not_stupid_bool>> contact_active_map;
    std::vector<float3, cudallocator<float3>> contact_history_map;

    /// X gravity in user units
    float X_accGrav;
    /// Y gravity in user units
    float Y_accGrav;
    /// Z gravity in user units
    float Z_accGrav;

    /// Entry "i" says how many spheres touch subdomain i
    std::vector<unsigned int, cudallocator<unsigned int>> SD_NumSpheresTouching;
    std::vector<unsigned int, cudallocator<unsigned int>> SD_SphereCompositeOffsets;

    /// Array containing the IDs of the spheres stored in the SDs associated with the box
    std::vector<unsigned int, cudallocator<unsigned int>> spheres_in_SD_composite;

    /// List of owner subdomains for each sphere
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
    GRAN_FRICTION_MODE friction_mode_s2w;
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

    /// Collect all the sphere data into the member struct
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

    /// Sort data based on owner subdomain to improve memory performance
    virtual void defragment_data();

    const float new_step_freq = .01;
    /// Calculates the new optimal timestep size
    virtual void determineNewStepSize_SU();

    /// Set the position of a boundary condition and account for the offset
    void setBCOffset(const BC_type&,
                     const BC_params_t<float, float3>& params_UU,
                     BC_params_t<int64_t, int64_t3>& params_SU,
                     double3 offset_UU);

    /// Update positions of each boundary condition using prescribed functions
    void updateBCPositions();

    /// Total time elapsed since beginning of simulation
    float elapsedSimTime;

    /// Normal stiffness for sphere-to-sphere contact (Hertzian spring constant)
    double K_n_s2s_UU;

    /// Normal stiffness for sphere-to-wall contact (Hertzian spring constant)
    double K_n_s2w_UU;

    /// Normal damping for sphere-to-sphere
    double Gamma_n_s2s_UU;

    /// Normal damping for sphere-to-wall
    double Gamma_n_s2w_UU;

    /// Tangential stiffness for sphere-to-sphere (Hertzian spring constant
    double K_t_s2s_UU;

    /// Tangential stiffness for sphere-to-wall (Hertzian spring constant
    double K_t_s2w_UU;

    /// Tangential damping for sphere-to-sphere
    double Gamma_t_s2s_UU;

    /// Tangential damping for sphere-to-wall
    double Gamma_t_s2w_UU;

    /// Rolling friction coefficient for sphere-to-sphere
    double rolling_coeff_s2s_UU;

    /// Rolling friction coefficient for sphere-to-wall
    /// Units and use are dependent on the rolling friction model used
    double rolling_coeff_s2w_UU;

    /// Store the ratio of the acceleration due to cohesion vs the acceleration due to gravity, makes simple API
    float cohesion_over_gravity;

    /// Store the ratio of the acceleration due to adhesion vs the acceleration due to gravity
    float adhesion_s2w_over_gravity;

    /// The reference point for UU to SU local coordinates
    int64_t3 BD_rest_frame_SU;

    /// List of generalized boundary conditions that constrain sphere motion
    std::vector<BC_type, cudallocator<BC_type>> BC_type_list;
    std::vector<BC_params_t<int64_t, int64_t3>, cudallocator<BC_params_t<int64_t, int64_t3>>> BC_params_list_SU;
    std::vector<BC_params_t<float, float3>, cudallocator<BC_params_t<float, float3>>> BC_params_list_UU;
    std::vector<GranPositionFunction> BC_offset_function_list;

    /// User defined radius of the sphere
    const float sphere_radius_UU;
    /// User defined density of the sphere
    const float sphere_density_UU;

    /// X-length of the big domain; defines the global X axis located at the CM of the box
    const float box_size_X;
    /// Y-length of the big domain; defines the global Y axis located at the CM of the box
    const float box_size_Y;
    /// Z-length of the big domain; defines the global Z axis located at the CM of the box
    const float box_size_Z;

    /// User-provided sphere positions in UU
    std::vector<ChVector<float>> user_sphere_positions;

    /// Allow the user to set the big domain to be fixed, ignoring any given position functions
    bool BD_is_fixed = true;
};

/// @} granular_physics
}  // namespace granular
}  // namespace chrono