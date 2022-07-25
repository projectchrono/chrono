// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut, Radu Serban
// =============================================================================

#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "chrono_gpu/ChApiGpu.h"
#include "chrono_gpu/ChGpuDefines.h"
#include "chrono_gpu/physics/ChGpuBoundaryConditions.h"
#include "chrono_gpu/cuda/ChGpuCUDAalloc.hpp"

typedef unsigned char not_stupid_bool;

namespace chrono {
namespace gpu {

/// <summary>
/// ChSolverStateData contains information that pertains the solver, at a certain point in time. It contains
/// information about the current sim time, current time step, max number of spheres in each SD, etc.
/// This is the type of information that changes (or not) from time step to time step.
/// </summary>
class ChSolverStateData {
  private:
    /// variable stores at each time step the largest number of spheres in any domain; this is useful when deciding
    /// on the number of threads in a block for a kernel call. It is also useful to settle once and for all whether
    /// we exceed the threshold MAX_COUNT_OF_SPHERES_PER_SD. Avoids having these checks in the kernel. Stored in
    /// managed memory.
    unsigned int* pMaxNumberSpheresInAnySD;
    unsigned int crntMaxNumberSpheresInAnySD;  // redundant with info above, but info above is on the device
    unsigned int largestMaxNumberSpheresInAnySD_thusFar;

    /// vector of unsigned int that lives on the device; used by CUB or by anybody else that needs scrap space.
    /// Please pay attention to the type the vector stores.
    std::vector<char, cudallocator<char>> deviceScratchSpace;

    /// current integration time step
    float crntStepSize_SU;  // DN: needs to be brought here from GranParams
    float crntSimTime_SU;   // DN: needs to be brought here from GranParams
  public:
    ChSolverStateData() {
        cudaMallocManaged(&pMaxNumberSpheresInAnySD, sizeof(unsigned int));
        largestMaxNumberSpheresInAnySD_thusFar = 0;
    }
    ~ChSolverStateData() { cudaFree(pMaxNumberSpheresInAnySD); }
    inline unsigned int* pMM_maxNumberSpheresInAnySD() {
        return pMaxNumberSpheresInAnySD;  ///< returns pointer to managed memory
    }
    /// keep track of the largest number of spheres that touched an SD thus far into the simulation
    inline void recordCrntMaxNumberSpheresInAnySD(unsigned int someVal) {
        crntMaxNumberSpheresInAnySD = someVal;
        if (someVal > largestMaxNumberSpheresInAnySD_thusFar)
            largestMaxNumberSpheresInAnySD_thusFar = someVal;
    }
    /// reports the largest number of spheres that touched any SD thus far into the simulation
    inline unsigned int MaxNumberSpheresInAnySDThusFar() const { return largestMaxNumberSpheresInAnySD_thusFar; }
    /// reports the largest number of spheres that touched any SD at the most recent broadphase CD function call
    inline unsigned int currentMaxNumberSpheresInAnySD() const { return crntMaxNumberSpheresInAnySD; }

    /// return raw pointer to swath of device memory that is at least "sizeNeeded" large
    inline char* pDeviceMemoryScratchSpace(size_t sizeNeeded) {
        if (deviceScratchSpace.size() < sizeNeeded) {
            deviceScratchSpace.resize(sizeNeeded, 0);
        }
        return deviceScratchSpace.data();
    }
};

// Underlying implementation of the Chrono::Gpu system.
// used to control and dispatch the GPU sphere-only solver.
class ChSystemGpu_impl {
  public:
    virtual ~ChSystemGpu_impl();

  protected:
    /// Structure with simulation parameters for sphere-based granular dynamics.
    /// This structure is stored in CUDA unified memory so that it can be accessed from both host and device.
    struct GranParams {
        float stepSize_SU;  ///< Timestep in SU

        CHGPU_FRICTION_MODE friction_mode;      ///< Which friction mode is active for the simulation
        CHGPU_ROLLING_MODE rolling_mode;        ///< Which rolling resistance model is active
        CHGPU_TIME_INTEGRATOR time_integrator;  ///< Which time integrator is active

        /// Ratio of normal force to peak tangent force, also arctan(theta) where theta is the friction angle
        /// sphere-to-sphere
        float static_friction_coeff_s2s;
        /// Ratio of normal force to peak tangent force, also arctan(theta) where theta is the friction angle
        /// sphere-to-wall
        float static_friction_coeff_s2w;

        float rolling_coeff_s2s_SU;  ///< Coefficient of rolling resistance sphere-to-sphere
        float rolling_coeff_s2w_SU;  ///< Coefficient of rolling resistance sphere-to-wall

        float spinning_coeff_s2s_SU;  ///< Coefficient of spinning resistance sphere-to-sphere
        float spinning_coeff_s2w_SU;  ///< Coefficient of spinning resistance sphere-to-wall

        float Gamma_n_s2s_SU;  ///< sphere-to-sphere normal contact damping coefficient, expressed in SU
        float Gamma_n_s2w_SU;  ///< sphere-to-wall normal contact damping coefficient, expressed in SU
        float Gamma_t_s2s_SU;  ///< sphere-to-sphere tangent contact damping coefficient, expressed in SU
        float Gamma_t_s2w_SU;  ///< sphere-to-wall tangent contact damping coefficient, expressed in SU

        float K_n_s2s_SU;  ///< sphere-to-sphere normal contact stiffness, expressed in SU
        float K_n_s2w_SU;  ///< sphere-to-wall normal contact stiffness, expressed in SU
        float K_t_s2s_SU;  ///< sphere-to-sphere tangent contact stiffness, expressed in SU
        float K_t_s2w_SU;  ///< sphere-to-wall tangent contact stiffness, expressed in SU

        /// use material-based property
        bool use_mat_based = false;

        /// effective sphere-to-sphere youngs modulus, expressed in SU
        float E_eff_s2s_SU;
        /// effective sphere-to-wall youngs modulus, expressed in SU
        float E_eff_s2w_SU;

        /// effective sphere-to-sphere shear modulus, expressed in SU
        float G_eff_s2s_SU;
        /// effective sphere-to-wall shear modulus, expressed in SU
        float G_eff_s2w_SU;

        /// effective sphere-to-sphere coefficient of restitution, expressed in SU
        float COR_s2s_SU;
        /// effective sphere-to-wall coefficient of restitution, expressed in SU
        float COR_s2w_SU;

        unsigned int sphereRadius_SU;  ///< Radius of the sphere, expressed in SU
        float sphereInertia_by_r;      ///< Moment of inertia of a sphere, normalized by the radius, expressed in SU

        unsigned int SD_size_X_SU;  ///< X-dimension of each subdomain box, expressed in SU
        unsigned int SD_size_Y_SU;  ///< Y-dimension of each subdomain box, expressed in SU
        unsigned int SD_size_Z_SU;  ///< Z-dimension of each subdomain box, expressed in SU

        unsigned int nSpheres;  ///< Total number of spheres in system, used for boundary condition multistep friction
        unsigned int nSDs;      ///< Total number of subdomains

        unsigned int nSDs_X;  ///< X-dimension of the big domain box in multiples of subdomains
        unsigned int nSDs_Y;  ///< Y-dimension of the big domain box in multiples of subdomains
        unsigned int nSDs_Z;  ///< Z-dimension of the big domain box in multiples of subdomains

        int64_t max_x_pos_unsigned;  ///< Maximum X dimension in the big domain frame ((int64_t)SD_size_X_SU * nSDs_X)
        int64_t max_y_pos_unsigned;  ///< Maximum Y dimension in the big domain frame ((int64_t)SD_size_Y_SU * nSDs_Y)
        int64_t max_z_pos_unsigned;  ///< Maximum Z dimension in the big domain frame ((int64_t)SD_size_Z_SU * nSDs_Z)

        float gravAcc_X_SU;  ///< X gravity in SU
        float gravAcc_Y_SU;  ///< Y gravity in SU
        float gravAcc_Z_SU;  ///< Z gravity in SU

        int64_t BD_frame_X;  ///< The bottom-left corner xPos of the big domain
        int64_t BD_frame_Y;  ///< The bottom-left corner yPos of the big domain
        int64_t BD_frame_Z;  ///< The bottom-left corner zPos of the big domain

        int64_t BD_offset_X;  /// X offset of big domain from its original frame; allows the subdomain to move
        int64_t BD_offset_Y;  /// Y offset of big domain from its original frame; allows the subdomain to move
        int64_t BD_offset_Z;  /// Z offset of big domain from its original frame; allows the subdomain to move

        float cohesionAcc_s2s;  ///< Constant acceleration of sphere-to-sphere cohesion
        float adhesionAcc_s2w;  ///< Accleration of adhesion

        double LENGTH_UNIT;  ///< 1 / C_L. Any length expressed in SU is a multiple of LENGTH_UNIT
        double TIME_UNIT;    ///< 1 / C_T. Any time quantity in SU is measured as a positive multiple of TIME_UNIT
        double MASS_UNIT;    ///< 1 / C_M. Any mass quantity is measured as a positive multiple of MASS_UNIT

        /// Make clear that the underlying assumption is unit SU mass
        constexpr static float sphere_mass_SU = 1.f;

        /// Used as a safety check to determine whether a system has lost stability
        float max_safe_vel = (float)UINT_MAX;

        bool recording_contactInfo = false;  ///< recording contact info
    };

    /// Structure of pointers to kinematic quantities of the ChSystemGpu_impl.
    /// These pointers must be in device-accessible memory.
    struct SphereData {
        int* sphere_local_pos_X;  ///< X position relative to owner subdomain in unified memory
        int* sphere_local_pos_Y;  ///< Y position relative to owner subdomain in unified memory
        int* sphere_local_pos_Z;  ///< Z position relative to owner subdomain in unified memory

        float* pos_X_dt;  ///< X velocity in unified memory
        float* pos_Y_dt;  ///< Y velocity in unified memory
        float* pos_Z_dt;  ///< Z velocity in unified memory

        float* sphere_Omega_X;  ///< X angular velocity in unified memory. Only used if friction is present
        float* sphere_Omega_Y;  ///< Y angular velocity in unified memory. Only used if friction is present
        float* sphere_Omega_Z;  ///< Z angular velocity in unified memory. Only used if friction is present

        float* sphere_acc_X;  ///< X angular acceleration in unified memory
        float* sphere_acc_Y;  ///< Y angular acceleration in unified memory
        float* sphere_acc_Z;  ///< Z angular acceleration in unified memory

        float* sphere_ang_acc_X;  ///< X angular acceleration in unified memory
        float* sphere_ang_acc_Y;  ///< Y angular acceleration in unified memory
        float* sphere_ang_acc_Z;  ///< Z angular acceleration in unified memory

        float* sphere_acc_X_old;  ///< Previous step X acceleration for multistep integrators
        float* sphere_acc_Y_old;  ///< Previous step Y acceleration for multistep integrators
        float* sphere_acc_Z_old;  ///< Previous step Z acceleration for multistep integrators

        float* sphere_ang_acc_X_old;  ///< Previous step X angular acceleration for multistep integrators
        float* sphere_ang_acc_Y_old;  ///< Previous step Y angular acceleration for multistep integrators
        float* sphere_ang_acc_Z_old;  ///< Previous step Z angular acceleration for multistep integrators

        not_stupid_bool* sphere_fixed;  ///< Flags indicating whether or not a sphere is fixed

        float* sphere_stats_buffer;  ///< A buffer array that can store any quantity that the user wish to reduce
        unsigned int*
            sphere_stats_buffer_int;  ///< A buffer array that stores int-valued sys info that the user quarries

        unsigned int* contact_partners_map;   ///< Contact partners for each sphere. Only in frictional simulations
        not_stupid_bool* contact_active_map;  ///< Whether the frictional contact at an index is active
        float3* contact_history_map;  ///< Tangential history for a given contact pair. Only for multistep friction
        float* contact_duration;      ///< Duration of persistent contact between pairs

        float3* normal_contact_force;       ///< Track normal contact force
        float3* tangential_friction_force;  ///< Track sliding friction force
        float3* rolling_friction_torque;    ///< Track rolling friction force
        float* char_collision_time;         ///< Track characteristic collision time
        float3* v_rot_array;                ///< Track v_rot array

        unsigned int* SD_NumSpheresTouching;         ///< Number of particles touching each subdomain
        unsigned int* SD_SphereCompositeOffsets;     ///< Offset of each subdomain in the big composite array
        unsigned int* SD_SphereCompositeOffsets_SP;  ///< like SD_SphereCompositeOffsets, scratch pad (SP) used
        unsigned int* spheres_in_SD_composite;       ///< Big composite array of sphere-subdomain membership

        unsigned int* sphere_owner_SDs;  ///< List of owner subdomains for each sphere
    };

    // The system is not default-constructible
    ChSystemGpu_impl() = delete;

    /// Construct Chrono::Gpu system with given sphere radius, density, big domain dimensions and the frame origin.
    ChSystemGpu_impl(float sphere_rad, float density, float3 boxDims, float3 O);

    /// Create big domain walls out of plane boundary conditions
    void CreateWallBCs();

    /// Create an axis-aligned sphere boundary condition
    size_t CreateBCSphere(float center[3], float radius, bool outward_normal, bool track_forces, float mass);

    /// Create an z-axis aligned cone boundary condition
    size_t CreateBCConeZ(float cone_tip[3],
                         float slope,
                         float hmax,
                         float hmin,
                         bool outward_normal,
                         bool track_forces);

    /// Create plane boundary condition
    /// Instead of always push_back, you can select a position in vector to store the BC info: this is for reserved BCs
    /// (box domain BCs) only. And if the position is SIZE_MAX then the behavior is push_back.
    size_t CreateBCPlane(float plane_pos[3], float plane_normal[3], bool track_forces, size_t position = SIZE_MAX);

    /// Create customized bc plate
    size_t CreateCustomizedPlate(float plate_pos_center[3], float plate_normal[3], float hdim_y);

    /// Create an z-axis aligned cylinder boundary condition
    size_t CreateBCCylinderZ(float center[3], float radius, bool outward_normal, bool track_forces);

    /// Disable a boundary condition by its ID, returns false if the BC does not exist
    bool DisableBCbyID(size_t BC_id) {
        size_t max_id = BC_params_list_SU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to disable invalid BC ID %zu\n", BC_id);
            return false;
        }

        if (BC_id <= NUM_RESERVED_BC_IDS - 1) {
            printf("ERROR: Trying to modify reserved BC ID %zu\n", BC_id);
            return false;
        }
        BC_params_list_UU.at(BC_id).active = false;
        BC_params_list_SU.at(BC_id).active = false;
        return true;
    }

    /// Enable a boundary condition by its ID, returns false if the BC does not exist
    bool EnableBCbyID(size_t BC_id) {
        size_t max_id = BC_params_list_SU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to enable invalid BC ID %zu\n", BC_id);
            return false;
        }
        if (BC_id <= NUM_RESERVED_BC_IDS - 1) {
            printf("ERROR: Trying to modify reserved BC ID %zu\n", BC_id);
            return false;
        }
        BC_params_list_UU.at(BC_id).active = true;
        BC_params_list_SU.at(BC_id).active = true;
        return true;
    }

    /// Enable a boundary condition by its ID, returns false if the BC does not exist
    bool SetBCOffsetFunction(size_t BC_id, const GranPositionFunction& offset_function) {
        size_t max_id = BC_params_list_SU.size();
        if (BC_id >= max_id) {
            printf("ERROR: Trying to set offset function for invalid BC ID %zu\n", BC_id);
            return false;
        }
        if (BC_id <= NUM_RESERVED_BC_IDS - 1) {
            printf("ERROR: Trying to modify reserved BC ID %zu\n", BC_id);
            return false;
        }
        BC_offset_function_list.at(BC_id) = offset_function;
        BC_params_list_UU.at(BC_id).fixed = false;
        BC_params_list_SU.at(BC_id).fixed = false;
        return true;
    }

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

    // Copy back the subdomain device data and save it to a file for error checking on the priming kernel
    //// RADU: is this function going to be implemented?!?
    ////void checkSDCounts(std::string ofile, bool write_out, bool verbose) const;

    /// Get the max z position of the spheres, allows easier co-simulation. True for getting max Z, false for getting
    /// minimum Z.
    double GetMaxParticleZ(bool getMax = true);

    /// Get the number of particles that are higher than a given Z coordinate
    unsigned int GetNumParticleAboveZ(float ZValue);

    /// Get the number of particles that are higher than a given X coordinate
    unsigned int GetNumParticleAboveX(float XValue);

    /// Return the total kinetic energy of all particles.
    float ComputeTotalKE();

    /// Return the squared sum of the 3 arrays.
    float computeArray3SquaredSum(std::vector<float, cudallocator<float>>& arrX,
                                  std::vector<float, cudallocator<float>>& arrY,
                                  std::vector<float, cudallocator<float>>& arrZ,
                                  size_t nSpheres);

    /// Return particle position.
    float3 GetParticlePosition(int nSphere) const;

    /// Set particle position
    void SetParticlePosition(int nSphere, double3 position);

    /// return absolute velocity
    float getAbsVelocity(int nSphere);

    // whether or not the particle is fixed
    bool IsFixed(int nSphere) const;

    /// Return particle linear velocity.
    float3 GetParticleLinVelocity(int nSphere) const;

    /// Return particle angular velocity.
    float3 GetParticleAngVelocity(int nSphere) const;

    /// Return particle acceleration
    float3 GetParticleLinAcc(int nSphere) const;

    /// Return number of particle-particle contacts.
    int GetNumContacts() const;

    /// Return position of BC plane.
    float3 GetBCPlanePosition(size_t plane_id) const;

    /// return position of BC sphere
    float3 GetBCSpherePosition(size_t bc_id) const;

    /// set bc sphere position
    void SetBCSpherePosition(size_t bc_id, const float3 pos);

    /// set bc sphere vleocity
    void SetBCSphereVelocity(size_t bc_id, const float3 velo);

    /// return velocity of BC sphere
    float3 GetBCSphereVelocity(size_t bc_id) const;

    /// Get the reaction forces on a boundary by ID, returns false if the forces are invalid (bad BC ID)
    bool GetBCReactionForces(size_t BC_id, float3& force) const;

    /// Set initial particle positions. MUST be called only once and MUST be called before initialize
    void SetParticles(const std::vector<float3>& points,
                      const std::vector<float3>& vels = std::vector<float3>(),
                      const std::vector<float3>& ang_vels = std::vector<float3>());

    /// set particle velocity, can be called during the simulation
    void SetParticleVelocity(int id, const double3& velocity);

    void SetBCPlaneRotation(size_t plane_id, double3 rotation_center, double3 rotation_omega);

    /// Advance simulation by duration in user units, return actual duration elapsed
    /// Requires initialize() to have been called
    virtual double AdvanceSimulation(float duration);

    /// This method figures out how big a SD is, and how many SDs are going to be necessary
    /// in order to cover the entire BD.
    /// Nomenclature: BD: Big domain, SD: Sub-domain.
    void partitionBD();

    /// Copy constant sphere data to device
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

    /// Wrap the device helper function
    int3 getSDTripletFromID(unsigned int SD_ID) const;

    /// Create a helper to do sphere initialization
    void initializeSpheres();

    /// Sorts particle positions spatially in order to improve memory efficiency
    void defragment_initial_positions();

    /// Sorts the user-provided contact history array in the order determined by defragment_initial_positions(),
    /// if that is called
    void defragment_friction_history(unsigned int history_offset);

    /// Setup sphere data, initialize local coords
    void setupSphereDataStructures();

    /// Helper function to convert a position in UU to its SU representation while also changing data type
    template <typename T1, typename T2>
    T1 convertToPosSU(T2 val) {
        return (T1)(val / gran_params->LENGTH_UNIT);
    }

    /// convert all BCs from UU to SU
    void convertBCUnits();

    /// Max velocity of all particles in system
    float get_max_vel() const;

    /// Get the maximum stiffness term in the system
    virtual double get_max_K() const;

    /// This method defines the mass, time, length Simulation Units. It also sets several other constants that enter the
    /// scaling of various physical quantities set by the user.
    virtual void switchToSimUnits();

    /// combine material properties of two types to get effective ones
    void combineMaterialSurface();

    /// Set the position function of a boundary condition and account for the offset
    void setBCOffset(const BC_type&,
                     const BC_params_t<float, float3>& params_UU,
                     BC_params_t<int64_t, int64_t3>& params_SU,
                     double3 offset_UU);

    /// Update positions of each boundary condition using prescribed functions
    void updateBCPositions();

    /// Write particle positions, vels and ang vels to a file stream (based on a format)
    void WriteRawParticles(std::ofstream& ptFile) const;
    void WriteCsvParticles(std::ofstream& ptFile) const;
    void WriteChPFParticles(std::ofstream& ptFile) const;
#ifdef USE_HDF5
    void WriteH5Particles(H5::H5File& ptFile) const;
#endif

    /// Write contact info file
    void WriteContactInfoFile(const std::string& outfilename) const;

    /// Get rolling friction torque between body i and j, return 0 if not in contact
    float3 getRollingFrictionTorque(int i, int j);

    /// get rolling friction v_rot
    float3 getRollingVrot(int i, int j);

    /// get rolling characterisitc contact time
    float getRollingCharContactTime(int i, int j);

    /// Get tangential friction force between body i and j, return 0 if not in contact
    float3 getSlidingFrictionForce(int i, int j);

    /// Get normal friction force between body i and j, return 0 if not in contact
    float3 getNormalForce(int i, int j);

    /// get list of neighbors in contact with particle ID
    void getNeighbors(int ID, std::vector<int>& neighborList);

    /// Rough estimate of the total amount of memory used by the system.
    size_t EstimateMemUsage() const;

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

    // Tuning for non-dimensionalization
    /// Safety factor on simulation time
    unsigned int psi_T;

    /// Safety factor on space adim
    unsigned int psi_L;

    /// Fraction of sphere radius which gives an upper bound on the length unit
    float psi_R;

    /// Holds the sphere and big-domain-related params in unified memory
    GranParams* gran_params;

    /// Holds system degrees of freedom
    SphereData* sphere_data;

    /// Contains information about the status of the granular simulator (solver)
    ChSolverStateData stateOfSolver_resources;

    /// Allows the code to be very verbose for debugging
    CHGPU_VERBOSITY verbosity;

    /// If dividing the longest box dimension into INT_MAX pieces gives better resolution than the deformation-based
    /// scaling, do that.
    bool use_min_length_unit;

    /// If true, on Initialize(), the order of particles will be re-arranged so those in the same SD locate close to
    /// each other
    bool defragment_on_start = true;

    /// Bit flags indicating what fields to write out during WriteParticleFile
    /// Set with the CHGPU_OUTPUT_FLAGS enum
    unsigned int output_flags;

    /// How to write the output files?
    /// Default is CSV
    CHGPU_OUTPUT_MODE file_write_mode;

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

    /// Fixity of each sphere
    std::vector<not_stupid_bool, cudallocator<not_stupid_bool>> sphere_fixed;

    /// A buffer array that can store any derived quantity from particles. When users quarry a quantity (such as kinetic
    /// energy using GetParticleKineticEnergy), this array is used to store that particle-wise quantity, and then
    /// potentially reduced via CUB.
    std::vector<float, cudallocator<float>> sphere_stats_buffer;
    std::vector<unsigned int, cudallocator<unsigned int>> sphere_stats_buffer_int;

    /// Set of contact partners for each sphere. Only used in frictional simulations
    std::vector<unsigned int, cudallocator<unsigned int>> contact_partners_map;
    /// Whether the frictional contact at an index is active
    std::vector<not_stupid_bool, cudallocator<not_stupid_bool>> contact_active_map;
    /// Tracks the tangential history vector for a given contact pair. Only used in multistep friction
    std::vector<float3, cudallocator<float3>> contact_history_map;
    /// Tracks the duration of contact between contact pairs. Only used in multistep friction
    std::vector<float, cudallocator<float>> contact_duration;
    /// Tracks the normal contact force for a given contact pair
    std::vector<float3, cudallocator<float3>> normal_contact_force;
    /// Tracks the tangential contact force for a given contact pair
    std::vector<float3, cudallocator<float3>> tangential_friction_force;
    /// Tracks the rolling resistance for a given contact pair
    std::vector<float3, cudallocator<float3>> rolling_friction_torque;
    /////////////////////DEBUG PURPOSE///////////////////////////
    /// Tracks the characteristic contact time for a given contact pair
    std::vector<float, cudallocator<float>> char_collision_time;
    /// Tracks v_rot
    std::vector<float3, cudallocator<float3>> v_rot_array;

    /// X gravity in user units
    float X_accGrav;
    /// Y gravity in user units
    float Y_accGrav;
    /// Z gravity in user units
    float Z_accGrav;

    /// Entry "i" says how many spheres touch subdomain i
    std::vector<unsigned int, cudallocator<unsigned int>> SD_NumSpheresTouching;
    ///  Entry "i" says where spheres touching ith SD are stored in the big composite array (the offset)
    std::vector<unsigned int, cudallocator<unsigned int>> SD_SphereCompositeOffsets;
    /// Scratch area, needed to populate data in the big composite array
    std::vector<unsigned int, cudallocator<unsigned int>> SD_SphereCompositeOffsets_ScratchPad;
    /// Array with the IDs of the spheres touching each SD associated with the box; organized SD after SD
    std::vector<unsigned int, cudallocator<unsigned int>> spheres_in_SD_composite;

    /// List of owner subdomains for each sphere
    std::vector<unsigned int, cudallocator<unsigned int>> sphere_owner_SDs;

    /// User provided timestep in UU
    float stepSize_UU;

    /// SU (adimensional) value of time step
    float stepSize_SU;

    /// how is the system integrated through time?
    CHGPU_TIME_INTEGRATOR time_integrator;

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

    /// material based property: youngs modulus, poisson ratio, cor
    bool use_mat_based = false;

    double YoungsModulus_sphere_UU;
    double YoungsModulus_wall_UU;
    double YoungsModulus_mesh_UU;

    double COR_sphere_UU;
    double COR_wall_UU;
    double COR_mesh_UU;

    double PoissonRatio_sphere_UU;
    double PoissonRatio_wall_UU;
    double PoissonRatio_mesh_UU;

    /// Rolling friction coefficient for sphere-to-sphere
    double rolling_coeff_s2s_UU;

    /// Rolling friction coefficient for sphere-to-wall
    /// Units and use are dependent on the rolling friction model used
    double rolling_coeff_s2w_UU;

    /// Spinning friction coefficient for sphere-to-sphere
    double spinning_coeff_s2s_UU;

    /// Spinning friction coefficient for sphere-to-wall
    /// Units and use are dependent on the spinning friction model used
    double spinning_coeff_s2w_UU;

    /// Store the ratio of the acceleration due to cohesion vs the acceleration due to gravity, makes simple API
    float cohesion_over_gravity;

    /// Store the ratio of the acceleration due to adhesion vs the acceleration due to gravity
    float adhesion_s2w_over_gravity;

    /// The reference point for UU to SU local coordinates
    int64_t3 BD_rest_frame_SU;

    /// List of generalized boundary conditions that constrain sphere motion
    std::vector<BC_type, cudallocator<BC_type>> BC_type_list;
    /// Sim-unit (adimensional) details of boundary conditions
    std::vector<BC_params_t<int64_t, int64_t3>, cudallocator<BC_params_t<int64_t, int64_t3>>> BC_params_list_SU;
    /// User-unit (dimensional) details of boundary conditions
    std::vector<BC_params_t<float, float3>, cudallocator<BC_params_t<float, float3>>> BC_params_list_UU;
    /// Offset motions functions for boundary conditions -- used for moving walls, wavetanks, etc.
    std::vector<GranPositionFunction> BC_offset_function_list;

    /// User defined radius of the sphere
    float sphere_radius_UU;
    /// User defined density of the sphere
    float sphere_density_UU;

    /// X-length of the big domain; defines the global X axis located at the CM of the box (as default)
    float box_size_X;
    /// Y-length of the big domain; defines the global Y axis located at the CM of the box (as default)
    float box_size_Y;
    /// Z-length of the big domain; defines the global Z axis located at the CM of the box (as default)
    float box_size_Z;

    /// XYZ coordinate of the center of the big box domain in the user-defined frame. Default is (0,0,0).
    float user_coord_O_X;
    float user_coord_O_Y;
    float user_coord_O_Z;

    /// User-provided sphere positions in UU
    std::vector<float3> user_sphere_positions;

    /// User-provided sphere velocities in UU
    std::vector<float3> user_sphere_vel;

    /// User-provided sphere angular velocities in UU
    std::vector<float3> user_sphere_ang_vel;

    /// User-provided sphere fixity as bools
    std::vector<bool> user_sphere_fixed;

    /// User-provided sphere contact pair information
    std::vector<unsigned int> user_partner_map;

    /// User-provided sphere (contact pair) friction history information in UU
    std::vector<float3> user_friction_history;

    /// The offset function for the big domain walls
    GranPositionFunction BDOffsetFunction;

    /// Allow the user to set the big domain to be fixed, ignoring any given position functions
    bool BD_is_fixed = true;

  public:
    // Do two things: make the naming nicer and require a const pointer everywhere

    /// Get handle for the gran params that skips namespacing and enforces const-ness
    typedef const ChSystemGpu_impl::GranParams* GranParamsPtr;

    /// Get handle for the sphere data that skips namespacing and enforces const-ness
    typedef const ChSystemGpu_impl::SphereData* GranSphereDataPtr;

    friend class ChSystemGpu;
    friend class ChSystemGpuMesh;
};

}  // namespace gpu
}  // namespace chrono
