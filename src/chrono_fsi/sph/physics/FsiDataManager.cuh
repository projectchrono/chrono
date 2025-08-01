// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration.
//
// =============================================================================

#ifndef CH_FSI_DATA_MANAGER_H
#define CH_FSI_DATA_MANAGER_H

#include <vector>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/detail/normal_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

#include "chrono_fsi/sph/ChFsiParamsSPH.h"

#include "chrono_fsi/sph/physics/MarkerType.cuh"
#include "chrono_fsi/sph/utils/UtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// typedef device iterators for shorthand SPH operation of thrust vectors of Real3
typedef thrust::device_vector<Real3>::iterator r3IterD;

/// typedef device iterators for shorthand SPH operation of thrust vectors of Real4
typedef thrust::device_vector<Real4>::iterator r4IterD;

/// typedef device tuple for holding SPH data pos,vel,[rho,pressure,mu,type]
typedef thrust::tuple<r4IterD, r3IterD, r4IterD, r3IterD, r3IterD> iterTupleSphD;
typedef thrust::zip_iterator<iterTupleSphD> zipIterSphD;

/// typedef host iterators for shorthand SPH operation of thrust vectors of Real3
typedef thrust::host_vector<Real3>::iterator r3IterH;

/// typedef host iterators for shorthand SPH operation of thrust vectors of Real4
typedef thrust::host_vector<Real4>::iterator r4IterH;

/// typedef host tuple for holding SPH data pos,vel,[rho,pressure,mu,type]
typedef thrust::tuple<r4IterH, r3IterH, r4IterH, r3IterH, r3IterH> iterTupleH;
typedef thrust::zip_iterator<iterTupleH> zipIterSphH;

/// typedef device iterators for shorthand rigid body states:
/// pos,lin_vel,lin_acc,rot,ang_Vel,ang_acc
typedef thrust::tuple<r3IterD, r3IterD, r3IterD, r4IterD, r3IterD, r3IterD> iterTupleRigidD;
typedef thrust::zip_iterator<iterTupleRigidD> zipIterRigidD;

/// typedef host iterators for shorthand rigid body states:
/// pos,lin_vel,lin_acc,rot,ang_Vel,ang_acc
typedef thrust::tuple<r3IterH, r3IterH, r3IterH, r4IterH, r3IterH, r3IterH> iterTupleRigidH;
typedef thrust::zip_iterator<iterTupleRigidH> zipIterRigidH;

/// typedef device iterators for shorthand chrono bodies operations
typedef thrust::tuple<r3IterH, r3IterH, r3IterH, r4IterH, r3IterH, r3IterH> iterTupleChronoBodiesH;
typedef thrust::zip_iterator<iterTupleChronoBodiesH> zipIterChronoBodiesH;

/// Struct to store the information of SPH particles on the device
struct SphMarkerDataD {
    thrust::device_vector<Real4> posRadD;     ///< Vector of the positions of particles + characteristic radius
    thrust::device_vector<Real3> velMasD;     ///< Vector of the velocities of particles
    thrust::device_vector<Real4> rhoPresMuD;  ///< Vector of the rho+pressure+mu+type of particles
    thrust::device_vector<Real3> tauXxYyZzD;  ///< Vector of the total stress (diagonal) of particles
    thrust::device_vector<Real3> tauXyXzYzD;  ///< Vector of the total stress (off-diagonal) of particles

    zipIterSphD iterator(int offset);
    void resize(size_t s);
};

/// Struct to store the information of SPH particles on the host.
struct SphMarkerDataH {
    thrust::host_vector<Real4> posRadH;     ///< Vector of the positions of particles
    thrust::host_vector<Real3> velMasH;     ///< Vector of the velocities of particles
    thrust::host_vector<Real4> rhoPresMuH;  ///< Vector of the rho+pressure+mu+type of particles
    thrust::host_vector<Real3> tauXxYyZzH;  ///< Vector of the total stress (diagonal) of particles
    thrust::host_vector<Real3> tauXyXzYzH;  ///< Vector of the total stress (off-diagonal) of particles

    zipIterSphH iterator(int offset);
    void resize(size_t s);
};

/// Rigid body states on host.
struct FsiBodyStateH {
    thrust::host_vector<Real3> pos;      ///< body positions
    thrust::host_vector<Real3> lin_vel;  ///< body linear velocities
    thrust::host_vector<Real3> lin_acc;  ///< body linear accelerations
    thrust::host_vector<Real4> rot;      ///< body orientations (quaternions)
    thrust::host_vector<Real3> ang_vel;  ///< body angular velocities (local frame)
    thrust::host_vector<Real3> ang_acc;  ///< body angular accelerations (local frame)

    zipIterRigidH iterator(int offset);
    void Resize(size_t s);
};

/// Rigid body states on device.
/// Data are managed in an SOA, with each array containing corresponding data from all bodies in the system.
struct FsiBodyStateD {
    thrust::device_vector<Real3> pos;      ///< body linear positions
    thrust::device_vector<Real3> lin_vel;  ///< body linear velocities
    thrust::device_vector<Real3> lin_acc;  ///< body linear accelerations
    thrust::device_vector<Real4> rot;      ///< body orientations (quaternions)
    thrust::device_vector<Real3> ang_vel;  ///< body angular velocities (local frame)
    thrust::device_vector<Real3> ang_acc;  ///< body angular accelerations (local frame)

    zipIterRigidD iterator(int offset);

    void CopyFromH(const FsiBodyStateH& bodyStateH);
    FsiBodyStateD& operator=(const FsiBodyStateD& other);
    void Resize(size_t s);
};

/// FEA mesh states on host.
/// Data are managed in an SOA, with each array containing corresponding data from all meshes (1D or 2D) in the system.
struct FsiMeshStateH {
    // States
    thrust::host_vector<Real3> pos;  ///< mesh node positions
    thrust::host_vector<Real3> vel;  ///< mesh node velocities
    thrust::host_vector<Real3> acc;  ///< mesh node accelerations
    thrust::host_vector<Real3> dir;  ///< node directions (unit vectors)

    void Resize(size_t s, bool use_node_directions);
    size_t GetSize() const { return pos.size(); }

    bool has_node_directions;
};

/// FEA mesh state on device.
/// Data are managed in an SOA, with each array containing corresponding data from all meshes (1D or 2D) in the system.
struct FsiMeshStateD {
    // States
    thrust::device_vector<Real3> pos;  ///< mesh node positions
    thrust::device_vector<Real3> vel;  ///< mesh node velocities
    thrust::device_vector<Real3> acc;  ///< mesh node accelerations
    thrust::device_vector<Real3> dir;  ///< node directions (unit vectors)

    /// Function to transfer CPU -> GPU.
    /// This function copies only `pos`, `vel`, and `acc`. It does not copy node directions `dir`.
    void CopyFromH(const FsiMeshStateH& meshStateH);

    /// Function to transfer node directions CPU -> GPU.
    /// This function copies only node directions `dir`. It does not copy `pos`, `vel`, and `acc`.
    void CopyDirectionsFromH(const FsiMeshStateH& meshStateH);

    /// Assignment operator. Only copies states.
    FsiMeshStateD& operator=(const FsiMeshStateD& other);

    /// Resize the data vectors to the specified size.
    /// If indicated, also resize the vector of node directions.
    void Resize(size_t s, bool use_node_directions);

    bool has_node_directions;
};

/// Struct to store neighbor search information on the device.
struct ProximityDataD {
    thrust::device_vector<uint> gridMarkerHashD;   ///< gridMarkerHash=s(i,j,k)= k*n_x*n_y + j*n_x + i (numAllMarkers);
    thrust::device_vector<uint> gridMarkerIndexD;  ///< Marker's index, can be original or sorted (numAllMarkers);
    thrust::device_vector<uint> cellStartD;  ///< Index of the particle starts a cell in sorted list (m_numGridCells)
    thrust::device_vector<uint> cellEndD;    ///< Index of the particle ends a cell in sorted list (m_numGridCells)
    thrust::device_vector<uint>
        mapOriginalToSorted;  ///< Index mapping from the original to the sorted (numAllMarkers);

    void resize(size_t s);
};

/// Struct to store CUDA device information.
struct CudaDeviceInfo {
    int deviceID;               ///< CUDA device ID
    cudaDeviceProp deviceProp;  ///< CUDA device properties
};

// -----------------------------------------------------------------------------

/// Number of rigid and flexible solid bodies, fluid SPH particles, solid SPH particles, boundary SPH particles.
/// This structure holds the number of SPH particles and rigid/flexible bodies.
///  Note that the order of makers in the memory is as follows:
///  -  (1) fluid particles (type = -1)
///  -  (2) particles attached to fixed objects (boundary particles with type = 0)
///  -  (3) particles attached to rigid bodies (type = 1)
///  -  (4) particles attached to flexible bodies (type = 2)
struct Counters {
    size_t numFsiBodies;      ///< number of rigid bodies
    size_t numFsiNodes1D;     ///< number of nodes in 1-D FEA mesh segments
    size_t numFsiNodes2D;     ///< number of nodes in 2-D FEA mesh faces
    size_t numFsiElements1D;  ///< number of 1-D FEA mesh segments
    size_t numFsiElements2D;  ///< number of 2-D FEA mesh faces

    size_t numGhostMarkers;     ///< number of Ghost SPH particles for Variable Resolution methods
    size_t numHelperMarkers;    ///< number of helper SPH particles used for merging particles
    size_t numFluidMarkers;     ///< number of fluid SPH particles
    size_t numBoundaryMarkers;  ///< number of BCE markers on boundaries
    size_t numRigidMarkers;     ///< number of BCE markers on rigid bodies
    size_t numFlexMarkers1D;    ///< number of BCE markers on flexible segments
    size_t numFlexMarkers2D;    ///< number of BCE markers on flexible faces
    size_t numBceMarkers;       ///< total number of BCE markers
    size_t numAllMarkers;       ///< total number of particles in the simulation

    size_t startBoundaryMarkers;  ///< index of first BCE marker on boundaries
    size_t startRigidMarkers;     ///< index of first BCE marker on first rigid body
    size_t startFlexMarkers1D;    ///< index of first BCE marker on first flex segment
    size_t startFlexMarkers2D;    ///< index of first BCE marker on first flex face
    size_t numActiveParticles;    ///< number of active particles
    size_t numExtendedParticles;  ///< number of extended particles
};

// -----------------------------------------------------------------------------

/// Data manager for the SPH-based FSI system.
struct FsiDataManager {
  public:
    FsiDataManager(std::shared_ptr<ChFsiParamsSPH> params);
    virtual ~FsiDataManager();

    /// Set the growth factor for buffer resizing
    void SetGrowthFactor(float factor) { GROWTH_FACTOR = factor; }

    /// Add an SPH particle given its position, physical properties, velocity, and stress.
    void AddSphParticle(Real3 pos,
                        Real rho,
                        Real pres,
                        Real mu,
                        Real3 vel = mR3(0.0),
                        Real3 tauXxYyZz = mR3(0.0),
                        Real3 tauXyXzYz = mR3(0.0));

    /// Add a BCE marker of given type at the specified position and with specified velocity.
    void AddBceMarker(MarkerType type, Real3 pos, Real3 vel);

    /// Initialize the underlying FSU system.
    /// Set reference arrays, set counters, and resize simulation arrays.
    void Initialize(unsigned int num_fsi_bodies,
                    unsigned int num_fsi_nodes1D,
                    unsigned int num_fsi_elements1D,
                    unsigned int num_fsi_nodes2D,
                    unsigned int num_fsi_elements2D,
                    bool use_node_directions);

    /// Find indices of all SPH particles inside the specified OBB.
    std::vector<int> FindParticlesInBox(const Real3& hsize,
                                        const Real3& pos,
                                        const Real3& ax,
                                        const Real3& ay,
                                        const Real3& az);

    /// Extract positions of all markers (SPH and BCE).
    std::vector<Real3> GetPositions();

    /// Extract velocities of all markers (SPH and BCE).
    std::vector<Real3> GetVelocities();

    /// Extract accelerations of all markers (SPH and BCE).
    std::vector<Real3> GetAccelerations();

    /// Extract forces applied to all markers (SPH and BCE).
    std::vector<Real3> GetForces();

    /// Extract fluid properties of all markers (SPH and BCE).
    /// For each SPH particle, the 3-dimensional vector contains density, pressure, and viscosity.
    std::vector<Real3> GetProperties();

    /// Extract positions of all markers (SPH and BCE) with indices in the provided array.
    std::vector<Real3> GetPositions(const std::vector<int>& indices);

    /// Extract velocities of all markers (SPH and BCE) with indices in the provided array.
    std::vector<Real3> GetVelocities(const std::vector<int>& indices);

    /// Extract accelerations of all markers (SPH and BCE) with indices in the provided array.
    std::vector<Real3> GetAccelerations(const std::vector<int>& indices);

    /// Extract forces applied to all markers (SPH and BCE) with indices in the provided array.
    std::vector<Real3> GetForces(const std::vector<int>& indices);

    /// Extract FSI forces on rigid bodies.
    std::vector<Real3> GetRigidForces();

    /// Extract FSI torques on rigid bodies.
    std::vector<Real3> GetRigidTorques();

    /// Extract FSI forces on flex1D nodes.
    std::vector<Real3> GetFlex1dForces();

    /// Extract FSI forces on flex2D nodes.
    std::vector<Real3> GetFlex2dForces();

    void ConstructReferenceArray();
    void SetCounters(unsigned int num_fsi_bodies,
                     unsigned int num_fsi_nodes1D,
                     unsigned int num_fsi_elements1D,
                     unsigned int num_fsi_nodes2D,
                     unsigned int num_fsi_elements2D);

    /// Reset device data at beginning of a step.
    /// Initializes device vectors to zero.
    void ResetData();

    /// Resize data arrays based on particle activity.
    void ResizeArrays(uint numExtended);

    /// Return device memory usage.
    size_t GetCurrentGPUMemoryUsage() const;

    // ------------------------

    std::shared_ptr<CudaDeviceInfo> cudaDeviceInfo;  ///< CUDA device information

    std::shared_ptr<ChFsiParamsSPH> paramsH;  ///< simulation parameters (host)
    std::shared_ptr<Counters> countersH;      ///< problem counters (host)

    std::shared_ptr<SphMarkerDataD> sphMarkers_D;         ///< information of SPH particles at state 1 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkers1_D;  ///< information of SPH particles at state 2 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkers2_D;  ///< sorted information of SPH particles at state 1 on device
    std::shared_ptr<SphMarkerDataH> sphMarkers_H;         ///< information of SPH particles on host

    // ------------------------

    // FSI solid states
    std::shared_ptr<FsiBodyStateH> fsiBodyState_H;  ///< rigid body state (host)
    std::shared_ptr<FsiBodyStateD> fsiBodyState_D;  ///< rigid body state (device)

    std::shared_ptr<FsiMeshStateH> fsiMesh1DState_H;  ///< 1-D FEA mesh state (host)
    std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D;  ///< 1-D FEA mesh state (device)
    std::shared_ptr<FsiMeshStateH> fsiMesh2DState_H;  ///< 2-D FEA mesh state (host)
    std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D;  ///< 2-D FEA mesh state (device)

    // FSI flex solid connectivity
    thrust::host_vector<int2> flex1D_Nodes_H;    ///< node indices for each 1-D flex segment (host)
    thrust::device_vector<int2> flex1D_Nodes_D;  ///< node indices for each 1-D flex segment (device)
    thrust::host_vector<int3> flex2D_Nodes_H;    ///< node indices for each 2-D flex face (host)
    thrust::device_vector<int3> flex2D_Nodes_D;  ///< node indices for each 2-D flex face (device)

    // FSI solid BCEs
    thrust::host_vector<Real3> rigid_BCEcoords_H;     ///< local coordinates for BCE markers on rigid bodies (host)
    thrust::device_vector<Real3> rigid_BCEcoords_D;   ///< local coordinates for BCE markers on rigid bodies (device)
    thrust::host_vector<Real3> flex1D_BCEcoords_H;    ///< local coords for BCE markers on 1-D flex segments (host)
    thrust::device_vector<Real3> flex1D_BCEcoords_D;  ///< local coords for BCE markers on 1-D flex segments (device)
    thrust::host_vector<Real3> flex2D_BCEcoords_H;    ///< local coords for BCE markers on 2-D flex faces (host)
    thrust::device_vector<Real3> flex2D_BCEcoords_D;  ///< local coors for BCE markers on 2-D flex faces (device)

    thrust::host_vector<uint> rigid_BCEsolids_H;      ///< body ID for BCE markers on rigid bodies (host)
    thrust::device_vector<uint> rigid_BCEsolids_D;    ///< body ID for BCE markers on rigid bodies (device)
    thrust::host_vector<uint3> flex1D_BCEsolids_H;    ///< mesh and segment IDs for BCE markers on 1-D segments (host)
    thrust::device_vector<uint3> flex1D_BCEsolids_D;  ///< mesh and segment IDs for BCE markers on 1-D segments (device)
    thrust::host_vector<uint3> flex2D_BCEsolids_H;    ///< mesh and face IDs for BCE markers on 2-D faces (host)
    thrust::device_vector<uint3> flex2D_BCEsolids_D;  ///< mesh and face IDs for BCE markers on 2-D faces (device)

    // FSI solid forces
    thrust::device_vector<Real3> rigid_FSI_ForcesD;   ///< surface-integrated forces to rigid bodies
    thrust::device_vector<Real3> rigid_FSI_TorquesD;  ///< surface-integrated torques to rigid bodies

    thrust::device_vector<Real3> flex1D_FSIforces_D;  ///< surface-integrated forces on FEA 1-D segment nodes
    thrust::device_vector<Real3> flex2D_FSIforces_D;  ///< surface-integrated forces on FEA 2-D face nodes

    // ------------------------

    std::shared_ptr<ProximityDataD> markersProximity_D;  ///< information of neighbor search on the device

    thrust::host_vector<int4> referenceArray;      ///< phases in the array of SPH particles
    thrust::host_vector<int4> referenceArray_FEA;  ///< phases in the array of SPH particles for flexible elements

    // Fluid data (device)
    thrust::device_vector<Real4> derivVelRhoD;          ///< particle dv/dt and d(rho)/dt for particles
    thrust::device_vector<Real4> derivVelRhoOriginalD;  ///< particle dv/dt and d(rho)/dt - unsorted

    thrust::device_vector<Real3> derivTauXxYyZzD;         ///< d(tau)/dt for particles
    thrust::device_vector<Real3> derivTauXyXzYzD;         ///< d(tau)/dt for particles
    thrust::device_vector<Real3> vel_XSPH_D;              ///< XSPH velocity for particles
    thrust::device_vector<Real3> vis_vel_SPH_D;           ///< ISPH velocity for particles
    thrust::device_vector<Real4> sr_tau_I_mu_i;           ///< ISPH strain-rate, stress, inertia number, friction
    thrust::device_vector<Real4> sr_tau_I_mu_i_Original;  ///< ISPH strain-rate, stress, inertia number, friction
    thrust::device_vector<Real3> bceAcc;                  ///< acceleration for boundary/rigid/flex body particles

    thrust::device_vector<int32_t> activityIdentifierOriginalD;          ///< active particle flags - unsorted
    thrust::device_vector<int32_t> activityIdentifierSortedD;            ///< active particle flags - sorted
    thrust::device_vector<int32_t> extendedActivityIdentifierOriginalD;  ///< active particle flags - unsorted
    thrust::device_vector<uint> prefixSumExtendedActivityIdD;            ///< prefix sum of extended particles
    thrust::device_vector<uint> activeListD;                             ///< active list of particles
    thrust::device_vector<uint> numNeighborsPerPart;                     ///< number of neighbors for each particle

    // List of all neighbors (indexed with information from numNeighborsPerPart)
    thrust::device_vector<uint> neighborList;    ///< neighbor list for all particles
    thrust::device_vector<uint> freeSurfaceIdD;  ///< identifiers for particles close to free surface

  private:
    // Memory management parameters
    uint m_max_extended_particles;  ///< Maximum number of extended particles seen so far
    uint m_resize_counter;          ///< Counter for number of resizes since last shrink
    float GROWTH_FACTOR;            ///< Buffer factor for growth (20%)
    float SHRINK_THRESHOLD;         ///< Shrink if using less than 50% of capacity
    uint SHRINK_INTERVAL;           ///< Shrink every N resizes
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
