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

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/detail/normal_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

#include "chrono_fsi/ChConfigFsi.h"

#include "chrono_fsi/sph/physics/ChParams.h"
#include "chrono_fsi/sph/physics/ChMarkerType.cuh"
#include "chrono_fsi/sph/math/CustomMath.h"
#include "chrono_fsi/sph/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {

class ChFluidSystemSPH;

namespace sph {

/// @addtogroup fsi_physics
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

    zipIterSphD iterator();
    void resize(size_t s);
};

/// Struct to store the information of SPH particles on the host.
struct SphMarkerDataH {
    thrust::host_vector<Real4> posRadH;     ///< Vector of the positions of particles
    thrust::host_vector<Real3> velMasH;     ///< Vector of the velocities of particles
    thrust::host_vector<Real4> rhoPresMuH;  ///< Vector of the rho+pressure+mu+type of particles
    thrust::host_vector<Real3> tauXxYyZzH;  ///< Vector of the total stress (diagonal) of particles
    thrust::host_vector<Real3> tauXyXzYzH;  ///< Vector of the total stress (off-diagonal) of particles

    zipIterSphH iterator();
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

    zipIterRigidH iterator();
    void resize(size_t s);
};

///  Rigid body states on device.
struct FsiBodyStateD {
    thrust::device_vector<Real3> pos;      ///< body linear positions
    thrust::device_vector<Real3> lin_vel;  ///< body linear velocities
    thrust::device_vector<Real3> lin_acc;  ///< body linear accelerations
    thrust::device_vector<Real4> rot;      ///< body orientations (quaternions)
    thrust::device_vector<Real3> ang_vel;  ///< body angular velocities (local frame)
    thrust::device_vector<Real3> ang_acc;  ///< body angular accelerations (local frame)

    zipIterRigidD iterator();
    void CopyFromH(const FsiBodyStateH& bodyStateH);
    FsiBodyStateD& operator=(const FsiBodyStateD& other);
    void resize(size_t s);
};

/// FEA mesh states on host.
struct FsiMeshStateH {
    thrust::host_vector<Real3> pos_fsi_fea_H;  ///< mesh node positions
    thrust::host_vector<Real3> vel_fsi_fea_H;  ///< mesh node velocities
    thrust::host_vector<Real3> acc_fsi_fea_H;  ///< mesh node accelerations

    // zipIterFlexH iterator();
    void resize(size_t s);
    size_t size() { return pos_fsi_fea_H.size(); };
};

/// FEA mesh state on device.
struct FsiMeshStateD {
    thrust::device_vector<Real3> pos_fsi_fea_D;  ///< mesh node positions
    thrust::device_vector<Real3> vel_fsi_fea_D;  ///< mesh node velocities
    thrust::device_vector<Real3> acc_fsi_fea_D;  ///< mesh node accelerations

    // zipIterFlexD iterator();
    void CopyFromH(const FsiMeshStateH& meshStateH);
    FsiMeshStateD& operator=(const FsiMeshStateD& other);
    void resize(size_t s);
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

    size_t startRigidMarkers;   ///< index of first BCE marker on first rigid body
    size_t startFlexMarkers1D;  ///< index of first BCE marker on first flex segment
    size_t startFlexMarkers2D;  ///< index of first BCE marker on first flex face
};

// -----------------------------------------------------------------------------

/// Data manager for the SPH-based FSI system.
class FsiDataManager {
  public:
    FsiDataManager(std::shared_ptr<SimParams> params);
    virtual ~FsiDataManager();

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
                    unsigned int num_fsi_elements2D);

    /// Find indices of all SPH particles inside the specified OBB.
    thrust::device_vector<int> FindParticlesInBox(const Real3& hsize,
                                                  const Real3& pos,
                                                  const Real3& ax,
                                                  const Real3& ay,
                                                  const Real3& az);

    /// Extract positions of all markers (SPH and BCE).
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetPositions();

    /// Extract velocities of all markers (SPH and BCE).
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetVelocities();

    /// Extract accelerations of all markers (SPH and BCE).
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetAccelerations();

    /// Extract forces applied to all markers (SPH and BCE).
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetForces();

    /// Extract fluid properties of all markers (SPH and BCE).
    /// For each SPH particle, the 3-dimensional vector contains density, pressure, and viscosity.
    thrust::device_vector<Real3> GetProperties();

    /// Extract positions of all markers (SPH and BCE) with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetPositions(const thrust::device_vector<int>& indices);

    /// Extract velocities of all markers (SPH and BCE) with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetVelocities(const thrust::device_vector<int>& indices);

    /// Extract accelerations of all markers (SPH and BCE) with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetAccelerations(const thrust::device_vector<int>& indices);

    /// Extract forces applied to all markers (SPH and BCE) with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetForces(const thrust::device_vector<int>& indices);

    std::shared_ptr<SimParams> paramsH;   ///< simulation parameters (host)
    std::shared_ptr<Counters> countersH;  ///< problem counters (host)

    std::shared_ptr<SphMarkerDataD> sphMarkers_D;         ///< Information of SPH particles at state 1 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkers1_D;  ///< Information of SPH particles at state 2 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkers2_D;  ///< Sorted information of SPH particles at state 1 on device
    std::shared_ptr<SphMarkerDataH> sphMarkers_H;         ///< Information of SPH particles on host

    std::shared_ptr<FsiBodyStateH> fsiBodyState_H;  ///< rigid body state (host)
    std::shared_ptr<FsiBodyStateD> fsiBodyState_D;  ///< rigid body state 2 (device)

    std::shared_ptr<FsiMeshStateH> fsiMesh1DState_H;  ///< 1-D FEA mesh state (host)
    std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D;  ///< 1-D FEA mesh state (device)
    std::shared_ptr<FsiMeshStateH> fsiMesh2DState_H;  ///< 2-D FEA mesh state (host)
    std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D;  ///< 2-D FEA mesh state (device)

    std::shared_ptr<ProximityDataD> markersProximity_D;  ///< Information of neighbor search on the device

    std::shared_ptr<CudaDeviceInfo> cudaDeviceInfo;  ///< CUDA device information

    // fluidfsiBodiesIndex (host)
    thrust::host_vector<int4> referenceArray;      ///< phases in the array of SPH particles
    thrust::host_vector<int4> referenceArray_FEA;  ///< phases in the array of SPH particles for flexible elements

    // Fluid data (device)
    thrust::device_vector<Real4> derivVelRhoD;  ///< dv/dt and d(rho)/dt for particles
    thrust::device_vector<Real4>
        derivVelRhoOriginalD;  ///< dv/dt and d(rho)/dt used for writing partilces in file - unsorted

    thrust::device_vector<Real3> derivTauXxYyZzD;  ///< d(tau)/dt for particles
    thrust::device_vector<Real3> derivTauXyXzYzD;  ///< d(tau)/dt for particles
    thrust::device_vector<Real3> vel_XSPH_D;       ///< XSPH velocity for particles
    thrust::device_vector<Real3> vis_vel_SPH_D;    ///< ISPH velocity for particles
    thrust::device_vector<Real4> sr_tau_I_mu_i;    ///< I2SPH strain-rate, stress, inertia number, friction
    thrust::device_vector<Real4>
        sr_tau_I_mu_i_Original;  ///< I2SPH strain-rate, stress, inertia number, friction - unsorted for writing
    thrust::device_vector<Real3> bceAcc;  ///< Acceleration for boundary/rigid/flex body particles

    thrust::device_vector<uint> activityIdentifierD;  ///< Identifies if a particle is an active particle or not
    thrust::device_vector<uint> extendedActivityIdD;  ///< Identifies if a particle is in an extended active domain
    thrust::device_vector<uint>
        numNeighborsPerPart;                   ///< Stores the number of neighbors the particle, given by the index, has
    thrust::device_vector<uint> neighborList;  ///< Stores the neighbor list - all neighbors are just stored one by one
                                               ///< - The above vector provides the info required to idenitfy which
                                               ///< particles neighbors are stored at which index of neighborList

    thrust::device_vector<uint> freeSurfaceIdD;  ///< Identifies if a particle is close to free surface

    // BCE
    thrust::device_vector<Real3> rigid_BCEcoords_D;   ///< rigid body BCE position (local reference frame)
    thrust::host_vector<Real3> flex1D_BCEcoords_H;    ///< local coords for BCE markers on 1-D flex segments (host)
    thrust::device_vector<Real3> flex1D_BCEcoords_D;  ///< local coords for BCE markers on 1-D flex segments (device)
    thrust::host_vector<Real3> flex2D_BCEcoords_H;    ///< local coords for BCE markers on 2-D flex faces (host)
    thrust::device_vector<Real3> flex2D_BCEcoords_D;  ///< local coors for BCE markers on 2-D flex faces (device)

    thrust::device_vector<uint> rigid_BCEsolids_D;    ///< associated body ID for BCE markers on rigid bodies
    thrust::host_vector<uint3> flex1D_BCEsolids_H;    ///< associated mesh and segment for BCE markers on 1-D segments
    thrust::device_vector<uint3> flex1D_BCEsolids_D;  ///< associated mesh and segment for BCE markers on 1-D segments
    thrust::host_vector<uint3> flex2D_BCEsolids_H;    ///< associated mesh and face for BCE markers on 2-D faces
    thrust::device_vector<uint3> flex2D_BCEsolids_D;  ///< associated mesh and face for BCE markers on 2-D faces

    // FSI bodies
    thrust::device_vector<Real3> rigid_FSI_ForcesD;   ///< surface-integrated forces to rigid bodies
    thrust::device_vector<Real3> rigid_FSI_TorquesD;  ///< surface-integrated torques to rigid bodies

    thrust::device_vector<Real3> flex1D_FSIforces_D;  ///< surface-integrated forces on FEA 1-D segment nodes
    thrust::device_vector<Real3> flex2D_FSIforces_D;  ///< surface-integrated forces on FEA 2-D face nodes

    thrust::host_vector<int2> flex1D_Nodes_H;    ///< node indices for each 1-D flex segment (host)
    thrust::device_vector<int2> flex1D_Nodes_D;  ///< node indices for each 1-D flex segment (device)
    thrust::host_vector<int3> flex2D_Nodes_H;    ///< node indices for each 2-D flex face (host)
    thrust::device_vector<int3> flex2D_Nodes_D;  ///< node indices for each 2-D flex face (device)

  private:
    void ConstructReferenceArray();
    void SetCounters(unsigned int num_fsi_bodies,
                     unsigned int num_fsi_nodes1D,
                     unsigned int num_fsi_elements1D,
                     unsigned int num_fsi_nodes2D,
                     unsigned int num_fsi_elements2D);

    /// Initialize the midpoint device data of the fluid system by copying from the full step.
    void CopyDeviceDataToHalfStep();

    /// Reset device data at beginning of a step.
    /// Initializes device vectors to zero.
    void ResetData();

    friend class chrono::fsi::ChFluidSystemSPH;
};

/// @} fsi_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
