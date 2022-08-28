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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration.
//
// =============================================================================

#ifndef CH_SYSTEMFSI_IMPL_H_
#define CH_SYSTEMFSI_IMPL_H_

#include "chrono/ChConfig.h"

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/detail/normal_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

#include "chrono_fsi/physics/ChFsiGeneral.h"
#include "chrono_fsi/math/custom_math.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {

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
/// pos,orientation in position, velocity and acceleration level
typedef thrust::tuple<r3IterD, r4IterD, r3IterD, r4IterD, r3IterD, r3IterD> iterTupleRigidD;
typedef thrust::zip_iterator<iterTupleRigidD> zipIterRigidD;

/// typedef host iterators for shorthand rigid body states:
/// pos,orientation in position, velocity and acceleration level
typedef thrust::tuple<r3IterH, r4IterH, r3IterH, r4IterH, r3IterH, r3IterH> iterTupleRigidH;
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

/// Struct to store the information of SPH particles on the host
struct SphMarkerDataH {
    thrust::host_vector<Real4> posRadH;     ///< Vector of the positions of particles
    thrust::host_vector<Real3> velMasH;     ///< Vector of the velocities of particles
    thrust::host_vector<Real4> rhoPresMuH;  ///< Vector of the rho+pressure+mu+type of particles
    thrust::host_vector<Real3> tauXxYyZzH;  ///< Vector of the total stress (diagonal) of particles
    thrust::host_vector<Real3> tauXyXzYzH;  ///< Vector of the total stress (off-diagonal) of particles

    zipIterSphH iterator();
    void resize(size_t s);
};

/// Struct to store the information of rigid bodies on the host
struct FsiBodiesDataH {
    thrust::host_vector<Real3> posRigid_fsiBodies_H;      ///< Vector of the linear positions of rigid bodies
    thrust::host_vector<Real4> velMassRigid_fsiBodies_H;  ///< Vector of the linear velocities of rigid bodies
    thrust::host_vector<Real3> accRigid_fsiBodies_H;      ///< Vector of the linear acceleration of rigid bodies
    thrust::host_vector<Real4>
        q_fsiBodies_H;  ///< Vector of the orientations (Euler parameters as Quaternion) of rigid bodies
    thrust::host_vector<Real3> omegaVelLRF_fsiBodies_H;  ///< Vector of the angular velocities of rigid bodies
    thrust::host_vector<Real3> omegaAccLRF_fsiBodies_H;  ///< Vector of the angular acceleration of rigid bodies

    zipIterRigidH iterator();
    void resize(size_t s);
};

/// Struct to store the information of rigid bodies on the device
struct FsiBodiesDataD {
    thrust::device_vector<Real3> posRigid_fsiBodies_D;      ///< Vector of the linear positions of rigid bodies
    thrust::device_vector<Real4> velMassRigid_fsiBodies_D;  ///< Vector of the linear velocities of rigid bodies
    thrust::device_vector<Real3> accRigid_fsiBodies_D;      ///< Vector of the linear acceleration of rigid bodies
    thrust::device_vector<Real4>
        q_fsiBodies_D;  ///< Vector of the orientations (Euler parameters as Quaternion) of rigid bodies
    thrust::device_vector<Real3> omegaVelLRF_fsiBodies_D;  ///< Vector of the angular velocities of rigid bodies
    thrust::device_vector<Real3> omegaAccLRF_fsiBodies_D;  ///< Vector of the angular acceleration of rigid bodies

    zipIterRigidD iterator();
    void CopyFromH(const FsiBodiesDataH& other);
    FsiBodiesDataD& operator=(const FsiBodiesDataD& other);
    void resize(size_t s);
};

/// Struct to store the information of mesh on the host
struct FsiMeshDataH {
    thrust::host_vector<Real3> pos_fsi_fea_H;  ///< Vector of the mesh position
    thrust::host_vector<Real3> vel_fsi_fea_H;  ///< Vector of the mesh velocity
    thrust::host_vector<Real3> acc_fsi_fea_H;  ///< Vector of the mesh acceleration
    thrust::host_vector<Real3> dir_fsi_fea_H;  ///< Vector of the mesh direction

    // zipIterFlexH iterator();
    void resize(size_t s);
    size_t size() { return pos_fsi_fea_H.size(); };
};

/// Struct to store the information of mesh on the device
struct FsiMeshDataD {
    thrust::device_vector<Real3> pos_fsi_fea_D;  ///< Vector of the mesh position
    thrust::device_vector<Real3> vel_fsi_fea_D;  ///< Vector of the mesh velocity
    thrust::device_vector<Real3> acc_fsi_fea_D;  ///< Vector of the mesh acceleration
    thrust::device_vector<Real3> dir_fsi_fea_D;  ///< Vector of the mesh direction

    // zipIterFlexD iterator();
    void CopyFromH(const FsiMeshDataH& other);
    FsiMeshDataD& operator=(const FsiMeshDataD& other);
    void resize(size_t s);
};

/// Struct to store the information of shell elements on the host
struct FsiShellsDataH {
    thrust::host_vector<Real3> posFlex_fsiBodies_nA_H;  ///< Vector of the node A position
    thrust::host_vector<Real3> posFlex_fsiBodies_nB_H;  ///< Vector of the node B position
    thrust::host_vector<Real3> posFlex_fsiBodies_nC_H;  ///< Vector of the node C position
    thrust::host_vector<Real3> posFlex_fsiBodies_nD_H;  ///< Vector of the node D position

    thrust::host_vector<Real3> velFlex_fsiBodies_nA_H;  ///< Vector of the node A velocity
    thrust::host_vector<Real3> velFlex_fsiBodies_nB_H;  ///< Vector of the node B velocity
    thrust::host_vector<Real3> velFlex_fsiBodies_nC_H;  ///< Vector of the node C velocity
    thrust::host_vector<Real3> velFlex_fsiBodies_nD_H;  ///< Vector of the node D velocity

    thrust::host_vector<Real3> accFlex_fsiBodies_nA_H;  ///< Vector of the node A acceleration
    thrust::host_vector<Real3> accFlex_fsiBodies_nB_H;  ///< Vector of the node B acceleration
    thrust::host_vector<Real3> accFlex_fsiBodies_nC_H;  ///< Vector of the node C acceleration
    thrust::host_vector<Real3> accFlex_fsiBodies_nD_H;  ///< Vector of the node D acceleration

    // zipIterFlexH iterator();
    void resize(size_t s);
};

/// Struct to store the information of shell elements on the device
struct FsiShellsDataD {
    thrust::device_vector<Real3> posFlex_fsiBodies_nA_D;  ///< Vector of the node A position
    thrust::device_vector<Real3> posFlex_fsiBodies_nB_D;  ///< Vector of the node B position
    thrust::device_vector<Real3> posFlex_fsiBodies_nC_D;  ///< Vector of the node C position
    thrust::device_vector<Real3> posFlex_fsiBodies_nD_D;  ///< Vector of the node D position

    thrust::device_vector<Real3> velFlex_fsiBodies_nA_D;  ///< Vector of the node A velocity
    thrust::device_vector<Real3> velFlex_fsiBodies_nB_D;  ///< Vector of the node B velocity
    thrust::device_vector<Real3> velFlex_fsiBodies_nC_D;  ///< Vector of the node C velocity
    thrust::device_vector<Real3> velFlex_fsiBodies_nD_D;  ///< Vector of the node D velocity

    thrust::device_vector<Real3> accFlex_fsiBodies_nA_D;  ///< Vector of the node A acceleration
    thrust::device_vector<Real3> accFlex_fsiBodies_nB_D;  ///< Vector of the node B acceleration
    thrust::device_vector<Real3> accFlex_fsiBodies_nC_D;  ///< Vector of the node C acceleration
    thrust::device_vector<Real3> accFlex_fsiBodies_nD_D;  ///< Vector of the node D acceleration

    // zipIterFlexD iterator();
    void CopyFromH(const FsiShellsDataH& other);
    FsiShellsDataD& operator=(const FsiShellsDataD& other);
    void resize(size_t s);
};

/// Struct to store neighbor search information on the device
struct ProximityDataD {
    thrust::device_vector<uint> gridMarkerHashD;   ///< gridMarkerHash=s(i,j,k)= k*n_x*n_y + j*n_x + i (numAllMarkers);
    thrust::device_vector<uint> gridMarkerIndexD;  ///< Marker's index, can be original or sorted (numAllMarkers);
    thrust::device_vector<uint> cellStartD;  ///< Index of the particle starts a cell in sorted list (m_numGridCells)
    thrust::device_vector<uint> cellEndD;    ///< Index of the particle ends a cell in sorted list (m_numGridCells)
    thrust::device_vector<uint>
        mapOriginalToSorted;  ///< Index mapping from the original to the sorted (numAllMarkers);

    void resize(size_t s);
};

/// Struct to store Chrono rigid bodies information on the host
struct ChronoBodiesDataH {
    ChronoBodiesDataH() {}
    ChronoBodiesDataH(size_t s);
    thrust::host_vector<Real3> pos_ChSystemH;  ///< Vector of the linear positions of rigid bodies
    thrust::host_vector<Real3> vel_ChSystemH;  ///< Vector of the linear velocities of rigid bodies
    thrust::host_vector<Real3> acc_ChSystemH;  ///< Vector of the linear accelerations of rigid bodies
    thrust::host_vector<Real4>
        quat_ChSystemH;  ///< Vector of the orientations (Euler parameters as Quaternion) of rigid bodies
    thrust::host_vector<Real3> omegaVelGRF_ChSystemH;  ///< Vector of the angular velocities of rigid bodies
    thrust::host_vector<Real3> omegaAccGRF_ChSystemH;  ///< Vector of the angular acceleraion of rigid bodies

    zipIterChronoBodiesH iterator();
    void resize(size_t s);
};

/// Struct to store Chrono shell elements information on the host
struct ChronoShellsDataH {
    ChronoShellsDataH() {}
    ChronoShellsDataH(size_t s);

    // zipIterChronoShellsH iterator();

    thrust::host_vector<Real3> posFlex_ChSystemH_nA_H;  ///< Vector of the node A position
    thrust::host_vector<Real3> posFlex_ChSystemH_nB_H;  ///< Vector of the node B position
    thrust::host_vector<Real3> posFlex_ChSystemH_nC_H;  ///< Vector of the node C position
    thrust::host_vector<Real3> posFlex_ChSystemH_nD_H;  ///< Vector of the node D position

    thrust::host_vector<Real3> velFlex_ChSystemH_nA_H;  ///< Vector of the node A velocity
    thrust::host_vector<Real3> velFlex_ChSystemH_nB_H;  ///< Vector of the node B velocity
    thrust::host_vector<Real3> velFlex_ChSystemH_nC_H;  ///< Vector of the node C velocity
    thrust::host_vector<Real3> velFlex_ChSystemH_nD_H;  ///< Vector of the node D velocity

    thrust::host_vector<Real3> accFlex_ChSystemH_nA_H;  ///< Vector of the node A acceleration
    thrust::host_vector<Real3> accFlex_ChSystemH_nB_H;  ///< Vector of the node B acceleration
    thrust::host_vector<Real3> accFlex_ChSystemH_nC_H;  ///< Vector of the node C acceleration
    thrust::host_vector<Real3> accFlex_ChSystemH_nD_H;  ///< Vector of the node D acceleration

    void resize(size_t s);
};

/// Struct to store Chrono mesh information on the host
struct ChronoMeshDataH {
    ChronoMeshDataH() {}
    ChronoMeshDataH(size_t s);

    thrust::host_vector<Real3> posFlex_ChSystemH_H;  ///< Vector of the mesh position
    thrust::host_vector<Real3> velFlex_ChSystemH_H;  ///< Vector of the mesh velocity
    thrust::host_vector<Real3> accFlex_ChSystemH_H;  ///< Vector of the mesh acceleration
    thrust::host_vector<Real3> dirFlex_ChSystemH_H;  ///< Vector of the mesh direction

    void resize(size_t s);
};

/// Struct to store fluid/granular system information that need to be passed to Chrono
struct FsiGeneralData {
    // ----------------
    //  host
    // ----------------
    // fluidfsiBodiesIndex
    thrust::host_vector<int4> referenceArray;  ///< Holds information of each phase in the array of SPH particles
    thrust::host_vector<int4>
        referenceArray_FEA;  ///< Holds information of each phase in the array of SPH particles for Flexible elements
    // ----------------
    //  device
    // ----------------
    // fluid
    thrust::device_vector<Real4> derivVelRhoD;      ///< dv/dt and d(rho)/dt for particles
    thrust::device_vector<Real4> derivVelRhoD_old;  ///< dv/dt and d(rho)/dt for particles
    thrust::device_vector<Real3> derivTauXxYyZzD;   ///< d(tau)/dt for particles
    thrust::device_vector<Real3> derivTauXyXzYzD;   ///< d(tau)/dt for particles
    thrust::device_vector<Real3> vel_XSPH_D;        ///< XSPH velocity for particles
    thrust::device_vector<Real3> vis_vel_SPH_D;     ///< IISPH velocity for particles
    thrust::device_vector<Real4> sr_tau_I_mu_i;     ///< I2SPH strain-rate, stress, inertia number, friction

    thrust::device_vector<uint> activityIdentifierD;  ///< Identifies if a particle is an active particle or not
    thrust::device_vector<uint> extendedActivityIdD;  ///< Identifies if a particle is in an extended active domain

    thrust::device_vector<uint> freeSurfaceIdD;  ///< Identifies if a particle is close to free surface

    // BCE
    thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D;  ///< Position of a particle attached to a rigid body in a local
    thrust::device_vector<Real3> FlexSPH_MeshPos_LRF_D;  ///< Position of a particle attached to a mesh in a local on device
    thrust::host_vector<Real3> FlexSPH_MeshPos_LRF_H;  ///< Position of a particle attached to a mesh in a local on host

    thrust::device_vector<uint> rigidIdentifierD;  ///< Identifies which rigid body a particle belongs to
    thrust::device_vector<uint> FlexIdentifierD;   ///< Identifies which flexible body a particle belongs to

    // fsi bodies
    thrust::device_vector<Real3> rigid_FSI_ForcesD;   ///< Vector of the surface-integrated forces to rigid bodies
    thrust::device_vector<Real3> rigid_FSI_TorquesD;  ///< Vector of the surface-integrated torques to rigid bodies
    thrust::device_vector<Real3> Flex_FSI_ForcesD;    ///< Vector of the surface-integrated force on FEA nodes

    thrust::host_vector<int2> CableElementsNodesH;   ///< Vector of the cable elements dodes on host
    thrust::device_vector<int2> CableElementsNodes;  ///< Vector of the cable elements dodes on device

    thrust::host_vector<int4> ShellElementsNodesH;   ///< Vector of the shell elements dodes on host
    thrust::device_vector<int4> ShellElementsNodes;  ///< Vector of the shell elements dodes on device
};

/// @brief Data related function implementations for FSI system
class ChSystemFsi_impl : public ChFsiGeneral {
  public:
    ChSystemFsi_impl(std::shared_ptr<SimParams> params);
    virtual ~ChSystemFsi_impl();

    /// Add an SPH particle given its position, physical properties, velocity, and stress
    void AddSPHParticle(Real4 pos,
                        Real4 rhoPresMu,
                        Real3 vel = mR3(0.0),
                        Real3 tauXxYyZz = mR3(0.0),
                        Real3 tauXyXzYz = mR3(0.0));

    /// Resize the simulation data based on the FSI system constructed.
    void ResizeData(size_t numRigidBodies, size_t numFlexBodies1D, size_t numFlexBodies2D, size_t numFlexNodes);

    /// Extract forces applied on all SPH particles.
    thrust::device_vector<Real4> GetParticleForces();

    /// Find indices of all SPH particles inside the specified OBB.
    thrust::device_vector<int> FindParticlesInBox(const Real3& hsize,
                                                  const Real3& pos,
                                                  const Real3& ax,
                                                  const Real3& ay,
                                                  const Real3& az);

    /// Extract positions of all SPH particles with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real4> GetParticlePositions(const thrust::device_vector<int>& indices);

    /// Extract velocities of all SPH particles with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetParticleVelocities(const thrust::device_vector<int>& indices);

    /// Extract forces applied to all SPH particles with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real4> GetParticleForces(const thrust::device_vector<int>& indices);

    std::shared_ptr<SimParams> paramsH;      ///< Parameters of the simulation
    std::shared_ptr<ChCounters> numObjects;  ///< Counters (SPH particles, BCE particles, bodies, etc)

    std::shared_ptr<SphMarkerDataD> sphMarkersD1;       ///< Information of SPH particles at state 1 on device
    std::shared_ptr<SphMarkerDataD> sphMarkersD2;       ///< Information of SPH particles at state 2 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkersD;  ///< Sorted information of SPH particles at state 1 on device
    std::shared_ptr<SphMarkerDataH> sphMarkersH;        ///< Information of SPH particles on host

    std::shared_ptr<FsiBodiesDataD> fsiBodiesD1;  ///< Information of rigid bodies at state 1 on device
    std::shared_ptr<FsiBodiesDataD> fsiBodiesD2;  ///< Information of rigid bodies at state 2 on device
    std::shared_ptr<FsiBodiesDataH> fsiBodiesH;   ///< Information of rigid bodies at state 1 on host
    std::shared_ptr<FsiMeshDataD> fsiMeshD;       ///< Information of mesh on device
    std::shared_ptr<FsiMeshDataH> fsiMeshH;       ///< Information of mesh on host

    std::shared_ptr<FsiGeneralData> fsiGeneralData;  ///< General FSI data needed in the simulation

    std::shared_ptr<ProximityDataD> markersProximityD;  ///< Information of neighbor search on the device

  private:
    void ArrangeDataManager();
    void ConstructReferenceArray();
    void InitNumObjects();
    void CalcNumObjects();

  public:
    /// Base class of FSI system.
    friend class ChSystemFsi;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
