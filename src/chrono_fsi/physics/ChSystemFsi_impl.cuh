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

#ifndef CH_SYSTEMFSI_IMPL_H_
#define CH_SYSTEMFSI_IMPL_H_

#include "chrono/ChConfig.h"

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/detail/normal_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

#include "chrono_fsi/physics/ChFsiBase.h"
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
    thrust::host_vector<Real4> lin_vel;  ///< body linear velocities
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
    thrust::device_vector<Real4> lin_vel;  ///< body linear velocities
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

/// FSI system information information exchanged with the Chrono system.
struct FsiData {
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
    thrust::device_vector<Real3> rigid_BCEcoords_D;   ///< Rigid body BCE position (local reference frame)
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
    thrust::device_vector<Real3> rigid_FSI_ForcesD;   ///< Vector of the surface-integrated forces to rigid bodies
    thrust::device_vector<Real3> rigid_FSI_TorquesD;  ///< Vector of the surface-integrated torques to rigid bodies

    thrust::device_vector<Real3> flex1D_FSIforces_D;  ///< surface-integrated forces on FEA 1-D segment nodes
    thrust::device_vector<Real3> flex2D_FSIforces_D;  ///< surface-integrated forces on FEA 2-D face nodes

    thrust::host_vector<int2> flex1D_Nodes_H;    ///< node indices for each 1-D flex segment (host)
    thrust::device_vector<int2> flex1D_Nodes_D;  ///< node indices for each 1-D flex segment (device)
    thrust::host_vector<int3> flex2D_Nodes_H;    ///< node indices for each 2-D flex face (host)
    thrust::device_vector<int3> flex2D_Nodes_D;  ///< node indices for each 2-D flex face (device)
};

/// Underlying implementation of an FSI system.
class ChSystemFsi_impl : public ChFsiBase {
  public:
    ChSystemFsi_impl(std::shared_ptr<SimParams> params);
    virtual ~ChSystemFsi_impl();

    /// Add an SPH particle given its position, physical properties, velocity, and stress.
    void AddSPHParticle(Real4 pos,
                        Real4 rhoPresMu,
                        Real3 vel = mR3(0.0),
                        Real3 tauXxYyZz = mR3(0.0),
                        Real3 tauXyXzYz = mR3(0.0));

    /// Initialize the underlying FSU system.
    /// Set reference arrays, set counters, and resize simulation arrays.
    void Initialize(size_t numRigidBodies,
                    size_t numFlexBodies1D,
                    size_t numFlexBodies2D,
                    size_t numFlexNodes1D,
                    size_t numFlexNodes2D);

    /// Extract forces applied on all SPH particles.
    thrust::device_vector<Real4> GetParticleForces();

    /// Extract accelerations of all SPH particles.
    thrust::device_vector<Real4> GetParticleAccelerations();

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

    /// Extract accelerations of all SPH particles with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real4> GetParticleAccelerations(const thrust::device_vector<int>& indices);

    std::shared_ptr<SphMarkerDataD> sphMarkers_D;         ///< Information of SPH particles at state 1 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkers1_D;  ///< Information of SPH particles at state 2 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkers2_D;  ///< Sorted information of SPH particles at state 1 on device
    std::shared_ptr<SphMarkerDataH> sphMarkers_H;         ///< Information of SPH particles on host

    std::shared_ptr<FsiBodyStateH> fsiBodyState_H;   ///< Rigid body state (host)
    std::shared_ptr<FsiBodyStateD> fsiBodyState1_D;  ///< Rigid body state 1 (device)
    std::shared_ptr<FsiBodyStateD> fsiBodyState2_D;  ///< Rigid body state 2 (device)

    std::shared_ptr<FsiMeshStateH> fsiMesh1DState_H;  ///< 1-D FEA mesh state (host)
    std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D;  ///< 1-D FEA mesh state (device)
    std::shared_ptr<FsiMeshStateH> fsiMesh2DState_H;  ///< 2-D FEA mesh state (host)
    std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D;  ///< 2-D FEA mesh state (device)

    std::shared_ptr<FsiData> fsiData;  ///< simulation FSI data

    std::shared_ptr<ProximityDataD> markersProximity_D;  ///< Information of neighbor search on the device
  private:
    void ArrangeDataManager();
    void ConstructReferenceArray();
    void InitNumObjects();
    void CalcNumObjects();

    friend class ChSystemFsi;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
