// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki
// =============================================================================
//
// Base class for managing data in chrono_fsi, aka fluid system.//
// =============================================================================

#ifndef CH_FSI_DATAMANAGER_H_
#define CH_FSI_DATAMANAGER_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChParams.cuh"
#include "chrono_fsi/custom_math.h"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

namespace chrono {
namespace fsi {

/// typedef device iterators for shorthand sph operation of thrust vectors of Real3
typedef thrust::device_vector<Real3>::iterator r3IterD;
/// typedef device iterators for shorthand sph operation of thrust vectors of Real4
typedef thrust::device_vector<Real4>::iterator r4IterD;
/// typedef device tuple for holding sph data pos,vel,[rho,pressure,mu,type]
typedef thrust::tuple<r3IterD, r3IterD, r4IterD> iterTupleSphD;
typedef thrust::zip_iterator<iterTupleSphD> zipIterSphD;

/// typedef host iterators for shorthand sph operation of thrust vectors of Real3
typedef thrust::host_vector<Real3>::iterator r3IterH;
/// typedef host iterators for shorthand sph operation of thrust vectors of Real4
typedef thrust::host_vector<Real4>::iterator r4IterH;
/// typedef host tuple for holding sph data pos,vel,[rho,pressure,mu,type]
typedef thrust::tuple<r3IterH, r3IterH, r4IterH> iterTupleH;
typedef thrust::zip_iterator<iterTupleH> zipIterSphH;

/// typedef device iterators for shorthand rigid body states:
/// pos,orientation in position, velocity and acceleration level
typedef thrust::tuple<r3IterD, r4IterD, r3IterD, r4IterD, r3IterD, r3IterD> iterTupleRigidD;
typedef thrust::zip_iterator<iterTupleRigidD> zipIterRigidD;

/// typedef host iterators for shorthand rigid body states:
/// pos,orientation in position, velocity and acceleration level
typedef thrust::tuple<r3IterH, r4IterH, r3IterH, r4IterH, r3IterH, r3IterH> iterTupleRigidH;
typedef thrust::zip_iterator<iterTupleRigidH> zipIterRigidH;

// typedef device iterators for shorthand chrono bodies operations
typedef thrust::tuple<r3IterH, r3IterH, r3IterH, r4IterH, r3IterH, r3IterH> iterTupleChronoBodiesH;
typedef thrust::zip_iterator<iterTupleChronoBodiesH> zipIterChronoBodiesH;

/**
 * @brief Number of fluid markers, solid bodies, solid markers, boundary markers
 * @details
 * 		This structure holds necessary information for the fsi system to be finalized. Note that the order of makers in
 * 		the memory is as follows: i) fluid markers, ii) markers attached to fixed objects (boundary markers), iii)
 * 		markers attached to rigid bodies, and iv) markers attached to flexible bodies
 */
struct NumberOfObjects {
    int numRigidBodies;       ///< Number of rigid bodies
    int numFlexBodies;        ///< Number of Flexible bodies*/
    int numFluidMarkers;      ///< Number of fluid SPH markers*/
    int numBoundaryMarkers;   ///< Number of boundary SPH markers*/
    int startRigidMarkers;    ///< Index of the first SPH marker that covers the first rigid body.
    int startFlexMarkers;     ///< Index of the first SPH marker that covers the first flexible body.
    int numRigid_SphMarkers;  ///< Number of SPH markers attached to rigid bodies
    int numFlex_SphMarkers;   ///< Number of SPH markers attached to flexible bodies
    int numAllMarkers;        ///< Total number of SPH markers in the simulation
};

/// Class for storing the information of SPH markers on the device
class SphMarkerDataD {
  public:
    thrust::device_vector<Real3> posRadD;     ///< Vector of the positions of markers
    thrust::device_vector<Real3> velMasD;     ///< Vector of the velocities of markers
    thrust::device_vector<Real4> rhoPresMuD;  ///< Vector of the rho+pressure+mu+type of markers

    zipIterSphD iterator();

    // resize
    void resize(int s);

  private:
};

/// Class for storing the information of SPH markers on the host
class SphMarkerDataH {
  public:
    thrust::host_vector<Real3> posRadH;     ///< Vector of the positions of markers
    thrust::host_vector<Real3> velMasH;     ///< Vector of the velocities of markers
    thrust::host_vector<Real4> rhoPresMuH;  ///< Vector of the rho+pressure+mu+type of markers

    zipIterSphH iterator();

    // resize
    void resize(int s);

  private:
};

/// Class for storing the information of rigid bodies of the simulation on the host
class FsiBodiesDataH {
  public:
    thrust::host_vector<Real3> posRigid_fsiBodies_H;      ///< Vector of the linear positions of rigid bodies
    thrust::host_vector<Real4> velMassRigid_fsiBodies_H;  ///< Vector of the linear velocities of rigid bodies
    thrust::host_vector<Real3> accRigid_fsiBodies_H;      ///< Vector of the linear acceleration of rigid bodies
    thrust::host_vector<Real4>
        q_fsiBodies_H;  ///< Vector of the orientations (Euler parameters as Quaternion) of rigid bodies
    thrust::host_vector<Real3> omegaVelLRF_fsiBodies_H;  ///< Vector of the angular velocities of rigid bodies
    thrust::host_vector<Real3> omegaAccLRF_fsiBodies_H;  ///< Vector of the angular acceleration of rigid bodies

    zipIterRigidH iterator();

    // resize
    void resize(int s);

  private:
};

/// Class for storing the information of rigid bodies of the simulation on the device
class FsiBodiesDataD {
  public:
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

    // resize
    void resize(int s);

  private:
};

/// Class for storing the neighbor search informations on the device
class ProximityDataD {
  public:
    thrust::device_vector<uint> gridMarkerHashD;   ///< gridMarkerHash=s(i,j,k)= k*n_x*n_y + j*n_x + i (numAllMarkers);
    thrust::device_vector<uint> gridMarkerIndexD;  ///< (numAllMarkers);
    thrust::device_vector<uint> cellStartD;  ///< Index of the marker starts a cell in sorted list (m_numGridCells)
    thrust::device_vector<uint> cellEndD;    ///< Index of the marker ends a cell in sorted list (m_numGridCells)
    thrust::device_vector<uint> mapOriginalToSorted;

    // resize
    void resize(int numAllMarkers);

  private:
};

class ChronoBodiesDataH {
  public:
    ChronoBodiesDataH() {}
    ChronoBodiesDataH(int s);
    thrust::host_vector<Real3> pos_ChSystemH;  ///< Vector of the linear positions of rigid bodies
    thrust::host_vector<Real3> vel_ChSystemH;  ///< Vector of the linear velocities of rigid bodies
    thrust::host_vector<Real3> acc_ChSystemH;  ///< Vector of the linear accelerations of rigid bodies
    thrust::host_vector<Real4>
        quat_ChSystemH;  ///< Vector of the orientations (Euler parameters as Quaternion) of rigid bodies
    thrust::host_vector<Real3> omegaVelGRF_ChSystemH;  ///< Vector of the angular velocities of rigid bodies
    thrust::host_vector<Real3> omegaAccGRF_ChSystemH;  ///< Vector of the angular acceleraion of rigid bodies

    zipIterChronoBodiesH iterator();

    // resize
    void resize(int s);

  private:
};

/// Class to hold information of the fluid system that needs to be passed to Chrono
class FsiGeneralData {
  public:
    // ----------------
    //  host
    // ----------------
    // fluidfsiBodeisIndex
    thrust::host_vector<::int4> referenceArray;  ///< Holds information of each phase in the array of sph markers
    // ----------------
    //  device
    // ----------------
    // fluid
    thrust::device_vector<Real4> derivVelRhoD;  ///< dv/dt and d(rho)/dt for markers
    thrust::device_vector<Real3> vel_XSPH_D;    ///< XSPH velocity for markers

    // BCE
    thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D;  ///< Position of a marker attached to a rigid body in a local
                                                          ///refrence frame of the body
    thrust::device_vector<uint> rigidIdentifierD;         ///< Identifies which rigid body a marker belongs to

    // fsi bodies
    thrust::device_vector<Real3> rigid_FSI_ForcesD;   ///< Vector of the surface-integrated forces to rigid bodies
    thrust::device_vector<Real3> rigid_FSI_TorquesD;  ///< Vector of the surface-integrated torques to rigid bodies

  private:
};

/// Data manager class that holds all the information of the SPH markers and MBD system
class CH_FSI_API ChFsiDataManager {
  public:
    ChFsiDataManager();
    ~ChFsiDataManager();

    void AddSphMarker(Real3 pos, Real3 vel, Real4 rhoPresMu);
    void ResizeDataManager();

    NumberOfObjects numObjects;

    SphMarkerDataD sphMarkersD1;       ///< Information of SPH markers at state 1 on device
    SphMarkerDataD sphMarkersD2;       ///< Information of SPH markers at state 2 on device
    SphMarkerDataD sortedSphMarkersD;  ///< Sorted information of SPH markers at state 1 on device
    SphMarkerDataH sphMarkersH;        ///< Information of SPH markers on host

    FsiBodiesDataD fsiBodiesD1;  ///< Information of rigid bodies at state 1 on device
    FsiBodiesDataD fsiBodiesD2;  ///< Information of rigid bodies at state 2 on device
    FsiBodiesDataH fsiBodiesH;   ///< Information of rigid bodies at state 1 on host

    FsiGeneralData fsiGeneralData;

    ProximityDataD markersProximityD;

  private:
    void ArrangeDataManager();
    void ConstructReferenceArray();
    void InitNumObjects();
    void
    CalcNumObjects();  ///< Calculates the number of rigid bodies, flexible bodies, etc. based on the type of markers
};

}  // end namespace fsi
}  // end namespace chrono

#endif /* CH_FSI_DATAMANAGER_H_ */
