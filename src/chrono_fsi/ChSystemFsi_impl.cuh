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
// Author: Milad Rakhsha, Arman Pazouki
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

    /// typedef device iterators for shorthand sph operation of thrust vectors of Real3
    typedef thrust::device_vector<Real3>::iterator r3IterD;
    /// typedef device iterators for shorthand sph operation of thrust vectors of Real4
    typedef thrust::device_vector<Real4>::iterator r4IterD;
    /// typedef device tuple for holding sph data pos,vel,[rho,pressure,mu,type]
    typedef thrust::tuple<r4IterD, r3IterD, r4IterD, r3IterD, r3IterD> iterTupleSphD;  
    typedef thrust::zip_iterator<iterTupleSphD> zipIterSphD;

    /// typedef host iterators for shorthand sph operation of thrust vectors of Real3
    typedef thrust::host_vector<Real3>::iterator r3IterH;
    /// typedef host iterators for shorthand sph operation of thrust vectors of Real4
    typedef thrust::host_vector<Real4>::iterator r4IterH;
    /// typedef host tuple for holding sph data pos,vel,[rho,pressure,mu,type]
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


    /// Struct to store the information of SPH markers on the device
    struct SphMarkerDataD {

        thrust::device_vector<Real4> posRadD;     ///< Vector of the positions of markers + characteristic radius
        thrust::device_vector<Real3> velMasD;     ///< Vector of the velocities of markers
        thrust::device_vector<Real4> rhoPresMuD;  ///< Vector of the rho+pressure+mu+type of markers
        thrust::device_vector<Real3> tauXxYyZzD;  ///< Vector of the shear stress (diagonal) of markers
        thrust::device_vector<Real3> tauXyXzYzD;  ///< Vector of the shear stress (off-diagonal) of markers

        zipIterSphD iterator();
        void resize(size_t s);
    };

    /// Struct to store the information of SPH markers on the host
    struct SphMarkerDataH {
        thrust::host_vector<Real4> posRadH;     ///< Vector of the positions of markers
        thrust::host_vector<Real3> velMasH;     ///< Vector of the velocities of markers
        thrust::host_vector<Real4> rhoPresMuH;  ///< Vector of the rho+pressure+mu+type of markers
        thrust::host_vector<Real3> tauXxYyZzH;  ///< Vector of the shear stress (diagonal) of markers
        thrust::host_vector<Real3> tauXyXzYzH;  ///< Vector of the shear stress (off-diagonal) of markers

        zipIterSphH iterator();
        void resize(size_t s);
    };

    /// Struct to store the information of rigid bodies on the host
    struct FsiBodiesDataH {
        thrust::host_vector<Real3> posRigid_fsiBodies_H;      ///< Vector of the linear positions of rigid bodies
        thrust::host_vector<Real4> velMassRigid_fsiBodies_H;  ///< Vector of the linear velocities of rigid bodies
        thrust::host_vector<Real3> accRigid_fsiBodies_H;      ///< Vector of the linear acceleration of rigid bodies
        thrust::host_vector<Real4> q_fsiBodies_H;  ///< Vector of the orientations (Euler parameters as Quaternion) of rigid bodies
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
        thrust::device_vector<Real4> q_fsiBodies_D;  ///< Vector of the orientations (Euler parameters as Quaternion) of rigid bodies
        thrust::device_vector<Real3> omegaVelLRF_fsiBodies_D;  ///< Vector of the angular velocities of rigid bodies
        thrust::device_vector<Real3> omegaAccLRF_fsiBodies_D;  ///< Vector of the angular acceleration of rigid bodies

        zipIterRigidD iterator();
        void CopyFromH(const FsiBodiesDataH& other);
        FsiBodiesDataD& operator=(const FsiBodiesDataD& other);
        void resize(size_t s);
    };

    struct FsiMeshDataH {
        thrust::host_vector<Real3> pos_fsi_fea_H;
        thrust::host_vector<Real3> vel_fsi_fea_H;
        thrust::host_vector<Real3> acc_fsi_fea_H;

        // zipIterFlexH iterator();
        
        void resize(size_t s);
        size_t size() { return pos_fsi_fea_H.size(); };
    };

    struct FsiMeshDataD {
        thrust::device_vector<Real3> pos_fsi_fea_D;
        thrust::device_vector<Real3> vel_fsi_fea_D;
        thrust::device_vector<Real3> acc_fsi_fea_D;

        // zipIterFlexD iterator();
        void CopyFromH(const FsiMeshDataH& other);
        FsiMeshDataD& operator=(const FsiMeshDataD& other);
        void resize(size_t s);
    };

    struct FsiShellsDataH {
        thrust::host_vector<Real3> posFlex_fsiBodies_nA_H;
        thrust::host_vector<Real3> posFlex_fsiBodies_nB_H;
        thrust::host_vector<Real3> posFlex_fsiBodies_nC_H;
        thrust::host_vector<Real3> posFlex_fsiBodies_nD_H;

        thrust::host_vector<Real3> velFlex_fsiBodies_nA_H;
        thrust::host_vector<Real3> velFlex_fsiBodies_nB_H;
        thrust::host_vector<Real3> velFlex_fsiBodies_nC_H;
        thrust::host_vector<Real3> velFlex_fsiBodies_nD_H;

        thrust::host_vector<Real3> accFlex_fsiBodies_nA_H;
        thrust::host_vector<Real3> accFlex_fsiBodies_nB_H;
        thrust::host_vector<Real3> accFlex_fsiBodies_nC_H;
        thrust::host_vector<Real3> accFlex_fsiBodies_nD_H;

        // zipIterFlexH iterator();
        void resize(size_t s);
    };

    struct FsiShellsDataD {
        thrust::device_vector<Real3> posFlex_fsiBodies_nA_D;
        thrust::device_vector<Real3> posFlex_fsiBodies_nB_D;
        thrust::device_vector<Real3> posFlex_fsiBodies_nC_D;
        thrust::device_vector<Real3> posFlex_fsiBodies_nD_D;

        thrust::device_vector<Real3> velFlex_fsiBodies_nA_D;
        thrust::device_vector<Real3> velFlex_fsiBodies_nB_D;
        thrust::device_vector<Real3> velFlex_fsiBodies_nC_D;
        thrust::device_vector<Real3> velFlex_fsiBodies_nD_D;

        thrust::device_vector<Real3> accFlex_fsiBodies_nA_D;
        thrust::device_vector<Real3> accFlex_fsiBodies_nB_D;
        thrust::device_vector<Real3> accFlex_fsiBodies_nC_D;
        thrust::device_vector<Real3> accFlex_fsiBodies_nD_D;

        // zipIterFlexD iterator();
        void CopyFromH(const FsiShellsDataH& other);
        FsiShellsDataD& operator=(const FsiShellsDataD& other);
        void resize(size_t s);
    };

    /// Struct to store neighbor search information on the device
    struct ProximityDataD {
        thrust::device_vector<uint> gridMarkerHashD;   ///< gridMarkerHash=s(i,j,k)= k*n_x*n_y + j*n_x + i (numAllMarkers);
        thrust::device_vector<uint> gridMarkerIndexD;  ///< (numAllMarkers);
        thrust::device_vector<uint> cellStartD;  ///< Index of the marker starts a cell in sorted list (m_numGridCells)
        thrust::device_vector<uint> cellEndD;    ///< Index of the marker ends a cell in sorted list (m_numGridCells)
        thrust::device_vector<uint> mapOriginalToSorted;

        void resize(size_t numAllMarkers);
    };

    struct ChronoBodiesDataH {
        ChronoBodiesDataH() {}
        ChronoBodiesDataH(size_t s);
        thrust::host_vector<Real3> pos_ChSystemH;  ///< Vector of the linear positions of rigid bodies
        thrust::host_vector<Real3> vel_ChSystemH;  ///< Vector of the linear velocities of rigid bodies
        thrust::host_vector<Real3> acc_ChSystemH;  ///< Vector of the linear accelerations of rigid bodies
        thrust::host_vector<Real4> quat_ChSystemH;  ///< Vector of the orientations (Euler parameters as Quaternion) of rigid bodies
        thrust::host_vector<Real3> omegaVelGRF_ChSystemH;  ///< Vector of the angular velocities of rigid bodies
        thrust::host_vector<Real3> omegaAccGRF_ChSystemH;  ///< Vector of the angular acceleraion of rigid bodies

        zipIterChronoBodiesH iterator();
        void resize(size_t s);
    };

    struct ChronoShellsDataH {
        ChronoShellsDataH() {}
        ChronoShellsDataH(size_t s);

        // zipIterChronoShellsH iterator();

        thrust::host_vector<Real3> posFlex_ChSystemH_nA_H;
        thrust::host_vector<Real3> posFlex_ChSystemH_nB_H;
        thrust::host_vector<Real3> posFlex_ChSystemH_nC_H;
        thrust::host_vector<Real3> posFlex_ChSystemH_nD_H;

        thrust::host_vector<Real3> velFlex_ChSystemH_nA_H;
        thrust::host_vector<Real3> velFlex_ChSystemH_nB_H;
        thrust::host_vector<Real3> velFlex_ChSystemH_nC_H;
        thrust::host_vector<Real3> velFlex_ChSystemH_nD_H;

        thrust::host_vector<Real3> accFlex_ChSystemH_nA_H;
        thrust::host_vector<Real3> accFlex_ChSystemH_nB_H;
        thrust::host_vector<Real3> accFlex_ChSystemH_nC_H;
        thrust::host_vector<Real3> accFlex_ChSystemH_nD_H;

        void resize(size_t s);
    };
    struct ChronoMeshDataH {
        ChronoMeshDataH() {}
        ChronoMeshDataH(size_t s);

        thrust::host_vector<Real3> posFlex_ChSystemH_H;
        thrust::host_vector<Real3> velFlex_ChSystemH_H;
        thrust::host_vector<Real3> accFlex_ChSystemH_H;

        void resize(size_t s);
    };
    /// Struct to store fluid system information that need to be passed to Chrono
    struct FsiGeneralData {
        // ----------------
        //  host
        // ----------------
        // fluidfsiBodiesIndex
        thrust::host_vector<int4> referenceArray;  ///< Holds information of each phase in the array of sph markers
        thrust::host_vector<int4> referenceArray_FEA;  ///< Holds information of each phase in the array of sph markers for Flexible elements
        // ----------------
        //  device
        // ----------------
        // fluid
        thrust::device_vector<Real4> derivVelRhoD;      ///< dv/dt and d(rho)/dt for markers
        thrust::device_vector<Real4> derivVelRhoD_old;  ///< dv/dt and d(rho)/dt for markers,

        thrust::device_vector<Real3> derivTauXxYyZzD;  ///< d(tau)/dt for markers
        thrust::device_vector<Real3> derivTauXyXzYzD;  ///< d(tau)/dt for markers

        thrust::device_vector<Real3> vel_XSPH_D;     ///< XSPH velocity for markers
        thrust::device_vector<Real3> vis_vel_SPH_D;  ///< IISPH velocity for markers
        thrust::device_vector<Real4> sr_tau_I_mu_i;  ///< I2SPH strain-rate, stress, inertia number, friction

        // BCE
        thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D;  ///< Position of a marker attached to a rigid body in a local
        thrust::device_vector<Real3> FlexSPH_MeshPos_LRF_D;
        thrust::host_vector<Real3> FlexSPH_MeshPos_LRF_H;

        thrust::device_vector<uint> rigidIdentifierD;  ///< Identifies which rigid body a marker belongs to
        thrust::device_vector<uint> FlexIdentifierD;

        // fsi bodies
        thrust::device_vector<Real3> rigid_FSI_ForcesD;   ///< Vector of the surface-integrated forces to rigid bodies
        thrust::device_vector<Real3> rigid_FSI_TorquesD;  ///< Vector of the surface-integrated torques to rigid bodies
        thrust::device_vector<Real3> Flex_FSI_ForcesD;    ///< Vector of the surface-integrated force on FEA nodes

        thrust::host_vector<int2> CableElementsNodesH;
        thrust::device_vector<int2> CableElementsNodes;

        thrust::host_vector<int4> ShellElementsNodesH;
        thrust::device_vector<int4> ShellElementsNodes;
    };

class ChSystemFsi_impl : public ChFsiGeneral {
  public:
    ChSystemFsi_impl();
    virtual ~ChSystemFsi_impl();

    void AddSphMarker(Real4 pos, Real4 rhoPresMu, Real3 vel = mR3(0.0), Real3 tauXxYyZz = mR3(0.0), Real3 tauXyXzYz = mR3(0.0));
    void ResizeDataManager(int numNode = 0);

    std::shared_ptr<NumberOfObjects> numObjects;

    std::shared_ptr<SphMarkerDataD> sphMarkersD1;       ///< Information of SPH markers at state 1 on device
    std::shared_ptr<SphMarkerDataD> sphMarkersD2;       ///< Information of SPH markers at state 2 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkersD;  ///< Sorted information of SPH markers at state 1 on device
    std::shared_ptr<SphMarkerDataH> sphMarkersH;        ///< Information of SPH markers on host

    std::shared_ptr<FsiBodiesDataD> fsiBodiesD1;  ///< Information of rigid bodies at state 1 on device
    std::shared_ptr<FsiBodiesDataD> fsiBodiesD2;  ///< Information of rigid bodies at state 2 on device
    std::shared_ptr<FsiBodiesDataH> fsiBodiesH;   ///< Information of rigid bodies at state 1 on host
    std::shared_ptr<FsiMeshDataD> fsiMeshD;
    std::shared_ptr<FsiMeshDataH> fsiMeshH;

    std::shared_ptr<FsiGeneralData> fsiGeneralData;

    std::shared_ptr<ProximityDataD> markersProximityD;

  private:
    void ArrangeDataManager();
    void ConstructReferenceArray();
    void InitNumObjects();
    void CalcNumObjects();  ///< Calculates the number of rigid bodies, flexible bodies, etc. based on the type of markers

  public:
    friend class ChSystemFsi;

};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
