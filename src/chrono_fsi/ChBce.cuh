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
// Base class for processing boundary condition enforcing (bce) marker forces
// in an FSI system.
//
// =============================================================================

#ifndef CH_BCE_CUH_
#define CH_BCE_CUH_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiDataManager.cuh"  //for FsiGeneralData
#include "chrono_fsi/ChFsiGeneral.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for processing boundary condition enforcing (BCE) marker forces
/// in an FSI system.
class CH_FSI_API ChBce : public ChFsiGeneral {
  public:
    thrust::device_vector<Real3> velMas_ModifiedBCE;    //(numRigidAndBoundaryMarkers);
    thrust::device_vector<Real4> rhoPreMu_ModifiedBCE;  //(numRigidAndBoundaryMarkers);

    ChBce(SphMarkerDataD* sortedSphMarkersD,
          ProximityDataD* markersProximityD,
          FsiGeneralData* otherFsiGeneralData,
          SimParams* otherParamsH,
          NumberOfObjects* otherNumObjects);

    ~ChBce();

    virtual void UpdateRigidMarkersPositionVelocity(SphMarkerDataD* sphMarkersD, FsiBodiesDataD* fsiBodiesD);

    virtual void Rigid_Forces_Torques(SphMarkerDataD* sphMarkersD, FsiBodiesDataD* fsiBodiesD);

    void ModifyBceVelocity(SphMarkerDataD* sphMarkersD, FsiBodiesDataD* fsiBodiesD);

    virtual void Populate_RigidSPH_MeshPos_LRF(SphMarkerDataD* sphMarkersD, FsiBodiesDataD* fsiBodiesD);

    virtual void Finalize(SphMarkerDataD* sphMarkersD, FsiBodiesDataD* fsiBodiesD);

  private:
    FsiGeneralData* fsiGeneralData;
    SphMarkerDataD* sortedSphMarkersD;
    ProximityDataD* markersProximityD;

    SimParams* paramsH;
    NumberOfObjects* numObjectsH;

    thrust::device_vector<Real4> totalSurfaceInteractionRigid4;
    thrust::device_vector<Real3> torqueMarkersD;
    thrust::device_vector<int> dummyIdentify;
    void CalcBceAcceleration(thrust::device_vector<Real3>& bceAcc,
                             const thrust::device_vector<Real4>& q_fsiBodies_D,
                             const thrust::device_vector<Real3>& accRigid_fsiBodies_D,
                             const thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,
                             const thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,
                             const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
                             const thrust::device_vector<uint>& rigidIdentifierD,
                             int numRigid_SphMarkers);

    void RecalcSortedVelocityPressure_BCE(thrust::device_vector<Real3>& velMas_ModifiedBCE,
                                          thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,
                                          const thrust::device_vector<Real3>& sortedPosRad,
                                          const thrust::device_vector<Real3>& sortedVelMas,
                                          const thrust::device_vector<Real4>& sortedRhoPreMu,
                                          const thrust::device_vector<uint>& cellStart,
                                          const thrust::device_vector<uint>& cellEnd,
                                          const thrust::device_vector<uint>& mapOriginalToSorted,
                                          const thrust::device_vector<Real3>& bceAcc,
                                          int2 updatePortion);

    void MakeRigidIdentifier();
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
