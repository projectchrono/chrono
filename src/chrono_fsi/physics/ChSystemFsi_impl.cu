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

#include <thrust/copy.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>
#include <thrust/transform.h>

#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"

namespace chrono {
namespace fsi {

struct sphTypeCompEqual {
    __host__ __device__ bool operator()(const Real4& o1, const Real4& o2) { return o1.w == o2.w; }
};
//---------------------------------------------------------------------------------------
zipIterSphD SphMarkerDataD::iterator() {
    return thrust::make_zip_iterator(thrust::make_tuple(posRadD.begin(), velMasD.begin(), rhoPresMuD.begin(),
                                                        tauXxYyZzD.begin(), tauXyXzYzD.begin()));
}

void SphMarkerDataD::resize(size_t s) {
    posRadD.resize(s);
    velMasD.resize(s);
    rhoPresMuD.resize(s);
    tauXxYyZzD.resize(s);
    tauXyXzYzD.resize(s);
}

//---------------------------------------------------------------------------------------
zipIterSphH SphMarkerDataH::iterator() {
    return thrust::make_zip_iterator(thrust::make_tuple(posRadH.begin(), velMasH.begin(), rhoPresMuH.begin(),
                                                        tauXxYyZzH.begin(), tauXyXzYzH.begin()));
}

// resize
void SphMarkerDataH::resize(size_t s) {
    posRadH.resize(s);
    velMasH.resize(s);
    rhoPresMuH.resize(s);
    tauXxYyZzH.resize(s);
    tauXyXzYzH.resize(s);
}

//---------------------------------------------------------------------------------------
zipIterRigidD FsiBodiesDataD::iterator() {
    return thrust::make_zip_iterator(
        thrust::make_tuple(posRigid_fsiBodies_D.begin(), velMassRigid_fsiBodies_D.begin(), accRigid_fsiBodies_D.begin(),
                           q_fsiBodies_D.begin(), omegaVelLRF_fsiBodies_D.begin(), omegaAccLRF_fsiBodies_D.begin()));
}

void FsiBodiesDataD::resize(size_t s) {
    posRigid_fsiBodies_D.resize(s);
    velMassRigid_fsiBodies_D.resize(s);
    accRigid_fsiBodies_D.resize(s);
    q_fsiBodies_D.resize(s);
    omegaVelLRF_fsiBodies_D.resize(s);
    omegaAccLRF_fsiBodies_D.resize(s);
}

void FsiShellsDataH::resize(size_t s) {
    posFlex_fsiBodies_nA_H.resize(s);
    posFlex_fsiBodies_nB_H.resize(s);
    posFlex_fsiBodies_nC_H.resize(s);
    posFlex_fsiBodies_nD_H.resize(s);

    velFlex_fsiBodies_nA_H.resize(s);
    velFlex_fsiBodies_nB_H.resize(s);
    velFlex_fsiBodies_nC_H.resize(s);
    velFlex_fsiBodies_nD_H.resize(s);

    accFlex_fsiBodies_nA_H.resize(s);
    accFlex_fsiBodies_nB_H.resize(s);
    accFlex_fsiBodies_nC_H.resize(s);
    accFlex_fsiBodies_nD_H.resize(s);
}

void FsiShellsDataD::resize(size_t s) {
    posFlex_fsiBodies_nA_D.resize(s);
    posFlex_fsiBodies_nB_D.resize(s);
    posFlex_fsiBodies_nC_D.resize(s);
    posFlex_fsiBodies_nD_D.resize(s);

    velFlex_fsiBodies_nA_D.resize(s);
    velFlex_fsiBodies_nB_D.resize(s);
    velFlex_fsiBodies_nC_D.resize(s);
    velFlex_fsiBodies_nD_D.resize(s);

    accFlex_fsiBodies_nA_D.resize(s);
    accFlex_fsiBodies_nB_D.resize(s);
    accFlex_fsiBodies_nC_D.resize(s);
    accFlex_fsiBodies_nD_D.resize(s);
}
void FsiMeshDataH::resize(size_t s) {
    pos_fsi_fea_H.resize(s);
    vel_fsi_fea_H.resize(s);
    acc_fsi_fea_H.resize(s);
    dir_fsi_fea_H.resize(s);
}
void FsiMeshDataD::resize(size_t s) {
    pos_fsi_fea_D.resize(s);
    vel_fsi_fea_D.resize(s);
    acc_fsi_fea_D.resize(s);
    dir_fsi_fea_D.resize(s);
}

void FsiBodiesDataD::CopyFromH(const FsiBodiesDataH& other) {
    thrust::copy(other.posRigid_fsiBodies_H.begin(), other.posRigid_fsiBodies_H.end(), posRigid_fsiBodies_D.begin());
    thrust::copy(other.velMassRigid_fsiBodies_H.begin(), other.velMassRigid_fsiBodies_H.end(),
                 velMassRigid_fsiBodies_D.begin());
    thrust::copy(other.accRigid_fsiBodies_H.begin(), other.accRigid_fsiBodies_H.end(), accRigid_fsiBodies_D.begin());
    thrust::copy(other.q_fsiBodies_H.begin(), other.q_fsiBodies_H.end(), q_fsiBodies_D.begin());
    thrust::copy(other.omegaVelLRF_fsiBodies_H.begin(), other.omegaVelLRF_fsiBodies_H.end(),
                 omegaVelLRF_fsiBodies_D.begin());
    thrust::copy(other.omegaAccLRF_fsiBodies_H.begin(), other.omegaAccLRF_fsiBodies_H.end(),
                 omegaAccLRF_fsiBodies_D.begin());
}

void FsiShellsDataD::CopyFromH(const FsiShellsDataH& other) {
    thrust::copy(other.posFlex_fsiBodies_nA_H.begin(), other.posFlex_fsiBodies_nA_H.end(),
                 posFlex_fsiBodies_nA_D.begin());
    thrust::copy(other.posFlex_fsiBodies_nB_H.begin(), other.posFlex_fsiBodies_nB_H.end(),
                 posFlex_fsiBodies_nB_D.begin());
    thrust::copy(other.posFlex_fsiBodies_nC_H.begin(), other.posFlex_fsiBodies_nC_H.end(),
                 posFlex_fsiBodies_nC_D.begin());
    thrust::copy(other.posFlex_fsiBodies_nD_H.begin(), other.posFlex_fsiBodies_nD_H.end(),
                 posFlex_fsiBodies_nD_D.begin());

    thrust::copy(other.velFlex_fsiBodies_nA_H.begin(), other.velFlex_fsiBodies_nA_H.end(),
                 velFlex_fsiBodies_nA_D.begin());
    thrust::copy(other.velFlex_fsiBodies_nB_H.begin(), other.velFlex_fsiBodies_nB_H.end(),
                 velFlex_fsiBodies_nB_D.begin());
    thrust::copy(other.velFlex_fsiBodies_nC_H.begin(), other.velFlex_fsiBodies_nC_H.end(),
                 velFlex_fsiBodies_nC_D.begin());
    thrust::copy(other.velFlex_fsiBodies_nD_H.begin(), other.velFlex_fsiBodies_nD_H.end(),
                 velFlex_fsiBodies_nD_D.begin());

    thrust::copy(other.accFlex_fsiBodies_nA_H.begin(), other.accFlex_fsiBodies_nA_H.end(),
                 accFlex_fsiBodies_nA_D.begin());
    thrust::copy(other.accFlex_fsiBodies_nB_H.begin(), other.accFlex_fsiBodies_nB_H.end(),
                 accFlex_fsiBodies_nB_D.begin());
    thrust::copy(other.accFlex_fsiBodies_nC_H.begin(), other.accFlex_fsiBodies_nC_H.end(),
                 accFlex_fsiBodies_nC_D.begin());
    thrust::copy(other.accFlex_fsiBodies_nD_H.begin(), other.accFlex_fsiBodies_nD_H.end(),
                 accFlex_fsiBodies_nD_D.begin());
}

void FsiMeshDataD::CopyFromH(const FsiMeshDataH& other) {
    thrust::copy(other.pos_fsi_fea_H.begin(), other.pos_fsi_fea_H.end(), pos_fsi_fea_D.begin());
    thrust::copy(other.vel_fsi_fea_H.begin(), other.vel_fsi_fea_H.end(), vel_fsi_fea_D.begin());
    thrust::copy(other.acc_fsi_fea_H.begin(), other.acc_fsi_fea_H.end(), acc_fsi_fea_D.begin());
    thrust::copy(other.dir_fsi_fea_H.begin(), other.dir_fsi_fea_H.end(), dir_fsi_fea_D.begin());
}

FsiBodiesDataD& FsiBodiesDataD::operator=(const FsiBodiesDataD& other) {
    if (this == &other) {
        return *this;
    }
    thrust::copy(other.posRigid_fsiBodies_D.begin(), other.posRigid_fsiBodies_D.end(), posRigid_fsiBodies_D.begin());
    thrust::copy(other.velMassRigid_fsiBodies_D.begin(), other.velMassRigid_fsiBodies_D.end(),
                 velMassRigid_fsiBodies_D.begin());
    thrust::copy(other.accRigid_fsiBodies_D.begin(), other.accRigid_fsiBodies_D.end(), accRigid_fsiBodies_D.begin());
    thrust::copy(other.q_fsiBodies_D.begin(), other.q_fsiBodies_D.end(), q_fsiBodies_D.begin());
    thrust::copy(other.omegaVelLRF_fsiBodies_D.begin(), other.omegaVelLRF_fsiBodies_D.end(),
                 omegaVelLRF_fsiBodies_D.begin());
    thrust::copy(other.omegaAccLRF_fsiBodies_D.begin(), other.omegaAccLRF_fsiBodies_D.end(),
                 omegaAccLRF_fsiBodies_D.begin());
    return *this;
}

FsiShellsDataD& FsiShellsDataD::operator=(const FsiShellsDataD& other) {
    if (this == &other) {
        return *this;
    }
    thrust::copy(other.posFlex_fsiBodies_nA_D.begin(), other.posFlex_fsiBodies_nA_D.end(),
                 posFlex_fsiBodies_nA_D.begin());

    thrust::copy(other.posFlex_fsiBodies_nB_D.begin(), other.posFlex_fsiBodies_nB_D.end(),
                 posFlex_fsiBodies_nB_D.begin());
    thrust::copy(other.posFlex_fsiBodies_nC_D.begin(), other.posFlex_fsiBodies_nC_D.end(),
                 posFlex_fsiBodies_nC_D.begin());
    thrust::copy(other.posFlex_fsiBodies_nD_D.begin(), other.posFlex_fsiBodies_nD_D.end(),
                 posFlex_fsiBodies_nD_D.begin());

    thrust::copy(other.velFlex_fsiBodies_nA_D.begin(), other.velFlex_fsiBodies_nA_D.end(),
                 velFlex_fsiBodies_nA_D.begin());
    thrust::copy(other.velFlex_fsiBodies_nB_D.begin(), other.velFlex_fsiBodies_nB_D.end(),
                 velFlex_fsiBodies_nB_D.begin());
    thrust::copy(other.velFlex_fsiBodies_nC_D.begin(), other.velFlex_fsiBodies_nC_D.end(),
                 velFlex_fsiBodies_nC_D.begin());
    thrust::copy(other.velFlex_fsiBodies_nD_D.begin(), other.velFlex_fsiBodies_nD_D.end(),
                 velFlex_fsiBodies_nD_D.begin());

    thrust::copy(other.accFlex_fsiBodies_nA_D.begin(), other.accFlex_fsiBodies_nA_D.end(),
                 posFlex_fsiBodies_nA_D.begin());
    thrust::copy(other.accFlex_fsiBodies_nB_D.begin(), other.accFlex_fsiBodies_nB_D.end(),
                 accFlex_fsiBodies_nB_D.begin());
    thrust::copy(other.accFlex_fsiBodies_nC_D.begin(), other.accFlex_fsiBodies_nC_D.end(),
                 accFlex_fsiBodies_nC_D.begin());
    thrust::copy(other.accFlex_fsiBodies_nD_D.begin(), other.accFlex_fsiBodies_nD_D.end(),
                 accFlex_fsiBodies_nD_D.begin());
    return *this;
}

FsiMeshDataD& FsiMeshDataD::operator=(const FsiMeshDataD& other) {
    if (this == &other) {
        return *this;
    }
    thrust::copy(other.pos_fsi_fea_D.begin(), other.pos_fsi_fea_D.end(), pos_fsi_fea_D.begin());
    thrust::copy(other.vel_fsi_fea_D.begin(), other.vel_fsi_fea_D.end(), vel_fsi_fea_D.begin());
    thrust::copy(other.acc_fsi_fea_D.begin(), other.acc_fsi_fea_D.end(), acc_fsi_fea_D.begin());
    thrust::copy(other.dir_fsi_fea_D.begin(), other.dir_fsi_fea_D.end(), dir_fsi_fea_D.begin());
    return *this;
}

//---------------------------------------------------------------------------------------
zipIterRigidH FsiBodiesDataH::iterator() {
    return thrust::make_zip_iterator(
        thrust::make_tuple(posRigid_fsiBodies_H.begin(), velMassRigid_fsiBodies_H.begin(), accRigid_fsiBodies_H.begin(),
                           q_fsiBodies_H.begin(), omegaVelLRF_fsiBodies_H.begin(), omegaAccLRF_fsiBodies_H.begin()));
}

void FsiBodiesDataH::resize(size_t s) {
    posRigid_fsiBodies_H.resize(s);
    velMassRigid_fsiBodies_H.resize(s);
    accRigid_fsiBodies_H.resize(s);
    q_fsiBodies_H.resize(s);
    omegaVelLRF_fsiBodies_H.resize(s);
    omegaAccLRF_fsiBodies_H.resize(s);
}

//---------------------------------------------------------------------------------------
void ProximityDataD::resize(size_t s) {
    gridMarkerHashD.resize(s);
    gridMarkerIndexD.resize(s);
    mapOriginalToSorted.resize(s);
}

//---------------------------------------------------------------------------------------
ChronoBodiesDataH::ChronoBodiesDataH(size_t s) {
    resize(s);
}

ChronoShellsDataH::ChronoShellsDataH(size_t s) {
    resize(s);
}

ChronoMeshDataH::ChronoMeshDataH(size_t s) {
    resize(s);
}
zipIterChronoBodiesH ChronoBodiesDataH::iterator() {
    return thrust::make_zip_iterator(thrust::make_tuple(pos_ChSystemH.begin(), vel_ChSystemH.begin(),
                                                        acc_ChSystemH.begin(), quat_ChSystemH.begin(),
                                                        omegaVelGRF_ChSystemH.begin(), omegaAccGRF_ChSystemH.begin()));
}

void ChronoBodiesDataH::resize(size_t s) {
    pos_ChSystemH.resize(s);
    vel_ChSystemH.resize(s);
    acc_ChSystemH.resize(s);
    quat_ChSystemH.resize(s);
    omegaVelGRF_ChSystemH.resize(s);
    omegaAccGRF_ChSystemH.resize(s);
}

void ChronoShellsDataH::resize(size_t s) {
    posFlex_ChSystemH_nA_H.resize(s);
    posFlex_ChSystemH_nB_H.resize(s);
    posFlex_ChSystemH_nC_H.resize(s);
    posFlex_ChSystemH_nD_H.resize(s);

    velFlex_ChSystemH_nA_H.resize(s);
    velFlex_ChSystemH_nB_H.resize(s);
    velFlex_ChSystemH_nC_H.resize(s);
    velFlex_ChSystemH_nD_H.resize(s);

    accFlex_ChSystemH_nA_H.resize(s);
    accFlex_ChSystemH_nB_H.resize(s);
    accFlex_ChSystemH_nC_H.resize(s);
    accFlex_ChSystemH_nD_H.resize(s);
}

void ChronoMeshDataH::resize(size_t s) {
    posFlex_ChSystemH_H.resize(s);
    velFlex_ChSystemH_H.resize(s);
    accFlex_ChSystemH_H.resize(s);
    dirFlex_ChSystemH_H.resize(s);
}

//---------------------------------------------------------------------------------------

ChSystemFsi_impl::ChSystemFsi_impl(std::shared_ptr<SimParams> params) : paramsH(params) {
    numObjects = chrono_types::make_shared<ChCounters>();
    InitNumObjects();
    sphMarkersD1 = chrono_types::make_shared<SphMarkerDataD>();
    sphMarkersD2 = chrono_types::make_shared<SphMarkerDataD>();
    sortedSphMarkersD = chrono_types::make_shared<SphMarkerDataD>();
    sphMarkersH = chrono_types::make_shared<SphMarkerDataH>();
    fsiBodiesD1 = chrono_types::make_shared<FsiBodiesDataD>();
    fsiBodiesD2 = chrono_types::make_shared<FsiBodiesDataD>();
    fsiBodiesH = chrono_types::make_shared<FsiBodiesDataH>();
    fsiMeshD = chrono_types::make_shared<FsiMeshDataD>();
    fsiMeshH = chrono_types::make_shared<FsiMeshDataH>();
    fsiGeneralData = chrono_types::make_shared<FsiGeneralData>();
    markersProximityD = chrono_types::make_shared<ProximityDataD>();
}

ChSystemFsi_impl::~ChSystemFsi_impl() {}

void ChSystemFsi_impl::AddSPHParticle(Real4 pos, Real4 rhoPresMu, Real3 vel, Real3 tauXxYyZz, Real3 tauXyXzYz) {
    sphMarkersH->posRadH.push_back(pos);
    sphMarkersH->velMasH.push_back(vel);
    sphMarkersH->rhoPresMuH.push_back(rhoPresMu);
    sphMarkersH->tauXyXzYzH.push_back(tauXyXzYz);
    sphMarkersH->tauXxYyZzH.push_back(tauXxYyZz);
}
void ChSystemFsi_impl::ArrangeDataManager() {
    thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkersH->rhoPresMuH;
    dummyRhoPresMuH.clear();
}

void ChSystemFsi_impl::InitNumObjects() {
    numObjects->numRigidBodies = 0;      // Number of rigid bodies
    numObjects->numFlexBodies1D = 0;     // Number of 1D Flexible bodies
    numObjects->numFlexBodies2D = 0;     // Number of 2D Flexible bodies
    numObjects->numFlexNodes = 0;        // Number of FE nodes
    numObjects->numGhostMarkers = 0;     // Number of ghost particles
    numObjects->numHelperMarkers = 0;    // Number of helper particles
    numObjects->numFluidMarkers = 0;     // Number of fluid SPH particles
    numObjects->numBoundaryMarkers = 0;  // Number of boundary SPH particles
    numObjects->startRigidMarkers = 0;   // Start index of the rigid SPH particles
    numObjects->startFlexMarkers = 0;    // Start index of the flexible SPH particles
    numObjects->numRigidMarkers = 0;     // Number of rigid SPH particles
    numObjects->numFlexMarkers = 0;      // Number of flexible SPH particles
    numObjects->numAllMarkers = 0;       // Total number of SPH particles
}

void ChSystemFsi_impl::CalcNumObjects() {
    InitNumObjects();
    size_t rSize = fsiGeneralData->referenceArray.size();

    for (size_t i = 0; i < rSize; i++) {
        int4 rComp4 = fsiGeneralData->referenceArray[i];
        int numMarkers = rComp4.y - rComp4.x;

        switch (rComp4.z) {
            case -3:
                numObjects->numHelperMarkers += numMarkers;
                break;
            case -2:
                numObjects->numGhostMarkers += numMarkers;
                break;
            case -1:
                numObjects->numFluidMarkers += numMarkers;
                break;
            case 0:
                numObjects->numBoundaryMarkers += numMarkers;
                break;
            case 1:
                numObjects->numRigidMarkers += numMarkers;
                numObjects->numRigidBodies++;
                break;
            case 2:
                numObjects->numFlexMarkers += numMarkers;
                numObjects->numFlexBodies1D++;
                break;
            case 3:
                numObjects->numFlexMarkers += numMarkers;
                numObjects->numFlexBodies2D++;
                break;
            default:
                std::cerr << "ERROR (CalcNumObjects): particle type not defined." << std::endl;
                throw std::runtime_error("Particle type not defined.");
                break;
        }
    }

    numObjects->numFluidMarkers += numObjects->numGhostMarkers + numObjects->numHelperMarkers;
    numObjects->numAllMarkers = numObjects->numFluidMarkers + numObjects->numBoundaryMarkers +
                                numObjects->numRigidMarkers + numObjects->numFlexMarkers;

    numObjects->startRigidMarkers = numObjects->numFluidMarkers + numObjects->numBoundaryMarkers;
    numObjects->startFlexMarkers =
        numObjects->numFluidMarkers + numObjects->numBoundaryMarkers + numObjects->numRigidMarkers;
}

void ChSystemFsi_impl::ConstructReferenceArray() {
    auto numAllMarkers = sphMarkersH->rhoPresMuH.size();

    thrust::host_vector<int> numComponentMarkers(numAllMarkers);
    thrust::fill(numComponentMarkers.begin(), numComponentMarkers.end(), 1);
    thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkersH->rhoPresMuH;
    thrust::copy(sphMarkersH->rhoPresMuH.begin(), sphMarkersH->rhoPresMuH.end(), dummyRhoPresMuH.begin());
    size_t numberOfComponents =
        (thrust::reduce_by_key(dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(), numComponentMarkers.begin(),
                               dummyRhoPresMuH.begin(), numComponentMarkers.begin(), sphTypeCompEqual()))
            .first -
        dummyRhoPresMuH.begin();

    dummyRhoPresMuH.resize(numberOfComponents);
    numComponentMarkers.resize(numberOfComponents);

    fsiGeneralData->referenceArray.clear();
    fsiGeneralData->referenceArray_FEA.clear();

    // Loop through all components loading referenceArray and referenceArray_FEA
    int start_index = 0;
    for (size_t i = 0; i < numberOfComponents; i++) {
        int compType = (int)std::floor(dummyRhoPresMuH[i].w + .1);
        int phaseType = -1;
        if (compType == -3) {
            phaseType = -1;  // For helper
        } else if (compType == -2) {
            phaseType = -1;  // For ghost
        } else if (compType == -1) {
            phaseType = -1;  // For fluid/granular
        } else if (compType == 0) {
            phaseType = 0;  // For boundary
        } else if (compType == 1) {
            phaseType = 1;  // For rigid
        } else if (compType == 2) {
            phaseType = 1;  // For 1D cable elements
        } else if (compType == 3) {
            phaseType = 1;  // For 2D shell elements
        } else {
            phaseType = 1;
        }
        auto new_entry = mI4(start_index, start_index + numComponentMarkers[i], compType, phaseType);
        start_index += numComponentMarkers[i];

        fsiGeneralData->referenceArray.push_back(new_entry);
        if (compType == 2 || compType == 3)
            fsiGeneralData->referenceArray_FEA.push_back(new_entry);
    }

    dummyRhoPresMuH.clear();
    numComponentMarkers.clear();
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi_impl::ResizeData(size_t numRigidBodies,
                                  size_t numFlexBodies1D,
                                  size_t numFlexBodies2D,
                                  size_t numFlexNodes) {
    ConstructReferenceArray();
    CalcNumObjects();

    if (numObjects->numAllMarkers != sphMarkersH->rhoPresMuH.size()) {
        std::cerr << "ERROR (ResizeData): mismatch in total number of markers." << std::endl;
        throw std::runtime_error("Mismatch in total number of markers.");
    }

    // Set number of interface objects
    numObjects->numRigidBodies = numRigidBodies;
    numObjects->numFlexBodies1D = numFlexBodies1D;
    numObjects->numFlexBodies2D = numFlexBodies2D;
    numObjects->numFlexNodes = numFlexNodes;

    sphMarkersD1->resize(numObjects->numAllMarkers);
    sphMarkersD2->resize(numObjects->numAllMarkers);
    sortedSphMarkersD->resize(numObjects->numAllMarkers);
    sphMarkersH->resize(numObjects->numAllMarkers);
    markersProximityD->resize(numObjects->numAllMarkers);

    fsiGeneralData->derivVelRhoD.resize(numObjects->numAllMarkers);
    fsiGeneralData->derivVelRhoD_old.resize(numObjects->numAllMarkers);

    fsiGeneralData->derivTauXxYyZzD.resize(numObjects->numAllMarkers);
    fsiGeneralData->derivTauXyXzYzD.resize(numObjects->numAllMarkers);

    fsiGeneralData->vel_XSPH_D.resize(numObjects->numAllMarkers);
    fsiGeneralData->vis_vel_SPH_D.resize(numObjects->numAllMarkers, mR3(1e-20));
    fsiGeneralData->sr_tau_I_mu_i.resize(numObjects->numAllMarkers, mR4(1e-20));

    fsiGeneralData->activityIdentifierD.resize(numObjects->numAllMarkers, 1);
    fsiGeneralData->extendedActivityIdD.resize(numObjects->numAllMarkers, 1);

    fsiGeneralData->freeSurfaceIdD.resize(numObjects->numAllMarkers, 0);

    thrust::copy(sphMarkersH->posRadH.begin(), sphMarkersH->posRadH.end(), sphMarkersD1->posRadD.begin());
    thrust::copy(sphMarkersH->velMasH.begin(), sphMarkersH->velMasH.end(), sphMarkersD1->velMasD.begin());
    thrust::copy(sphMarkersH->rhoPresMuH.begin(), sphMarkersH->rhoPresMuH.end(), sphMarkersD1->rhoPresMuD.begin());
    thrust::copy(sphMarkersH->tauXxYyZzH.begin(), sphMarkersH->tauXxYyZzH.end(), sphMarkersD1->tauXxYyZzD.begin());
    thrust::copy(sphMarkersH->tauXyXzYzH.begin(), sphMarkersH->tauXyXzYzH.end(), sphMarkersD1->tauXyXzYzD.begin());

    thrust::copy(sphMarkersD1->posRadD.begin(), sphMarkersD1->posRadD.end(), sphMarkersD2->posRadD.begin());
    thrust::copy(sphMarkersD1->velMasD.begin(), sphMarkersD1->velMasD.end(), sphMarkersD2->velMasD.begin());
    thrust::copy(sphMarkersD1->rhoPresMuD.begin(), sphMarkersD1->rhoPresMuD.end(), sphMarkersD2->rhoPresMuD.begin());
    thrust::copy(sphMarkersD1->tauXxYyZzD.begin(), sphMarkersD1->tauXxYyZzD.end(), sphMarkersD2->tauXxYyZzD.begin());
    thrust::copy(sphMarkersD1->tauXyXzYzD.begin(), sphMarkersD1->tauXyXzYzD.end(), sphMarkersD2->tauXyXzYzD.begin());

    fsiBodiesD1->resize(numObjects->numRigidBodies);
    fsiBodiesD2->resize(numObjects->numRigidBodies);
    fsiBodiesH->resize(numObjects->numRigidBodies);

    fsiGeneralData->rigid_FSI_ForcesD.resize(numObjects->numRigidBodies);
    fsiGeneralData->rigid_FSI_TorquesD.resize(numObjects->numRigidBodies);

    fsiGeneralData->rigidIdentifierD.resize(numObjects->numRigidMarkers);
    fsiGeneralData->rigidSPH_MeshPos_LRF_D.resize(numObjects->numRigidMarkers);

    fsiGeneralData->FlexIdentifierD.resize(numObjects->numFlexMarkers);
    fsiGeneralData->FlexSPH_MeshPos_LRF_D.resize(numObjects->numFlexMarkers);
    fsiGeneralData->FlexSPH_MeshPos_LRF_H.resize(numObjects->numFlexMarkers);

    fsiGeneralData->CableElementsNodesD.resize(fsiGeneralData->CableElementsNodesH.size());
    fsiGeneralData->ShellElementsNodesD.resize(fsiGeneralData->ShellElementsNodesH.size());

    thrust::copy(fsiGeneralData->CableElementsNodesH.begin(), fsiGeneralData->CableElementsNodesH.end(),
                 fsiGeneralData->CableElementsNodesD.begin());
    thrust::copy(fsiGeneralData->ShellElementsNodesH.begin(), fsiGeneralData->ShellElementsNodesH.end(),
                 fsiGeneralData->ShellElementsNodesD.begin());

    fsiMeshD->resize(numObjects->numFlexNodes);
    fsiMeshH->resize(numObjects->numFlexNodes);
    fsiGeneralData->Flex_FSI_ForcesD.resize(numObjects->numFlexNodes);
}

//--------------------------------------------------------------------------------------------------------------------------------

struct axpby_functor {
    axpby_functor(Real a, Real b) : m_a(a), m_b(b) {}
    __host__ __device__ Real4 operator()(const Real4& x, const Real4& y) const { return m_a * x + m_b * y; }

    const Real m_a;
    const Real m_b;
};

thrust::device_vector<Real4> ChSystemFsi_impl::GetParticleForces() {
    const auto n = numObjects->numFluidMarkers;

    // Copy data for SPH particles only
    thrust::device_vector<Real4> dvD(n);
    thrust::copy_n(fsiGeneralData->derivVelRhoD.begin(), n, dvD.begin());

    // Average dvD = beta * derivVelRhoD + (1-beta) * derivVelRhoD_old
    Real beta = paramsH->Beta;
    thrust::transform(dvD.begin(), dvD.end(), fsiGeneralData->derivVelRhoD_old.begin(), dvD.begin(),
                      axpby_functor(beta, 1 - beta));

    return dvD;
}

thrust::device_vector<Real4> ChSystemFsi_impl::GetParticleAccelerations() {
    const auto n = numObjects->numFluidMarkers;

    // Copy data for SPH particles only
    thrust::device_vector<Real4> accD(n);
    thrust::copy_n(fsiGeneralData->derivVelRhoD.begin(), n, accD.begin());

    // Average accD = beta * derivVelRhoD + (1-beta) * derivVelRhoD_old
    Real beta = paramsH->Beta / paramsH->markerMass;
    thrust::transform(accD.begin(), accD.end(), fsiGeneralData->derivVelRhoD_old.begin(), accD.begin(),
                      axpby_functor(beta , 1 - beta));

    return accD;
}

//--------------------------------------------------------------------------------------------------------------------------------

struct in_box {
    in_box() {}

    __device__ bool operator()(const Real4 v) {
        // Convert location in box frame
        auto d = mR3(v) - pos;
        auto w = mR3(                              //
            ax.x * d.x + ax.y * d.y + ax.z * d.z,  //
            ay.x * d.x + ay.y * d.y + ay.z * d.z,  //
            az.x * d.x + az.y * d.y + az.z * d.z   //
        );
        // Check w between all box limits
        return (w.x >= -hsize.x && w.x <= +hsize.x) && (w.y >= -hsize.y && w.y <= +hsize.y) &&
               (w.z >= -hsize.z && w.z <= +hsize.z);
    }

    Real3 hsize;
    Real3 pos;
    Real3 ax;
    Real3 ay;
    Real3 az;
};

thrust::device_vector<int> ChSystemFsi_impl::FindParticlesInBox(const Real3& hsize,
                                                                const Real3& pos,
                                                                const Real3& ax,
                                                                const Real3& ay,
                                                                const Real3& az) {
    // Extract indices of SPH particles contained in the OBB
    auto& ref = fsiGeneralData->referenceArray;
    auto& pos_D = sphMarkersD2->posRadD;

    // Find start and end locations for SPH particles (exclude ghost and BCE markers)
    int haveHelper = (ref[0].z == -3) ? 1 : 0;
    int haveGhost = (ref[0].z == -2 || ref[1].z == -2) ? 1 : 0;
    auto sph_start = ref[haveHelper + haveGhost].x;
    auto sph_end = ref[haveHelper + haveGhost].y;
    auto num_sph = sph_end - sph_start;

    // Preallocate output vector of indices
    thrust::device_vector<int> indices_D(num_sph);

    // Extract indices of SPH particles inside OBB
    thrust::counting_iterator<int> first(0);
    thrust::counting_iterator<int> last(num_sph);
    in_box predicate;
    predicate.hsize = hsize;
    predicate.pos = pos;
    predicate.ax = ax;
    predicate.ay = ay;
    predicate.az = az;
    auto end = thrust::copy_if(thrust::device,     // execution policy
                               first, last,        // range of all particle indices
                               pos_D.begin(),      // stencil vector
                               indices_D.begin(),  // beginning of destination
                               predicate           // predicate for stencil elements
    );

    // Trim the output vector of indices
    size_t num_active = (size_t)(end - indices_D.begin());
    indices_D.resize(num_active);

    return indices_D;
}

thrust::device_vector<Real4> ChSystemFsi_impl::GetParticlePositions(const thrust::device_vector<int>& indices) {
    // Gather positions from particles with specified indices
    const auto& allpos = sphMarkersD2->posRadD;

    thrust::device_vector<Real4> pos(allpos.size());

    auto end = thrust::gather(thrust::device,                  // execution policy
                              indices.begin(), indices.end(),  // range of gather locations
                              allpos.begin(),                  // beginning of source
                              pos.begin()                      // beginning of destination
    );

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - pos.begin());
    assert(num_active == indices.size());
    pos.resize(num_active);

    return pos;
}

thrust::device_vector<Real3> ChSystemFsi_impl::GetParticleVelocities(const thrust::device_vector<int>& indices) {
    // Gather positions from particles with specified indices
    auto allvel = sphMarkersD2->velMasD;

    thrust::device_vector<Real3> vel(allvel.size());

    auto end = thrust::gather(thrust::device,                  // execution policy
                              indices.begin(), indices.end(),  // range of gather locations
                              allvel.begin(),                  // beginning of source
                              vel.begin()                      // beginning of destination
    );

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - vel.begin());
    assert(num_active == indices.size());
    vel.resize(num_active);

    return vel;
}

thrust::device_vector<Real4> ChSystemFsi_impl::GetParticleForces(const thrust::device_vector<int>& indices) {
    auto allforces = GetParticleForces();

    thrust::device_vector<Real4> forces(allforces.size());

    auto end = thrust::gather(thrust::device,                  // execution policy
                              indices.begin(), indices.end(),  // range of gather locations
                              allforces.begin(),               // beginning of source
                              forces.begin()                   // beginning of destination
    );

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - forces.begin());
    assert(num_active == indices.size());
    forces.resize(num_active);

    return forces;
}

thrust::device_vector<Real4> ChSystemFsi_impl::GetParticleAccelerations(const thrust::device_vector<int>& indices) {
    auto allacc = GetParticleAccelerations();

    thrust::device_vector<Real4> acc(allacc.size());

    auto end = thrust::gather(thrust::device,                   // execution policy
                              indices.begin(), indices.end(),   // range of gather locations
                              allacc.begin(),                   // beginning of source
                              acc.begin()                       // beginning of destination
    );

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - acc.begin());
    assert(num_active == indices.size());
    acc.resize(num_active);

    return acc;
}

}  // end namespace fsi
}  // end namespace chrono
