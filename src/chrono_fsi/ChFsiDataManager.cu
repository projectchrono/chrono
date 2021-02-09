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
// Base class for managing data in chrono_fsi, aka fluid system.//
// =============================================================================

#include <thrust/sort.h>
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {

struct sphTypeCompLess {
    __host__ __device__ bool operator()(const Real4& o1, const Real4& o2) { return o1.w <= o2.w; }
};

struct sphTypeCompEqual {
    __host__ __device__ bool operator()(const Real4& o1, const Real4& o2) { return o1.w == o2.w; }
};

//---------------------------------------------------------------------------------------
zipIterSphD SphMarkerDataD::iterator() {
    return thrust::make_zip_iterator(thrust::make_tuple(posRadD.begin(), velMasD.begin(), rhoPresMuD.begin(),
                                                        tauXxYyZzD.begin(), tauXyXzYzD.begin()));
}


// resize
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
    return thrust::make_zip_iterator(thrust::make_tuple(posRigid_fsiBodies_D.begin(), 
                                                        velMassRigid_fsiBodies_D.begin(), 
                                                        accRigid_fsiBodies_D.begin(),
                                                        q_fsiBodies_D.begin(), 
                                                        omegaVelLRF_fsiBodies_D.begin(), 
                                                        omegaAccLRF_fsiBodies_D.begin()));
}

// resize
void FsiBodiesDataD::resize(size_t s) {
    posRigid_fsiBodies_D.resize(s);
    velMassRigid_fsiBodies_D.resize(s);
    accRigid_fsiBodies_D.resize(s);
    q_fsiBodies_D.resize(s);
    omegaVelLRF_fsiBodies_D.resize(s);
    omegaAccLRF_fsiBodies_D.resize(s);
}

// resize
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

// resize
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
}
// resize
void FsiMeshDataD::resize(size_t s) {
    pos_fsi_fea_D.resize(s);
    vel_fsi_fea_D.resize(s);
    acc_fsi_fea_D.resize(s);
}

void FsiBodiesDataD::CopyFromH(const FsiBodiesDataH& other) {
    thrust::copy(other.posRigid_fsiBodies_H.begin(), other.posRigid_fsiBodies_H.end(), 
                 posRigid_fsiBodies_D.begin());
    thrust::copy(other.velMassRigid_fsiBodies_H.begin(), other.velMassRigid_fsiBodies_H.end(),
                 velMassRigid_fsiBodies_D.begin());
    thrust::copy(other.accRigid_fsiBodies_H.begin(), other.accRigid_fsiBodies_H.end(), 
                 accRigid_fsiBodies_D.begin());
    thrust::copy(other.q_fsiBodies_H.begin(), other.q_fsiBodies_H.end(), 
                 q_fsiBodies_D.begin());
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
}

FsiBodiesDataD& FsiBodiesDataD::operator=(const FsiBodiesDataD& other) {
    if (this == &other) {
        return *this;
    }
    thrust::copy(other.posRigid_fsiBodies_D.begin(), other.posRigid_fsiBodies_D.end(), 
                 posRigid_fsiBodies_D.begin());
    thrust::copy(other.velMassRigid_fsiBodies_D.begin(), other.velMassRigid_fsiBodies_D.end(),
                 velMassRigid_fsiBodies_D.begin());
    thrust::copy(other.accRigid_fsiBodies_D.begin(), other.accRigid_fsiBodies_D.end(), 
                 accRigid_fsiBodies_D.begin());
    thrust::copy(other.q_fsiBodies_D.begin(), other.q_fsiBodies_D.end(), 
                 q_fsiBodies_D.begin());
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
    return *this;
}

//---------------------------------------------------------------------------------------

zipIterRigidH FsiBodiesDataH::iterator() {
    return thrust::make_zip_iterator(
        thrust::make_tuple(posRigid_fsiBodies_H.begin(), velMassRigid_fsiBodies_H.begin(), accRigid_fsiBodies_H.begin(),
                           q_fsiBodies_H.begin(), omegaVelLRF_fsiBodies_H.begin(), omegaAccLRF_fsiBodies_H.begin()));
}

// resize
void FsiBodiesDataH::resize(size_t s) {
    posRigid_fsiBodies_H.resize(s);
    velMassRigid_fsiBodies_H.resize(s);
    accRigid_fsiBodies_H.resize(s);
    q_fsiBodies_H.resize(s);
    omegaVelLRF_fsiBodies_H.resize(s);
    omegaAccLRF_fsiBodies_H.resize(s);
}

//---------------------------------------------------------------------------------------
void ProximityDataD::resize(size_t numAllMarkers) {
    gridMarkerHashD.resize(numAllMarkers);
    gridMarkerIndexD.resize(numAllMarkers);
    mapOriginalToSorted.resize(numAllMarkers);
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

// resize
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
}
//---------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------

ChFsiDataManager::ChFsiDataManager() {
    numObjects = chrono_types::make_shared<NumberOfObjects>();
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

ChFsiDataManager::~ChFsiDataManager() {}

void ChFsiDataManager::AddSphMarker(Real4 pos, Real3 vel, Real4 rhoPresMu, Real3 tauXxYyZz, Real3 tauXyXzYz) {
    sphMarkersH->posRadH.push_back(pos);
    sphMarkersH->velMasH.push_back(vel);
    sphMarkersH->rhoPresMuH.push_back(rhoPresMu);
    sphMarkersH->tauXxYyZzH.push_back(tauXxYyZz);
    sphMarkersH->tauXyXzYzH.push_back(tauXyXzYz);
}

void ChFsiDataManager::ArrangeDataManager() {
    thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkersH->rhoPresMuH;

    // arrange data based on type: fluid, boundary, bce1, bce2, ....
    //  thrust::sort_by_key(dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(), sphMarkersH->iterator(), sphTypeCompLess());
    dummyRhoPresMuH.clear();
}

void ChFsiDataManager::InitNumObjects() {
    numObjects->numRigidBodies = 0;  /* Number of rigid bodies */
    numObjects->numFlexBodies1D = 0; /* Number of Flexible bodies*/
    numObjects->numFlexBodies2D = 0; /* Number of Flexible bodies*/
    numObjects->numFlexNodes = 0;    /* Number of FE nodes*/
    numObjects->numGhostMarkers = 0;
    numObjects->numHelperMarkers = 0;
    numObjects->numFluidMarkers = 0;     /* Number of fluid SPH markers*/
    numObjects->numBoundaryMarkers = 0;  /* Number of boundary SPH markers */
    numObjects->startRigidMarkers = 0;   /* */
    numObjects->startFlexMarkers = 0;    /* */
    numObjects->numRigid_SphMarkers = 0; /* */
    numObjects->numFlex_SphMarkers = 0;  /* */
    numObjects->numAllMarkers = 0;       /* Total number of SPH markers */
}

void ChFsiDataManager::CalcNumObjects() {
    InitNumObjects();
    size_t rSize = fsiGeneralData->referenceArray.size();
    bool flagRigid = false;
    bool flagFlex = false;
    std::cout << "ChFsiDataManager::CalcNumObjects: rSize=" << rSize << " \n";

    for (size_t i = 0; i < rSize; i++) {
        int4 rComp4 = fsiGeneralData->referenceArray[i];
        int numMerkers = rComp4.y - rComp4.x;
        //        std::cout << "rComp4.z=" << rComp4.z << " \n";

        switch (rComp4.z) {
            case -3:
                numObjects->numHelperMarkers += numMerkers;
                std::cout << "Added " << numMerkers << " helper  markers\n";
                break;
            case -2:
                numObjects->numGhostMarkers += numMerkers;
                std::cout << "Added " << numMerkers << " ghost  markers\n";
                break;
            case -1:
                numObjects->numFluidMarkers += numMerkers;
                std::cout << "Added " << numMerkers << " fluid  markers\n";
                break;
            case 0:
                numObjects->numBoundaryMarkers += numMerkers;
                std::cout << "Added " << numMerkers << " boundary  markers\n";
                break;
            case 1:
                numObjects->numRigid_SphMarkers += numMerkers;
                std::cout << "Added " << numMerkers << " rigid  markers\n";
                numObjects->numRigidBodies++;
                flagRigid = true;
                break;
            case 2:
                numObjects->numFlex_SphMarkers += numMerkers;
                numObjects->numFlexBodies1D++;
                flagFlex = true;
                break;
            case 3:
                numObjects->numFlex_SphMarkers += numMerkers;
                numObjects->numFlexBodies2D++;
                flagFlex = true;
                break;
            default:
                std::cout << "Error! particle type not defined! Thrown from CalcNumObjects\n";
                break;
        }
    }

    std::cout << "numObjects->numFlexNodes" << numObjects->numFlexNodes << std::endl;

    std::cout << "numObjects->numGhostMarkers" << numObjects->numGhostMarkers << std::endl;
    numObjects->numFluidMarkers += numObjects->numGhostMarkers + numObjects->numHelperMarkers;

    numObjects->numAllMarkers = numObjects->numFluidMarkers 
                              + numObjects->numBoundaryMarkers 
                              + numObjects->numRigid_SphMarkers 
                              + numObjects->numFlex_SphMarkers;

    numObjects->startRigidMarkers = numObjects->numFluidMarkers + numObjects->numBoundaryMarkers;

    numObjects->startFlexMarkers =
        (flagFlex) ? (numObjects->numFluidMarkers + numObjects->numBoundaryMarkers + numObjects->numRigid_SphMarkers)
                   : numObjects->numAllMarkers;

    printf(
        "# Markers: helpers:%zd, Ghost:%zd, Fluid:%zd boundary:%zd ridigSph:%zd flexSph:%zd, start-rigid:%zd, start-flex:%zd, "
        "All:%zd \n",
        numObjects->numHelperMarkers, numObjects->numGhostMarkers, numObjects->numFluidMarkers,
        numObjects->numBoundaryMarkers, numObjects->numRigid_SphMarkers, numObjects->numFlex_SphMarkers,
        numObjects->startRigidMarkers, numObjects->startFlexMarkers, numObjects->numAllMarkers);
}

void ChFsiDataManager::ConstructReferenceArray() {
    //  ArrangeDataManager();

    CalcNumObjects();

    // determine the number of each component
    if (numObjects->numAllMarkers != sphMarkersH->rhoPresMuH.size()) {
        printf(
            "\nChFsiDataManager::ConstructReferenceArray()    numObjects->numAllMarkers=%zd, "
            "sphMarkersH->rhoPresMuH.size()=%zd\n",
            numObjects->numAllMarkers, sphMarkersH->rhoPresMuH.size());
        throw std::runtime_error("Error! numObjects wrong! thrown from ConstructReferenceArray !\n");
    }
    thrust::host_vector<int> numComponentMarkers(numObjects->numAllMarkers);
    thrust::fill(numComponentMarkers.begin(), numComponentMarkers.end(), 1);
    thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkersH->rhoPresMuH;
    thrust::copy(sphMarkersH->rhoPresMuH.begin(), sphMarkersH->rhoPresMuH.end(), dummyRhoPresMuH.begin());
    size_t numberOfComponents =
        (thrust::reduce_by_key(dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(), numComponentMarkers.begin(),
                               dummyRhoPresMuH.begin(), numComponentMarkers.begin(), sphTypeCompEqual()))
            .first -
        dummyRhoPresMuH.begin();
    // if (numberOfComponents == 0) {
    // 	std::cout << "Error! no marker found! Thrown from
    // ConstructReferenceArray\n";
    // 	return;
    // }

    printf("numberOfComponents=%zd\n", numberOfComponents);

    fsiGeneralData->referenceArray.resize(numberOfComponents);
    dummyRhoPresMuH.resize(numberOfComponents);
    numComponentMarkers.resize(numberOfComponents);
    int savedNumber = 0;
    for (size_t i = 0; i < numberOfComponents; i++) {
        int compType = (int)std::floor(dummyRhoPresMuH[i].w + .1);
        int phaseType = -1;
        if (compType <= -2) {
            phaseType = -1;
        } else if (compType == -1) {
            phaseType = -1;
        } else if (compType == 0) {
            phaseType = 0;
        } else if (compType == 1) {
            phaseType = 1;
        } else if (compType == 2) {
            phaseType = 1;  // For Cable Elements
        } else if (compType == 3) {
            phaseType = 2;  // For Shell Elements
        } else {
            phaseType = 1;
        }
        fsiGeneralData->referenceArray[i] = mI4(savedNumber, savedNumber + numComponentMarkers[i], compType, phaseType);
        savedNumber += numComponentMarkers[i];
    }
    dummyRhoPresMuH.clear();
    numComponentMarkers.clear();

    printf("reference array \n");
    for (size_t i = 0; i < fsiGeneralData->referenceArray.size(); i++) {
        int4 num = fsiGeneralData->referenceArray[i];
        printf("%d %d %d %d \n", num.x, num.y, num.z, num.w);
    }
}

////--------------------------------------------------------------------------------------------------------------------------------
void ChFsiDataManager::ResizeDataManager(int numNodes) {
    ConstructReferenceArray();
    if (numObjects->numAllMarkers != sphMarkersH->rhoPresMuH.size()) {
        throw std::runtime_error("Error! numObjects wrong! thrown from FinalizeDataManager !\n");
    }

    //    int numNodes = fsiGeneralData->CableElementsNodesH.size();
    numObjects->numFlexNodes = numNodes;

    //  printf("ChSystemFsi::FinalizeData pos[%d]=%f,%f,%f, pos[%d]=%f,%f,%f\n", 22995, sphMarkersH->posRadH[22995].x,
    //         sphMarkersH->posRadH[22995].y, sphMarkersH->posRadH[22995].z, 23054, sphMarkersH->posRadH[23054].x,
    //         sphMarkersH->posRadH[23054].y, sphMarkersH->posRadH[23054].z);
    //  printf("\n\n fsiData->ResizeDataManager (numAllMarkers)...\n");

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

    printf("fsiData->ResizeDataManager (sphMarkersH)...\n");

    //  printf("ChFsiDataManager::ResizeDataManager pos[%d]=%f,%f,%f, pos[%d]=%f,%f,%f\n", 22995,
    //         sphMarkersH->posRadH[22995].x, sphMarkersH->posRadH[22995].y, sphMarkersH->posRadH[22995].z, 23054,
    //         sphMarkersH->posRadH[23054].x, sphMarkersH->posRadH[23054].y, sphMarkersH->posRadH[23054].z);
    // Arman: implement this in one shot function in class
    thrust::copy(sphMarkersH->posRadH.begin(), sphMarkersH->posRadH.end(), sphMarkersD1->posRadD.begin());
    thrust::copy(sphMarkersH->velMasH.begin(), sphMarkersH->velMasH.end(), sphMarkersD1->velMasD.begin());
    thrust::copy(sphMarkersH->rhoPresMuH.begin(), sphMarkersH->rhoPresMuH.end(), sphMarkersD1->rhoPresMuD.begin());
    thrust::copy(sphMarkersH->tauXxYyZzH.begin(), sphMarkersH->tauXxYyZzH.end(), sphMarkersD1->tauXxYyZzD.begin());
    thrust::copy(sphMarkersH->tauXyXzYzH.begin(), sphMarkersH->tauXyXzYzH.end(), sphMarkersD1->tauXyXzYzD.begin());
    printf("fsiData->ResizeDataManager (sphMarkersD)...\n");

    thrust::copy(sphMarkersD1->posRadD.begin(), sphMarkersD1->posRadD.end(), sphMarkersD2->posRadD.begin());
    thrust::copy(sphMarkersD1->velMasD.begin(), sphMarkersD1->velMasD.end(), sphMarkersD2->velMasD.begin());
    thrust::copy(sphMarkersD1->rhoPresMuD.begin(), sphMarkersD1->rhoPresMuD.end(), sphMarkersD2->rhoPresMuD.begin());
    thrust::copy(sphMarkersD1->tauXxYyZzD.begin(), sphMarkersD1->tauXxYyZzD.end(), sphMarkersD2->tauXxYyZzD.begin());
    thrust::copy(sphMarkersD1->tauXyXzYzD.begin(), sphMarkersD1->tauXyXzYzD.end(), sphMarkersD2->tauXyXzYzD.begin());
    printf("fsiData->ResizeDataManager (Rigid)...\n");

    // copy rigids
    fsiBodiesD1->resize(numObjects->numRigidBodies);
    fsiBodiesD2->resize(numObjects->numRigidBodies);
    fsiBodiesH->resize(numObjects->numRigidBodies);
    fsiGeneralData->rigid_FSI_ForcesD.resize(numObjects->numRigidBodies);
    fsiGeneralData->rigid_FSI_TorquesD.resize(numObjects->numRigidBodies);
    fsiGeneralData->rigidIdentifierD.resize(numObjects->numRigid_SphMarkers);
    fsiGeneralData->rigidSPH_MeshPos_LRF_D.resize(numObjects->numRigid_SphMarkers);
    fsiGeneralData->FlexSPH_MeshPos_LRF_D.resize(numObjects->numFlex_SphMarkers);
    fsiGeneralData->FlexSPH_MeshPos_LRF_H.resize(numObjects->numFlex_SphMarkers);

    printf("fsiData->ResizeDataManager (Flex)...\n");

    fsiGeneralData->FlexIdentifierD.resize(numObjects->numFlex_SphMarkers);

    if (fsiGeneralData->CableElementsNodesH.size() != numObjects->numFlexBodies1D) {
        printf("******************************************************************************\n");
        printf("******************************************************************************\n");
        printf("******************************Be Careful**************************************\n");
        printf("There might be 1D Flexible bodies in Chrono that are not a part of ChSystemFSI\n");
        printf("I am going to transfer nodal data for such elements back and forth although they\n");
        printf("are not part of FSI calculation. If you want to have some 1D element that are  \n");
        printf("inside the ChSystem mesh but not FSI system, you can ignore this warning ...\n");
        printf("******************************************************************************\n");
        printf("******************************************************************************\n");
        printf("******************************************************************************\n");
        fsiGeneralData->CableElementsNodes.resize(fsiGeneralData->CableElementsNodesH.size());
    } else
        fsiGeneralData->CableElementsNodes.resize(numObjects->numFlexBodies1D);

    fsiGeneralData->ShellElementsNodes.resize(numObjects->numFlexBodies2D);
    printf("numObjects->numFlexBodies1D= %zd, numObjects->numFlexBodies2D= %zd \n", numObjects->numFlexBodies1D,
           numObjects->numFlexBodies2D);
    printf("fsiGeneralData->CableElementsNodesH.size()= %zd, fsiGeneralData->ShellElementsNodesH= %zd \n",
           fsiGeneralData->CableElementsNodesH.size(), fsiGeneralData->ShellElementsNodesH.size());
    thrust::copy(fsiGeneralData->CableElementsNodesH.begin(), fsiGeneralData->CableElementsNodesH.end(),
                 fsiGeneralData->CableElementsNodes.begin());
    thrust::copy(fsiGeneralData->ShellElementsNodesH.begin(), fsiGeneralData->ShellElementsNodesH.end(),
                 fsiGeneralData->ShellElementsNodes.begin());

    fsiMeshD->resize(numObjects->numFlexNodes);
    fsiMeshH->resize(numObjects->numFlexNodes);
    fsiGeneralData->Flex_FSI_ForcesD.resize(numObjects->numFlexNodes);
}

}  // end namespace fsi
}  // end namespace chrono
