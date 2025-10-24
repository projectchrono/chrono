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
// Author: Milad Rakhsha, Arman Pazouki, Radu Serban
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration.
//
// =============================================================================

////#define DEBUG_LOG

#include <algorithm>

#include <thrust/execution_policy.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/count.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/gather.h>
#include <thrust/for_each.h>
#include <thrust/functional.h>
#include <thrust/transform.h>
#include <thrust/partition.h>
#include <thrust/zip_function.h>

#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/physics/SphDataManager.cuh"
#include "chrono_fsi/sph/math/SphCustomMath.cuh"

namespace chrono {
namespace fsi {
namespace sph {

//---------------------------------------------------------------------------------------

zipIterSphD SphMarkerDataD::iterator(int offset) {
    return thrust::make_zip_iterator(thrust::make_tuple(posRadD.begin() + offset, velMasD.begin() + offset,
                                                        rhoPresMuD.begin() + offset, tauXxYyZzD.begin() + offset,
                                                        tauXyXzYzD.begin() + offset));
}

void SphMarkerDataD::resize(size_t s) {
    posRadD.resize(s);
    velMasD.resize(s);
    rhoPresMuD.resize(s);
    tauXxYyZzD.resize(s);
    tauXyXzYzD.resize(s);
}

zipIterSphH SphMarkerDataH::iterator(int offset) {
    return thrust::make_zip_iterator(thrust::make_tuple(posRadH.begin() + offset, velMasH.begin() + offset,
                                                        rhoPresMuH.begin() + offset, tauXxYyZzH.begin() + offset,
                                                        tauXyXzYzH.begin() + offset));
}

void SphMarkerDataH::resize(size_t s) {
    posRadH.resize(s);
    velMasH.resize(s);
    rhoPresMuH.resize(s);
    tauXxYyZzH.resize(s);
    tauXyXzYzH.resize(s);
}

//---------------------------------------------------------------------------------------

zipIterRigidH FsiBodyStateH::iterator(int offset) {
    return thrust::make_zip_iterator(thrust::make_tuple(pos.begin() + offset, lin_vel.begin() + offset,
                                                        lin_acc.begin() + offset, rot.begin() + offset,
                                                        ang_vel.begin() + offset, ang_acc.begin() + offset));
}

void FsiBodyStateH::Resize(size_t s) {
    pos.resize(s);
    lin_vel.resize(s);
    lin_acc.resize(s);
    rot.resize(s);
    ang_vel.resize(s);
    ang_acc.resize(s);
}

zipIterRigidD FsiBodyStateD::iterator(int offset) {
    return thrust::make_zip_iterator(thrust::make_tuple(pos.begin() + offset, lin_vel.begin() + offset,
                                                        lin_acc.begin() + offset, rot.begin() + offset,
                                                        ang_vel.begin() + offset, ang_acc.begin() + offset));
}

void FsiBodyStateD::Resize(size_t s) {
    pos.resize(s);
    lin_vel.resize(s);
    lin_acc.resize(s);
    rot.resize(s);
    ang_vel.resize(s);
    ang_acc.resize(s);
}

void FsiBodyStateD::CopyFromH(const FsiBodyStateH& bodyStateH) {
    thrust::copy(bodyStateH.pos.begin(), bodyStateH.pos.end(), pos.begin());
    thrust::copy(bodyStateH.lin_vel.begin(), bodyStateH.lin_vel.end(), lin_vel.begin());
    thrust::copy(bodyStateH.lin_acc.begin(), bodyStateH.lin_acc.end(), lin_acc.begin());
    thrust::copy(bodyStateH.rot.begin(), bodyStateH.rot.end(), rot.begin());
    thrust::copy(bodyStateH.ang_vel.begin(), bodyStateH.ang_vel.end(), ang_vel.begin());
    thrust::copy(bodyStateH.ang_acc.begin(), bodyStateH.ang_acc.end(), ang_acc.begin());
}

FsiBodyStateD& FsiBodyStateD::operator=(const FsiBodyStateD& other) {
    if (this == &other) {
        return *this;
    }
    thrust::copy(other.pos.begin(), other.pos.end(), pos.begin());
    thrust::copy(other.lin_vel.begin(), other.lin_vel.end(), lin_vel.begin());
    thrust::copy(other.lin_acc.begin(), other.lin_acc.end(), lin_acc.begin());
    thrust::copy(other.rot.begin(), other.rot.end(), rot.begin());
    thrust::copy(other.ang_vel.begin(), other.ang_vel.end(), ang_vel.begin());
    thrust::copy(other.ang_acc.begin(), other.ang_acc.end(), ang_acc.begin());
    return *this;
}

//---------------------------------------------------------------------------------------

void FsiMeshStateH::Resize(size_t s, bool use_node_directions) {
    pos.resize(s);
    vel.resize(s);
    acc.resize(s);
    if (use_node_directions)
        dir.resize(s);
    has_node_directions = use_node_directions;
}

void FsiMeshStateD::Resize(size_t s, bool use_node_directions) {
    pos.resize(s);
    vel.resize(s);
    acc.resize(s);
    if (use_node_directions)
        dir.resize(s);
    has_node_directions = use_node_directions;
}

void FsiMeshStateD::CopyFromH(const FsiMeshStateH& meshStateH) {
    thrust::copy(meshStateH.pos.begin(), meshStateH.pos.end(), pos.begin());
    thrust::copy(meshStateH.vel.begin(), meshStateH.vel.end(), vel.begin());
    thrust::copy(meshStateH.acc.begin(), meshStateH.acc.end(), acc.begin());
}

void FsiMeshStateD::CopyDirectionsFromH(const FsiMeshStateH& meshStateH) {
    if (meshStateH.has_node_directions)
        thrust::copy(meshStateH.dir.begin(), meshStateH.dir.end(), dir.begin());
}

FsiMeshStateD& FsiMeshStateD::operator=(const FsiMeshStateD& other) {
    if (this == &other) {
        return *this;
    }
    thrust::copy(other.pos.begin(), other.pos.end(), pos.begin());
    thrust::copy(other.vel.begin(), other.vel.end(), vel.begin());
    thrust::copy(other.acc.begin(), other.acc.end(), acc.begin());
    if (other.has_node_directions)
        thrust::copy(other.dir.begin(), other.dir.end(), dir.begin());
    return *this;
}

//---------------------------------------------------------------------------------------

void ProximityDataD::resize(size_t s) {
    mapOriginalToSorted.resize(s);
}

//---------------------------------------------------------------------------------------

FsiDataManager::FsiDataManager(std::shared_ptr<ChFsiParamsSPH> params) : paramsH(params) {
    countersH = chrono_types::make_shared<Counters>();

    sphMarkers_D = chrono_types::make_shared<SphMarkerDataD>();
    sortedSphMarkers1_D = chrono_types::make_shared<SphMarkerDataD>();
    sortedSphMarkers2_D = chrono_types::make_shared<SphMarkerDataD>();
    sphMarkers_H = chrono_types::make_shared<SphMarkerDataH>();

    fsiBodyState_D = chrono_types::make_shared<FsiBodyStateD>();
    fsiBodyState_H = chrono_types::make_shared<FsiBodyStateH>();

    fsiMesh1DState_D = chrono_types::make_shared<FsiMeshStateD>();
    fsiMesh1DState_H = chrono_types::make_shared<FsiMeshStateH>();
    fsiMesh2DState_D = chrono_types::make_shared<FsiMeshStateD>();
    fsiMesh2DState_H = chrono_types::make_shared<FsiMeshStateH>();

    markersProximity_D = chrono_types::make_shared<ProximityDataD>();

    cudaDeviceInfo = chrono_types::make_shared<CudaDeviceInfo>();

    // Resizing parameters
    m_max_extended_particles = 0;
    m_resize_counter = 0;
    GROWTH_FACTOR = 1.2f;
    SHRINK_THRESHOLD = 0.75f;
    SHRINK_INTERVAL = 50;
}

FsiDataManager::~FsiDataManager() {}

void FsiDataManager::AddSphParticle(Real3 pos,
                                    Real rho,
                                    Real pres,
                                    Real mu,
                                    Real3 vel,
                                    Real3 tauXxYyZz,
                                    Real3 tauXyXzYz) {
    sphMarkers_H->posRadH.push_back(mR4(pos, paramsH->h));
    sphMarkers_H->velMasH.push_back(vel);
    sphMarkers_H->rhoPresMuH.push_back(mR4(rho, pres, mu, -1));

    //// TODO: do this only for elasticSPH!
    sphMarkers_H->tauXyXzYzH.push_back(tauXyXzYz);
    sphMarkers_H->tauXxYyZzH.push_back(tauXxYyZz);
}

void FsiDataManager::AddBceMarker(MarkerType type, Real3 pos, Real3 vel) {
    sphMarkers_H->posRadH.push_back(mR4(pos, paramsH->h));
    sphMarkers_H->velMasH.push_back(vel);
    sphMarkers_H->rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->base_pressure, paramsH->mu0, GetMarkerCode(type)));

    //// TODO: do this only for elasticSPH!
    sphMarkers_H->tauXyXzYzH.push_back(mR3(0.0));
    sphMarkers_H->tauXxYyZzH.push_back(mR3(0.0));
}

void FsiDataManager::SetCounters(unsigned int num_fsi_bodies,
                                 unsigned int num_fsi_nodes1D,
                                 unsigned int num_fsi_elements1D,
                                 unsigned int num_fsi_nodes2D,
                                 unsigned int num_fsi_elements2D) {
    countersH->numFsiBodies = num_fsi_bodies;
    countersH->numFsiElements1D = num_fsi_elements1D;
    countersH->numFsiElements2D = num_fsi_elements2D;
    countersH->numFsiNodes1D = num_fsi_nodes1D;
    countersH->numFsiNodes2D = num_fsi_nodes2D;

    countersH->numGhostMarkers = 0;       // Number of ghost particles
    countersH->numHelperMarkers = 0;      // Number of helper particles
    countersH->numFluidMarkers = 0;       // Number of fluid SPH particles
    countersH->numBoundaryMarkers = 0;    // Number of boundary BCE markers
    countersH->numRigidMarkers = 0;       // Number of rigid BCE markers
    countersH->numFlexMarkers1D = 0;      // Number of flexible 1-D segment BCE markers
    countersH->numFlexMarkers2D = 0;      // Number of flexible 2-D face BCE markers
    countersH->numBceMarkers = 0;         // Total number of BCE markers
    countersH->numAllMarkers = 0;         // Total number of SPH + BCE particles
    countersH->startBoundaryMarkers = 0;  // Start index of the boundary BCE markers
    countersH->startRigidMarkers = 0;     // Start index of the rigid BCE markers
    countersH->startFlexMarkers1D = 0;    // Start index of the 1-D flexible BCE markers
    countersH->startFlexMarkers2D = 0;    // Start index of the 2-D flexible BCE markers

    size_t rSize = referenceArray.size();

    for (size_t i = 0; i < rSize; i++) {
        int4 rComp4 = referenceArray[i];
        int numMarkers = rComp4.y - rComp4.x;

        switch (rComp4.z) {
            case -3:
                countersH->numHelperMarkers += numMarkers;
                break;
            case -2:
                countersH->numGhostMarkers += numMarkers;
                break;
            case -1:
                countersH->numFluidMarkers += numMarkers;
                break;
            case 0:
                countersH->numBoundaryMarkers += numMarkers;
                break;
            case 1:
                countersH->numRigidMarkers += numMarkers;
                break;
            case 2:
                countersH->numFlexMarkers1D += numMarkers;
                break;
            case 3:
                countersH->numFlexMarkers2D += numMarkers;
                break;
            default:
                std::cerr << "ERROR SetCounters: particle type not defined." << std::endl;
                throw std::runtime_error("SetCounters: Particle type not defined.");
                break;
        }
    }

    countersH->numFluidMarkers += countersH->numGhostMarkers + countersH->numHelperMarkers;
    countersH->numBceMarkers = countersH->numBoundaryMarkers + countersH->numRigidMarkers +  //
                               countersH->numFlexMarkers1D + countersH->numFlexMarkers2D;
    countersH->numAllMarkers = countersH->numFluidMarkers + countersH->numBceMarkers;

    countersH->startBoundaryMarkers = countersH->numFluidMarkers;
    countersH->startRigidMarkers = countersH->startBoundaryMarkers + countersH->numBoundaryMarkers;
    countersH->startFlexMarkers1D = countersH->startRigidMarkers + countersH->numRigidMarkers;
    countersH->startFlexMarkers2D = countersH->startFlexMarkers1D + countersH->numFlexMarkers1D;
}

struct sphTypeCompEqual {
    __host__ bool operator()(const Real4& o1, const Real4& o2) { return o1.w == o2.w; }
};

void FsiDataManager::ConstructReferenceArray() {
    auto numAllMarkers = sphMarkers_H->rhoPresMuH.size();

    thrust::host_vector<int> numComponentMarkers(numAllMarkers);
    thrust::fill(numComponentMarkers.begin(), numComponentMarkers.end(), 1);
    thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkers_H->rhoPresMuH;

    auto new_end = thrust::reduce_by_key(thrust::host,                                    // execution policy
                                         dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(),  // keys first, last
                                         numComponentMarkers.begin(),                     // values first
                                         dummyRhoPresMuH.begin(),                         // keys out
                                         numComponentMarkers.begin(),                     // values out
                                         sphTypeCompEqual());

    size_t numberOfComponents = new_end.first - dummyRhoPresMuH.begin();

    dummyRhoPresMuH.resize(numberOfComponents);
    numComponentMarkers.resize(numberOfComponents);

    referenceArray.clear();
    referenceArray_FEA.clear();

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

        referenceArray.push_back(new_entry);
        if (compType == 2 || compType == 3)
            referenceArray_FEA.push_back(new_entry);
    }

    dummyRhoPresMuH.clear();
    numComponentMarkers.clear();
}

void FsiDataManager::ResetData() {
    auto zero4 = mR4(0);
    auto zero3 = mR3(0);

    thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), zero4);
    thrust::fill(derivVelRhoOriginalD.begin(), derivVelRhoOriginalD.end(), zero4);
    thrust::fill(freeSurfaceIdD.begin(), freeSurfaceIdD.end(), 0);

    thrust::fill(vel_XSPH_D.begin(), vel_XSPH_D.end(), zero3);

    if (paramsH->integration_scheme == IntegrationScheme::IMPLICIT_SPH)
        thrust::fill(sr_tau_I_mu_i.begin(), sr_tau_I_mu_i.end(), zero4);

    //// TODO: ISPH only
    thrust::fill(bceAcc.begin(), bceAcc.end(), zero3);

    //// TODO: elasticSPH only
    thrust::fill(derivTauXxYyZzD.begin(), derivTauXxYyZzD.end(), zero3);
    thrust::fill(derivTauXyXzYzD.begin(), derivTauXyXzYzD.end(), zero3);

    //// Time step vectors
    thrust::fill(courantViscousTimeStepD.begin(), courantViscousTimeStepD.end(), std::numeric_limits<Real>::max());
    thrust::fill(accelerationTimeStepD.begin(), accelerationTimeStepD.end(), std::numeric_limits<Real>::max());
}

// ------------------------------------------------------------------------------

void FsiDataManager::ResizeArrays(uint numExtended) {
    bool should_shrink = false;
    m_resize_counter++;

    // On first allocation or if we exceed current capacity, grow with buffer
    if (numExtended > m_max_extended_particles) {
        // Add buffer for future growth
        uint new_capacity = static_cast<uint>(numExtended * GROWTH_FACTOR);

        // Reserve space in all arrays
        markersProximity_D->gridMarkerHashD.reserve(new_capacity);
        markersProximity_D->gridMarkerIndexD.reserve(new_capacity);
        sortedSphMarkers2_D->posRadD.reserve(new_capacity);
        sortedSphMarkers2_D->velMasD.reserve(new_capacity);
        sortedSphMarkers2_D->rhoPresMuD.reserve(new_capacity);
        sortedSphMarkers2_D->tauXxYyZzD.reserve(new_capacity);
        sortedSphMarkers2_D->tauXyXzYzD.reserve(new_capacity);
        sortedSphMarkers1_D->posRadD.reserve(new_capacity);
        sortedSphMarkers1_D->velMasD.reserve(new_capacity);
        sortedSphMarkers1_D->rhoPresMuD.reserve(new_capacity);
        sortedSphMarkers1_D->tauXxYyZzD.reserve(new_capacity);
        sortedSphMarkers1_D->tauXyXzYzD.reserve(new_capacity);
        freeSurfaceIdD.reserve(new_capacity);
        vel_XSPH_D.reserve(new_capacity);
        courantViscousTimeStepD.reserve(new_capacity);
        accelerationTimeStepD.reserve(new_capacity);

        m_max_extended_particles = new_capacity;
    }

    // Check if we should shrink based on counter and memory usage
    if (m_resize_counter >= SHRINK_INTERVAL) {
        float usage_ratio = float(numExtended) / markersProximity_D->gridMarkerHashD.capacity();
        should_shrink = (usage_ratio < SHRINK_THRESHOLD);
        m_resize_counter = 0;
    }

    // Always resize to actual size needed
    markersProximity_D->gridMarkerHashD.resize(numExtended);
    markersProximity_D->gridMarkerIndexD.resize(numExtended);
    sortedSphMarkers2_D->posRadD.resize(numExtended);
    sortedSphMarkers2_D->velMasD.resize(numExtended);
    sortedSphMarkers2_D->rhoPresMuD.resize(numExtended);
    sortedSphMarkers2_D->tauXxYyZzD.resize(numExtended);
    sortedSphMarkers2_D->tauXyXzYzD.resize(numExtended);
    sortedSphMarkers1_D->posRadD.resize(numExtended);
    sortedSphMarkers1_D->velMasD.resize(numExtended);
    sortedSphMarkers1_D->rhoPresMuD.resize(numExtended);
    sortedSphMarkers1_D->tauXxYyZzD.resize(numExtended);
    sortedSphMarkers1_D->tauXyXzYzD.resize(numExtended);
    derivVelRhoD.resize(numExtended);
    derivTauXxYyZzD.resize(numExtended);
    derivTauXyXzYzD.resize(numExtended);
    freeSurfaceIdD.resize(numExtended);
    vel_XSPH_D.resize(numExtended);
    courantViscousTimeStepD.resize(numExtended);
    accelerationTimeStepD.resize(numExtended);
    // Only shrink periodically if needed
    if (should_shrink) {
        markersProximity_D->gridMarkerHashD.shrink_to_fit();
        markersProximity_D->gridMarkerIndexD.shrink_to_fit();
        sortedSphMarkers2_D->posRadD.shrink_to_fit();
        sortedSphMarkers2_D->velMasD.shrink_to_fit();
        sortedSphMarkers2_D->rhoPresMuD.shrink_to_fit();
        sortedSphMarkers2_D->tauXxYyZzD.shrink_to_fit();
        sortedSphMarkers2_D->tauXyXzYzD.shrink_to_fit();
        sortedSphMarkers1_D->posRadD.shrink_to_fit();
        sortedSphMarkers1_D->velMasD.shrink_to_fit();
        sortedSphMarkers1_D->rhoPresMuD.shrink_to_fit();
        sortedSphMarkers1_D->tauXxYyZzD.shrink_to_fit();
        sortedSphMarkers1_D->tauXyXzYzD.shrink_to_fit();
        derivVelRhoD.shrink_to_fit();
        derivTauXxYyZzD.shrink_to_fit();
        derivTauXyXzYzD.shrink_to_fit();
        freeSurfaceIdD.shrink_to_fit();
        vel_XSPH_D.shrink_to_fit();
        courantViscousTimeStepD.shrink_to_fit();
        accelerationTimeStepD.shrink_to_fit();
        // Update max particles to match new capacity
        m_max_extended_particles = (uint)markersProximity_D->gridMarkerHashD.capacity();
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void FsiDataManager::Initialize(unsigned int num_fsi_bodies,
                                unsigned int num_fsi_nodes1D,
                                unsigned int num_fsi_elements1D,
                                unsigned int num_fsi_nodes2D,
                                unsigned int num_fsi_elements2D,
                                NodeDirections node_directions_mode) {
    ConstructReferenceArray();
    SetCounters(num_fsi_bodies, num_fsi_nodes1D, num_fsi_elements1D, num_fsi_nodes2D, num_fsi_elements2D);

    if (countersH->numAllMarkers != sphMarkers_H->rhoPresMuH.size()) {
        std::cerr << "ERROR (Initialize): mismatch in total number of markers." << std::endl;
        throw std::runtime_error("Mismatch in total number of markers.");
    }

    sphMarkers_D->resize(countersH->numAllMarkers);
    sphMarkers_H->resize(countersH->numAllMarkers);
    markersProximity_D->resize(countersH->numAllMarkers);

    derivVelRhoOriginalD.resize(countersH->numAllMarkers);  // unsorted

    if (paramsH->integration_scheme == IntegrationScheme::IMPLICIT_SPH) {
        Real tiny = Real(1e-20);
        vis_vel_SPH_D.resize(countersH->numAllMarkers, mR3(tiny));
        sr_tau_I_mu_i.resize(countersH->numAllMarkers, mR4(tiny));           // sorted
        sr_tau_I_mu_i_Original.resize(countersH->numAllMarkers, mR4(tiny));  // unsorted
    }

    //// TODO: why is this sized for both WCSPH and ISPH?!?
    bceAcc.resize(countersH->numAllMarkers, mR3(0));  // Rigid/flex body accelerations from motion

    activityIdentifierOriginalD.resize(countersH->numAllMarkers, 1);
    activityIdentifierSortedD.resize(countersH->numAllMarkers, 1);
    extendedActivityIdentifierOriginalD.resize(countersH->numAllMarkers, 1);
    prefixSumExtendedActivityIdD.resize(countersH->numAllMarkers, 1);
    activeListD.resize(countersH->numAllMarkers, 1);
    // Number of neighbors for the particle of given index
    numNeighborsPerPart.resize(countersH->numAllMarkers + 1, 0);

    thrust::copy(sphMarkers_H->posRadH.begin(), sphMarkers_H->posRadH.end(), sphMarkers_D->posRadD.begin());
    thrust::copy(sphMarkers_H->velMasH.begin(), sphMarkers_H->velMasH.end(), sphMarkers_D->velMasD.begin());
    thrust::copy(sphMarkers_H->rhoPresMuH.begin(), sphMarkers_H->rhoPresMuH.end(), sphMarkers_D->rhoPresMuD.begin());
    thrust::copy(sphMarkers_H->tauXxYyZzH.begin(), sphMarkers_H->tauXxYyZzH.end(), sphMarkers_D->tauXxYyZzD.begin());
    thrust::copy(sphMarkers_H->tauXyXzYzH.begin(), sphMarkers_H->tauXyXzYzH.end(), sphMarkers_D->tauXyXzYzD.begin());

    fsiBodyState_D->Resize(countersH->numFsiBodies);
    fsiBodyState_H->Resize(countersH->numFsiBodies);

    rigid_FSI_ForcesD.resize(countersH->numFsiBodies);
    rigid_FSI_TorquesD.resize(countersH->numFsiBodies);

    bool use_node_directions = (node_directions_mode != NodeDirections::NONE);
    fsiMesh1DState_D->Resize(countersH->numFsiNodes1D, use_node_directions);
    fsiMesh1DState_H->Resize(countersH->numFsiNodes1D, use_node_directions);
    fsiMesh2DState_D->Resize(countersH->numFsiNodes2D, use_node_directions);
    fsiMesh2DState_H->Resize(countersH->numFsiNodes2D, use_node_directions);

    flex1D_FSIforces_D.resize(countersH->numFsiNodes1D);
    flex2D_FSIforces_D.resize(countersH->numFsiNodes2D);
}

//--------------------------------------------------------------------------------------------------------------------------------

struct extract_functor {
    extract_functor() {}
    __device__ Real3 operator()(Real4& x) const { return mR3(x); }
};

struct scale_functor {
    scale_functor(Real a) : m_a(a) {}
    Real3 operator()(Real3& x) const { return m_a * x; }
    const Real m_a;
};

std::vector<Real3> FsiDataManager::GetPositions() {
    auto& pos4_D = sphMarkers_D->posRadD;

    // Extract positions only (drop radius)
    thrust::device_vector<Real3> pos_D(pos4_D.size());
    thrust::transform(thrust::device,  // execution policy
                      pos4_D.begin(), pos4_D.end(), pos_D.begin(), extract_functor());

    // Copy to output
    std::vector<Real3> pos_H(pos_D.size());
    thrust::copy(pos_D.begin(), pos_D.end(), pos_H.begin());
    return pos_H;
}

std::vector<Real3> FsiDataManager::GetVelocities() {
    const auto& vel_D = sphMarkers_D->velMasD;

    // Copy to output
    std::vector<Real3> vel_H(vel_D.size());
    thrust::copy(vel_D.begin(), vel_D.end(), vel_H.begin());
    return vel_H;
}

std::vector<Real3> FsiDataManager::GetAccelerations() {
    // Copy data for SPH particles only
    const auto n = countersH->numFluidMarkers;
    thrust::device_vector<Real4> acc4_D(n);
    thrust::copy_n(derivVelRhoD.begin(), n, acc4_D.begin());

    // Extract acceleration (drop density)
    thrust::device_vector<Real3> acc_D(n);
    thrust::transform(acc4_D.begin(), acc4_D.end(), acc_D.begin(), extract_functor());

    // Copy to output
    std::vector<Real3> acc_H(acc_D.size());
    thrust::copy(acc_D.begin(), acc_D.end(), acc_H.begin());
    return acc_H;
}

std::vector<Real3> FsiDataManager::GetForces() {
    std::vector<Real3> frc_H = GetAccelerations();
    std::transform(frc_H.begin(), frc_H.end(), frc_H.begin(), scale_functor(paramsH->markerMass));
    return frc_H;
}

std::vector<Real3> FsiDataManager::GetProperties() {
    auto& prop4_D = sphMarkers_D->rhoPresMuD;

    // Extract fluid properties only (drop particle type)
    thrust::device_vector<Real3> prop_D(prop4_D.size());
    thrust::transform(prop4_D.begin(), prop4_D.end(), prop_D.begin(), extract_functor());

    // Copy to output
    std::vector<Real3> prop_H(prop_D.size());
    thrust::copy(prop_D.begin(), prop_D.end(), prop_H.begin());
    return prop_H;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<Real3> FsiDataManager::GetPositions(const std::vector<int>& indices) {
    thrust::device_vector<int> indices_D(indices.size());
    thrust::copy(indices.begin(), indices.end(), indices_D.begin());

    // Get all extended positions
    auto& allpos4_D = sphMarkers_D->posRadD;

    // Gather only those for specified indices
    thrust::device_vector<Real4> pos4_D(allpos4_D.size());
    auto end = thrust::gather(thrust::device,                      // execution policy
                              indices_D.begin(), indices_D.end(),  // range of gather locations
                              allpos4_D.begin(),                   // beginning of source
                              pos4_D.begin()                       // beginning of destination
    );

    // Extract positions only (drop radius)
    thrust::device_vector<Real3> pos_D(pos4_D.size());
    thrust::transform(pos4_D.begin(), pos4_D.end(), pos_D.begin(), extract_functor());

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - pos4_D.begin());
    assert(num_active == indices_D.size());
    pos_D.resize(num_active);

    // Copy to output
    std::vector<Real3> pos_H(pos_D.size());
    thrust::copy(pos_D.begin(), pos_D.end(), pos_H.begin());
    return pos_H;
}

std::vector<Real3> FsiDataManager::GetVelocities(const std::vector<int>& indices) {
    thrust::device_vector<int> indices_D(indices.size());
    thrust::copy(indices.begin(), indices.end(), indices_D.begin());

    // Get all velocities
    auto allvel_D = sphMarkers_D->velMasD;

    // Gather only those for specified indices
    thrust::device_vector<Real3> vel_D(allvel_D.size());
    auto end = thrust::gather(thrust::device,                      // execution policy
                              indices_D.begin(), indices_D.end(),  // range of gather locations
                              allvel_D.begin(),                    // beginning of source
                              vel_D.begin()                        // beginning of destination
    );

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - vel_D.begin());
    assert(num_active == indices_D.size());
    vel_D.resize(num_active);

    // Copy to output
    std::vector<Real3> vel_H(vel_D.size());
    thrust::copy(vel_D.begin(), vel_D.end(), vel_H.begin());
    return vel_H;
}

std::vector<Real3> FsiDataManager::GetAccelerations(const std::vector<int>& indices) {
    thrust::device_vector<int> indices_D(indices.size());
    thrust::copy(indices.begin(), indices.end(), indices_D.begin());

    // Get all extended accelerations
    const auto n = countersH->numFluidMarkers;
    thrust::device_vector<Real4> allacc4_D(n);
    thrust::copy_n(derivVelRhoD.begin(), n, allacc4_D.begin());

    // Gather only those for specified indices
    thrust::device_vector<Real4> acc4_D(allacc4_D.size());
    auto end = thrust::gather(thrust::device,                      // execution policy
                              indices_D.begin(), indices_D.end(),  // range of gather locations
                              allacc4_D.begin(),                   // beginning of source
                              acc4_D.begin()                       // beginning of destination
    );

    // Extract acceleration (drop density)
    thrust::device_vector<Real3> acc_D(n);
    thrust::transform(acc4_D.begin(), acc4_D.end(), acc_D.begin(), extract_functor());

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - acc4_D.begin());
    assert(num_active == indices_D.size());
    acc_D.resize(num_active);

    // Copy to output
    std::vector<Real3> acc_H(acc_D.size());
    thrust::copy(acc_D.begin(), acc_D.end(), acc_H.begin());
    return acc_H;
}

std::vector<Real3> FsiDataManager::GetForces(const std::vector<int>& indices) {
    std::vector<Real3> frc_H = GetAccelerations(indices);
    std::transform(frc_H.begin(), frc_H.end(), frc_H.begin(), scale_functor(paramsH->markerMass));
    return frc_H;
}

//--------------------------------------------------------------------------------------------------------------------------------

std::vector<Real3> FsiDataManager::GetRigidForces() {
    std::vector<Real3> out_H(rigid_FSI_ForcesD.size());
    thrust::copy(rigid_FSI_ForcesD.begin(), rigid_FSI_ForcesD.end(), out_H.begin());
    return out_H;
}

std::vector<Real3> FsiDataManager::GetRigidTorques() {
    std::vector<Real3> out_H(rigid_FSI_TorquesD.size());
    thrust::copy(rigid_FSI_TorquesD.begin(), rigid_FSI_TorquesD.end(), out_H.begin());
    return out_H;
}

std::vector<Real3> FsiDataManager::GetFlex1dForces() {
    std::vector<Real3> out_H(flex1D_FSIforces_D.size());
    thrust::copy(flex1D_FSIforces_D.begin(), flex1D_FSIforces_D.end(), out_H.begin());
    return out_H;
}

std::vector<Real3> FsiDataManager::GetFlex2dForces() {
    std::vector<Real3> out_H(flex2D_FSIforces_D.size());
    thrust::copy(flex2D_FSIforces_D.begin(), flex2D_FSIforces_D.end(), out_H.begin());
    return out_H;
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

std::vector<int> FsiDataManager::FindParticlesInBox(const Real3& hsize,
                                                    const Real3& pos,
                                                    const Real3& ax,
                                                    const Real3& ay,
                                                    const Real3& az) {
    // Extract indices of SPH particles contained in the OBB
    auto& ref = referenceArray;
    auto& pos_D = sphMarkers_D->posRadD;

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

    // Copy to output
    std::vector<int> indices_H;
    thrust::copy(indices_D.begin(), indices_D.end(), indices_H.begin());
    return indices_H;
}

size_t FsiDataManager::GetCurrentGPUMemoryUsage() const {
    size_t total_bytes = 0;

    // SPH marker data
    total_bytes += sphMarkers_D->posRadD.capacity() * sizeof(Real4);
    total_bytes += sphMarkers_D->velMasD.capacity() * sizeof(Real3);
    total_bytes += sphMarkers_D->rhoPresMuD.capacity() * sizeof(Real4);
    total_bytes += sphMarkers_D->tauXxYyZzD.capacity() * sizeof(Real3);
    total_bytes += sphMarkers_D->tauXyXzYzD.capacity() * sizeof(Real3);

    // Sorted SPH marker data (state 1)
    total_bytes += sortedSphMarkers1_D->posRadD.capacity() * sizeof(Real4);
    total_bytes += sortedSphMarkers1_D->velMasD.capacity() * sizeof(Real3);
    total_bytes += sortedSphMarkers1_D->rhoPresMuD.capacity() * sizeof(Real4);
    total_bytes += sortedSphMarkers1_D->tauXxYyZzD.capacity() * sizeof(Real3);
    total_bytes += sortedSphMarkers1_D->tauXyXzYzD.capacity() * sizeof(Real3);

    // Sorted SPH marker data (state 2)
    total_bytes += sortedSphMarkers2_D->posRadD.capacity() * sizeof(Real4);
    total_bytes += sortedSphMarkers2_D->velMasD.capacity() * sizeof(Real3);
    total_bytes += sortedSphMarkers2_D->rhoPresMuD.capacity() * sizeof(Real4);

    // Proximity data
    total_bytes += markersProximity_D->gridMarkerHashD.capacity() * sizeof(uint);
    total_bytes += markersProximity_D->gridMarkerIndexD.capacity() * sizeof(uint);
    total_bytes += markersProximity_D->cellStartD.capacity() * sizeof(uint);
    total_bytes += markersProximity_D->cellEndD.capacity() * sizeof(uint);
    total_bytes += markersProximity_D->mapOriginalToSorted.capacity() * sizeof(uint);

    // FSI body state data
    total_bytes += fsiBodyState_D->pos.capacity() * sizeof(Real3);
    total_bytes += fsiBodyState_D->lin_vel.capacity() * sizeof(Real3);
    total_bytes += fsiBodyState_D->lin_acc.capacity() * sizeof(Real3);
    total_bytes += fsiBodyState_D->rot.capacity() * sizeof(Real4);
    total_bytes += fsiBodyState_D->ang_vel.capacity() * sizeof(Real3);
    total_bytes += fsiBodyState_D->ang_acc.capacity() * sizeof(Real3);

    // FSI mesh state data (1D)
    total_bytes += fsiMesh1DState_D->pos.capacity() * sizeof(Real3);
    total_bytes += fsiMesh1DState_D->vel.capacity() * sizeof(Real3);
    total_bytes += fsiMesh1DState_D->acc.capacity() * sizeof(Real3);

    // FSI mesh state data (2D)
    total_bytes += fsiMesh2DState_D->pos.capacity() * sizeof(Real3);
    total_bytes += fsiMesh2DState_D->vel.capacity() * sizeof(Real3);
    total_bytes += fsiMesh2DState_D->acc.capacity() * sizeof(Real3);

    // Fluid data
    total_bytes += derivVelRhoD.capacity() * sizeof(Real4);
    total_bytes += derivVelRhoOriginalD.capacity() * sizeof(Real4);
    total_bytes += derivTauXxYyZzD.capacity() * sizeof(Real3);
    total_bytes += derivTauXyXzYzD.capacity() * sizeof(Real3);
    total_bytes += vel_XSPH_D.capacity() * sizeof(Real3);
    total_bytes += vis_vel_SPH_D.capacity() * sizeof(Real3);
    total_bytes += bceAcc.capacity() * sizeof(Real3);
    if (paramsH->integration_scheme == IntegrationScheme::IMPLICIT_SPH) {
        total_bytes += sr_tau_I_mu_i.capacity() * sizeof(Real4);
        total_bytes += sr_tau_I_mu_i_Original.capacity() * sizeof(Real4);
    }

    // Activity and neighbor data
    total_bytes += activityIdentifierOriginalD.capacity() * sizeof(int32_t);
    total_bytes += activityIdentifierSortedD.capacity() * sizeof(int32_t);
    total_bytes += extendedActivityIdentifierOriginalD.capacity() * sizeof(int32_t);
    total_bytes += prefixSumExtendedActivityIdD.capacity() * sizeof(uint);
    total_bytes += activeListD.capacity() * sizeof(uint);
    total_bytes += numNeighborsPerPart.capacity() * sizeof(uint);
    total_bytes += neighborList.capacity() * sizeof(uint);
    total_bytes += freeSurfaceIdD.capacity() * sizeof(uint);

    // BCE data
    total_bytes += rigid_BCEcoords_D.capacity() * sizeof(Real3);
    total_bytes += flex1D_BCEcoords_D.capacity() * sizeof(Real3);
    total_bytes += flex2D_BCEcoords_D.capacity() * sizeof(Real3);
    total_bytes += rigid_BCEsolids_D.capacity() * sizeof(uint);
    total_bytes += flex1D_BCEsolids_D.capacity() * sizeof(uint3);
    total_bytes += flex2D_BCEsolids_D.capacity() * sizeof(uint3);

    // FSI forces
    total_bytes += rigid_FSI_ForcesD.capacity() * sizeof(Real3);
    total_bytes += rigid_FSI_TorquesD.capacity() * sizeof(Real3);
    total_bytes += flex1D_FSIforces_D.capacity() * sizeof(Real3);
    total_bytes += flex2D_FSIforces_D.capacity() * sizeof(Real3);

    // FSI nodes
    total_bytes += flex1D_Nodes_D.capacity() * sizeof(int2);
    total_bytes += flex2D_Nodes_D.capacity() * sizeof(int3);

    return total_bytes;
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
