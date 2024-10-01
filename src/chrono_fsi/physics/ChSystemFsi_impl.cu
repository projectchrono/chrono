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

zipIterSphH SphMarkerDataH::iterator() {
    return thrust::make_zip_iterator(thrust::make_tuple(posRadH.begin(), velMasH.begin(), rhoPresMuH.begin(),
                                                        tauXxYyZzH.begin(), tauXyXzYzH.begin()));
}

void SphMarkerDataH::resize(size_t s) {
    posRadH.resize(s);
    velMasH.resize(s);
    rhoPresMuH.resize(s);
    tauXxYyZzH.resize(s);
    tauXyXzYzH.resize(s);
}

//---------------------------------------------------------------------------------------

zipIterRigidD FsiBodyStateD::iterator() {
    return thrust::make_zip_iterator(thrust::make_tuple(pos.begin(), lin_vel.begin(), lin_acc.begin(),  //
                                                        rot.begin(), ang_vel.begin(), ang_acc.begin()));
}

void FsiBodyStateD::resize(size_t s) {
    pos.resize(s);
    lin_vel.resize(s);
    lin_acc.resize(s);
    rot.resize(s);
    ang_vel.resize(s);
    ang_acc.resize(s);
}

void FsiMeshStateH::resize(size_t s) {
    pos_fsi_fea_H.resize(s);
    vel_fsi_fea_H.resize(s);
    acc_fsi_fea_H.resize(s);
}

void FsiMeshStateD::resize(size_t s) {
    pos_fsi_fea_D.resize(s);
    vel_fsi_fea_D.resize(s);
    acc_fsi_fea_D.resize(s);
}

void FsiBodyStateD::CopyFromH(const FsiBodyStateH& bodyStateH) {
    thrust::copy(bodyStateH.pos.begin(), bodyStateH.pos.end(), pos.begin());
    thrust::copy(bodyStateH.lin_vel.begin(), bodyStateH.lin_vel.end(), lin_vel.begin());
    thrust::copy(bodyStateH.lin_acc.begin(), bodyStateH.lin_acc.end(), lin_acc.begin());
    thrust::copy(bodyStateH.rot.begin(), bodyStateH.rot.end(), rot.begin());
    thrust::copy(bodyStateH.ang_vel.begin(), bodyStateH.ang_vel.end(), ang_vel.begin());
    thrust::copy(bodyStateH.ang_acc.begin(), bodyStateH.ang_acc.end(), ang_acc.begin());
}

void FsiMeshStateD::CopyFromH(const FsiMeshStateH& meshStateH) {
    thrust::copy(meshStateH.pos_fsi_fea_H.begin(), meshStateH.pos_fsi_fea_H.end(), pos_fsi_fea_D.begin());
    thrust::copy(meshStateH.vel_fsi_fea_H.begin(), meshStateH.vel_fsi_fea_H.end(), vel_fsi_fea_D.begin());
    thrust::copy(meshStateH.acc_fsi_fea_H.begin(), meshStateH.acc_fsi_fea_H.end(), acc_fsi_fea_D.begin());
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

FsiMeshStateD& FsiMeshStateD::operator=(const FsiMeshStateD& other) {
    if (this == &other) {
        return *this;
    }
    thrust::copy(other.pos_fsi_fea_D.begin(), other.pos_fsi_fea_D.end(), pos_fsi_fea_D.begin());
    thrust::copy(other.vel_fsi_fea_D.begin(), other.vel_fsi_fea_D.end(), vel_fsi_fea_D.begin());
    thrust::copy(other.acc_fsi_fea_D.begin(), other.acc_fsi_fea_D.end(), acc_fsi_fea_D.begin());
    return *this;
}

//---------------------------------------------------------------------------------------
zipIterRigidH FsiBodyStateH::iterator() {
    return thrust::make_zip_iterator(thrust::make_tuple(pos.begin(), lin_vel.begin(), lin_acc.begin(),  //
                                                        rot.begin(), ang_vel.begin(), ang_acc.begin()));
}

void FsiBodyStateH::resize(size_t s) {
    pos.resize(s);
    lin_vel.resize(s);
    lin_acc.resize(s);
    rot.resize(s);
    ang_vel.resize(s);
    ang_acc.resize(s);
}

//---------------------------------------------------------------------------------------
void ProximityDataD::resize(size_t s) {
    gridMarkerHashD.resize(s);
    gridMarkerIndexD.resize(s);
    mapOriginalToSorted.resize(s);
}

//---------------------------------------------------------------------------------------

ChSystemFsi_impl::ChSystemFsi_impl(std::shared_ptr<SimParams> params) : ChFsiBase(params, nullptr) {
    numObjectsH = chrono_types::make_shared<ChCounters>();
    InitNumObjects();
    sphMarkers_D = chrono_types::make_shared<SphMarkerDataD>();
    sortedSphMarkers1_D = chrono_types::make_shared<SphMarkerDataD>();
    sortedSphMarkers2_D = chrono_types::make_shared<SphMarkerDataD>();
    sphMarkers_H = chrono_types::make_shared<SphMarkerDataH>();

    fsiBodyState1_D = chrono_types::make_shared<FsiBodyStateD>();
    fsiBodyState2_D = chrono_types::make_shared<FsiBodyStateD>();
    fsiBodyState_H = chrono_types::make_shared<FsiBodyStateH>();

    fsiMesh1DState_D = chrono_types::make_shared<FsiMeshStateD>();
    fsiMesh1DState_H = chrono_types::make_shared<FsiMeshStateH>();
    fsiMesh2DState_D = chrono_types::make_shared<FsiMeshStateD>();
    fsiMesh2DState_H = chrono_types::make_shared<FsiMeshStateH>();

    fsiData = chrono_types::make_shared<FsiData>();
    markersProximity_D = chrono_types::make_shared<ProximityDataD>();
}

ChSystemFsi_impl::~ChSystemFsi_impl() {}

void ChSystemFsi_impl::AddSPHParticle(Real4 pos, Real4 rhoPresMu, Real3 vel, Real3 tauXxYyZz, Real3 tauXyXzYz) {
    sphMarkers_H->posRadH.push_back(pos);
    sphMarkers_H->velMasH.push_back(vel);
    sphMarkers_H->rhoPresMuH.push_back(rhoPresMu);
    sphMarkers_H->tauXyXzYzH.push_back(tauXyXzYz);
    sphMarkers_H->tauXxYyZzH.push_back(tauXxYyZz);
}
void ChSystemFsi_impl::ArrangeDataManager() {
    thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkers_H->rhoPresMuH;
    dummyRhoPresMuH.clear();
}

void ChSystemFsi_impl::InitNumObjects() {
    numObjectsH->numRigidBodies = 0;      // Number of rigid bodies
    numObjectsH->numFlexBodies1D = 0;     // Number of 1D Flexible bodies
    numObjectsH->numFlexBodies2D = 0;     // Number of 2D Flexible bodies
    numObjectsH->numFlexNodes1D = 0;      // Number of 1D Flexible nodes
    numObjectsH->numFlexNodes2D = 0;      // Number of 2D Flexible nodes
    numObjectsH->numGhostMarkers = 0;     // Number of ghost particles
    numObjectsH->numHelperMarkers = 0;    // Number of helper particles
    numObjectsH->numFluidMarkers = 0;     // Number of fluid SPH particles
    numObjectsH->numBoundaryMarkers = 0;  // Number of boundary BCE markers
    numObjectsH->numRigidMarkers = 0;     // Number of rigid BCE markers
    numObjectsH->numFlexMarkers1D = 0;    // Number of flexible 1-D segment BCE markers
    numObjectsH->numFlexMarkers2D = 0;    // Number of flexible 2-D face BCE markers
    numObjectsH->numBceMarkers = 0;       // Total number of BCE markers
    numObjectsH->numAllMarkers = 0;       // Total number of SPH + BCE particles
    numObjectsH->startRigidMarkers = 0;   // Start index of the rigid BCE markers
    numObjectsH->startFlexMarkers1D = 0;  // Start index of the 1-D flexible BCE markers
    numObjectsH->startFlexMarkers2D = 0;  // Start index of the 2-D flexible BCE markers
}

void ChSystemFsi_impl::CalcNumObjects() {
    InitNumObjects();
    size_t rSize = fsiData->referenceArray.size();

    for (size_t i = 0; i < rSize; i++) {
        int4 rComp4 = fsiData->referenceArray[i];
        int numMarkers = rComp4.y - rComp4.x;

        switch (rComp4.z) {
            case -3:
                numObjectsH->numHelperMarkers += numMarkers;
                break;
            case -2:
                numObjectsH->numGhostMarkers += numMarkers;
                break;
            case -1:
                numObjectsH->numFluidMarkers += numMarkers;
                break;
            case 0:
                numObjectsH->numBoundaryMarkers += numMarkers;
                break;
            case 1:
                numObjectsH->numRigidMarkers += numMarkers;
                break;
            case 2:
                numObjectsH->numFlexMarkers1D += numMarkers;
                break;
            case 3:
                numObjectsH->numFlexMarkers2D += numMarkers;
                break;
            default:
                std::cerr << "ERROR (CalcNumObjects): particle type not defined." << std::endl;
                throw std::runtime_error("Particle type not defined.");
                break;
        }
    }

    numObjectsH->numFluidMarkers += numObjectsH->numGhostMarkers + numObjectsH->numHelperMarkers;
    numObjectsH->numBceMarkers = numObjectsH->numBoundaryMarkers + numObjectsH->numRigidMarkers +  //
                                 numObjectsH->numFlexMarkers1D + numObjectsH->numFlexMarkers2D;
    numObjectsH->numAllMarkers = numObjectsH->numFluidMarkers + numObjectsH->numBceMarkers;

    numObjectsH->startRigidMarkers = numObjectsH->numFluidMarkers + numObjectsH->numBoundaryMarkers;
    numObjectsH->startFlexMarkers1D = numObjectsH->startRigidMarkers + numObjectsH->numRigidMarkers;
    numObjectsH->startFlexMarkers2D = numObjectsH->startFlexMarkers1D + numObjectsH->numFlexMarkers1D;
}

void ChSystemFsi_impl::ConstructReferenceArray() {
    auto numAllMarkers = sphMarkers_H->rhoPresMuH.size();

    thrust::host_vector<int> numComponentMarkers(numAllMarkers);
    thrust::fill(numComponentMarkers.begin(), numComponentMarkers.end(), 1);
    thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkers_H->rhoPresMuH;

    auto new_end = thrust::reduce_by_key(dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(),  // keys first, last
                                         numComponentMarkers.begin(),                     // values first
                                         dummyRhoPresMuH.begin(),                         // keys out
                                         numComponentMarkers.begin(),                     // values out
                                         sphTypeCompEqual());

    size_t numberOfComponents = new_end.first - dummyRhoPresMuH.begin();

    dummyRhoPresMuH.resize(numberOfComponents);
    numComponentMarkers.resize(numberOfComponents);

    fsiData->referenceArray.clear();
    fsiData->referenceArray_FEA.clear();

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

        fsiData->referenceArray.push_back(new_entry);
        if (compType == 2 || compType == 3)
            fsiData->referenceArray_FEA.push_back(new_entry);
    }

    dummyRhoPresMuH.clear();
    numComponentMarkers.clear();
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi_impl::Initialize(size_t numRigidBodies,
                                  size_t numFlexBodies1D,
                                  size_t numFlexBodies2D,
                                  size_t numFlexNodes1D,
                                  size_t numFlexNodes2D) {
    ConstructReferenceArray();
    CalcNumObjects();

    if (numObjectsH->numAllMarkers != sphMarkers_H->rhoPresMuH.size()) {
        std::cerr << "ERROR (Initialize): mismatch in total number of markers." << std::endl;
        throw std::runtime_error("Mismatch in total number of markers.");
    }

    // Set number of interface objects
    numObjectsH->numRigidBodies = numRigidBodies;
    numObjectsH->numFlexBodies1D = numFlexBodies1D;
    numObjectsH->numFlexBodies2D = numFlexBodies2D;
    numObjectsH->numFlexNodes1D = numFlexNodes1D;
    numObjectsH->numFlexNodes2D = numFlexNodes2D;

    sphMarkers_D->resize(numObjectsH->numAllMarkers);
    sortedSphMarkers1_D->resize(numObjectsH->numAllMarkers);
    sortedSphMarkers2_D->resize(numObjectsH->numAllMarkers);
    sphMarkers_H->resize(numObjectsH->numAllMarkers);
    markersProximity_D->resize(numObjectsH->numAllMarkers);

    fsiData->derivVelRhoD.resize(numObjectsH->numAllMarkers);          // sorted
    fsiData->derivVelRhoOriginalD.resize(numObjectsH->numAllMarkers);  // unsorted

    fsiData->derivTauXxYyZzD.resize(numObjectsH->numAllMarkers);
    fsiData->derivTauXyXzYzD.resize(numObjectsH->numAllMarkers);

    fsiData->vel_XSPH_D.resize(numObjectsH->numAllMarkers);  // TODO (Huzaifa): Check if this is always sorted or not
    fsiData->vis_vel_SPH_D.resize(numObjectsH->numAllMarkers, mR3(1e-20));
    fsiData->sr_tau_I_mu_i.resize(numObjectsH->numAllMarkers, mR4(1e-20));           // sorted
    fsiData->sr_tau_I_mu_i_Original.resize(numObjectsH->numAllMarkers, mR4(1e-20));  // unsorted
    fsiData->bceAcc.resize(numObjectsH->numAllMarkers, mR3(0.0));  // Rigid/flex body accelerations from motion

    fsiData->activityIdentifierD.resize(numObjectsH->numAllMarkers, 1);
    fsiData->extendedActivityIdD.resize(numObjectsH->numAllMarkers, 1);

    fsiData->numNeighborsPerPart.resize(numObjectsH->numAllMarkers + 1,
                                        0);  // Stores the number of neighbors the particle given by the index has
    fsiData->freeSurfaceIdD.resize(numObjectsH->numAllMarkers, 0);

    thrust::copy(sphMarkers_H->posRadH.begin(), sphMarkers_H->posRadH.end(), sphMarkers_D->posRadD.begin());
    thrust::copy(sphMarkers_H->velMasH.begin(), sphMarkers_H->velMasH.end(), sphMarkers_D->velMasD.begin());
    thrust::copy(sphMarkers_H->rhoPresMuH.begin(), sphMarkers_H->rhoPresMuH.end(), sphMarkers_D->rhoPresMuD.begin());
    thrust::copy(sphMarkers_H->tauXxYyZzH.begin(), sphMarkers_H->tauXxYyZzH.end(), sphMarkers_D->tauXxYyZzD.begin());
    thrust::copy(sphMarkers_H->tauXyXzYzH.begin(), sphMarkers_H->tauXyXzYzH.end(), sphMarkers_D->tauXyXzYzD.begin());

    fsiBodyState1_D->resize(numObjectsH->numRigidBodies);
    fsiBodyState2_D->resize(numObjectsH->numRigidBodies);
    fsiBodyState_H->resize(numObjectsH->numRigidBodies);

    fsiData->rigid_FSI_ForcesD.resize(numObjectsH->numRigidBodies);
    fsiData->rigid_FSI_TorquesD.resize(numObjectsH->numRigidBodies);

    fsiData->rigid_BCEsolids_D.resize(numObjectsH->numRigidMarkers);
    fsiData->rigid_BCEcoords_D.resize(numObjectsH->numRigidMarkers);

    fsiMesh1DState_D->resize(numObjectsH->numFlexNodes1D);
    fsiMesh1DState_H->resize(numObjectsH->numFlexNodes1D);
    fsiMesh2DState_D->resize(numObjectsH->numFlexNodes2D);
    fsiMesh2DState_H->resize(numObjectsH->numFlexNodes2D);

    fsiData->flex1D_FSIforces_D.resize(numObjectsH->numFlexNodes1D);
    fsiData->flex2D_FSIforces_D.resize(numObjectsH->numFlexNodes2D);
}

//--------------------------------------------------------------------------------------------------------------------------------

struct scale_functor {
    scale_functor(Real a) : m_a(a) {}
    __host__ __device__ Real4 operator()(Real4& x) const { return m_a * x; }
    const Real m_a;
};

thrust::device_vector<Real4> ChSystemFsi_impl::GetParticleAccelerations() {
    const auto n = numObjectsH->numFluidMarkers;

    // Copy data for SPH particles only
    thrust::device_vector<Real4> accD(n);
    thrust::copy_n(fsiData->derivVelRhoD.begin(), n, accD.begin());

    return accD;
}

thrust::device_vector<Real4> ChSystemFsi_impl::GetParticleForces() {
    thrust::device_vector<Real4> frcD = GetParticleAccelerations();
    thrust::transform(frcD.begin(), frcD.end(), frcD.begin(), scale_functor(paramsH->markerMass));

    return frcD;
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
    auto& ref = fsiData->referenceArray;
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

    return indices_D;
}

// Gather positions from particles with specified indices
thrust::device_vector<Real4> ChSystemFsi_impl::GetParticlePositions(const thrust::device_vector<int>& indices) {
    const auto& allpos = sphMarkers_D->posRadD;

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

// Gather velocities from particles with specified indices
thrust::device_vector<Real3> ChSystemFsi_impl::GetParticleVelocities(const thrust::device_vector<int>& indices) {
    auto allvel = sphMarkers_D->velMasD;

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

// Gather accelerations from particles with specified indices
thrust::device_vector<Real4> ChSystemFsi_impl::GetParticleAccelerations(const thrust::device_vector<int>& indices) {
    auto allacc = GetParticleAccelerations();

    thrust::device_vector<Real4> acc(allacc.size());

    auto end = thrust::gather(thrust::device,                  // execution policy
                              indices.begin(), indices.end(),  // range of gather locations
                              allacc.begin(),                  // beginning of source
                              acc.begin()                      // beginning of destination
    );

    // Trim the output vector of particle positions
    size_t num_active = (size_t)(end - acc.begin());
    assert(num_active == indices.size());
    acc.resize(num_active);

    return acc;
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

}  // end namespace fsi
}  // end namespace chrono
