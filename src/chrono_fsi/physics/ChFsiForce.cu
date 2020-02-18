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
// Author: Milad Rakhsha
// =============================================================================
//
// Base class for processing sph force in fsi system.//
// =============================================================================

#include <thrust/extrema.h>
#include <thrust/sort.h>
#include "chrono_fsi/physics/ChFsiForce.cuh"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

//==========================================================================================================================================
namespace chrono {
namespace fsi {

ChFsiForce::ChFsiForce(std::shared_ptr<ChBce> otherBceWorker,
                       std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,
                       std::shared_ptr<ProximityDataD> otherMarkersProximityD,
                       std::shared_ptr<FsiGeneralData> otherFsiGeneralData,
                       std::shared_ptr<SimParams> otherParamsH,
                       std::shared_ptr<NumberOfObjects> otherNumObjects)
    : bceWorker(otherBceWorker),
      sortedSphMarkersD(otherSortedSphMarkersD),
      markersProximityD(otherMarkersProximityD),
      fsiGeneralData(otherFsiGeneralData),
      numObjectsH(otherNumObjects),
      paramsH(otherParamsH) {
    fsiCollisionSystem =
        chrono_types::make_shared<ChCollisionSystemFsi>(sortedSphMarkersD, markersProximityD, paramsH, numObjectsH);
    sphMarkersD = NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForce::Finalize() {
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(NumberOfObjects));
    printf("ChFsiForce::Finalize() numAllMarkers=%zd\n", numObjectsH->numAllMarkers);

    vel_XSPH_Sorted_D.resize(numObjectsH->numAllMarkers);
    vel_vis_Sorted_D.resize(numObjectsH->numAllMarkers);
    derivVelRhoD_Sorted_D.resize(numObjectsH->numAllMarkers);
    fsiCollisionSystem->Finalize();
}
//--------------------------------------------------------------------------------------------------------------------------------

ChFsiForce::~ChFsiForce() {}

void ChFsiForce::SetLinearSolver(ChFsiLinearSolver::SolverType other_solverType) {
    switch (other_solverType) {
        case ChFsiLinearSolver::SolverType::BICGSTAB:
            myLinearSolver = chrono_types::make_shared<ChFsiLinearSolverBiCGStab>();
            break;
        case ChFsiLinearSolver::SolverType::GMRES:
            myLinearSolver = chrono_types::make_shared<ChFsiLinearSolverGMRES>();
            break;
            /// Extend this function with your own linear solvers
        default:

            myLinearSolver = chrono_types::make_shared<ChFsiLinearSolverBiCGStab>();
            std::cout << "The ChFsiLinearSolver you chose has not been implemented, reverting back to "
                         "ChFsiLinearSolverBiCGStab\n";
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
// use invasive to avoid one extra copy. However, keep in mind that sorted is
// changed.
void ChFsiForce::CopySortedToOriginal_Invasive_R3(thrust::device_vector<Real3>& original,
                                                  thrust::device_vector<Real3>& sorted,
                                                  const thrust::device_vector<uint>& gridMarkerIndex) {
    thrust::device_vector<uint> dummyMarkerIndex = gridMarkerIndex;
    thrust::sort_by_key(dummyMarkerIndex.begin(), dummyMarkerIndex.end(), sorted.begin());
    dummyMarkerIndex.clear();
    thrust::copy(sorted.begin(), sorted.end(), original.begin());
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForce::CopySortedToOriginal_NonInvasive_R3(thrust::device_vector<Real3>& original,
                                                     const thrust::device_vector<Real3>& sorted,
                                                     const thrust::device_vector<uint>& gridMarkerIndex) {
    thrust::device_vector<Real3> dummySorted = sorted;
    CopySortedToOriginal_Invasive_R3(original, dummySorted, gridMarkerIndex);
}
//--------------------------------------------------------------------------------------------------------------------------------
// use invasive to avoid one extra copy. However, keep in mind that sorted is
// changed.
void ChFsiForce::CopySortedToOriginal_Invasive_R4(thrust::device_vector<Real4>& original,
                                                  thrust::device_vector<Real4>& sorted,
                                                  const thrust::device_vector<uint>& gridMarkerIndex) {
    thrust::device_vector<uint> dummyMarkerIndex = gridMarkerIndex;
    thrust::sort_by_key(dummyMarkerIndex.begin(), dummyMarkerIndex.end(), sorted.begin());
    dummyMarkerIndex.clear();
    thrust::copy(sorted.begin(), sorted.end(), original.begin());
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForce::CopySortedToOriginal_NonInvasive_R4(thrust::device_vector<Real4>& original,
                                                     thrust::device_vector<Real4>& sorted,
                                                     const thrust::device_vector<uint>& gridMarkerIndex) {
    thrust::device_vector<Real4> dummySorted = sorted;
    CopySortedToOriginal_Invasive_R4(original, dummySorted, gridMarkerIndex);
}
//--------------------------------------------------------------------------------------------------------------------------------

}  // namespace fsi
}  // namespace chrono
