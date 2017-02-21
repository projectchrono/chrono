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
// Utility class for generating BCE markers.//
// =============================================================================

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/ChDeviceUtils.cuh"
#include "chrono_fsi/ChFsiTypeConvert.h"
#include "chrono_fsi/custom_math.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

namespace chrono {
namespace fsi {
namespace utils {

// =============================================================================
// left off here
// TransformToCOG
// This utility function converts a given position and orientation, specified
// with respect to a body's reference frame, into a frame defined with respect
// to the body's centroidal frame.  Note that by default, a body's reference
// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
void TransformBceFrameToCOG(ChBody* body, const ChVector<>& pos, const ChMatrix33<>& rot, ChFrame<>& frame) {
    frame = ChFrame<>(pos, rot);
    if (ChBodyAuxRef* body_ar = dynamic_cast<ChBodyAuxRef*>(body)) {
        frame = frame >> body_ar->GetFrame_REF_to_COG();
    }
}

ChVector<> TransformBCEToCOG(ChBody* body, const ChVector<>& pos) {
    ChFrame<> frame;
    TransformBceFrameToCOG(body, pos, QUNIT, frame);
    return frame.GetPos();
}

ChVector<> TransformBCEToCOG(ChBody* body, const Real3& pos3) {
    ChVector<> pos = ChFsiTypeConvert::Real3ToChVector(pos3);
    return TransformBCEToCOG(body, pos);
}
// =============================================================================
void CreateBceGlobalMarkersFromBceLocalPos(ChFsiDataManager* fsiData,
                                           SimParams* paramsH,
                                           const thrust::host_vector<Real3>& posRadBCE,
                                           std::shared_ptr<ChBody> body,
                                           ChVector<> collisionShapeRelativePos,
                                           ChQuaternion<> collisionShapeRelativeRot,
                                           bool isSolid) {
    if (fsiData->fsiGeneralData.referenceArray.size() < 1) {
        printf(
            "\n\n\n\n Error! fluid need to be initialized before boundary. "
            "Reference array should have two "
            "components \n\n\n\n");
        std::cin.get();
    }
    ::int4 refSize4 = fsiData->fsiGeneralData.referenceArray[fsiData->fsiGeneralData.referenceArray.size() - 1];
    int type = 0;
    if (isSolid) {
        type = refSize4.w + 1;
    }
    if (type < 0) {
        printf(
            "\n\n\n\n Error! reference array type is not correct. It does not "
            "denote boundary or rigid \n\n\n\n");
        std::cin.get();
    } else if (type > 0 && (fsiData->fsiGeneralData.referenceArray.size() - 1 != type)) {
        printf("\n\n\n\n Error! reference array size does not match type \n\n\n\n");
        std::cin.get();
    }

    //#pragma omp parallel for  // it is very wrong to do it in parallel. race
    // condition will occur
    for (int i = 0; i < posRadBCE.size(); i++) {
        ChVector<> posLoc_collisionShape = ChFsiTypeConvert::Real3ToChVector(posRadBCE[i]);
        ChVector<> posLoc_body = ChTransform<>::TransformLocalToParent(posLoc_collisionShape, collisionShapeRelativePos,
                                                                       collisionShapeRelativeRot);
        ChVector<> posLoc_COG = TransformBCEToCOG(body.get(), posLoc_body);
        ChVector<> posGlob = ChTransform<>::TransformLocalToParent(posLoc_COG, body->GetPos(), body->GetRot());
        fsiData->sphMarkersH.posRadH.push_back(ChFsiTypeConvert::ChVectorToReal3(posGlob));

        ChVector<> vAbs = body->PointSpeedLocalToParent(posLoc_COG);
        Real3 v3 = ChFsiTypeConvert::ChVectorToReal3(vAbs);
        fsiData->sphMarkersH.velMasH.push_back(v3);

        fsiData->sphMarkersH.rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, type));
    }

    // ------------------------
    // Modify number of objects
    // ------------------------

    int numBce = posRadBCE.size();
    fsiData->numObjects.numAllMarkers += numBce;
    if (type == 0) {
        fsiData->numObjects.numBoundaryMarkers += numBce;
        if (fsiData->fsiGeneralData.referenceArray.size() == 1) {
            fsiData->fsiGeneralData.referenceArray.push_back(mI4(refSize4.y, refSize4.y + numBce, 0, 0));
        } else if (fsiData->fsiGeneralData.referenceArray.size() == 2) {
            refSize4.y = refSize4.y + numBce;
            fsiData->fsiGeneralData.referenceArray[1] = refSize4;
        } else {
            printf(
                "Error! reference array size is greater than 2 while marker type "
                "is 0 \n\n");
            std::cin.get();
        }
    } else {
        if (fsiData->fsiGeneralData.referenceArray.size() < 2) {
            printf(
                "Error! Boundary markers are not initialized while trying to "
                "initialize rigid marker!\n\n");
            std::cin.get();
        }
        fsiData->numObjects.numRigid_SphMarkers += numBce;
        fsiData->numObjects.numRigidBodies += 1;
        fsiData->numObjects.startRigidMarkers = fsiData->fsiGeneralData.referenceArray[1].y;
        fsiData->fsiGeneralData.referenceArray.push_back(
            mI4(refSize4.y, refSize4.y + numBce, 1, type));  // 1: for rigid
        if (fsiData->numObjects.numRigidBodies != fsiData->fsiGeneralData.referenceArray.size() - 2) {
            printf("Error! num rigid bodies does not match reference array size!\n\n");
            std::cin.get();
        }
    }

    //	SetNumObjects(numObjects, fsiGeneralData.referenceArray, numAllMarkers);
}
// =============================================================================
void CreateBceGlobalMarkersFromBceLocalPosBoundary(ChFsiDataManager* fsiData,
                                                   SimParams* paramsH,
                                                   const thrust::host_vector<Real3>& posRadBCE,
                                                   std::shared_ptr<ChBody> body,
                                                   ChVector<> collisionShapeRelativePos,
                                                   ChQuaternion<> collisionShapeRelativeRot) {
    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body, collisionShapeRelativePos,
                                          collisionShapeRelativeRot, false);
}
// =============================================================================
void AddSphereBce(ChFsiDataManager* fsiData,
                  SimParams* paramsH,
                  std::shared_ptr<ChBody> body,
                  ChVector<> relPos,
                  ChQuaternion<> relRot,
                  Real radius) {
    thrust::host_vector<Real3> posRadBCE;
    CreateBCE_On_Sphere(posRadBCE, radius, paramsH);

    //	if (fsiData->sphMarkersH.posRadH.size() !=
    // fsiData->numObjects.numAllMarkers) {
    //		printf("Error! numMarkers, %d, does not match posRadH.size(),
    //%d\n",
    //				fsiData->numObjects.numAllMarkers,
    // fsiData->sphMarkersH.posRadH.size());
    //		std::cin.get();
    //	}

    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body);

    posRadBCE.clear();
}
// =============================================================================

void AddCylinderBce(ChFsiDataManager* fsiData,
                    SimParams* paramsH,
                    std::shared_ptr<ChBody> body,
                    ChVector<> relPos,
                    ChQuaternion<> relRot,
                    Real radius,
                    Real height) {
    thrust::host_vector<Real3> posRadBCE;
    CreateBCE_On_Cylinder(posRadBCE, radius, height, paramsH);

    //	if (fsiData->sphMarkersH.posRadH.size() !=
    // fsiData->numObjects.numAllMarkers) {
    //		printf("Error! numMarkers, %d, does not match posRadH.size(),
    //%d\n",
    //				fsiData->numObjects.numAllMarkers,
    // fsiData->sphMarkersH.posRadH.size());
    //		std::cin.get();
    //	}

    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body);
    posRadBCE.clear();
}

// =============================================================================
// Arman note, the function in the current implementation creates boundary bce
// (accesses only referenceArray[1])

// Arman thrust::host_vector<uint>& bodyIndex,

// Arman later on, you can remove numObjects since the Finalize function will
// take care of setting up the numObjects

void AddBoxBce(ChFsiDataManager* fsiData,
               SimParams* paramsH,
               std::shared_ptr<ChBody> body,
               ChVector<> relPos,
               ChQuaternion<> relRot,
               const ChVector<>& size) {
    thrust::host_vector<Real3> posRadBCE;

    CreateBCE_On_Box(posRadBCE, ChFsiTypeConvert::ChVectorToReal3(size), 12, paramsH);
    //	if (fsiData->sphMarkersH.posRadH.size() !=
    // fsiData->numObjects.numAllMarkers) {
    //		printf("Error! numMarkers, %d, does not match posRadH.size(),
    //%d\n",
    //				fsiData->numObjects.numAllMarkers,
    // fsiData->sphMarkersH.posRadH.size());
    //		std::cin.get();
    //	}

    CreateBceGlobalMarkersFromBceLocalPosBoundary(fsiData, paramsH, posRadBCE, body, relPos, relRot);
    posRadBCE.clear();
}

// =============================================================================
void AddBCE_FromFile(ChFsiDataManager* fsiData,
                     SimParams* paramsH,
                     std::shared_ptr<ChBody> body,
                     std::string dataPath) {
    //----------------------------
    //  chassis
    //----------------------------
    thrust::host_vector<Real3> posRadBCE;

    LoadBCE_fromFile(posRadBCE, dataPath);

    //	if (fsiData->sphMarkersH.posRadH.size() !=
    // fsiData->numObjects.numAllMarkers) {
    //		printf("Error! numMarkers, %d, does not match posRadH.size(),
    //%d\n",
    //				fsiData->numObjects.numAllMarkers,
    // fsiData->sphMarkersH.posRadH.size());
    //		std::cin.get();
    //	}

    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body);
    posRadBCE.clear();
}

// =============================================================================
void CreateSphereFSI(ChFsiDataManager* fsiData,
                     ChSystem& mphysicalSystem,
                     std::vector<std::shared_ptr<ChBody>>* fsiBodeisPtr,
                     SimParams* paramsH,
                     std::shared_ptr<ChMaterialSurface> mat_prop,
                     Real density,
                     ChVector<> pos,
                     Real radius) {
    //	ChVector<> pos = ChVector<>(-9.5, .20, 3);
    //	Real radius = 0.3;

    auto body = std::make_shared<ChBody>(std::make_shared<collision::ChCollisionModelParallel>());
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetMaterialSurface(mat_prop);
    body->SetPos(pos);
    double volume = chrono::utils::CalcSphereVolume(radius);
    ChVector<> gyration = chrono::utils::CalcSphereGyration(radius).Get_Diag();
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    chrono::utils::AddSphereGeometry(body.get(), radius);
    body->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(body);
    fsiBodeisPtr->push_back(body);

    AddSphereBce(fsiData, paramsH, body, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), radius);
}
// =============================================================================
void CreateCylinderFSI(ChFsiDataManager* fsiData,
                       ChSystem& mphysicalSystem,
                       std::vector<std::shared_ptr<ChBody>>* fsiBodeisPtr,
                       SimParams* paramsH,
                       std::shared_ptr<ChMaterialSurface> mat_prop,
                       Real density,
                       ChVector<> pos,
                       ChQuaternion<> rot,
                       Real radius,
                       Real length) {
    auto body = std::make_shared<ChBody>(std::make_shared<collision::ChCollisionModelParallel>());
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetMaterialSurface(mat_prop);
    body->SetPos(pos);
    body->SetRot(rot);
    double volume = chrono::utils::CalcCylinderVolume(radius, 0.5 * length);
    ChVector<> gyration = chrono::utils::CalcCylinderGyration(radius, 0.5 * length).Get_Diag();
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    chrono::utils::AddCylinderGeometry(body.get(), radius, 0.5 * length);
    body->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(body);

    fsiBodeisPtr->push_back(body);
    AddCylinderBce(fsiData, paramsH, body, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), radius, length);
}
// =============================================================================
void CreateBoxFSI(ChFsiDataManager* fsiData,
                  ChSystem& mphysicalSystem,
                  std::vector<std::shared_ptr<ChBody>>* fsiBodeisPtr,
                  SimParams* paramsH,
                  std::shared_ptr<ChMaterialSurface> mat_prop,
                  Real density,
                  ChVector<> pos,
                  ChQuaternion<> rot,
                  const ChVector<>& hsize) {
    auto body = std::make_shared<ChBody>(std::make_shared<collision::ChCollisionModelParallel>());
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetMaterialSurface(mat_prop);
    body->SetPos(pos);
    body->SetRot(rot);
    double volume = chrono::utils::CalcBoxVolume(hsize);
    ChVector<> gyration = chrono::utils::CalcBoxGyration(hsize).Get_Diag();
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    chrono::utils::AddBoxGeometry(body.get(), hsize);
    body->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(body);

    fsiBodeisPtr->push_back(body);
    AddBoxBce(fsiData, paramsH, body, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), hsize);
}

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono
