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
// Author: Arman Pazouki, Milad Rakhsha
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

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"

namespace chrono {
namespace fsi {
namespace utils {
void FinalizeDomainCreating(SimParams* paramsH) {
    paramsH->NUM_BOUNDARY_LAYERS = 3;
    paramsH->ApplyInFlowOutFlow = false;
    paramsH->Apply_BC_U = false;  ///< You should go to custom_math.h all the way to end of file and set your function
    paramsH->ApplyInFlowOutFlow = false;
    int3 side0 = mI3(floor((paramsH->cMax.x - paramsH->cMin.x) / (2 * paramsH->HSML)),
                     floor((paramsH->cMax.y - paramsH->cMin.y) / (2 * paramsH->HSML)),
                     floor((paramsH->cMax.z - paramsH->cMin.z) / (2 * paramsH->HSML)));
    Real3 binSize3 = mR3((paramsH->cMax.x - paramsH->cMin.x) / side0.x, (paramsH->cMax.y - paramsH->cMin.y) / side0.y,
                         (paramsH->cMax.z - paramsH->cMin.z) / side0.z);
    paramsH->binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
    paramsH->binSize0 = binSize3.x;
    paramsH->boxDims = paramsH->cMax - paramsH->cMin;
    paramsH->straightChannelBoundaryMin = paramsH->cMin;  // mR3(0, 0, 0);  // 3D channel
    paramsH->straightChannelBoundaryMax = paramsH->cMax;  // SmR3(3, 2, 3) * paramsH->sizeScale;
    paramsH->deltaPress = mR3(0);
    int3 SIDE = mI3(int((paramsH->cMax.x - paramsH->cMin.x) / paramsH->binSize0 + .1),
                    int((paramsH->cMax.y - paramsH->cMin.y) / paramsH->binSize0 + .1),
                    int((paramsH->cMax.z - paramsH->cMin.z) / paramsH->binSize0 + .1));
    Real mBinSize = paramsH->binSize0;
    paramsH->gridSize = SIDE;
    paramsH->worldOrigin = paramsH->cMin;
    paramsH->cellSize = mR3(mBinSize, mBinSize, mBinSize);
}

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
                                           const thrust::host_vector<Real4>& posRadBCE,
                                           std::shared_ptr<ChBody> body,
                                           ChVector<> collisionShapeRelativePos,
                                           ChQuaternion<> collisionShapeRelativeRot,
                                           bool isSolid,
                                           bool add_to_fluid_helpers) {
    if (fsiData->fsiGeneralData.referenceArray.size() < 1 && !add_to_fluid_helpers) {
        printf(
            "\n\n\n\n Error! fluid need to be initialized before boundary. "
            "Reference array should have two "
            "components \n\n\n\n");
        std::cin.get();
    }

    if (fsiData->fsiGeneralData.referenceArray.size() == 0)
        fsiData->fsiGeneralData.referenceArray.push_back(mI4(0, posRadBCE.size(), -3, -1));

    ::int4 refSize4 = fsiData->fsiGeneralData.referenceArray.back();
    int type = 0;
    int object = 0;
    if (isSolid) {
        object = refSize4.w + 1;
        type = 1;
        //        printf("adding solid, type is %d, ref size=%d\n", type,
        //        fsiData->fsiGeneralData.referenceArray.size());
    }
    if (type < 0) {
        printf(
            "\n\n\n\n Error! reference array type is not correct. It does not "
            "denote boundary or rigid \n\n\n\n");
        std::cin.get();
    }

    if (add_to_fluid_helpers)
        type = -3;

    for (int i = 0; i < posRadBCE.size(); i++) {
        ChVector<> posLoc_collisionShape = ChFsiTypeConvert::Real3ToChVector(mR3(posRadBCE[i]));
        ChVector<> posLoc_body = ChTransform<>::TransformLocalToParent(posLoc_collisionShape, collisionShapeRelativePos,
                                                                       collisionShapeRelativeRot);
        ChVector<> posLoc_COG = TransformBCEToCOG(body.get(), posLoc_body);
        ChVector<> posGlob = ChTransform<>::TransformLocalToParent(posLoc_COG, body->GetPos(), body->GetRot());
        fsiData->sphMarkersH.posRadH.push_back(mR4(ChFsiTypeConvert::ChVectorToReal3(posGlob), posRadBCE[i].w));

        ChVector<> vAbs = body->PointSpeedLocalToParent(posLoc_COG);
        Real3 v3 = ChFsiTypeConvert::ChVectorToReal3(vAbs);
        fsiData->sphMarkersH.velMasH.push_back(v3);
        fsiData->sphMarkersH.rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, (double)type));
    }

    // ------------------------
    // Modify number of objects
    // ------------------------

    int numBce = posRadBCE.size();
    printf("type=%d", type);

    fsiData->numObjects.numAllMarkers += numBce;
    if (type == -3 && fsiData->fsiGeneralData.referenceArray.size() != 1) {
        fsiData->fsiGeneralData.referenceArray.push_back(mI4(refSize4.y, refSize4.y + posRadBCE.size(), -3, -1));
    } else if (type == 0 && !add_to_fluid_helpers) {
        fsiData->numObjects.numBoundaryMarkers += numBce;
        if (refSize4.w == -1) {
            printf("pushing back to refarr\n");
            fsiData->fsiGeneralData.referenceArray.push_back(mI4(refSize4.y, refSize4.y + numBce, 0, 0));
        } else if (refSize4.w == 0) {
            refSize4.y = refSize4.y + numBce;
            fsiData->fsiGeneralData.referenceArray[fsiData->fsiGeneralData.referenceArray.size() - 1] = refSize4;
        }
        //        else {
        //            printf(
        //                "Error! reference array size is greater than 2 while marker type "
        //                "is 0 \n\n");
        //            std::cin.get();
        //        }
    } else if (!add_to_fluid_helpers) {
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
            mI4(refSize4.y, refSize4.y + numBce, 1, object));  // 1: for rigid
        //        if (fsiData->numObjects.numRigidBodies != fsiData->fsiGeneralData.referenceArray.size() - 2) {
        //            printf("Error! num rigid bodies does not match reference array size!\n\n");
        //            std::cin.get();
        //        }

        printf("refSize4.y=%d, refSize4.y + numBce = %d, %d, type=,%d", refSize4.y, refSize4.y + numBce, 1, object);
    }

    //	SetNumObjects(numObjects, fsiGeneralData.referenceArray, numAllMarkers);
}
// =============================================================================

void CreateBceGlobalMarkersFromBceLocalPos_CableANCF(ChFsiDataManager* fsiData,
                                                     SimParams* paramsH,
                                                     const thrust::host_vector<Real4>& posRadBCE,
                                                     std::shared_ptr<chrono::fea::ChElementCableANCF> cable) {
    int type = 2;

    fea::ChElementCableANCF::ShapeVector N;
    double dx = (cable->GetNodeB()->GetX0() - cable->GetNodeA()->GetX0()).Length();

    chrono::ChVector<> Element_Axis = (cable->GetNodeB()->GetX0() - cable->GetNodeA()->GetX0()).GetNormalized();
    printf(" Element_Axis= %f, %f, %f\n", Element_Axis.x(), Element_Axis.y(), Element_Axis.z());

    chrono::ChVector<> Old_axis = ChVector<>(1, 0, 0);
    chrono::ChQuaternion<double> Rotation = (Q_from_Vect_to_Vect(Old_axis, Element_Axis));
    Rotation.Normalize();
    chrono::ChVector<> new_y_axis = Rotation.Rotate(ChVector<>(0, 1, 0));
    chrono::ChVector<> new_z_axis = Rotation.Rotate(ChVector<>(0, 0, 1));
    //  printf(" Rotation Q for this element is = (%f,%f,%f,%f)\n", Rotation.e0, Rotation.e1, Rotation.e2, Rotation.e3);
    //  printf(" new_x_axis element is = (%f,%f,%f)\n", Element_Axis.x, Element_Axis.y, Element_Axis.z);
    //  printf(" new_y_axis element is = (%f,%f,%f)\n", new_y_axis.x, new_y_axis.y, new_y_axis.z);
    //  printf(" new_z_axis element is = (%f,%f,%f)\n", new_z_axis.x, new_z_axis.y, new_z_axis.z);

    chrono::ChVector<> physic_to_natural(1 / dx, 1, 1);

    chrono::ChVector<> nAp = cable->GetNodeA()->GetPos();
    chrono::ChVector<> nBp = cable->GetNodeB()->GetPos();

    chrono::ChVector<> nAv = cable->GetNodeA()->GetPos_dt();
    chrono::ChVector<> nBv = cable->GetNodeB()->GetPos_dt();

    chrono::ChVector<> nAa = cable->GetNodeA()->GetPos_dtdt();
    chrono::ChVector<> nBa = cable->GetNodeB()->GetPos_dtdt();

    int posRadSizeModified = 0;

    printf(" posRadBCE.size()= :%d\n", posRadBCE.size());
    for (int i = 0; i < posRadBCE.size(); i++) {
        //    chrono::ChVector<> posGlob =
        chrono::ChVector<> pos_physical = ChFsiTypeConvert::Real3ToChVector(mR3(posRadBCE[i]));

        chrono::ChVector<> pos_natural = pos_physical * physic_to_natural;

        //    cable->ShapeFunctions(N, pos_natural.x);

        Real2 Nnew = Cables_ShapeFunctions(pos_natural.x());

        chrono::ChVector<> x_dir = (nBp - nAp);
        chrono::ChVector<> Normal;
        //    printf(" N0 =%f, nAp.z= %f, N2=%f, nAp.z=%f\n", N(0), nAp.z, N(2), nBp.z);

        chrono::ChVector<> Correct_Pos =
            Nnew.x * nAp + Nnew.y * nBp + new_y_axis * pos_physical.y() + new_z_axis * pos_physical.z();

        printf(" physic_to_natural is = (%f,%f,%f)\n", physic_to_natural.x(), physic_to_natural.y(),
               physic_to_natural.z());
        printf(" pos_physical is = (%f,%f,%f)\n", pos_physical.x(), pos_physical.y(), pos_physical.z());
        printf(" pos_natural is = (%f,%f,%f)\n ", pos_natural.x(), pos_natural.y(), pos_natural.z());
        printf(" Correct_Pos is = (%f,%f,%f)\n\n\n ", Correct_Pos.x(), Correct_Pos.y(), Correct_Pos.z());

        if ((Correct_Pos.x() < paramsH->cMin.x || Correct_Pos.x() > paramsH->cMax.x) ||
            (Correct_Pos.y() < paramsH->cMin.y || Correct_Pos.y() > paramsH->cMax.y) ||
            (Correct_Pos.z() < paramsH->cMin.z || Correct_Pos.z() > paramsH->cMax.z)) {
            continue;
        }
        //    printf("fsiData->sphMarkersH.posRadH.push_back :%f,%f,%f\n", Correct_Pos.x, Correct_Pos.y, Correct_Pos.z);

        bool addthis = true;
        for (int p = 0; p < fsiData->sphMarkersH.posRadH.size() - 1; p++) {
            if (length(mR3(fsiData->sphMarkersH.posRadH[p]) - ChFsiTypeConvert::ChVectorToReal3(Correct_Pos)) < 1e-5 &&
                fsiData->sphMarkersH.rhoPresMuH[p].w != -1) {
                addthis = false;
                printf("remove this particle %f,%f,%f because of its overlap with a particle at %f,%f,%f\n",
                       fsiData->sphMarkersH.posRadH[p].x, fsiData->sphMarkersH.posRadH[p].y,
                       fsiData->sphMarkersH.posRadH[p].z, Correct_Pos.x(), Correct_Pos.y(), Correct_Pos.z());

                break;
            }
        }

        //// RADU
        //// N is not set anywhere (call to ShapeFunctions above is commented out), yet used below.

        if (addthis) {
            fsiData->sphMarkersH.posRadH.push_back(mR4(ChFsiTypeConvert::ChVectorToReal3(Correct_Pos), posRadBCE[i].w));
            fsiData->fsiGeneralData.FlexSPH_MeshPos_LRF_H.push_back(ChFsiTypeConvert::ChVectorToReal3(pos_physical));
            chrono::ChVector<> Correct_Vel = N(0) * nAv + N(2) * nBv + ChVector<double>(1e-20);
            Real3 v3 = ChFsiTypeConvert::ChVectorToReal3(Correct_Vel);
            fsiData->sphMarkersH.velMasH.push_back(v3);
            fsiData->sphMarkersH.rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, type));
            posRadSizeModified++;
        }
    }

    // ------------------------
    // Modify number of objects
    // ------------------------
    int numObjects = fsiData->fsiGeneralData.referenceArray.size();
    int numBce = posRadSizeModified;
    fsiData->numObjects.numAllMarkers += numBce;

    int numRigid = fsiData->numObjects.numRigidBodies;
    fsiData->numObjects.numFlex_SphMarkers += numBce;
    fsiData->numObjects.numFlexBodies1D += 1;
    fsiData->numObjects.startFlexMarkers = fsiData->fsiGeneralData.referenceArray[numRigid + 1].y;
    int start_flex = fsiData->numObjects.startFlexMarkers;

    int4 last = fsiData->fsiGeneralData.referenceArray[fsiData->fsiGeneralData.referenceArray.size() - 1];
    fsiData->fsiGeneralData.referenceArray.push_back(
        mI4(last.y, last.y + numBce, type, fsiData->numObjects.numFlexBodies1D));  // 2: for Shell

    fsiData->fsiGeneralData.referenceArray_FEA.push_back(
        mI4(last.y, last.y + numBce, type, fsiData->numObjects.numFlexBodies1D));  // 2: for Shell

    printf(" push_back Index %d. ", fsiData->fsiGeneralData.referenceArray.size() - 1);
    int4 test = fsiData->fsiGeneralData.referenceArray[fsiData->fsiGeneralData.referenceArray.size() - 1];
    printf(" x=%d, y=%d, z=%d, w=%d\n", test.x, test.y, test.z, test.w);

    if (fsiData->numObjects.numFlexBodies1D !=
        fsiData->fsiGeneralData.referenceArray.size() - 2 - fsiData->numObjects.numRigidBodies) {
        printf("Error! num rigid Flexible does not match reference array size!\n\n");
        std::cin.get();
    }
    numObjects = fsiData->fsiGeneralData.referenceArray.size();
    printf("numObjects : %d\n ", numObjects);
    printf("numObjects.startFlexMarkers  : %d\n ", fsiData->numObjects.startFlexMarkers);
}
// =============================================================================

void CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(ChFsiDataManager* fsiData,
                                                     SimParams* paramsH,
                                                     const thrust::host_vector<Real4>& posRadBCE,
                                                     std::shared_ptr<chrono::fea::ChElementShellANCF> shell,
                                                     double kernel_h = 0) {
    int type = 3;

    fea::ChElementShellANCF::ShapeVector N;
    int posRadSizeModified = 0;

    double my_h = (kernel_h == 0) ? paramsH->HSML : kernel_h;

    Real dx = shell->GetLengthX();
    Real dy = shell->GetLengthY();
    chrono::ChVector<> physic_to_natural(2 / dx, 2 / dy, 1);
    chrono::ChVector<> nAp = shell->GetNodeA()->GetPos();
    chrono::ChVector<> nBp = shell->GetNodeB()->GetPos();
    chrono::ChVector<> nCp = shell->GetNodeC()->GetPos();
    chrono::ChVector<> nDp = shell->GetNodeD()->GetPos();

    chrono::ChVector<> nAv = shell->GetNodeA()->GetPos_dt();
    chrono::ChVector<> nBv = shell->GetNodeB()->GetPos_dt();
    chrono::ChVector<> nCv = shell->GetNodeC()->GetPos_dt();
    chrono::ChVector<> nDv = shell->GetNodeD()->GetPos_dt();

    chrono::ChVector<> nAa = shell->GetNodeA()->GetPos_dtdt();
    chrono::ChVector<> nBa = shell->GetNodeB()->GetPos_dtdt();
    chrono::ChVector<> nCa = shell->GetNodeC()->GetPos_dtdt();
    chrono::ChVector<> nDa = shell->GetNodeD()->GetPos_dtdt();
    printf(" posRadBCE.size()= :%d\n", posRadBCE.size());
    for (int i = 0; i < posRadBCE.size(); i++) {
        //    chrono::ChVector<> posGlob =
        chrono::ChVector<> pos_physical = ChFsiTypeConvert::Real3ToChVector(mR3(posRadBCE[i]));
        chrono::ChVector<> pos_natural = pos_physical * physic_to_natural;
        //        printf("pos_natural :%f,%f,%f\n", pos_natural.x(), pos_natural.y(), pos_natural.z());

        shell->ShapeFunctions(N, pos_natural.x(), pos_natural.y(), pos_natural.z());
        chrono::ChVector<> x_dir = (nBp - nAp + nCp - nDp);
        chrono::ChVector<> y_dir = (nCp - nBp + nDp - nAp);
        chrono::ChVector<> Normal;
        Normal.Cross(x_dir, y_dir);
        Normal.Normalize();
        //    printf("GetNormalized :%f,%f,%f\n", Normal.x, Normal.y, Normal.z);

        chrono::ChVector<> Correct_Pos = N(0) * nAp + N(2) * nBp + N(4) * nCp + N(6) * nDp +
                                         Normal * pos_physical.z() * my_h * paramsH->MULT_INITSPACE_Shells;

        if ((Correct_Pos.x() < paramsH->cMin.x || Correct_Pos.x() > paramsH->cMax.x) ||
            (Correct_Pos.y() < paramsH->cMin.y || Correct_Pos.y() > paramsH->cMax.y) ||
            (Correct_Pos.z() < paramsH->cMin.z || Correct_Pos.z() > paramsH->cMax.z)) {
            continue;
        }
        //    printf("fsiData->sphMarkersH.posRadH.push_back :%f,%f,%f\n", Correct_Pos.x, Correct_Pos.y, Correct_Pos.z);

        // Note that the fluid markers are removed differently
        bool addthis = true;
        for (int p = 0; p < fsiData->sphMarkersH.posRadH.size() - 1; p++) {
            if (length(mR3(fsiData->sphMarkersH.posRadH[p]) - ChFsiTypeConvert::ChVectorToReal3(Correct_Pos)) < 1e-8 &&
                fsiData->sphMarkersH.rhoPresMuH[p].w != -1) {
                addthis = false;
                //        printf("remove this particle %f,%f,%f because of its overlap with a particle at %f,%f,%f\n",
                //               fsiData->sphMarkersH.posRadH[p].x, fsiData->sphMarkersH.posRadH[p].y,
                //               fsiData->sphMarkersH.posRadH[p].z,
                //               Correct_Pos.x, Correct_Pos.y, Correct_Pos.z);
                break;
            }
        }

        if (addthis) {
            fsiData->sphMarkersH.posRadH.push_back(mR4(ChFsiTypeConvert::ChVectorToReal3(Correct_Pos), posRadBCE[i].w));
            fsiData->fsiGeneralData.FlexSPH_MeshPos_LRF_H.push_back(ChFsiTypeConvert::ChVectorToReal3(pos_natural));

            chrono::ChVector<> Correct_Vel = N(0) * nAv + N(2) * nBv + N(4) * nCv + N(6) * nDv;
            Real3 v3 = ChFsiTypeConvert::ChVectorToReal3(Correct_Vel);
            fsiData->sphMarkersH.velMasH.push_back(v3);
            fsiData->sphMarkersH.rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, type));
            posRadSizeModified++;
        }
    }
    fsiData->sphMarkersH.rhoPresMuH.size();
    //  printf(" CreateBceGlobalMarkersFromBceLocalPos_ShellANCF : fsiData->sphMarkersH.rhoPresMuH.size() %d. ",
    //         fsiData->sphMarkersH.rhoPresMuH.size());

    // ------------------------
    // Modify number of objects
    // ------------------------
    int numObjects = fsiData->fsiGeneralData.referenceArray.size();
    int numBce = posRadSizeModified;
    fsiData->numObjects.numAllMarkers += numBce;

    int numRigid = fsiData->numObjects.numRigidBodies;
    fsiData->numObjects.numFlex_SphMarkers += numBce;
    fsiData->numObjects.numFlexBodies2D += 1;
    fsiData->numObjects.startFlexMarkers = fsiData->fsiGeneralData.referenceArray[numRigid + 1].y;
    int start_flex = fsiData->numObjects.startFlexMarkers;

    int4 last = fsiData->fsiGeneralData.referenceArray[fsiData->fsiGeneralData.referenceArray.size() - 1];
    fsiData->fsiGeneralData.referenceArray.push_back(
        mI4(last.y, last.y + numBce, type, fsiData->numObjects.numFlexBodies2D));  // 2: for Shell

    fsiData->fsiGeneralData.referenceArray_FEA.push_back(
        mI4(last.y, last.y + numBce, type, fsiData->numObjects.numFlexBodies2D));  // 2: for Shell

    printf(" referenceArray size %d. ", fsiData->fsiGeneralData.referenceArray.size());
    int4 test = fsiData->fsiGeneralData.referenceArray[fsiData->fsiGeneralData.referenceArray.size() - 1];
    printf(" x=%d, y=%d, z=%d, w=%d\n", test.x, test.y, test.z, test.w);

    if (fsiData->numObjects.numFlexBodies2D != fsiData->fsiGeneralData.referenceArray.size() - 2 -
                                                   fsiData->numObjects.numRigidBodies -
                                                   fsiData->numObjects.numFlexBodies1D) {
        printf("Error! num rigid Flexible does not match reference array size!\n\n");
        std::cin.get();
    }
    numObjects = fsiData->fsiGeneralData.referenceArray.size();
    printf("numObjects : %d\n ", numObjects);
}

// =============================================================================
void CreateBceGlobalMarkersFromBceLocalPosBoundary(ChFsiDataManager* fsiData,
                                                   SimParams* paramsH,
                                                   const thrust::host_vector<Real4>& posRadBCE,
                                                   std::shared_ptr<ChBody> body,
                                                   ChVector<> collisionShapeRelativePos,
                                                   ChQuaternion<> collisionShapeRelativeRot,
                                                   bool isSolid) {
    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body, collisionShapeRelativePos,
                                          collisionShapeRelativeRot, isSolid);
}
// =============================================================================
void AddSphereBce(ChFsiDataManager* fsiData,
                  SimParams* paramsH,
                  std::shared_ptr<ChBody> body,
                  ChVector<> relPos,
                  ChQuaternion<> relRot,
                  Real radius) {
    thrust::host_vector<Real4> posRadBCE;
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
                    Real height,
                    Real kernel_h,
                    bool cartesian) {
    thrust::host_vector<Real4> posRadBCE;
    CreateBCE_On_Cylinder(posRadBCE, radius, height, paramsH, kernel_h, cartesian);
    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body, relPos, relRot);
    posRadBCE.clear();
}
// =============================================================================

void AddCylinderSurfaceBce(ChFsiDataManager* fsiData,
                           SimParams* paramsH,
                           std::shared_ptr<ChBody> body,
                           ChVector<> relPos,
                           ChQuaternion<> relRot,
                           Real radius,
                           Real height,
                           Real kernel_h) {
    thrust::host_vector<Real4> posRadBCE;
    thrust::host_vector<Real3> normals;
    CreateBCE_On_surface_of_Cylinder(posRadBCE, normals, radius, height, kernel_h);
    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body, relPos, relRot, false, true);
    posRadBCE.clear();
    normals.clear();
}
// =============================================================================

void AddSphereSurfaceBce(ChFsiDataManager* fsiData,
                         SimParams* paramsH,
                         std::shared_ptr<ChBody> body,
                         ChVector<> relPos,
                         ChQuaternion<> relRot,
                         Real radius,
                         Real kernel_h) {
    thrust::host_vector<Real4> posRadBCE;
    thrust::host_vector<Real3> normals;
    CreateBCE_On_surface_of_Sphere(posRadBCE, radius, kernel_h);
    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body, relPos, relRot, false, true);
    posRadBCE.clear();
    normals.clear();
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
               const ChVector<>& size,
               int plane,
               bool isSolid) {
    thrust::host_vector<Real4> posRadBCE;

    CreateBCE_On_Box(posRadBCE, ChFsiTypeConvert::ChVectorToReal3(size), plane, paramsH);
    //	if (fsiData->sphMarkersH.posRadH.size() !=
    // fsiData->numObjects.numAllMarkers) {
    //		printf("Error! numMarkers, %d, does not match posRadH.size(),
    //%d\n",
    //				fsiData->numObjects.numAllMarkers,
    // fsiData->sphMarkersH.posRadH.size());
    //		std::cin.get();
    //	}
    //    printf("in AddBoxBce Ref size=%d,posRadBCE.size()=%d\n", fsiData->fsiGeneralData.referenceArray.size(),
    //           posRadBCE.size());

    CreateBceGlobalMarkersFromBceLocalPosBoundary(fsiData, paramsH, posRadBCE, body, relPos, relRot, isSolid);
    posRadBCE.clear();
}

// =============================================================================
void AddBCE_FromFile(ChFsiDataManager* fsiData,
                     SimParams* paramsH,
                     std::shared_ptr<ChBody> body,
                     std::string dataPath,
                     ChVector<> collisionShapeRelativePos,
                     ChQuaternion<> collisionShapeRelativeRot,
                     double scale) {
    //----------------------------
    //  chassis
    //----------------------------
    thrust::host_vector<Real4> posRadBCE;

    LoadBCE_fromFile(posRadBCE, dataPath, scale);

    //	if (fsiData->sphMarkersH.posRadH.size() !=
    // fsiData->numObjects.numAllMarkers) {
    //		printf("Error! numMarkers, %d, does not match posRadH.size(),
    //%d\n",
    //				fsiData->numObjects.numAllMarkers,
    // fsiData->sphMarkersH.posRadH.size());
    //		std::cin.get();
    //	}

    CreateBceGlobalMarkersFromBceLocalPos(fsiData, paramsH, posRadBCE, body, collisionShapeRelativePos,
                                          collisionShapeRelativeRot);
    posRadBCE.clear();
}

// =============================================================================
void CreateSphereFSI(ChFsiDataManager* fsiData,
                     ChSystem& mphysicalSystem,
                     std::vector<std::shared_ptr<ChBody>>* fsiBodeisPtr,
                     SimParams* paramsH,
                     std::shared_ptr<ChMaterialSurfaceNSC> mat_prop,
                     Real density,
                     ChVector<> pos,
                     Real radius) {
    //	ChVector<> pos = ChVector<>(-9.5, .20, 3);
    //	Real radius = 0.3;

    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetMaterialSurface(mat_prop);
    body->SetPos(pos);
    double volume = chrono::utils::CalcSphereVolume(radius);
    ChVector<> gyration = chrono::utils::CalcSphereGyration(radius).diagonal();
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
                       std::shared_ptr<ChMaterialSurfaceSMC> mat_prop,
                       Real density,
                       ChVector<> pos,
                       ChQuaternion<> rot,
                       Real radius,
                       Real length) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetMaterialSurface(mat_prop);
    body->SetPos(pos);
    body->SetRot(rot);
    double volume = chrono::utils::CalcCylinderVolume(radius, 0.5 * length);
    ChVector<> gyration = chrono::utils::CalcCylinderGyration(radius, 0.5 * length).diagonal();
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    chrono::utils::AddCylinderGeometry(body.get(), radius, 0.5 * length);
    body->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(body);

    fsiBodeisPtr->push_back(body);
    AddCylinderBce(fsiData, paramsH, body, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), radius, length,
                   paramsH->HSML * paramsH->MULT_INITSPACE);
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
    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetMaterialSurface(mat_prop);
    body->SetPos(pos);
    body->SetRot(rot);
    double volume = chrono::utils::CalcBoxVolume(hsize);
    ChVector<> gyration = chrono::utils::CalcBoxGyration(hsize).diagonal();
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
// =============================================================================

void AddBCE_ShellANCF(ChFsiDataManager* fsiData,
                      SimParams* paramsH,
                      std::vector<std::shared_ptr<chrono::fea::ChElementShellANCF>>* fsiShellsPtr,
                      std::shared_ptr<chrono::fea::ChMesh> my_mesh,
                      bool multiLayer,
                      bool removeMiddleLayer,
                      int SIDE) {
    thrust::host_vector<Real4> posRadBCE;
    int numShells = my_mesh->GetNelements();
    printf("number of shells to be meshed is %d\n", numShells);
    for (int i = 0; i < numShells; i++) {
        auto thisShell = std::dynamic_pointer_cast<fea::ChElementShellANCF>(my_mesh->GetElement(i));
        fsiShellsPtr->push_back(thisShell);
        CreateBCE_On_shell(posRadBCE, paramsH, thisShell, multiLayer, removeMiddleLayer, SIDE);
        CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(fsiData, paramsH, posRadBCE, thisShell);

        posRadBCE.clear();
    }
}

// =============================================================================

void AddBCE_ShellFromMesh(ChFsiDataManager* fsiData,
                          SimParams* paramsH,
                          std::vector<std::shared_ptr<chrono::fea::ChElementShellANCF>>* fsiShellsPtr,
                          std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzD>>* fsiNodesPtr,
                          std::shared_ptr<chrono::fea::ChMesh> my_mesh,
                          std::vector<std::vector<int>> elementsNodes,
                          std::vector<std::vector<int>> NodeNeighborElement,
                          bool multiLayer,
                          bool removeMiddleLayer,
                          int SIDE) {
    thrust::host_vector<Real4> posRadBCE;
    int numShells = my_mesh->GetNelements();
    std::vector<int> remove;

    for (int i = 0; i < NodeNeighborElement.size(); i++) {
        auto thisNode = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(my_mesh->GetNode(i));
        fsiNodesPtr->push_back(thisNode);
    }

    for (int i = 0; i < numShells; i++) {
        remove.resize(4);
        std::fill(remove.begin(), remove.begin() + 4, 0);
        auto thisShell = std::dynamic_pointer_cast<fea::ChElementShellANCF>(my_mesh->GetElement(i));
        fsiShellsPtr->push_back(thisShell);
        // Look into the nodes of this element
        int myNumNodes = (elementsNodes[i].size() > 4) ? 4 : elementsNodes[i].size();

        for (int j = 0; j < myNumNodes; j++) {
            int thisNode = elementsNodes[i][j] - 1;
            //      printf("Considering elementsNodes[%d][%d]=%d\n", i, j, thisNode);

            // Look into the elements attached to thisNode
            for (int k = 0; k < NodeNeighborElement[thisNode].size(); k++) {
                // If this neighbor element has more than one common node with the previous node this means that we must
                // not
                // add BCEs to this edge anymore. Because that edge has already been given BCE markers
                // The kth element of this node:
                int neighborElement = NodeNeighborElement[thisNode][k];
                if (neighborElement >= i)
                    continue;
                //        printf("neighborElement %d\n", neighborElement);

                int JNumNodes = (elementsNodes[neighborElement].size() > 4) ? 4 : elementsNodes[neighborElement].size();

                for (int inode = 0; inode < myNumNodes; inode++) {
                    for (int jnode = 0; jnode < JNumNodes; jnode++) {
                        if (elementsNodes[i][inode] - 1 == elementsNodes[neighborElement][jnode] - 1 &&
                            thisNode != elementsNodes[i][inode] - 1 && i > neighborElement) {
                            //              printf("node %d is common between %d and %d\n", elementsNodes[i][inode] - 1,
                            //              i,
                            //              neighborElement);
                            remove[inode] = 1;
                        }
                    }
                }
            }
        }

        //    printf("remove: %d, %d, %d, %d\n", remove[0], remove[1], remove[2], remove[3]);

        CreateBCE_On_ChElementShellANCF(posRadBCE, paramsH, thisShell, remove, multiLayer, removeMiddleLayer, SIDE);
        CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(fsiData, paramsH, posRadBCE, thisShell);
        posRadBCE.clear();
    }
}
// =============================================================================
void AddBCE_FromMesh(ChFsiDataManager* fsiData,
                     SimParams* paramsH,
                     std::shared_ptr<chrono::fea::ChMesh> my_mesh,
                     std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzD>>* fsiNodesPtr,
                     std::vector<std::shared_ptr<chrono::fea::ChElementCableANCF>>* fsiCablesPtr,
                     std::vector<std::shared_ptr<chrono::fea::ChElementShellANCF>>* fsiShellsPtr,
                     std::vector<std::vector<int>> NodeNeighborElement,
                     std::vector<std::vector<int>> _1D_elementsNodes,
                     std::vector<std::vector<int>> _2D_elementsNodes,
                     bool add1DElem,
                     bool add2DElem,
                     bool multiLayer,
                     bool removeMiddleLayer,
                     int SIDE,
                     int SIDE2D,
                     double kernel_h) {
    thrust::host_vector<Real4> posRadBCE;
    int numElems = my_mesh->GetNelements();
    std::vector<int> remove2D;
    std::vector<int> remove1D;

    for (int i = 0; i < my_mesh->GetNnodes(); i++) {
        auto thisNode = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(my_mesh->GetNode(i));
        fsiNodesPtr->push_back(thisNode);
    }

    for (int i = 0; i < numElems; i++) {
        ///////////////////////////
        // Check for Cable Elements
        if (_1D_elementsNodes.size() > 0) {
            if (auto thisCable = std::dynamic_pointer_cast<fea::ChElementCableANCF>(my_mesh->GetElement(i))) {
                remove1D.resize(2);
                std::fill(remove1D.begin(), remove1D.end(), 0);
                fsiCablesPtr->push_back(thisCable);

                int myNumNodes = (_1D_elementsNodes[i].size() > 2) ? 2 : _1D_elementsNodes[i].size();

                for (int j = 0; j < myNumNodes; j++) {
                    int thisNode = _1D_elementsNodes[i][j];

                    // Look into the elements attached to thisNode
                    for (int k = 0; k < NodeNeighborElement[thisNode].size(); k++) {
                        int neighborElement = NodeNeighborElement[thisNode][k];
                        //                        printf("element=%d, neighborElement=%d\n", i, neighborElement);

                        if (neighborElement >= i)
                            continue;
                        remove1D[j] = 1;
                    }
                }
                //                printf("1D element %d, removed firstNode=%d, SecondNode=%d\n", i, remove1D[1],
                //                remove1D[2]);

                if (add1DElem) {
                    CreateBCE_On_ChElementCableANCF(posRadBCE, paramsH, thisCable, remove1D, multiLayer,
                                                    removeMiddleLayer, SIDE);
                    CreateBceGlobalMarkersFromBceLocalPos_CableANCF(fsiData, paramsH, posRadBCE, thisCable);
                }
                posRadBCE.clear();
            }
        }
        int Curr_size = _1D_elementsNodes.size();

        ///////////////////////////
        // Check for Shell Elements
        if (_2D_elementsNodes.size() > 0) {
            if (auto thisShell = std::dynamic_pointer_cast<fea::ChElementShellANCF>(my_mesh->GetElement(i))) {
                remove2D.resize(4);
                std::fill(remove2D.begin(), remove2D.begin() + 4, 0);

                fsiShellsPtr->push_back(thisShell);
                // Look into the nodes of this element
                int myNumNodes =
                    (_2D_elementsNodes[i - Curr_size].size() > 4) ? 4 : _2D_elementsNodes[i - Curr_size].size();

                for (int j = 0; j < myNumNodes; j++) {
                    int thisNode = _2D_elementsNodes[i - Curr_size][j];
                    //          printf("Considering elementsNodes[%d][%d]=%d\n", i - Curr_size, j, thisNode);

                    // Look into the elements attached to thisNode
                    for (int k = 0; k < NodeNeighborElement[thisNode].size(); k++) {
                        // If this neighbor element has more than one common node with the previous node this means that
                        // we must not
                        // add BCEs to this edge anymore. Because that edge has already been given BCE markers
                        // The kth element of this node:
                        int neighborElement = NodeNeighborElement[thisNode][k] - Curr_size;
                        //            printf("Considering neighbor NodeNeighborElement[%d][%d]=%d\n", thisNode, k,
                        //            neighborElement);

                        if (neighborElement >= i - Curr_size)
                            continue;

                        int JNumNodes = (_2D_elementsNodes[neighborElement].size() > 4)
                                            ? 4
                                            : _2D_elementsNodes[neighborElement].size();

                        for (int inode = 0; inode < myNumNodes; inode++) {
                            for (int jnode = 0; jnode < JNumNodes; jnode++) {
                                if (_2D_elementsNodes[i - Curr_size][inode] ==
                                        _2D_elementsNodes[neighborElement][jnode] &&
                                    thisNode != _2D_elementsNodes[i - Curr_size][inode] && i > neighborElement) {
                                    remove2D[inode] = 1;
                                    //                  printf("removing _2D_elementsNodes[%d][%d]=%d\n", i - Curr_size,
                                    //                  inode,
                                    //                         _2D_elementsNodes[i - Curr_size][inode]);
                                }
                            }
                        }
                    }
                }
                if (add2DElem) {
                    CreateBCE_On_ChElementShellANCF(posRadBCE, paramsH, thisShell, remove2D, multiLayer,
                                                    removeMiddleLayer, SIDE2D, kernel_h);
                    CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(fsiData, paramsH, posRadBCE, thisShell, kernel_h);
                }
                posRadBCE.clear();
            }
        }
        ///////////////////////////
        // Check for break Elements
    }
}
}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono
