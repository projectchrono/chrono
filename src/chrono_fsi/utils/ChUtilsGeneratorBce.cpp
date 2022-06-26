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
// Utility class for generating fluid markers.//
// =============================================================================

#include <fstream>
#include <sstream>
#include <cmath>

#include "chrono/core/ChMathematics.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/utils/ChUtilsGeneratorBce.h"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"

namespace chrono {
namespace fsi {
namespace utils {

// =============================================================================

// TransformToCOG
// This utility function converts a given position and orientation, specified
// with respect to a body's reference frame, into a frame defined with respect
// to the body's centroidal frame.  Note that by default, a body's reference
// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
void TransformBceFrameToCOG(std::shared_ptr<ChBody> body,
                            const ChVector<>& pos,
                            const ChMatrix33<>& rot,
                            ChFrame<>& frame) {
    frame = ChFrame<>(pos, rot);
    if (std::shared_ptr<ChBodyAuxRef> body_ar = std::dynamic_pointer_cast<ChBodyAuxRef>(body)) {
        frame = frame >> body_ar->GetFrame_REF_to_COG();
    }
}

ChVector<> TransformBCEToCOG(std::shared_ptr<ChBody> body, const ChVector<>& pos) {
    ChFrame<> frame;
    TransformBceFrameToCOG(body, pos, QUNIT, frame);
    return frame.GetPos();
}

ChVector<> TransformBCEToCOG(std::shared_ptr<ChBody> body, const Real3& pos3) {
    ChVector<> pos = ChUtilsTypeConvert::Real3ToChVector(pos3);
    return TransformBCEToCOG(body, pos);
}



// =============================================================================
void CreateBCE_On_Sphere(thrust::host_vector<Real4>& posRadBCE, Real rad, std::shared_ptr<SimParams> paramsH) {
    Real spacing = paramsH->MULT_INITSPACE * paramsH->HSML;

    for (Real r = 0.5 * spacing; r < rad; r += spacing) {
        int numphi = (int)std::floor(3.1415 * r / spacing);
        for (size_t p = 0; p < numphi; p++) {
            Real phi = p * 3.1415 / numphi;
            int numTheta = (int)std::floor(2 * 3.1415 * r * sin(phi) / spacing);
            for (Real t = 0.0; t < numTheta; t++) {
                Real teta = t * 2 * 3.1415 / numTheta;
                Real3 BCE_Pos_local = mR3(r * sin(phi) * cos(teta), r * sin(phi) * sin(teta), r * cos(phi));
                posRadBCE.push_back(mR4(BCE_Pos_local, spacing));
            }
        }
        posRadBCE.push_back(mR4(0, 0, r, spacing));
        posRadBCE.push_back(mR4(0, 0, -r, spacing));
    }
}
// =============================================================================
void CreateBCE_On_surface_of_Sphere(thrust::host_vector<Real4>& posRadBCE, Real rad, Real kernel_h) {
    Real spacing = kernel_h;
    Real r = rad;
    int numphi = (int)std::floor(3.1415 * r / spacing);

    for (size_t p = 0; p < numphi; p++) {
        Real phi = p * 3.1415 / numphi;
        int numTheta = (int)std::floor(2 * 3.1415 * r * sin(phi) / spacing);
        for (Real t = 0.0; t < numTheta; t++) {
            Real teta = t * 2 * 3.1415 / numTheta;
            Real3 BCE_Pos_local = mR3(r * sin(phi) * cos(teta), r * sin(phi) * sin(teta), r * cos(phi));
            posRadBCE.push_back(mR4(BCE_Pos_local, spacing));
        }
    }
}
// =============================================================================
void CreateBCE_On_Cylinder(thrust::host_vector<Real4>& posRadBCE,
                           Real cyl_rad,
                           Real cyl_h,
                           std::shared_ptr<SimParams> paramsH,
                           Real kernel_h,
                           bool cartesian) {
    Real spacing = kernel_h * paramsH->MULT_INITSPACE;
    int num_layers = (int)std::floor(1.00001 * cyl_h / spacing) + 1;

    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * cyl_h + spacing * si;
        if (cartesian)
            for (Real x = -cyl_rad; x <= cyl_rad; x += spacing) {
                for (Real y = -cyl_rad; y <= cyl_rad; y += spacing) {
                    if (x * x + y * y <= cyl_rad * cyl_rad)
                        posRadBCE.push_back(mR4(x, s, y, kernel_h));
                }
            }
        else {
            Real3 centerPointLF = mR3(0, s, 0);
            posRadBCE.push_back(mR4(0, s, 0, kernel_h));
            int numr = (int)std::floor(1.00001 * cyl_rad / spacing);
            for (size_t ir = 0; ir < numr; ir++) {
                Real r = 0.5 * spacing + ir * spacing;
                int numTheta = (int)std::floor(2 * 3.1415 * r / spacing);
                for (size_t t = 0; t < numTheta; t++) {
                    Real teta = t * 2 * 3.1415 / numTheta;
                    Real3 BCE_Pos_local = mR3(r * cos(teta), 0, r * sin(teta)) + centerPointLF;
                    posRadBCE.push_back(mR4(BCE_Pos_local, kernel_h));
                }
            }
        }
    }
}
// =============================================================================
void CreateBCE_On_Cone(thrust::host_vector<Real4>& posRadBCE,
                       Real cone_rad,
                       Real cone_h,
                       std::shared_ptr<SimParams> paramsH,
                       Real kernel_h,
                       bool cartesian) {
    Real spacing = kernel_h * paramsH->MULT_INITSPACE;
    int num_layers = (int)std::floor(cone_h / spacing);

    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * cone_h + spacing / 2 + (cone_h / num_layers) * si;
        Real cone_h0 = spacing / 2 + (cone_h / num_layers) * si;
        Real cone_r0 = cone_h0 / cone_h * cone_rad;
        // if (cartesian)
        //     for (Real x = -cyl_rad; x <= cyl_rad; x += spacing) {
        //         for (Real y = -cyl_rad; y <= cyl_rad; y += spacing) {
        //             if (x * x + y * y <= cyl_rad * cyl_rad)
        //                 posRadBCE.push_back(mR4(x, s, y, kernel_h));
        //         }
        //     }
        // else {
        Real3 centerPointLF = mR3(0, 0, s);
        posRadBCE.push_back(mR4(0, 0, s, kernel_h));
        Real numr = std::floor(cone_r0 / spacing) + 2;
        // Real spacing_r = cone_r0 / numr;
        if (si > 0) {
            for (size_t ir = 1; ir < numr; ir++) {
                Real r = cone_r0 - 0.5 * spacing - (ir - 1) * spacing;
                if (r > 0.1 * spacing) {
                    int numTheta = (int)std::floor(2 * 3.1415 * r / spacing) + 2;
                    Real det_Theta = 2 * 3.1415 / numTheta;
                    for (size_t t = 0; t < numTheta; t++) {
                        Real teta = t * det_Theta + 0.0 * (ir - 1) * det_Theta;
                        Real3 BCE_Pos_local = mR3(r * cos(teta), r * sin(teta), 0.0) + centerPointLF;
                        posRadBCE.push_back(mR4(BCE_Pos_local, kernel_h));
                    }
                }
            }
        }
        // if create a cylinder at the end of the cone, set as 1==1, otherwise 1==0
        if (1 == 0) {
            Real cone_r1 = 0.5 * cone_rad;
            Real3 centerPointLF_1 = mR3(0, 0, s + cone_h);
            Real numr1 = std::floor(cone_r1 / spacing) + 2;
            Real spacing_r = cone_r1 / numr1;
            // if (si > 0) {
            for (size_t ir = 1; ir < numr1; ir++) {
                Real r = ir * spacing_r;
                int numTheta = (int)std::floor(2 * 3.1415 * r / spacing) + 2;
                Real det_Theta = 2 * 3.1415 / numTheta;
                for (size_t t = 0; t < numTheta; t++) {
                    Real teta = t * det_Theta + 0.0 * (ir - 1) * det_Theta;
                    Real3 BCE_Pos_local = mR3(r * cos(teta), r * sin(teta), 0.0) + centerPointLF_1;
                    posRadBCE.push_back(mR4(BCE_Pos_local, kernel_h));
                }
            }
            // }
        }
    }
}
// =============================================================================
void CreateBCE_On_surface_of_Cylinder(thrust::host_vector<Real4>& posRadBCE,
                                      thrust::host_vector<Real3>& normals,
                                      Real cyl_rad,
                                      Real cyl_h,
                                      Real spacing) {
    int num_layers = (int)std::floor(cyl_h / spacing);

    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * cyl_h + (cyl_h / num_layers) * si;
        Real3 centerPointLF = mR3(0, s, 0);
        Real numr = std::floor(cyl_rad / spacing);
        for (size_t ir = 1; ir < numr; ir++) {
            Real r = spacing + ir * cyl_rad / numr;
            int numTheta = (int)std::floor(2 * 3.1415 * r / spacing);
            for (size_t t = 0; t < numTheta; t++) {
                Real teta = t * 2 * 3.1415 / numTheta;
                Real3 BCE_Pos_local = mR3(r * cos(teta), 0, r * sin(teta)) + centerPointLF;
                if ( ir == numr - 1) {
                    posRadBCE.push_back(mR4(BCE_Pos_local, spacing));
                }
            }
        }
    }
}

// =============================================================================
// note, the function in the current implementation creates boundary BCE (zero
// velocity)
// x=1, y=2, z =3; therefore 12 means creating markers on the top surface
// parallel to xy plane,
// similarly -12 means bottom face paralel to xy. similarly 13, -13, 23, -23
void CreateBCE_On_Box(thrust::host_vector<Real4>& posRadBCE,
                      const Real3& hsize,
                      int face,
                      std::shared_ptr<SimParams> paramsH) {
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    int nFX = (int)round(hsize.x / initSpace0);
    int nFY = (int)round(hsize.y / initSpace0);
    int nFZ = (int)round(hsize.z / initSpace0);

    Real initSpaceX = hsize.x / nFX;
    Real initSpaceY = hsize.y / nFY;
    Real initSpaceZ = hsize.z / nFZ;

    int2 iBound = mI2(-nFX, nFX);
    int2 jBound = mI2(-nFY, nFY);
    int2 kBound = mI2(-nFZ, nFZ);
    
    switch (face) {
        case 12:
            kBound = mI2(nFZ - paramsH->NUM_BOUNDARY_LAYERS + 1, nFZ);
            break;
        case -12:
            kBound = mI2(-nFZ, -nFZ + paramsH->NUM_BOUNDARY_LAYERS - 1);
            break;
        case 13:
            jBound = mI2(nFY - paramsH->NUM_BOUNDARY_LAYERS + 1, nFY);
            break;
        case -13:
            jBound = mI2(-nFY, -nFY + paramsH->NUM_BOUNDARY_LAYERS - 1);
            break;
        case 23:
            iBound = mI2(nFX - paramsH->NUM_BOUNDARY_LAYERS + 1, nFX);
            break;
        case -23:
            iBound = mI2(-nFX, -nFX + paramsH->NUM_BOUNDARY_LAYERS - 1);
            break;
        case 123:
            break;
        default:
            printf("wrong argument box bce initialization\n");
            break;
    }

    for (int i = iBound.x; i <= iBound.y; i++) {
        for (int j = jBound.x; j <= jBound.y; j++) {
            for (int k = kBound.x; k <= kBound.y; k++) {
                Real3 relMarkerPos = mR3(i * initSpaceX, j * initSpaceY, k * initSpaceZ);

                if ((relMarkerPos.x < paramsH->cMin.x || relMarkerPos.x > paramsH->cMax.x) ||
                    (relMarkerPos.y < paramsH->cMin.y || relMarkerPos.y > paramsH->cMax.y) ||
                    (relMarkerPos.z < paramsH->cMin.z || relMarkerPos.z > paramsH->cMax.z)) {
                    continue;
                }
                posRadBCE.push_back(mR4(relMarkerPos, paramsH->HSML));
            }
        }
    }
}
// =============================================================================
void LoadBCE_fromFile(thrust::host_vector<Real4>& posRadBCE, std::string fileName, double scale, double hsml) {
    std::string ddSt;
    char buff[256];
    int numBce = 0;
    const int cols = 3;
    std::cout << "Reading BCE data from: " << fileName << " ...\n";
    std::ifstream inMarker;
    inMarker.open(fileName);
    if (!inMarker) {
        std::cerr << "   Error! Unable to open file: " << fileName << std::endl;
    }
    getline(inMarker, ddSt);
    Real q[cols];
    while (getline(inMarker, ddSt)) {
        std::stringstream linestream(ddSt);
        for (size_t i = 0; i < cols; i++) {
            linestream.getline(buff, 500, ',');
            q[i] = atof(buff);
        }
        posRadBCE.push_back(mR4(q[0] * scale, q[1] * scale, q[2] * scale, hsml));
        numBce++;
    }

    std::cout << "Loaded " << numBce << " BCE data from: " << fileName << std::endl;
}

void CreateBCE_On_shell(thrust::host_vector<Real4>& posRadBCE,
                        std::shared_ptr<SimParams> paramsH,
                        std::shared_ptr<chrono::fea::ChElementShellANCF_3423> shell,
                        bool multiLayer,
                        bool removeMiddleLayer,
                        int SIDE) {
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    double dx = shell->GetLengthX() / 2 - initSpace0 / 2;
    double dy = shell->GetLengthY() / 2 - initSpace0 / 2;

    double nX = dx / initSpace0 - std::floor(dx / initSpace0);
    double nY = dy / initSpace0 - std::floor(dy / initSpace0);
    int nFX = (int)std::floor(dx / initSpace0);
    int nFY = (int)std::floor(dy / initSpace0);
    if (nX > 0.5)
        nFX++;
    if (nY > 0.5)
        nFY++;

    Real initSpaceX = dx / nFX;
    Real initSpaceY = dy / nFY;

    int2 iBound = mI2(-nFX, nFX);
    int2 jBound = mI2(-nFY, nFY);
    int2 kBound;
    // If multi-layer BCE is required
    if (SIDE > 0 && multiLayer)  // Do SIDE number layers in one side
        kBound = mI2(0, SIDE);
    else if (SIDE < 0 && multiLayer)  // Do SIDE number layers in the other side
        kBound = mI2(SIDE, 0);
    else if (SIDE == 0 && multiLayer)  // Do 1 layer on each side. Note that there would be 3 layers in total
        kBound = mI2(-1, 1);           // The middle layer would be on the shell
    else                               // IF you do not want multi-layer just use one layer on the shell
        kBound = mI2(0, 0);            // This will create some marker deficiency and reduce the accuracy but look nicer

    for (int i = iBound.x; i <= iBound.y; i++) {
        for (int j = jBound.x; j <= jBound.y; j++) {
            for (int k = kBound.x; k <= kBound.y; k++) {
                Real3 relMarkerPos = mR3(i * initSpaceX, j * initSpaceY, k);

                if (k == 0 && SIDE == 0 && multiLayer && removeMiddleLayer) {
                    // skip the middle layer for this specific case
                    paramsH->MULT_INITSPACE_Shells = 0.5;
                    continue;
                }

                posRadBCE.push_back(mR4(relMarkerPos, initSpace0));
            }
        }
    }
}  // =============================================================================
void CreateBCE_On_ChElementCableANCF(thrust::host_vector<Real4>& posRadBCE,
                                     std::shared_ptr<SimParams> paramsH,
                                     std::shared_ptr<chrono::fea::ChElementCableANCF> cable,
                                     std::vector<int> remove,
                                     bool multiLayer,
                                     bool removeMiddleLayer,
                                     int SIDE) {
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    double dx = (cable->GetNodeB()->GetX0() - cable->GetNodeA()->GetX0()).Length();
    double nX = dx / initSpace0 - std::floor(dx / initSpace0);
    int nFX = (int)std::floor(dx / initSpace0);
    if (nX > 0.5)
        nFX++;

    Real initSpaceX;
    if (nFX != 0)
        initSpaceX = dx / nFX;
    else
        initSpaceX = dx;

    Real initSpaceZ = initSpace0;
    int2 iBound = mI2(0, nFX);

    for (int i = iBound.x; i <= iBound.y; i++) {
        bool con1 = (remove[1] && (i == iBound.y));
        bool con2 = (remove[0] && (i == iBound.x));
        if (con1 || con2)
            continue;

        Real3 relMarkerPos;
        double CONSTANT = 1.0;  // sqrt(2) / 2;
        if (multiLayer) {
            for (int j = 1; j <= SIDE; j++) {
                relMarkerPos = mR3(i * initSpaceX, j * initSpaceZ, 0) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, initSpace0));
                relMarkerPos = mR3(i * initSpaceX, -j * initSpaceZ, 0) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, initSpace0));
                relMarkerPos = mR3(i * initSpaceX, 0, j * initSpaceZ) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, initSpace0));
                relMarkerPos = mR3(i * initSpaceX, 0, -j * initSpaceZ) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, initSpace0));
            }
        }

        if (!removeMiddleLayer) {
            relMarkerPos = mR3(i * initSpaceX, 0, 0);
            posRadBCE.push_back(mR4(relMarkerPos, initSpace0));
        }
    }
}
// =============================================================================
void CreateBCE_On_ChElementShellANCF(thrust::host_vector<Real4>& posRadBCE,
                                     std::shared_ptr<SimParams> paramsH,
                                     std::shared_ptr<chrono::fea::ChElementShellANCF_3423> shell,
                                     std::vector<int> remove,
                                     bool multiLayer,
                                     bool removeMiddleLayer,
                                     int SIDE,
                                     double kernel_h) {
    Real initSpace0;
    if (kernel_h == 0)
        initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    else
        initSpace0 = paramsH->MULT_INITSPACE * kernel_h;

    double dx = shell->GetLengthX() / 2;
    double dy = shell->GetLengthY() / 2;
    printf("CreateBCE_On_ChElementShellANCF: dx,dy=%f,%f\n", dx, dy);

    double nX = dx / initSpace0 - std::floor(dx / initSpace0);
    double nY = dy / initSpace0 - std::floor(dy / initSpace0);
    int nFX = (int)std::floor(dx / initSpace0);
    int nFY = (int)std::floor(dy / initSpace0);
    if (nX > 0.5)
        nFX++;
    if (nY > 0.5)
        nFY++;

    Real initSpaceX = dx / nFX;
    Real initSpaceY = dy / nFY;

    int2 iBound = mI2(-nFX, nFX);
    int2 jBound = mI2(-nFY, nFY);
    int2 kBound;
    // If multi-layer BCE is required
    if (SIDE > 0 && multiLayer)  // Do SIDE number layers in one side
        kBound = mI2(0, SIDE);
    else if (SIDE < 0 && multiLayer)  // Do SIDE number layers in the other side
        kBound = mI2(SIDE, 0);
    else if (SIDE == 0 && multiLayer)  // Do 1 layer on each side. Note that there would be 3 layers in total
        kBound = mI2(-1, 1);           // The middle layer would be on the shell
    else                               // IF you do not want multi-layer just use one layer on the shell
        kBound = mI2(0, 0);            // This will create some marker deficiency and reduce the accuracy but look nicer

    for (int k = kBound.x; k <= kBound.y; k++) {
        for (int j = jBound.x; j <= jBound.y; j++) {
            for (int i = iBound.x; i <= iBound.y; i++) {
                Real3 relMarkerPos = mR3(i * initSpaceX, j * initSpaceY, k);
                if (k == 0 && SIDE == 0 && multiLayer && removeMiddleLayer) {
                    // skip the middle layer for this specific case
                    printf(
                        "---------------paramsH->MULT_INITSPACE_Shells was changed in CreateBCE_On_Mesh to 0.5. "
                        "\n");
                    paramsH->MULT_INITSPACE_Shells = 0.5;
                    continue;
                }

                // It has to skip puting BCE on the nodes if one of the following conditions is true
                bool con1 = (remove[0] && remove[1] && j == jBound.x);
                bool con2 = (remove[2] && remove[3] && j == jBound.y);
                bool con3 = (remove[1] && remove[2] && i == iBound.y);
                bool con4 = (remove[3] && remove[0] && i == iBound.x);

                if (con1 || con2 || con3 || con4)
                    continue;

                posRadBCE.push_back(mR4(relMarkerPos, (kernel_h == 0) ? paramsH->HSML : kernel_h));
            }
        }
    }
}
// =============================================================================

}  // namespace utils
}  // namespace fsi
}  // namespace chrono
