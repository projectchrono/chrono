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
void CreateBCE_On_Sphere(thrust::host_vector<Real4>& posRadBCE, 
                         Real rad, 
                         const SimParams& paramsH) {
    Real kernel_h = paramsH.HSML;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;

    for (Real r = 0.5 * spacing; r < rad; r += spacing) {
        int numphi = (int)std::floor(3.1415 * r / spacing);
        for (size_t p = 0; p < numphi; p++) {
            Real phi = p * 3.1415 / numphi;
            int numTheta = (int)std::floor(2 * 3.1415 * r * sin(phi) / spacing);
            for (Real t = 0.0; t < numTheta; t++) {
                Real teta = t * 2 * 3.1415 / numTheta;
                Real3 BCE_Pos_local = 
                    mR3(r * sin(phi) * cos(teta), r * sin(phi) * sin(teta), r * cos(phi));
                posRadBCE.push_back(mR4(BCE_Pos_local, kernel_h));
            }
        }
        posRadBCE.push_back(mR4(0, 0, r, kernel_h));
        posRadBCE.push_back(mR4(0, 0, -r, kernel_h));
    }
}

// =============================================================================
void CreateBCE_On_Cylinder(thrust::host_vector<Real4>& posRadBCE,
                           Real rad,
                           Real height,
                           const SimParams& paramsH,
                           bool cartesian) {
    Real kernel_h = paramsH.HSML;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;
    int num_layers = (int)std::floor(1.00001 * height / spacing) + 1;

    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * height + spacing * si;
        if (cartesian)
            for (Real x = -rad; x <= rad; x += spacing) {
                for (Real y = -rad; y <= rad; y += spacing) {
                    if (x * x + y * y <= rad * rad)
                        posRadBCE.push_back(mR4(x, s, y, kernel_h));
                }
            }
        else {
            Real3 centerPointLF = mR3(0, s, 0);
            posRadBCE.push_back(mR4(0, s, 0, kernel_h));
            int numr = (int)std::floor(1.00001 * rad / spacing);
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
void CreateBCE_On_surface_of_Cylinder(thrust::host_vector<Real4>& posRadBCE,
                                      thrust::host_vector<Real3>& normals,
                                      Real rad,
                                      Real height,
                                      const SimParams& paramsH) {
    Real kernel_h = paramsH.HSML;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;
    int num_layers = (int)std::floor(height / spacing);

    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * height + (height / num_layers) * si;
        Real3 centerPointLF = mR3(0, s, 0);
        Real numr = std::floor(rad / spacing);
        for (size_t ir = 1; ir < numr; ir++) {
            Real r = spacing + ir * rad / numr;
            int numTheta = (int)std::floor(2 * 3.1415 * r / spacing);
            for (size_t t = 0; t < numTheta; t++) {
                Real teta = t * 2 * 3.1415 / numTheta;
                Real3 BCE_Pos_local = mR3(r * cos(teta), 0, r * sin(teta)) + centerPointLF;
                if (ir == numr - 1) {
                    posRadBCE.push_back(mR4(BCE_Pos_local, kernel_h));
                }
            }
        }
    }
}

// =============================================================================
void CreateBCE_On_Cylinder_Annulus(thrust::host_vector<Real4>& posRadBCE,
                                   Real rad_in,
                                   Real rad_out,
                                   Real height,
                                   const SimParams& paramsH,
                                   bool cartesian) {
    Real kernel_h = paramsH.HSML;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;
    int num_layers = (int)std::floor(1.00001 * height / spacing) + 1;

    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * height + spacing * si;
        if (cartesian)
            for (Real x = -rad_out; x <= rad_out; x += spacing) {
                for (Real y = -rad_out; y <= rad_out; y += spacing) {
                    Real xxyy = x * x + y * y;
                    if ( xxyy <= rad_out * rad_out && xxyy > rad_in * rad_in)
                        posRadBCE.push_back(mR4(x, s, y, kernel_h));
                }
            }
        else {
            Real3 centerPointLF = mR3(0, s, 0);
            int numr = (int)std::floor(1.00001 * rad_out / spacing);
            for (size_t ir = 0; ir < numr; ir++) {
                Real r = 0.5 * spacing + ir * spacing;
                if (r >= rad_in){
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
}

// =============================================================================
void CreateBCE_On_Cone(thrust::host_vector<Real4>& posRadBCE,
                       Real rad,
                       Real height,
                       const SimParams& paramsH,
                       bool cartesian) {
    Real kernel_h = paramsH.HSML;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;
    int num_layers = (int)std::floor(height / spacing);

    for (size_t si = 0; si < num_layers; si++) {
        Real s = -0.5 * height + spacing / 2 + (height / num_layers) * si;
        Real h0 = spacing / 2 + (height / num_layers) * si;
        Real r0 = h0 / height * rad;
        // if (cartesian)
        //     for (Real x = -rad; x <= rad; x += spacing) {
        //         for (Real y = -rad; y <= rad; y += spacing) {
        //             if (x * x + y * y <= rad * rad)
        //                 posRadBCE.push_back(mR4(x, s, y, kernel_h));
        //         }
        //     }
        // else {
        Real3 centerPointLF = mR3(0, 0, s);
        posRadBCE.push_back(mR4(0, 0, s, kernel_h));
        Real numr = std::floor(r0 / spacing) + 2;
        // Real spacing_r = r0 / numr;
        if (si > 0) {
            for (size_t ir = 1; ir < numr; ir++) {
                Real r = r0 - 0.5 * spacing - (ir - 1) * spacing;
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
    }
}

// =============================================================================
// note, the function in the current implementation creates boundary BCE (zero
// velocity). x=1, y=2, z =3; therefore 12 means creating markers on the top 
// surface parallel to xy plane, similarly -12 means bottom face paralel to xy. 
// similarly 13, -13, 23, -23.
void CreateBCE_On_Box(thrust::host_vector<Real4>& posRadBCE, const Real3& hsize, int face, const SimParams& paramsH) {
    Real kernel_h = paramsH.HSML;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;
    int num_bndry_layers = paramsH.NUM_BOUNDARY_LAYERS;

    int nFX = (int)round(hsize.x / spacing);
    int nFY = (int)round(hsize.y / spacing);
    int nFZ = (int)round(hsize.z / spacing);

    Real initSpaceX = hsize.x / nFX;
    Real initSpaceY = hsize.y / nFY;
    Real initSpaceZ = hsize.z / nFZ;

    int2 iBound = mI2(-nFX, nFX);
    int2 jBound = mI2(-nFY, nFY);
    int2 kBound = mI2(-nFZ, nFZ);
    
    switch (face) {
        case 12:
            kBound = mI2(nFZ - num_bndry_layers + 1, nFZ);
            break;
        case -12:
            kBound = mI2(-nFZ, -nFZ + num_bndry_layers - 1);
            break;
        case 13:
            jBound = mI2(nFY - num_bndry_layers + 1, nFY);
            break;
        case -13:
            jBound = mI2(-nFY, -nFY + num_bndry_layers - 1);
            break;
        case 23:
            iBound = mI2(nFX - num_bndry_layers + 1, nFX);
            break;
        case -23:
            iBound = mI2(-nFX, -nFX + num_bndry_layers - 1);
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

                if ((relMarkerPos.x < paramsH.cMin.x || relMarkerPos.x > paramsH.cMax.x) ||
                    (relMarkerPos.y < paramsH.cMin.y || relMarkerPos.y > paramsH.cMax.y) ||
                    (relMarkerPos.z < paramsH.cMin.z || relMarkerPos.z > paramsH.cMax.z)) {
                    continue;
                }
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
            }
        }
    }
}

// =============================================================================
void CreateBCE_cableFEM(thrust::host_vector<Real4>& posRadBCE,
                        std::shared_ptr<chrono::fea::ChElementCableANCF> cable,
                        std::vector<int> remove,
                        bool multiLayer,
                        bool removeMiddleLayer,
                        int SIDE,
                        const SimParams& paramsH) {
    Real kernel_h = paramsH.HSML;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;

    double dx = (cable->GetNodeB()->GetX0() - cable->GetNodeA()->GetX0()).Length();
    double nX = dx / spacing - std::floor(dx / spacing);
    int nFX = (int)std::floor(dx / spacing);
    if (nX > 0.5)
        nFX++;

    Real initSpaceX;
    if (nFX != 0)
        initSpaceX = dx / nFX;
    else
        initSpaceX = dx;

    Real initSpaceZ = spacing;
    int2 iBound = mI2(0, nFX);

    for (int i = iBound.x; i <= iBound.y; i++) {
        bool con1 = (remove[1] && (i == iBound.y));
        bool con2 = (remove[0] && (i == iBound.x));
        if (con1 || con2)
            continue;

        Real3 relMarkerPos;
        double CONSTANT = 1.0;
        if (multiLayer) {
            for (int j = 1; j <= SIDE; j++) {
                relMarkerPos = mR3(i * initSpaceX, j * initSpaceZ, 0) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
                relMarkerPos = mR3(i * initSpaceX, -j * initSpaceZ, 0) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
                relMarkerPos = mR3(i * initSpaceX, 0, j * initSpaceZ) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
                relMarkerPos = mR3(i * initSpaceX, 0, -j * initSpaceZ) * CONSTANT;
                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
            }
        }

        if (!removeMiddleLayer) {
            relMarkerPos = mR3(i * initSpaceX, 0, 0);
            posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
        }
    }
}

// =============================================================================
void CreateBCE_shellFEM(thrust::host_vector<Real4>& posRadBCE,
                        std::shared_ptr<chrono::fea::ChElementShellANCF_3423> shell,
                        std::vector<int> remove,
                        std::vector<int> remove_s,
                        bool multiLayer,
                        bool removeMiddleLayer,
                        int SIDE,
                        const SimParams& paramsH) {
    Real kernel_h = paramsH.HSML;
    Real spacing = paramsH.MULT_INITSPACE * paramsH.HSML;

    double dx = shell->GetLengthX() / 2;
    double dy = shell->GetLengthY() / 2;

    double nX = dx / spacing - std::floor(dx / spacing);
    double nY = dy / spacing - std::floor(dy / spacing);
    int nFX = (int)std::floor(dx / spacing);
    int nFY = (int)std::floor(dy / spacing);
    if (nX > 0.5)
        nFX++;
    if (nY > 0.5)
        nFY++;

    Real initSpaceX;
    Real initSpaceY;
    int2 iBound;
    int2 jBound;

    if (dx < nFX * spacing) {
        iBound = mI2(-nFX * 2, nFX * 2);
        initSpaceX = dx / nFX;
    } else {
        iBound = mI2(-nFX * 2 - 1, nFX * 2 + 1);
        initSpaceX = dx / (0.5 + nFX);
    }

    if (dy < nFY * spacing) {
        jBound = mI2(-nFY * 2, nFY * 2);
        initSpaceY = dy / nFY;
    } else {
        jBound = mI2(-nFY * 2 - 1, nFY * 2 + 1);
        initSpaceY = dy / (0.5 + nFY);
    }

    int2 kBound;
    // If multi-layer BCE is required
    if (SIDE > 0 && multiLayer)         // Do SIDE number layers in one side
        kBound = mI2(0, SIDE);
    else if (SIDE < 0 && multiLayer)    // Do SIDE number layers in the other side
        kBound = mI2(SIDE, 0);
    else if (SIDE == 0 && multiLayer)   // Do 1 layer on each side. Note that there would be 3 layers in total
        kBound = mI2(-1, 1);            // The middle layer would be on the shell
    else                                // IF you do not want multi-layer just use one layer on the shell
        kBound = mI2(0, 0);             // This will create some marker deficiency and reduce the accuracy but look nicer

    for (int k = kBound.x; k <= kBound.y; k++) {
        //// RADU TODO
        ////    There should be no side-effect in this function!!!!!
        ////if (k == 0 && SIDE == 0 && multiLayer && removeMiddleLayer) {
        ////    // skip the middle layer for this specific case
        ////    // change value of paramsH->MULT_INITSPACE_Shells
        ////    paramsH.MULT_INITSPACE_Shells = 0.5;
        ////    continue;
        ////}
        for (int j = jBound.x; j <= jBound.y; j=j+2) {
            for (int i = iBound.x; i <= iBound.y; i=i+2) {
                Real3 relMarkerPos = mR3(i * initSpaceX / 2.0 , j * initSpaceY / 2.0 , k);

                // It has to skip puting BCE on the nodes if one of the following conditions is true
                bool con1 = (remove_s[0] && j == jBound.x);
                bool con2 = (remove_s[2] && j == jBound.y);
                bool con3 = (remove_s[1] && i == iBound.y);
                bool con4 = (remove_s[3] && i == iBound.x);
                bool con5 = (remove[0] && remove[1] && (!remove_s[0]) && j == jBound.x && (i==iBound.x || i == iBound.y));
                bool con6 = (remove[2] && remove[3] && (!remove_s[2]) && j == jBound.y && (i==iBound.x || i == iBound.y));
                bool con7 = (remove[1] && remove[2] && (!remove_s[1]) && i == iBound.y && (j==jBound.x || j == jBound.y));
                bool con8 = (remove[3] && remove[0] && (!remove_s[3]) && i == iBound.x && (j==jBound.x || j == jBound.y));

                if (con1 || con2 || con3 || con4 || con5 || con6 || con7 || con8 )
                    continue;

                posRadBCE.push_back(mR4(relMarkerPos, kernel_h));
            }
        }
    }
}
// =============================================================================

}  // namespace utils
}  // namespace fsi
}  // namespace chrono
