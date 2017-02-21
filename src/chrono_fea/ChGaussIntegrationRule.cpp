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
// Authors: Andrea Favali
// =============================================================================

#include "chrono_fea/ChGaussIntegrationRule.h"

namespace chrono {
namespace fea {

ChGaussIntegrationRule::ChGaussIntegrationRule() {}

ChGaussIntegrationRule::~ChGaussIntegrationRule() {}

// Create and assign at the 'GpVector' pointer the vector of the Gauss Integration Points.
// The vector is resized according to the number of points required.
// Note: the coordinates used are local (area coordinates L1=A1/Atot).
void ChGaussIntegrationRule::SetIntOnTriangle(int nPoints, std::vector<ChGaussPoint*>* GpVector) {
    double weight;
    ChVector<> coord;
    std::vector<double> c, w;

    // Resize the vector containing Gauss Points
    (*GpVector).resize(nPoints);

    switch (nPoints) {
        case 1:

            coord.x() = 0.0;
            coord.y() = 0.0;
            coord.z() = 0.0;
            weight = 4.0;
            (*GpVector)[0] = new ChGaussPoint(1, &coord, weight);
            break;

        //  case 4:

        // c.resize(2);
        // w.resize(2);
        //      c[0] = -0.577350269189626;
        //      c[1] =  0.577350269189626;

        //      w[0] = 1.0;
        //      w[1] = 1.0;

        //      for ( i = 0; i < 2; i++ ) {
        //          for ( j = 0; j < 2; j++ ) {
        //                  coord.x() = c[ i ];
        //			coord.y() = c[ j ];
        //                  weight = w[ i ] * w[ j ];
        //			//ChGaussPoint* my_gp = new ChGaussPoint( 4 *i + 2 *j + k , &coord, weight);
        //                  ( * GpVector ) [ 2 * i + j ] = new ChGaussPoint( 2 * i + j + 1 , &coord, weight);
        //          }
        //      }

        // break;

        //  case 9:

        // c.resize(3);
        // w.resize(3);
        //      c[0] = -0.774596669241483;
        //      c[1] =  0.0;
        //      c[2] =  0.774596669241483;

        //      w[0] =  0.555555555555555;
        //      w[1] =  0.888888888888888;
        //      w[2] =  0.555555555555555;

        //      for ( i = 0; i < 3; i++ ) {
        //          for ( j = 0; j < 3; j++ ) {
        //                  coord.x() = c[ i ];
        //			coord.y() = c[ j ];
        //                  weight = w[ i ] * w[ j ];
        //                  ( * GpVector ) [ 3 * i + j ] = new ChGaussPoint( 3 * i + j + 1, &coord, weight);
        //          }
        //      }

        //      break;

        default:
            GetLog() << "SetIntOnCube: unsupported number of integration points: " << nPoints;

    }  //__end of Switch
}  //__end of Set On Triangle

// Create and assign at the 'GpVector' pointer the vector of the Gauss Integration Points
// The vector is resized according to the number of points required
void ChGaussIntegrationRule::SetIntOnSquare(int nPoints, std::vector<ChGaussPoint*>* GpVector) {
    int i, j;
    double weight;
    ChVector<> coord;
    std::vector<double> c, w;

    // Resize the vector containing Gauss Points
    // GpVector = new std::vector<ChGaussPoint> [ nPoints ];
    (*GpVector).resize(nPoints);

    switch (nPoints) {
        case 1:

            coord.x() = 0.0;
            coord.y() = 0.0;
            weight = 4.0;
            (*GpVector)[0] = new ChGaussPoint(1, &coord, weight);
            break;

        case 4:

            c.resize(2);
            w.resize(2);
            c[0] = -0.577350269189626;
            c[1] = 0.577350269189626;

            w[0] = 1.0;
            w[1] = 1.0;

            for (i = 0; i < 2; i++) {
                for (j = 0; j < 2; j++) {
                    coord.x() = c[i];
                    coord.y() = c[j];
                    weight = w[i] * w[j];
                    // ChGaussPoint* my_gp = new ChGaussPoint( 4 *i + 2 *j + k , &coord, weight);
                    (*GpVector)[2 * i + j] = new ChGaussPoint(2 * i + j + 1, &coord, weight);
                }
            }

            break;

        case 9:

            c.resize(3);
            w.resize(3);
            c[0] = -0.774596669241483;
            c[1] = 0.0;
            c[2] = 0.774596669241483;

            w[0] = 0.555555555555555;
            w[1] = 0.888888888888888;
            w[2] = 0.555555555555555;

            for (i = 0; i < 3; i++) {
                for (j = 0; j < 3; j++) {
                    coord.x() = c[i];
                    coord.y() = c[j];
                    weight = w[i] * w[j];
                    (*GpVector)[3 * i + j] = new ChGaussPoint(3 * i + j + 1, &coord, weight);
                }
            }

            break;

        case 16:

            c.resize(4);
            w.resize(4);
            c[0] = -0.861136311594053;
            c[1] = -0.339981043584856;
            c[2] = 0.339981043584856;
            c[3] = 0.861136311594053;

            w[0] = 0.347854845137454;
            w[1] = 0.652145154862546;
            w[2] = 0.652145154862546;
            w[3] = 0.347854845137454;

            for (i = 0; i < 4; i++) {
                for (j = 0; j < 4; j++) {
                    coord.x() = c[i];
                    coord.y() = c[j];
                    weight = w[i] * w[j];
                    (*GpVector)[4 * i + j] = new ChGaussPoint(4 * i + j + 1, &coord, weight);
                }
            }

            break;

        default:
            GetLog() << "SetIntOnCube: unsupported number of integration points: " << nPoints;

    }  //__end of Switch
}  //__end of Set On Square

// Create and assign at the 'GpVector' pointer the vector of the Gauss Integration Points
// The vector is resized according to the number of points required
void ChGaussIntegrationRule::SetIntOnCube(int nPoints, std::vector<ChGaussPoint*>* GpVector) {
    int i, j, k;
    double weight;
    ChVector<> coord;
    std::vector<double> c, w;

    // Resize the vector containing Gauss Points
    // GpVector = new std::vector<ChGaussPoint> [ nPoints ];
    (*GpVector).resize(nPoints);

    switch (nPoints) {
        case 1:

            coord.x() = 0.0;
            coord.y() = 0.0;
            coord.z() = 0.0;
            weight = 8.0;
            (*GpVector)[0] = new ChGaussPoint(1, &coord, weight);
            break;

        case 8:

            c.resize(2);
            w.resize(2);
            c[0] = -0.577350269189626;
            c[1] = 0.577350269189626;

            w[0] = 1.0;
            w[1] = 1.0;

            for (i = 0; i < 2; i++) {
                for (j = 0; j < 2; j++) {
                    for (k = 0; k < 2; k++) {
                        coord.x() = c[i];
                        coord.y() = c[j];
                        coord.z() = c[k];
                        weight = w[i] * w[j] * w[k];
                        // ChGaussPoint* my_gp = new ChGaussPoint( 4 *i + 2 *j + k , &coord, weight);
                        (*GpVector)[4 * i + 2 * j + k] = new ChGaussPoint(4 * i + 2 * j + k + 1, &coord, weight);
                    }
                }
            }

            break;

        case 27:

            c.resize(3);
            w.resize(3);
            c[0] = -0.774596669241483;
            c[1] = 0.0;
            c[2] = 0.774596669241483;

            w[0] = 0.555555555555555;
            w[1] = 0.888888888888888;
            w[2] = 0.555555555555555;

            for (i = 0; i < 3; i++) {
                for (j = 0; j < 3; j++) {
                    for (k = 0; k < 3; k++) {
                        coord.x() = c[i];
                        coord.y() = c[j];
                        coord.z() = c[k];
                        weight = w[i] * w[j] * w[k];
                        (*GpVector)[9 * i + 3 * j + k] = new ChGaussPoint(9 * i + 3 * j + k + 1, &coord, weight);
                    }
                }
            }

            break;

        case 64:

            c.resize(4);
            w.resize(4);
            c[0] = -0.861136311594053;
            c[1] = -0.339981043584856;
            c[2] = 0.339981043584856;
            c[3] = 0.861136311594053;

            w[0] = 0.347854845137454;
            w[1] = 0.652145154862546;
            w[2] = 0.652145154862546;
            w[3] = 0.347854845137454;

            for (i = 0; i < 4; i++) {
                for (j = 0; j < 4; j++) {
                    for (k = 0; k < 4; k++) {
                        coord.x() = c[i];
                        coord.y() = c[j];
                        coord.z() = c[k];
                        weight = w[i] * w[j] * w[k];
                        (*GpVector)[16 * i + 4 * j + k] = new ChGaussPoint(16 * i + 4 * j + k + 1, &coord, weight);
                    }
                }
            }

            break;

        default:
            GetLog() << "SetIntOnCube: unsupported number of integration points: " << nPoints;

    }  //__end of Switch
}  //__end of Set On Cube

}  // end namespace fea
}  // end namespace chrono
