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
// Authors: Asher Elmquist
// =============================================================================
//
// Unit test for MatrMultiplyAVX and MatrMultiplyTAVX.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/core/ChLog.h"
#include "chrono_sensor/ChGPSSensor.h"

using namespace chrono;
using namespace sensor;

#define GPS_TEST_EPSILLON 1e-9

TEST(ChGPSSensor, gps_conversion) {
    // create some test coordinates are reference locations for representative coverage of the globe
    std::vector<ChVector<double>> test_coords = {{0, 0, 0}, {-100, -100, -100}, {100, 100, 100}};
    std::vector<ChVector<double>> test_refs = {{0, 0, 0}, {-75, 70, 500}, {20, -60, -200}};

    // make sure every pair of coordinate-reference locations is handled by the conversion in an invertible way
    for (int i = 0; i < test_coords.size(); i++) {
        ChVector<double> coord_0 = test_coords[i];
        const ChVector<double> coord_0_const(coord_0);
        for (int j = 0; j < test_refs.size(); j++) {
            ChVector<double> ref_0 = test_refs[j];
            const ChVector<double> ref_0_const(ref_0);

            Cartesian2GPS(coord_0, ref_0);
            GPS2Cartesian(coord_0, ref_0);

            // make sure our conversions are inverses
            ASSERT_LT(abs(coord_0.x() - coord_0_const.x()), GPS_TEST_EPSILLON);
            ASSERT_LT(abs(coord_0.y() - coord_0_const.y()), GPS_TEST_EPSILLON);
            ASSERT_LT(abs(coord_0.z() - coord_0_const.z()), GPS_TEST_EPSILLON);

            // make sure our conversions never modify the reference location
            ASSERT_LT(abs(ref_0.x() - ref_0_const.x()), GPS_TEST_EPSILLON);
            ASSERT_LT(abs(ref_0.y() - ref_0_const.y()), GPS_TEST_EPSILLON);
            ASSERT_LT(abs(ref_0.z() - ref_0_const.z()), GPS_TEST_EPSILLON);
        }
    }
}
