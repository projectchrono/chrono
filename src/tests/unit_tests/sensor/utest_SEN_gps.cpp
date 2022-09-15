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

#include <vector>

#include "gtest/gtest.h"

#include "chrono/core/ChLog.h"
#include "chrono_sensor/utils/ChGPSUtils.h"

using namespace chrono;
using namespace sensor;

#define GPS_TEST_EPSILLON 1e-9

TEST(ChGPSSensor, gps_conversion) {
    std::cout << "Start GPS unit test" << std::endl;

    // create some test coordinates are reference locations for representative coverage of the globe
    std::vector<ChVector<double>> test_coords = {{0, 0, 0}, {-100, -100, -100}, {100, 100, 100}};
    std::vector<ChVector<double>> test_refs = {{0, 0, 0}, {-75, 70, 500}, {20, -60, -200}};

    std::cout << "Testing " << test_coords.size() << " coord sets" << std::endl;

    // make sure every pair of coordinate-reference locations is handled by the conversion in an invertible way
    for (int i = 0; i < test_coords.size(); i++) {
        ChVector<double> coord_copy(test_coords[i]);
        const ChVector<double> coord_const(test_coords[i]);
        for (int j = 0; j < test_refs.size(); j++) {
            ChVector<double> ref = test_refs[j];

            Cartesian2GPS(coord_copy, ref);
            GPS2Cartesian(coord_copy, ref);

            // make sure our conversions are inverses
            ASSERT_LT(std::abs(coord_copy.x() - coord_const.x()), GPS_TEST_EPSILLON);
            ASSERT_LT(std::abs(coord_copy.y() - coord_const.y()), GPS_TEST_EPSILLON);
            ASSERT_LT(std::abs(coord_copy.z() - coord_const.z()), GPS_TEST_EPSILLON);
        }
    }

    std::cout << "Done GPS unit test" << std::endl;
}
