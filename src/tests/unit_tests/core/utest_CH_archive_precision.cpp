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
//
// Testing accuracy up to 0 ULP on archives
//
// =============================================================================

#include "gtest/gtest.h"

#include <cctype>
#include <ios>
#include <limits>
#include <sstream>
#include <vector>

#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"

using namespace chrono;

namespace {

struct NumberHolder {
    double d = 0;
    float f = 0;

    void ArchiveOut(ChArchiveOut& archive_out) {
        archive_out << CHNVP(d);
        archive_out << CHNVP(f);
    }
    void ArchiveIn(ChArchiveIn& archive_in) {
        archive_in >> CHNVP(d);
        archive_in >> CHNVP(f);
    }
};

// Values that cannot survive 6 significant digits. Spread over several magnitudes, because the
// error of a fixed significant-digit budget scales with the value.
std::vector<double> TestValues() {
    return {
        1.0 / 3.0,             //
        12345.678901234567,    //
        3.14159265358979312,   //
        -2.718281828459045,    //
        1.2345678901234567e+8  //
    };
}

template <typename OutArchive, typename InArchive>
NumberHolder RoundTrip(double d, float f) {
    std::stringstream buffer;
    {
        NumberHolder written;
        written.d = d;
        written.f = f;
        OutArchive archive_out(buffer);
        archive_out << CHNVP(written, "holder");
    }
    NumberHolder read;
    InArchive archive_in(buffer);
    archive_in >> CHNVP(read, "holder");
    return read;
}

}  // namespace

TEST(ChArchiveJSON, DoubleRoundTripIsExact) {
    for (double v : TestValues()) {
        const float fv = static_cast<float>(v);  // not used
        NumberHolder got = RoundTrip<ChArchiveOutJSON, ChArchiveInJSON>(v, fv);

        testing::internal::FloatingPoint<double> fp_a_double(got.d);
        testing::internal::FloatingPoint<double> fp_b_double(v);
        EXPECT_EQ(fp_a_double.bits(), fp_b_double.bits()) << "JSON does not match the bit pattern on double: " << v;

        testing::internal::FloatingPoint<float> fp_a_float(got.f);
        testing::internal::FloatingPoint<float> fp_b_float(fv);
        EXPECT_EQ(fp_a_float.bits(), fp_b_float.bits()) << "JSON does not match the bit pattern on float: " << fv;
    }
}

TEST(ChArchiveXML, DoubleRoundTripIsExact) {
    for (double v : TestValues()) {
        const float fv = static_cast<float>(v);  // not used
        NumberHolder got = RoundTrip<ChArchiveOutXML, ChArchiveInXML>(v, fv);

        testing::internal::FloatingPoint<double> fp_a_double(got.d);
        testing::internal::FloatingPoint<double> fp_b_double(v);
        EXPECT_EQ(fp_a_double.bits(), fp_b_double.bits()) << "XML does not match the bit pattern on double: " << v;

        testing::internal::FloatingPoint<float> fp_a_float(got.f);
        testing::internal::FloatingPoint<float> fp_b_float(fv);
        EXPECT_EQ(fp_a_float.bits(), fp_b_float.bits()) << "XML does not match the bit pattern on float: " << fv;
    }
}

TEST(ChArchiveBinary, DoubleRoundTripIsExact) {
    for (double v : TestValues()) {
        const float fv = static_cast<float>(v);
        NumberHolder got = RoundTrip<ChArchiveOutBinary, ChArchiveInBinary>(v, fv);

        testing::internal::FloatingPoint<double> fp_a_double(got.d);
        testing::internal::FloatingPoint<double> fp_b_double(v);
        EXPECT_EQ(fp_a_double.bits(), fp_b_double.bits()) << "Binary archive does not match the bit pattern on double: " << v;

        testing::internal::FloatingPoint<float> fp_a_float(got.f);
        testing::internal::FloatingPoint<float> fp_b_float(fv);
        EXPECT_EQ(fp_a_float.bits(), fp_b_float.bits()) << "Binary archive does not match the bit pattern on float: " << fv;
    }
}
