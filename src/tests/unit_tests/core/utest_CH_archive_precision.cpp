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
// Floating point fidelity of the archive backends.
//
// The text backends used to write floating point values at the default ostream precision, which is
// 6 significant digits, while a double needs 17 to be recovered exactly. Saving a model to JSON or
// XML and loading it back therefore returned different numbers, with no warning: 1.0/3.0 came back
// as 0.333333, and 12345.678901234567 came back as 12345.7, an absolute error of about 0.022.
//
// Nothing caught this because the existing round-trip tests compare with a tolerance of 1e-5 on
// states of magnitude near 1, where 6 significant digits happens to be good enough. Six digits is a
// relative budget, so the absolute error grows with the magnitude of the value.
//
// The binary backend was never affected: it writes the raw bytes instead of printing them. It is
// used here as a control, so that these tests describe the text backends specifically rather than
// asserting that serialization is exact in general.
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

// Count the significant decimal digits in the first number found in some serialized text.
size_t SignificantDigits(const std::string& text) {
    size_t digits = 0;
    bool in_number = false, seen_nonzero = false;
    for (char c : text) {
        if (std::isdigit(static_cast<unsigned char>(c))) {
            in_number = true;
            if (c != '0')
                seen_nonzero = true;
            if (seen_nonzero)
                ++digits;
        } else if (c == '.' || c == '-' || c == '+' || (in_number && (c == 'e' || c == 'E'))) {
            continue;
        } else if (in_number) {
            break;
        }
    }
    return digits;
}

}  // namespace

TEST(ChArchiveJSON, DoubleRoundTripIsExact) {
    for (double v : TestValues()) {
        const float fv = static_cast<float>(v);
        NumberHolder got = RoundTrip<ChArchiveOutJSON, ChArchiveInJSON>(v, fv);
        EXPECT_DOUBLE_EQ(got.d, v) << "JSON lost precision on " << v;
        EXPECT_FLOAT_EQ(got.f, fv) << "JSON lost precision on float " << fv;
    }
}

TEST(ChArchiveXML, DoubleRoundTripIsExact) {
    for (double v : TestValues()) {
        const float fv = static_cast<float>(v);
        NumberHolder got = RoundTrip<ChArchiveOutXML, ChArchiveInXML>(v, fv);
        EXPECT_DOUBLE_EQ(got.d, v) << "XML lost precision on " << v;
        EXPECT_FLOAT_EQ(got.f, fv) << "XML lost precision on float " << fv;
    }
}

// Control: the binary backend writes raw bytes and was never affected. If this ever fails, the
// problem is something other than text formatting.
TEST(ChArchiveBinary, DoubleRoundTripIsExact) {
    for (double v : TestValues()) {
        const float fv = static_cast<float>(v);
        NumberHolder got = RoundTrip<ChArchiveOutBinary, ChArchiveInBinary>(v, fv);
        EXPECT_DOUBLE_EQ(got.d, v) << "binary lost precision on " << v;
        EXPECT_FLOAT_EQ(got.f, fv) << "binary lost precision on float " << fv;
    }
}

// Exactness is necessary but not sufficient. Printing a float at the double's seventeen digits also
// round trips, but it exposes eight digits of binary noise: 0.6f becomes 0.60000002384185791. A float
// must be written at its own precision, which is nine digits.
TEST(ChArchiveJSON, FloatIsNotOverPrinted) {
    std::ostringstream stream;
    {
        NumberHolder holder;
        holder.d = 0;
        holder.f = 0.6f;
        ChArchiveOutJSON archive_out(stream);
        archive_out << CHNVP(holder, "holder");
    }
    const std::string text = stream.str();
    const size_t at = text.find("\"f\"");
    ASSERT_NE(at, std::string::npos) << "could not locate the float in:\n" << text;

    const size_t digits = SignificantDigits(text.substr(at + 3));
    EXPECT_LE(digits, static_cast<size_t>(std::numeric_limits<float>::max_digits10))
        << "float written with " << digits << " significant digits, which is more than a float carries.\n"
        << "serialized text was:\n"
        << text;
}

// The output archives take a stream owned by the caller, which may well be reused afterwards, so
// raising the precision must not be a permanent side effect on that stream.
TEST(ChArchiveJSON, DoesNotLeakStreamPrecision) {
    std::ostringstream stream;
    const std::streamsize original = stream.precision(4);
    ASSERT_EQ(stream.precision(), 4);
    {
        NumberHolder holder;
        ChArchiveOutJSON archive_out(stream);
        archive_out << CHNVP(holder, "holder");
        EXPECT_EQ(stream.precision(), std::numeric_limits<double>::max_digits10)
            << "the archive should raise the precision while it is writing";
    }
    EXPECT_EQ(stream.precision(), 4) << "the archive must restore the caller's stream precision";
    stream.precision(original);
}

TEST(ChArchiveXML, DoesNotLeakStreamPrecision) {
    std::ostringstream stream;
    const std::streamsize original = stream.precision(4);
    ASSERT_EQ(stream.precision(), 4);
    {
        NumberHolder holder;
        ChArchiveOutXML archive_out(stream);
        archive_out << CHNVP(holder, "holder");
        EXPECT_EQ(stream.precision(), std::numeric_limits<double>::max_digits10)
            << "the archive should raise the precision while it is writing";
    }
    EXPECT_EQ(stream.precision(), 4) << "the archive must restore the caller's stream precision";
    stream.precision(original);
}
