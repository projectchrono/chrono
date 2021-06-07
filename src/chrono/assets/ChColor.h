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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHCOLOR_H
#define CHCOLOR_H

#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChStream.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Class for setting a color (used by ChVisualization)
class ChApi ChColor {
  public:
    float R;  /// red channel (0,1)
    float G;  /// green channel (0,1)
    float B;  /// blue channel (0,1)
    float A;  /// alpha channel (0,1)

    /// Constructors
    ChColor() : R(1), G(1), B(1), A(0) {}
    ChColor(float mR, float mG, float mB, float mA = 0) : R(mR), G(mG), B(mB), A(mA) {}
    ChColor(const ChColor& other) : R(other.R), G(other.G), B(other.B), A(other.A) {}

    /// Assignment: copy from another color
    ChColor& operator=(const ChColor& other) {
        if (&other == this)
            return *this;
        R = other.R;
        G = other.G;
        B = other.B;
        A = other.A;
        return *this;
    }

    /// Compute a false color from a scalar value. Uses a cold-to-hot colormap.
    /// The 'v' scalar value is mapped in the vmin-vmax range.
    /// If out_of_range_as_bw option is true, when v>vmax the color is white and for v<vmin the color is black.
    static ChColor ComputeFalseColor(double v, double vmin, double vmax, bool out_of_range_as_bw = false);

    /// Method to allow serialization of transient data to archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive);
};

CH_CLASS_VERSION(ChColor, 0)

}  // end namespace chrono

#endif
