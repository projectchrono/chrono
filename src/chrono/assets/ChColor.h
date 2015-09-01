//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCOLOR_H
#define CHCOLOR_H

#include "core/ChStream.h"
#include "core/ChRunTimeType.h"
#include "serialization/ChArchive.h"



namespace chrono {

/// Class for setting a color (used by ChVisualization)

class ChApi ChColor {
    // Chrono RTTI, needed for serialization
    CH_RTTI_ROOT(ChColor);

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


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);

        // serialize all member data:
        marchive << CHNVP(R);
        marchive << CHNVP(G);
        marchive << CHNVP(B);
        marchive << CHNVP(A);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();

        // stream in all member data:
        marchive >> CHNVP(R);
        marchive >> CHNVP(G);
        marchive >> CHNVP(B);
        marchive >> CHNVP(A);
    }

    void StreamOUT(ChStreamOutAscii& mstream) {
        mstream << "\nRGB=" << R << "\n" << G << "\n" << B << " A=" << A << "\n";
    }
    void StreamOUT(ChStreamOutBinary& mstream) {
        mstream << R;
        mstream << G;
        mstream << B;
        mstream << A;
    }
    void StreamIN(ChStreamInBinary& mstream) {
        mstream >> R;
        mstream >> G;
        mstream >> B;
        mstream >> A;
    }
};

}  // END_OF_NAMESPACE____

#endif
