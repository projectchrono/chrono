//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_LINESEGMENT_H
#define CHC_LINESEGMENT_H

#include <math.h>

#include "ChCLine.h"

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_LINESEGMENT 18

///
/// SEGMENT
///
/// Geometric object representing a segment in 3D space
/// with two end points
///

class ChApi ChLineSegment : public ChLine {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChLineSegment, ChLine);

  public:
    //
    // DATA
    //

    ChVector<> pA;
    ChVector<> pB;

  public:
    //
    // CONSTRUCTORS
    //

    ChLineSegment(const ChVector<> mA = VNULL, const ChVector<> mB = VNULL) {
        pA = mA;
        pB = mB;
    }

    ~ChLineSegment(){};

    ChLineSegment(const ChLineSegment& source) {
        pA = source.pA;
        pB = source.pB;
    }

    void Copy(const ChLineSegment* source) {
        ChLine::Copy(source);
        pA = source->pA;
        pB = source->pB;
    }

    ChGeometry* Duplicate() { return new ChLineSegment(*this); };

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() { return CH_GEOCLASS_LINESEGMENT; };

    virtual int Get_complexity() { return 2; };

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(Vector& pos, const double parU, const double parV = 0., const double parW = 0.) {
        pos = pA * (1 - parU) + pB * parU;
    }

    /// Returns curve length. sampling does not matter
    double Length(int sampling) { return (pA - pB).Length(); }

    //
    // CUSTOM FUNCTIONS
    //

    //
    // STREAMING
    //

    void StreamOUT(ChStreamOutBinary& mstream) {
        // class version number
        mstream.VersionWrite(1);

        // serialize parent class too
        ChLine::StreamOUT(mstream);

        // stream out all member data
        mstream << pA;
        mstream << pB;
    }

    void StreamIN(ChStreamInBinary& mstream) {
        // class version number
        int version = mstream.VersionRead();

        // deserialize parent class too
        ChLine::StreamIN(mstream);

        // stream in all member data
        mstream >> pA;
        mstream >> pB;
    }
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif  // END of header
