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

#ifndef CHC_LINEARC_H
#define CHC_LINEARC_H

#include <math.h>

#include "ChCLine.h"

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_LINEARC 19

///
/// ARC
///
/// Geometric object representing an arc or a circle in 3D space.
///

class ChApi ChLineArc : public ChLine {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChLineArc, ChLine);

  public:
    //
    // DATA
    //

    ChCoordsys<> origin;  // center position and plane of the arc: xy used for plane, z for axis.
    double radius;
    double angle1;  // in radians
    double angle2;  // in radians

  public:
    //
    // CONSTRUCTORS
    //

    // - Creation by default.
    // - Creation by origin coordsystem, radius, two angles (if two angles not specified, is a full circle)
    ChLineArc(const ChCoordsys<> morigin = CSYSNULL,
              const double mradius = 1,
              const double mangle1 = 0,
              const double mangle2 = CH_C_2PI) {
        origin = morigin;
        radius = mradius;
        angle1 = mangle1;
        angle2 = mangle2;
    }

    ~ChLineArc(){};

    ChLineArc(const ChLineArc& source) {
        origin = source.origin;
        radius = source.radius;
        angle1 = source.angle1;
        angle2 = source.angle2;
    }

    void Copy(const ChLineArc* source) {
        ChLine::Copy(source);
        origin = source->origin;
        radius = source->radius;
        angle1 = source->angle1;
        angle2 = source->angle2;
    }

    ChGeometry* Duplicate() { return new ChLineArc(*this); };

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() { return CH_GEOCLASS_LINEARC; };

    virtual int Get_complexity() { return 2; };

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(Vector& pos, const double parU, const double parV = 0., const double parW = 0.) {
        double mangle = angle1 * (1 - parU) + angle2 * (parU);
        ChVector<> localP(radius * cos(mangle), radius * sin(mangle), 0);
        pos = localP >> origin;  // translform to absolute coordinates
    }

    /// Returns curve length. sampling does not matter
    double Length(int sampling) { return fabs(radius * (angle1 - angle2)); }

    //
    // CUSTOM FUNCTIONS
    //

    // shortcut for setting angle1 in degrees instead than radians
    void SetAngle1deg(double a1) { angle1 = a1 * CH_C_DEG_TO_RAD; }

    // shortcut for setting angle2 in degrees instead than radians
    void SetAngle2deg(double a2) { angle2 = a2 * CH_C_DEG_TO_RAD; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(origin);
        marchive << CHNVP(radius);
        marchive << CHNVP(angle1);
        marchive << CHNVP(angle2);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(origin);
        marchive >> CHNVP(radius);
        marchive >> CHNVP(angle1);
        marchive >> CHNVP(angle2);
    }

    //***OBSOLETE***
    void StreamOUT(ChStreamOutBinary& mstream) {
        // class version number
        mstream.VersionWrite(1);

        // serialize parent class too
        ChLine::StreamOUT(mstream);

        // stream out all member data
        mstream << origin;
        mstream << radius;
        mstream << angle1;
        mstream << angle2;
    }

    //***OBSOLETE***
    void StreamIN(ChStreamInBinary& mstream) {
        // class version number
        int version = mstream.VersionRead();

        // deserialize parent class too
        ChLine::StreamIN(mstream);

        // stream in all member data
        mstream >> origin;
        mstream >> radius;
        mstream >> angle1;
        mstream >> angle2;
    }
};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif  // END of header
