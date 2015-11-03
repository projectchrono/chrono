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

#ifndef CHC_LINEPOLY_H
#define CHC_LINEPOLY_H


#include <math.h>

#include "ChCLine.h"

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_LINEPOLY 5

///
/// POLY LINE
///
/// Geometric object representing a line in 3D space, which
/// is controlled by control points.
///

class ChApi ChLinePoly : public ChLine {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChLinePoly, ChLine);

  private:
    //
    // DATA
    //

    std::vector< ChVector<> > points;  // control points
    int degree;

  public:
    //
    // CONSTRUCTORS
    //

    ChLinePoly(int mnumpoints = 1);

    ~ChLinePoly();

    ChLinePoly(const ChLinePoly& source) {
        Copy(&source);
    }

    void Copy(const ChLinePoly* source);

    ChGeometry* Duplicate() {
        ChGeometry* mgeo = new ChLinePoly(*this);
        return mgeo;
    };

    //
    // OVERRIDE BASE CLASS FUNCTIONS
    //

    virtual int GetClassType() { return CH_GEOCLASS_LINEPOLY; };

    virtual bool Get_closed();
    virtual void Set_closed(bool mc);

    virtual int Get_complexity() { return (int)points.size(); };
    virtual void Set_complexity(int mc){};

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(Vector& pos, const double parU, const double parV = 0., const double parW = 0.);

    /// Returns curve length. sampling does not matter
    double Length(int sampling);

    /// Draw into the current graph viewport of a ChFile_ps file
    int DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate);

    //
    // CUSTOM FUNCTIONS
    //

    /// Gets the number of control points
    virtual size_t Get_numpoints();

    /// Get the degree of the curve (1= linear,
    /// 2= quadric, 3= cubic, etc.)
    virtual int Get_degree();

    /// Get the n-th control point
    virtual Vector Get_point(size_t mnum);

    /// Set the n-th control point
    virtual int Set_point(int mnum, Vector mpoint);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(points);
        marchive << CHNVP(degree);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(points);
        marchive >> CHNVP(degree);
    }


};

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____

#endif  // END of header
