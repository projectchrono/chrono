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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_LINE_H
#define CHC_LINE_H

#include <cmath>

#include "chrono/geometry/ChGeometry.h"
#include "chrono/core/ChFilePS.h"

namespace chrono {
namespace geometry {

/// Base class for all geometric objects representing lines in 3D space.

class ChApi ChLine : public ChGeometry {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLine)

  protected:
    bool closed;
    int complexityU;

  public:
    ChLine() : closed(false), complexityU(2) {}
    ChLine(const ChLine& source);
    virtual ~ChLine() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLine* Clone() const override { return new ChLine(*this); }

    /// Get the class type as unique numerical ID (faster
    /// than using ChronoRTTI mechanism).
    /// Each inherited class must return an unique ID.
    virtual GeometryType GetClassType() const override { return LINE; }

    /// Tell if the curve is closed
    virtual bool Get_closed() const { return closed; }
    virtual void Set_closed(bool mc) { closed = mc; }

    /// Tell the complexity
    virtual int Get_complexity() const { return complexityU; }
    virtual void Set_complexity(int mc) { complexityU = mc; }

    /// This is a line
    virtual int GetManifoldDimension() const override { return 1; }

    /// Find the parameter resU for the nearest point on curve to "point".
    bool FindNearestLinePoint(ChVector<>& point, double& resU, double approxU, double tol) const;

    /// Returns curve length. Typical sampling 1..5 (1 already gives correct result with degree1 curves)
    virtual double Length(int sampling) const;

    /// Return the start point of the line.
    /// By default, evaluates line at U=0.
    virtual ChVector<> GetEndA() const {
        ChVector<> pos;
        Evaluate(pos, 0, 0, 0);
        return pos;
    }

    /// Return the end point of the line.
    /// By default, evaluates line at U=1.
    virtual ChVector<> GetEndB() const {
        ChVector<> pos;
        Evaluate(pos, 1, 0, 0);
        return pos;
    }

    /// Returns adimensional information on "how much" this curve is similar to another
    /// in its overall shape (doesnot matter parametrization or start point). Try with 20 samples.
    /// The return value is somewhat the "average distance between the two curves".
    /// Note that the result is affected by "weight" of curves. If it chnges from default 1.0, the
    /// distance extimation is higher/lower (ex: if a curve defines low 'weight' in its central segment,
    /// its CurveCurveDistance from another segment is not much affected by errors near the central segment).
    double CurveCurveDist(ChLine* compline, int samples) const;

    /// Same as before, but returns "how near" is complinesegm to
    /// whatever segment of this line (does not matter the percentual of line).
    /// Again, this is affected by "weight" of curves. If weight changes along curves ->'weighted' distance
    double CurveSegmentDist(ChLine* complinesegm, int samples) const;

    /// Same as above, but instead of making average of the distances,
    /// these functions return the maximum of the distances...
    double CurveCurveDistMax(ChLine* compline, int samples) const;
    double CurveSegmentDistMax(ChLine* complinesegm, int samples) const;

    /// Draw into the current graph viewport of a ChFile_ps file
    virtual bool DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate);

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChLine>();
        // serialize parent class
        ChGeometry::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(closed);
        marchive << CHNVP(complexityU);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChLine>();
        // deserialize parent class
        ChGeometry::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(closed);
        marchive >> CHNVP(complexityU);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChLine,0)

}  // end namespace chrono

#endif
