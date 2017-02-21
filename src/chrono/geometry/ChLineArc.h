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

#ifndef CHC_LINEARC_H
#define CHC_LINEARC_H

#include <cmath>

#include "chrono/geometry/ChLine.h"

namespace chrono {
namespace geometry {

/// Geometric object representing an arc or a circle in 3D space.
/// By default it is evaluated clockwise from angle1 to angle2.

class ChApi ChLineArc : public ChLine {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLineArc)

  public:
    ChCoordsys<> origin;    ///< center position and plane of the arc: xy used for plane, z for axis.
    double radius;          ///< arc radius
    double angle1;          ///< start angle in radians
    double angle2;          ///< end angle in radians
    bool counterclockwise;  ///< flag indicating arc direction

  public:
    ChLineArc(const ChCoordsys<> morigin = CSYSNULL,
              const double mradius = 1,
              const double mangle1 = CH_C_2PI,
              const double mangle2 = 0,
              const bool mcounterclockwise = false);
    ChLineArc(const ChLineArc& source);
    ~ChLineArc() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineArc* Clone() const override { return new ChLineArc(*this); }

    virtual GeometryType GetClassType() const override { return LINE_ARC; }

    virtual int Get_complexity() const override { return 2; }

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(ChVector<>& pos,
                          const double parU,
                          const double parV = 0.,
                          const double parW = 0.) const override;

    /// Returns curve length. sampling does not matter
    double Length(int sampling) const override { return fabs(radius * (angle1 - angle2)); }

    // Shortcut for setting evaluation direction
    void SetCounterclockwise(bool mcc) { counterclockwise = mcc; }

    // Shortcut for setting angle1 in degrees instead than radians
    void SetAngle1deg(double a1) { angle1 = a1 * CH_C_DEG_TO_RAD; }

    // shortcut for setting angle2 in degrees instead than radians
    void SetAngle2deg(double a2) { angle2 = a2 * CH_C_DEG_TO_RAD; }

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChLineArc>();
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(origin);
        marchive << CHNVP(radius);
        marchive << CHNVP(angle1);
        marchive << CHNVP(angle2);
        marchive << CHNVP(counterclockwise);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChLineArc>();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(origin);
        marchive >> CHNVP(radius);
        marchive >> CHNVP(angle1);
        marchive >> CHNVP(angle2);
        marchive >> CHNVP(counterclockwise);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChLineArc,0)

}  // end namespace chrono

#endif
