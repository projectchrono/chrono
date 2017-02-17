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

#ifndef CHFUNCT_RECORDER_H
#define CHFUNCT_RECORDER_H

#include <list>
#include <iterator>

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

class ChApi ChRecPoint {
  public:
    double x;  ///< argument value
    double y;  ///< function value
    double w;  ///< weight

    ChRecPoint() {}
    ChRecPoint(double mx, double my, double mw) : x(mx), y(my), w(mw) {}

    void ArchiveOUT(ChArchiveOut& marchive) {
        marchive << CHNVP(x);
        marchive << CHNVP(y);
        marchive << CHNVP(w);
    }

    void ArchiveIN(ChArchiveIn& marchive) {
        marchive >> CHNVP(x);
        marchive >> CHNVP(y);
        marchive >> CHNVP(w);
    }
};

/// Recorder function:
///
/// y = interpolation of array of (x,y) data,
///     where (x,y) points can be inserted randomly.

class ChApi ChFunction_Recorder : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Recorder)

  private:
    std::list<ChRecPoint> m_points;  ///< the list of points
    mutable std::list<ChRecPoint>::const_iterator m_last;

  public:
    ChFunction_Recorder() : m_last(m_points.end()) {}
    ChFunction_Recorder(const ChFunction_Recorder& other);
    ~ChFunction_Recorder() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Recorder* Clone() const override { return new ChFunction_Recorder(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_RECORDER; }

    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    void AddPoint(double mx, double my, double mw = 1);

    void Reset() {
        m_points.clear();
        m_last = m_points.end();
    }

    const std::list<ChRecPoint>& GetPoints() { return m_points; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Recorder>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data: copy to vector and store
        std::vector<ChRecPoint> tmpvect{std::begin(m_points), std::end(m_points)};
        marchive << CHNVP(tmpvect);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Recorder>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data: load vector of points and copy to list
        std::vector<ChRecPoint> tmpvect;
        marchive >> CHNVP(tmpvect);
        m_points.clear();
        std::copy(tmpvect.begin(), tmpvect.end(), std::back_inserter(m_points));
    }
};

CH_CLASS_VERSION(ChFunction_Recorder,0)

}  // end namespace chrono

#endif
