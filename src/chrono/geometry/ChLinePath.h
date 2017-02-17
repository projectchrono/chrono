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

#ifndef CHC_LINEPATH_H
#define CHC_LINEPATH_H

#include <cmath>

#include "chrono/geometry/ChLine.h"

namespace chrono {
namespace geometry {

/// Geometric object representing an sequence of other ChLine objects,
/// The ChLine objects are assumed to be properly concatenated and to have C0 continuity.

class ChApi ChLinePath : public ChLine {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinePath)

  public:
    std::vector<std::shared_ptr<ChLine> > lines;
    std::vector<double> end_times;
    std::vector<double> durations;

  public:
    ChLinePath() {}
    ChLinePath(const ChLinePath& source);
    ~ChLinePath() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinePath* Clone() const override { return new ChLinePath(*this); }

    virtual GeometryType GetClassType() const override { return LINE_PATH; }

    virtual int Get_complexity() { return 2; }

    /// Return curve length.
    /// Sampling does not matter.
    virtual double Length(int sampling) const override;

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(ChVector<>& pos,
                          const double parU,
                          const double parV = 0.,
                          const double parW = 0.) const override;

    /// Return the start point of the line.
    virtual ChVector<> GetEndA() const override { return (lines.front())->GetEndA(); }

    /// Return the end point of the line.
    virtual ChVector<> GetEndB() const override { return (lines.back())->GetEndB(); }

    /// Get count of sub-lines that have been added:
    size_t GetSubLinesCount() { return lines.size(); }

    /// Access the nth line
    std::shared_ptr<ChLine> GetSubLineN(size_t n) { return lines[n]; }

    /// Get the nth line duration
    double GetSubLineDurationN(size_t n) { return durations[n]; }

    /// Set the nth line duration
    void SetSubLineDurationN(size_t n, double mduration);

    /// Queue a line (push it back to the array of lines)
    void AddSubLine(std::shared_ptr<ChLine> mline,  ///< line to add
                    double duration = 1             ///< duration of the abscyssa when calling the Evaluate() function
                    );

    /// Queue a line (push it back to the array of lines)
    void AddSubLine(ChLine& mline,       ///< line to add
                    double duration = 1  ///< duration of the abscyssa when calling the Evaluate() function
                    );

    /// Insert a line at a specified index  n  in line array.
    /// Note that  n  cannot be higher than GetLineCount().
    void InsertSubLine(size_t n,  ///< index of line, 0 is first, etc.
                       std::shared_ptr<ChLine>
                           mline,           ///< line to add
                       double duration = 1  ///< duration of the abscyssa when calling the Evaluate() function
                       );

    /// Insert a line at a specified index  n  in line array.
    /// Note that  n  cannot be higher than GetLineCount().
    void InsertSubLine(size_t n,            ///< index of line, 0 is first, etc.
                       ChLine& mline,       ///< line to add
                       double duration = 1  ///< duration of the abscyssa when calling the Evaluate() function
                       );

    /// Erase a line from a specified index  n  in line array.
    /// Note that  n  cannot be higher than GetLineCount().
    void EraseSubLine(size_t n  //<<< index of line, 0 is first, etc.
                      );

    /// Tells the duration of the path, sum of the durations of all sub-lines.
    /// This is useful because ifyou use the Evaluate() function on the path, the U
    /// parameter should range between 0 and the max duration.
    double GetPathDuration() const;

    /// Shrink or stretch all the durations of the sub-lines so that the
    /// total duration of the path is equal to a specified value.
    /// For example, you can normalize to 1 so you can use Evaluate() with U in
    /// the 0..1 range like with other lines.
    void SetPathDuration(double mUduration);

    /// Check if the path is topologically connected,
    /// i.e. if all the sub lines are queued to have C0 continuity
    double GetContinuityMaxError() const;

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChLinePath>();
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(lines);
        marchive << CHNVP(end_times);
        marchive << CHNVP(durations);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChLinePath>();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(lines);
        marchive >> CHNVP(end_times);
        marchive >> CHNVP(durations);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChLinePath,0)

}  // end namespace chrono

#endif