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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFUNCT_SEQUENCE_H
#define CHFUNCT_SEQUENCE_H

#include <list>
#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Node for the list of functions in a ChFunctionSequence object.
class ChApi ChFseqNode {
  public:
    std::shared_ptr<ChFunction> fx;
    double duration;
    double weight;
    double t_start;
    double t_end;
    double Iy;
    double Iydt;
    double Iydtdt;
    bool y_cont;
    bool ydt_cont;
    bool ydtdt_cont;

    void SetDuration(double mdur);
    void SetTend(double mt_end) {
        t_end = mt_end;
        if (t_end < t_start)
            t_end = t_start;
        duration = t_end - t_start;
    }

    ChFseqNode();
    ChFseqNode(std::shared_ptr<ChFunction> myfx, double mdur);
    ChFseqNode(const ChFseqNode& other);
    ~ChFseqNode() {}

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);
};

CH_CLASS_VERSION(ChFseqNode, 0)

/// Sequence function:
///   `y = sequence_of_functions(f1(y), f2(y), f3(y))`
/// All other function types can be inserted into this.
class ChApi ChFunctionSequence : public ChFunction {
public:
    ChFunctionSequence() : m_start(0) {}
    ChFunctionSequence(const ChFunctionSequence& other);
    ~ChFunctionSequence() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionSequence* Clone() const override { return new ChFunctionSequence(*this); }

    virtual Type GetType() const override { return ChFunction::Type::SEQUENCE; }

    virtual double GetVal(double x) const override;
    virtual double GetDer(double x) const override;
    virtual double GetDer2(double x) const override;

    /// The sequence of functions starts at this x value.
    void SetStartArg(double start) { m_start = start; }

    /// The sequence of functions starts at this x value.
    double GetStartArg() const { return m_start; }

    /// Access the list of the sub-functions.
    std::list<ChFseqNode>& GetFunctions() { return m_functions; }

    /// Scans all the seq.of functions and setup the timings and continuity
    /// offsets, to satisfy all constraints.
    /// This must be called whenever a new function is inserted, or its
    /// timing and continuity constraints are changed.
    void Setup();

    /// Insert function after the fx with defined "position" index in list.
    ///  - If index is higher than available objects, it simply goes to the end.
    ///  - If index = 0 insert at the beginning,
    ///  - If index = -1 insert at the end.
    /// Inserted functions will be deleted automatically when this object will be deleted.
    /// The fx segment has its own 'weight': use 1.0 for default, or different weights
    /// if you want that GetWeight() will give different results depending on the "x" parameter.
    /// Set c0=true if you want to force C0 continuity with previous function (an offset
    /// will be implicitly added to the function, as y=f(x)+Offset). Same for C1 and C2 continuity,
    /// using c1 and c2 flags.
    bool InsertFunct(std::shared_ptr<ChFunction> myfx,  ///< the function to insert
                     double duration,                   ///< duration of the time segment for this function
                     double weight = 1,                 ///< optional weight scalar
                     bool c0 = false,                   ///< impose continuity to previous f() by offsetting/slanting
                     bool c1 = false,                   ///< impose continuity to previous f() by offsetting/slanting
                     bool c2 = false,                   ///< impose continuity to previous f() by offsetting/slanting
                     int position = -1                  ///< position index, 0,1,2,3.. (if -1 insert at the end)
    );

    /// Remove and deletes function with defined "position", and returns true.
    ///	 - If index = 0, removes always head (beginning),
    ///  - If index = -1 removes tail (end).
    ///  - If index > max number of current nodes, removes tail anyway, but returns false.
    bool RemoveFunct(int index);

    /// Returns the ChFunction with given "index".
    ///  - If index = 0, returns always head (beginning),
    ///  - If index = -1 returns tail (end).
    ///  - If index > max number of current nodes, returns tail fx anyway.
    std::shared_ptr<ChFunction> GetFunction(int index);

    /// As above, but returns the function node (containing function pointer,
    /// function duration, continuity flags with previous node, etc.)
    ChFseqNode* GetNode(int index);

    /// As above, but returning duration. (return value is reference, so it
    /// can be also changed later, but remember Setup() for the
    /// ChFunctionSequence after you modified this return value by reference ***TO DO***).
    /// If no function, returns 0.
    double GetWidth(int index);

    virtual double GetWeight(double x) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

protected:
    std::list<ChFseqNode> m_functions;  ///< the list of sub functions
    double m_start;                     ///< start time for sequence
};

/// @} chrono_functions

}  // end namespace chrono

#endif
