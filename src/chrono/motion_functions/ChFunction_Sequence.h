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

#ifndef CHFUNCT_SEQUENCE_H
#define CHFUNCT_SEQUENCE_H

#include <list>
#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// Node for the list of functions in a ChFunction_Sequence object.
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
    void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite<ChFseqNode>();

        // serialize all member data:
        marchive << CHNVP(fx);
        marchive << CHNVP(duration);
        marchive << CHNVP(weight);
        marchive << CHNVP(t_start);
        marchive << CHNVP(t_end);
        marchive << CHNVP(Iy);
        marchive << CHNVP(Iydt);
        marchive << CHNVP(Iydtdt);
        marchive << CHNVP(y_cont);
        marchive << CHNVP(ydt_cont);
        marchive << CHNVP(ydtdt_cont);
    }

    /// Method to allow deserialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead<ChFseqNode>();

        // stream in all member data:
        marchive >> CHNVP(fx);
        marchive >> CHNVP(duration);
        marchive >> CHNVP(weight);
        marchive >> CHNVP(t_start);
        marchive >> CHNVP(t_end);
        marchive >> CHNVP(Iy);
        marchive >> CHNVP(Iydt);
        marchive >> CHNVP(Iydtdt);
        marchive >> CHNVP(y_cont);
        marchive >> CHNVP(ydt_cont);
        marchive >> CHNVP(ydtdt_cont);
    }
};

CH_CLASS_VERSION(ChFseqNode,0)


/// Sequence function:
///   y = sequence_of_functions(f1(y), f2(y), f3(y))
/// All other function types can be inserted into this.
/// This function is very important because very complex motion
/// laws can be created by sequencing many basic ChFunctions.

class ChApi ChFunction_Sequence : public ChFunction {

    CH_FACTORY_TAG(ChFunction_Sequence)

  private:
    std::list<ChFseqNode> functions;  ///< the list of sub functions
    double start;                     ///< start time for sequence

  public:
    ChFunction_Sequence() : start(0) {}
    ChFunction_Sequence(const ChFunction_Sequence& other);
    ~ChFunction_Sequence() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Sequence* Clone() const override { return new ChFunction_Sequence(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_SEQUENCE; }

    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    /// The sequence of functions starts at this x value.
    void Set_start(double m_start) { start = m_start; }
    double Get_start() const { return start; }

    /// Access the list of the sub-functions.
    std::list<ChFseqNode>& Get_list() { return functions; }

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
    /// if you want that Get_weight() will give different results depending on the "x" parameter.
    /// Set c0=true if you want to force C0 continuity with previous function (an offset
    /// will be implicitly added to the function, as y=f(x)+Offset). Same for C1 and C2 continuity,
    /// using c1 and c2 flags.
    bool InsertFunct(std::shared_ptr<ChFunction> myfx,  ///< the function to insert
                     double duration,                   ///< duration of the time segment for this function
                     double weight = 1,                 ///< optional weight scalar
                     bool c0 = false,
                     bool c1 = false,
                     bool c2 = false,     ///< impose continuity to previous f() by offsetting/slanting
                     int position = -1);  ///< position index, 0,1,2,3.. (if -1 insert at the end)

    /// Remove and deletes function with defined "position", and returns true.
    ///	 - If position = 0, removes always head (beginning),
    ///  - If position = -1 removes tail (end).
    ///  - If position > max number of current nodes, removes tail anyway, but returns false.
    bool KillFunct(int position);

    /// Returns the ChFunction with given "position".
    ///  - If position = 0, returns always head (beginning),
    ///  - If position = -1 returns tail (end).
    ///  - If position > max number of current nodes, returns tail fx anyway.
    std::shared_ptr<ChFunction> GetNthFunction(int position);

    /// As above, but returns the function node (containing function pointer,
    /// function duration, continuity flags with previous node, etc.)
    ChFseqNode* GetNthNode(int position);

    /// As above, but returning duration. (return value is reference, so it
    /// can be also changed later, but remember Setup() for the
    /// ChFunction_Sequence after you modified this return value by reference ***TO DO***).
    /// If no function, returns 0.
    double GetNthDuration(int position);

    virtual double Get_weight(double x) const override;

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    virtual int HandleNumber() const override;
    virtual bool HandleAccess(int handle_id, double mx, double my, bool set_mode) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFunction_Sequence>();
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(start);
        marchive << CHNVP(functions);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFunction_Sequence>();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(start);
        marchive >> CHNVP(functions);
    }
};

}  // end namespace chrono

#endif
