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

#include "chrono/core/ChLists.h"
#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

class ChApi ChRecPoint {
  public:
    double x;
    double y;
    double w;  ///< weight

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

#define CH_RECORDER_EPSILON 1.e-10

/// Recorder function:
///
/// y = interpolation of array of (x,y) data,
///     where (x,y) points can be inserted randomly.

class ChApi ChFunction_Recorder : public ChFunction {
    CH_RTTI(ChFunction_Recorder, ChFunction);

  private:
    mutable ChList<ChRecPoint> points;     ///< the list of points
    mutable ChNode<ChRecPoint>* lastnode;  ///< speed optimization: remember the last used pointer

  public:
    ChFunction_Recorder() : lastnode(NULL) {}
    ChFunction_Recorder(const ChFunction_Recorder& other);
    ~ChFunction_Recorder() { points.KillAll(); }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Recorder* Clone() const override { return new ChFunction_Recorder(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_RECORDER; }

    virtual double Get_y(double x) const override;
    virtual double Get_y_dx(double x) const override;
    virtual double Get_y_dxdx(double x) const override;

    int AddPoint(double mx, double my, double mw);
    int AddPoint(double mx, double my) { return AddPoint(mx, my, 1.0); }
    int AddPointClean(double mx, double my, double dx_clean);  // also clean nodes to the right, upt to dx interval
 
    void Reset() {
        points.KillAll();
        lastnode = NULL;
    }

    ChList<ChRecPoint>* GetPointList() { return &points; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        std::vector<ChRecPoint> tmpvect;  // promote to modern array
        for (ChNode<ChRecPoint>* mnode = points.GetHead(); mnode != NULL; mnode = mnode->next) {
            ChRecPoint tmprec;
            tmprec.x = mnode->data->x;
            tmprec.y = mnode->data->y;
            tmprec.w = mnode->data->w;
            tmpvect.push_back(tmprec);
        }
        marchive << CHNVP(tmpvect);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        std::vector<ChRecPoint> tmpvect;  // load from modern array
        marchive >> CHNVP(tmpvect);
        points.KillAll();
        for (int i = 0; i < tmpvect.size(); i++) {
            ChRecPoint* mpt = new ChRecPoint;
            mpt->x = tmpvect[i].x;
            mpt->y = tmpvect[i].y;
            mpt->w = tmpvect[i].w;
            points.AddTail(mpt);
        }
    }
};

}  // end namespace chrono

#endif
