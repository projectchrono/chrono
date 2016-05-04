//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHFUNCT_RECORDER_H
#define CHFUNCT_RECORDER_H

//////////////////////////////////////////////////
//
//   ChFunction_Recorder.h
//
//   Function objects,
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Base.h"

namespace chrono {

#define FUNCT_RECORDER 11

class ChApi ChRecPoint {
  public:
    double x;
    double y;
    double w;  // weight

    virtual ~ChRecPoint() = default;

    virtual void ArchiveOUT(ChArchiveOut& marchive) const {
        marchive << CHNVP_OUT(x);
        marchive << CHNVP_OUT(y);
        marchive << CHNVP_OUT(w);
    }

    virtual void ArchiveIN(ChArchiveIn& marchive) {
        marchive >> CHNVP_IN(x);
        marchive >> CHNVP_IN(y);
        marchive >> CHNVP_IN(w);
    }
};

#define CH_RECORDER_EPSILON 1.e-10

/////////////////////////////////////////////
/// RECORDER FUNCTION
/// y = interpolation of array of (x,y) data,
///     where (x,y) points can be inserted randomly.

class ChApi ChFunction_Recorder : public ChFunction {
    CH_RTTI(ChFunction_Recorder, ChFunction);

  private:
    ChList<ChRecPoint> points;     // the list of points
    ChNode<ChRecPoint>* lastnode;  // speed optimization: remember the last used pointer

  public:
    ChFunction_Recorder() { lastnode = NULL; };
    ~ChFunction_Recorder() { points.KillAll(); };
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
    void Copy(ChFunction_Recorder* source);
#pragma GCC diagnostic pop
    ChFunction* new_Duplicate() override;

    int AddPoint(double mx, double my, double mw);
    int AddPoint(double mx, double my) { return AddPoint(mx, my, 1.0); };
    int AddPointClean(double mx, double my, double dx_clean);  // also clean nodes to the right, upt to dx interval
    void Reset() {
        points.KillAll();
        lastnode = NULL;
    };

    ChList<ChRecPoint>* GetPointList() { return &points; };

    double Get_y(double x) override;
    double Get_y_dx(double x) override;
    double Get_y_dxdx(double x) override;

    void Estimate_x_range(double& xmin, double& xmax) override;

    int Get_Type() override { return (FUNCT_RECORDER); }

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) const override
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChFunction::ArchiveOUT(marchive);
        // serialize all member data:
        std::vector< ChRecPoint > tmpvect; // promote to modern array
        for (const ChNode<ChRecPoint>* mnode = points.GetHead(); mnode != NULL; mnode = mnode->next)
        {
            ChRecPoint tmprec; 
            tmprec.x = mnode->data->x;
            tmprec.y = mnode->data->y;
            tmprec.w = mnode->data->w;
            tmpvect.push_back( tmprec );
        }
        marchive << CHNVP_OUT(tmpvect);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override
    {
        // version number
        // int version =
        marchive.VersionRead();
        // deserialize parent class
        ChFunction::ArchiveIN(marchive);
        // stream in all member data:
        std::vector< ChRecPoint > tmpvect; // load from modern array
        marchive >> CHNVP_IN(tmpvect);
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

}  // END_OF_NAMESPACE____

#endif
