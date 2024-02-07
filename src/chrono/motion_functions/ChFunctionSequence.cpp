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

#include "chrono/motion_functions/ChFunctionSequence.h"
#include "chrono/motion_functions/ChFunctionConst.h"
#include "chrono/motion_functions/ChFunctionFillet3.h"

namespace chrono {

ChFseqNode::ChFseqNode() {
    fx = chrono_types::make_shared<ChFunctionConst>(0);
    duration = 1;
    weight = 1;
    t_start = 0;
    t_end = t_start + duration;
    Iy = Iydt = Iydtdt = 0.0;
    y_cont = ydt_cont = ydtdt_cont = false;
}

ChFseqNode::ChFseqNode(std::shared_ptr<ChFunction> myfx, double mdur) {
    fx = myfx;
    duration = mdur;
    weight = 1;
    t_start = 0;
    t_end = t_start + duration;
    Iy = Iydt = Iydtdt = 0.0;
    y_cont = ydt_cont = ydtdt_cont = false;
}

ChFseqNode::ChFseqNode(const ChFseqNode& other) {
    fx = other.fx;
    duration = other.duration;
    weight = other.weight;
    t_start = other.t_start;
    t_end = other.t_end;
    Iy = other.Iy;
    Iydt = other.Iydt;
    Iydtdt = other.Iydtdt;
    y_cont = other.y_cont;
    ydt_cont = other.ydt_cont;
    ydtdt_cont = other.ydtdt_cont;
}

void ChFseqNode::SetDuration(double mdur) {
    duration = mdur;
    if (duration < 0)
        duration = 0;
    t_end = t_start + duration;
}

void ChFseqNode::ArchiveOut(ChArchiveOut& marchive) {
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

void ChFseqNode::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFseqNode>();

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

// -------------------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionSequence)

ChFunctionSequence::ChFunctionSequence(const ChFunctionSequence& other) {
    start = other.start;
    functions = other.functions;
}

bool ChFunctionSequence::InsertFunct(std::shared_ptr<ChFunction> myfx,
                                      double duration,
                                      double weight,
                                      bool c0,
                                      bool c1,
                                      bool c2,
                                      int position) {
    ChFseqNode mfxsegment(myfx, duration);
    mfxsegment.y_cont = c0;
    mfxsegment.ydt_cont = c1;
    mfxsegment.ydtdt_cont = c2;
    mfxsegment.weight = weight;

    bool inserted = false;

    if (position == -1) {
        functions.push_back(mfxsegment);
        inserted = true;
    }
    if (!inserted) {
        std::list<ChFseqNode>::iterator iter;
        size_t i = 0;
        for (iter = functions.begin(); iter != functions.end(); ++iter, ++i) {
            if (i == position) {
                functions.insert(iter, mfxsegment);
                inserted = true;
                break;
            }
        }
    }
    if (!inserted) {
        functions.push_back(mfxsegment);
    }
    // update the continuity offsets and timings
    this->Setup();
    return inserted;
}

bool ChFunctionSequence::KillFunct(int position) {
    if (functions.size() == 0)
        return false;
    if ((position == -1) || (position > functions.size())) {
        functions.erase(functions.end());
        return true;
    }
    if (position == 0) {
        functions.erase(functions.begin());
        return true;
    }
    std::list<ChFseqNode>::iterator iter;
    size_t i = 1;
    for (iter = functions.begin(); iter != functions.end(); ++iter, ++i) {
        if (i == position) {
            functions.erase(iter);
            this->Setup();
            return true;
        }
    }

    this->Setup();
    return false;
}

std::shared_ptr<ChFunction> ChFunctionSequence::GetNthFunction(int position) {
    ChFseqNode* mnode = GetNthNode(position);
    if (mnode)
        return mnode->fx;
    return std::shared_ptr<ChFunction>();
}

double ChFunctionSequence::GetNthDuration(int position) {
    ChFseqNode* mnode = GetNthNode(position);
    if (mnode)
        return mnode->duration;
    return 0.0;
}

ChFseqNode* ChFunctionSequence::GetNthNode(int position) {
    if (functions.size() == 0)
        return 0;
    if ((position == -1) || (position > functions.size())) {
        return &(*(functions.end()));
    }
    if (position == 0) {
        return &(*(functions.begin()));
    }
    size_t i = 1;
    for (auto iter = functions.begin(); iter != functions.end(); ++iter, ++i) {
        if (i == position) {
            return &(*iter);
        }
    }
    return 0;
}

void ChFunctionSequence::Setup() {
    double basetime = this->start;
    double lastIy = 0;
    double lastIy_dt = 0;
    double lastIy_dtdt = 0;

    std::list<ChFseqNode>::iterator iter_next;
    for (auto iter = functions.begin(); iter != functions.end(); ++iter) {
        iter->t_start = basetime;
        iter->t_end = basetime + iter->duration;
        iter->Iy = 0;
        iter->Iydt = 0;
        iter->Iydtdt = 0;

        if (auto mfillet = std::dynamic_pointer_cast<ChFunctionFillet3>(iter->fx)) {
            mfillet->Set_y1(lastIy);
            mfillet->Set_dy1(lastIy_dt);

            iter_next = iter;
            ++iter_next;
            if (iter_next != functions.end()) {
                mfillet->Set_y2(iter_next->fx->Get_y(0));
                mfillet->Set_dy2(iter_next->fx->Get_y_dx(0));
            } else {
                mfillet->Set_y2(0);
                mfillet->Set_dy2(0);
            }
            mfillet->Set_end(iter->duration);
            iter->Iy = iter->Iydt = iter->Iydtdt = 0;
        } else  // generic continuity conditions
        {
            if (iter->y_cont)
                iter->Iy = lastIy - iter->fx->Get_y(0);
            if (iter->ydt_cont)
                iter->Iydt = lastIy_dt - iter->fx->Get_y_dx(0);
            if (iter->ydtdt_cont)
                iter->Iydtdt = lastIy_dtdt - iter->fx->Get_y_dxdx(0);
        }

        lastIy = iter->fx->Get_y(iter->duration) + iter->Iy + iter->Iydt * iter->duration +
                 iter->Iydtdt * iter->duration * iter->duration;
        lastIy_dt = iter->fx->Get_y_dx(iter->duration) + iter->Iydt + iter->Iydtdt * iter->duration;
        lastIy_dtdt = iter->fx->Get_y_dxdx(iter->duration) + iter->Iydtdt;

        basetime += iter->duration;
    }
}

double ChFunctionSequence::Get_y(double x) const {
    double res = 0;
    double localtime;
    for (auto iter = functions.begin(); iter != functions.end(); ++iter) {
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            localtime = x - iter->t_start;
            res = iter->fx->Get_y(localtime) + iter->Iy + iter->Iydt * localtime + iter->Iydtdt * localtime * localtime;
        }
    }
    return res;
}

double ChFunctionSequence::Get_y_dx(double x) const {
    double res = 0;
    double localtime;
    for (auto iter = functions.begin(); iter != functions.end(); ++iter) {
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            localtime = x - iter->t_start;
            res = iter->fx->Get_y_dx(localtime) + iter->Iydt + iter->Iydtdt * localtime;
        }
    }
    return res;
}

double ChFunctionSequence::Get_y_dxdx(double x) const {
    double res = 0;
    double localtime;
    for (auto iter = functions.begin(); iter != functions.end(); ++iter) {
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            localtime = x - iter->t_start;
            res = iter->fx->Get_y_dxdx(localtime) + iter->Iydtdt;
        }
    }
    return res;
}

double ChFunctionSequence::Get_weight(double x) const {
    double res = 1.0;
    for (auto iter = functions.begin(); iter != functions.end(); ++iter) {
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            res = iter->weight;
        }
    }
    return res;
}

void ChFunctionSequence::Estimate_x_range(double& xmin, double& xmax) const {
    xmin = start;
    xmax = functions.end()->t_end;
    if (xmin == xmax)
        xmax = xmin + 1.1;
}

int ChFunctionSequence::HandleNumber() const {
    int tot = 1;
    for (auto iter = functions.begin(); iter != functions.end(); ++iter) {
        tot++;
    }
    return tot;
}

bool ChFunctionSequence::HandleAccess(int handle_id, double mx, double my, bool set_mode) {
    if (handle_id == 0) {
        if (!set_mode)
            mx = Get_start();
        else
            Set_start(mx);
        return true;
    }
    int tot = 1;
    for (auto iter = functions.begin(); iter != functions.end(); ++iter) {
        if (handle_id == tot) {
            if (!set_mode)
                mx = iter->t_end;
            else {
                iter->SetTend(mx);
                Setup();
            }
            return true;
        }
    }

    return false;
}

void ChFunctionSequence::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionSequence>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(start);
    marchive << CHNVP(functions);
}

void ChFunctionSequence::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionSequence>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(start);
    marchive >> CHNVP(functions);
}

}  // end namespace chrono
