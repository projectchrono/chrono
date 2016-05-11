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

///////////////////////////////////////////////////
//
//   ChFunction_Sequence.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChFunction_Sequence.h"
#include "ChFunction_Fillet3.h"
#include "ChFunction_Const.h"

namespace chrono {

ChFseqNode::ChFseqNode() {
    fx = std::make_shared<ChFunction_Const>(0);
    duration = 1;
    weight = 1;
    t_start = 0;
    t_end = t_start + duration;
    Iy = Iydt = Iydtdt = 0.0;
    y_cont = ydt_cont = ydtdt_cont = FALSE;
}

ChFseqNode::ChFseqNode(std::shared_ptr<ChFunction> myfx, double mdur) {
    fx = myfx;
    duration = mdur;
    weight = 1;
    t_start = 0;
    t_end = t_start + duration;
    Iy = Iydt = Iydtdt = 0.0;
    y_cont = ydt_cont = ydtdt_cont = FALSE;
}

ChFseqNode::~ChFseqNode() {
    // no need to delete wrapped function, it is handled with shared pointer
}

void ChFseqNode::Copy(const ChFseqNode* source) {
    fx = source->fx;		//***? shallow copy (now sharing same object)...
    //fx = std::shared_ptr<ChFunction>(source->fx->new_Duplicate());  //***? ..or deep copy? make optional with flag?
    duration = source->duration;
    weight = source->weight;
    t_start = source->t_start;
    t_end = source->t_end;
    Iy = source->Iy;
    Iydt = source->Iydt;
    Iydtdt = source->Iydtdt;
    y_cont = source->y_cont;
    ydt_cont = source->ydt_cont;
    ydtdt_cont = source->ydtdt_cont;
}

void ChFseqNode::SetDuration(double mdur) {
    duration = mdur;
    if (duration < 0)
        duration = 0;
    t_end = t_start + duration;
}


/////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Sequence> a_registration_sequence;

ChFunction_Sequence::ChFunction_Sequence() {
    start = 0;
}

ChFunction_Sequence::~ChFunction_Sequence() {
}

void ChFunction_Sequence::Copy(ChFunction_Sequence* source) {
    start = source->start;
    functions = source->functions;
}

ChFunction* ChFunction_Sequence::new_Duplicate() {
    ChFunction_Sequence* m_func;
    m_func = new ChFunction_Sequence;
    m_func->Copy(this);
    return (m_func);
}

int ChFunction_Sequence::InsertFunct(std::shared_ptr<ChFunction> myfx,
                                     double duration,
                                     double weight,
                                     bool c0,
                                     bool c1,
                                     bool c2,
                                     int position) {
    ChFseqNode mfxsegment(myfx,duration);
    mfxsegment.y_cont = c0;
    mfxsegment.ydt_cont = c1;
    mfxsegment.ydtdt_cont = c2;
    mfxsegment.weight = weight;

    int inserted = FALSE;

    if (position == -1) {
        functions.push_back(mfxsegment);
        inserted = TRUE;
    }
    if (!inserted) {
        std::list< ChFseqNode >::iterator iter;
        size_t i = 0;
        for (iter = functions.begin(); iter != functions.end(); ++iter, ++i){
              if (i == position) {
                functions.insert(iter, mfxsegment);
                inserted = TRUE;
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


int ChFunction_Sequence::KillFunct(int position) {
    if (functions.size() == 0)
        return FALSE;
    if ((position == -1) || (position > functions.size())) {
        functions.erase(functions.end());
        return TRUE;
    }
    if (position == 0) {
        functions.erase(functions.begin());
        return TRUE;
    }
    std::list< ChFseqNode >::iterator iter;
    size_t i = 1;
    for (iter = functions.begin(); iter != functions.end(); ++iter, ++i){
         if (i == position) {
              functions.erase(iter);
              this->Setup();
              return TRUE;
         }
    }

    this->Setup();
    return FALSE;
}

std::shared_ptr<ChFunction> ChFunction_Sequence::GetNthFunction(int position) {
    ChFseqNode* mnode = GetNthNode(position);
    if (mnode)
        return mnode->fx;    
    return std::shared_ptr<ChFunction>();
}

double ChFunction_Sequence::GetNthDuration(int position) {
    ChFseqNode* mnode = GetNthNode(position);
    if (mnode)
        return mnode->duration;    
    return 0.0;
}

ChFseqNode* ChFunction_Sequence::GetNthNode(int position) {
    if (functions.size() == 0)
        return 0;
    if ((position == -1) || (position > functions.size())) {
        return  &(*(functions.end()));
    }
    if (position == 0) {
        return  &(*(functions.begin()));;
    }
    std::list< ChFseqNode >::iterator iter;
    size_t i = 1;
    for (iter = functions.begin(); iter != functions.end(); ++iter, ++i){
         if (i == position) {
              return &(*iter);
         }
    }
    return 0;
}

void ChFunction_Sequence::Setup() {
    double basetime = this->start;
    double lastIy = 0;
    double lastIy_dt = 0;
    double lastIy_dtdt = 0;

    std::list< ChFseqNode >::iterator iter;
    std::list< ChFseqNode >::iterator iter_next;
    for (iter = functions.begin(); iter != functions.end(); ++iter){
        iter->t_start = basetime;
        iter->t_end = basetime + iter->duration;
        iter->Iy = 0;
        iter->Iydt = 0;
        iter->Iydtdt = 0;

        if (auto mfillet = std::dynamic_pointer_cast<ChFunction_Fillet3>(iter->fx)) {
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

        lastIy = iter->fx->Get_y(iter->duration) + iter->Iy +
                 iter->Iydt * iter->duration +
                 iter->Iydtdt * iter->duration * iter->duration;
        lastIy_dt = iter->fx->Get_y_dx(iter->duration) + iter->Iydt +
                    iter->Iydtdt * iter->duration;
        lastIy_dtdt = iter->fx->Get_y_dxdx(iter->duration) + iter->Iydtdt;

        basetime += iter->duration;
    }
}

double ChFunction_Sequence::Get_y(double x) {
    double res = 0;
    double localtime;
    std::list< ChFseqNode >::iterator iter;
    for (iter = functions.begin(); iter != functions.end(); ++iter){
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            localtime = x - iter->t_start;
            res = iter->fx->Get_y(localtime) + iter->Iy + iter->Iydt * localtime +
                  iter->Iydtdt * localtime * localtime;
        }
    }
    return res;
}

double ChFunction_Sequence::Get_y_dx(double x) {
    double res = 0;
    double localtime;
    std::list< ChFseqNode >::iterator iter;
    for (iter = functions.begin(); iter != functions.end(); ++iter){
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            localtime = x - iter->t_start;
            res = iter->fx->Get_y_dx(localtime) + iter->Iydt + iter->Iydtdt * localtime;
        }
    }
    return res;
}

double ChFunction_Sequence::Get_y_dxdx(double x) {
    double res = 0;
    double localtime;
    std::list< ChFseqNode >::iterator iter;
    for (iter = functions.begin(); iter != functions.end(); ++iter){
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            localtime = x - iter->t_start;
            res = iter->fx->Get_y_dxdx(localtime) + iter->Iydtdt;
        }
    }
    return res;
}

double ChFunction_Sequence::Get_weight(double x) {
    double res = 1.0;
    std::list< ChFseqNode >::iterator iter;
    for (iter = functions.begin(); iter != functions.end(); ++iter){
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            res = iter->weight;
        }
    }
    return res;
}

void ChFunction_Sequence::Estimate_x_range(double& xmin, double& xmax) {
    xmin = start;
    xmax = functions.end()->t_end;
    if (xmin == xmax)
        xmax = xmin + 1.1;
}

int ChFunction_Sequence::MakeOptVariableTree(ChList<chjs_propdata>* mtree) {
    int i = 0;

    // inherit parent behaviour
    ChFunction::MakeOptVariableTree(mtree);

    // expand tree for all children..
    int cnt = 1;
    char msubduration[50];
    char msubfunction[50];
    std::list< ChFseqNode >::iterator iter;
    for (iter = functions.begin(); iter != functions.end(); ++iter){
        sprintf(msubduration, "node_n(%d).duration", cnt);

        chjs_propdata* mdataA = new chjs_propdata;
        strcpy(mdataA->propname, msubduration);
        strcpy(mdataA->label, mdataA->propname);
        mdataA->haschildren = FALSE;
        mtree->AddTail(mdataA);
        i++;

        sprintf(msubfunction, "node_n(%d).fx", cnt);

        chjs_propdata* mdataB = new chjs_propdata;
        strcpy(mdataB->propname, msubfunction);
        strcpy(mdataB->label, mdataB->propname);
        mdataB->haschildren = TRUE;
        mtree->AddTail(mdataB);

        i += iter->fx->MakeOptVariableTree(&mdataB->children);

        cnt++;
    }

    return i;
}

int ChFunction_Sequence::HandleNumber() {
    int tot = 1;
    std::list< ChFseqNode >::iterator iter;
    for (iter = functions.begin(); iter != functions.end(); ++iter){
        tot++;
    }
    return tot;
}

int ChFunction_Sequence::HandleAccess(int handle_id, double mx, double my, bool set_mode) {
    if (handle_id == 0) {
        if (!set_mode)
            mx = this->Get_start();
        else
            this->Set_start(mx);
        return TRUE;
    }
    int tot = 1;
    std::list< ChFseqNode >::iterator iter;
    for (iter = functions.begin(); iter != functions.end(); ++iter){
        if (handle_id == tot) {
            if (!set_mode)
                mx = iter->t_end;
            else {
                iter->SetTend(mx);
                this->Setup();
            }
            return TRUE;
        }
    }

    return FALSE;
}


}  // END_OF_NAMESPACE____

// eof
