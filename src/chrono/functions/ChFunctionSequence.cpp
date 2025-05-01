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

#include "chrono/functions/ChFunctionSequence.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionFillet3.h"

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

void ChFseqNode::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFseqNode>();

    // serialize all member data:
    archive_out << CHNVP(fx);
    archive_out << CHNVP(duration);
    archive_out << CHNVP(weight);
    archive_out << CHNVP(t_start);
    archive_out << CHNVP(t_end);
    archive_out << CHNVP(Iy);
    archive_out << CHNVP(Iydt);
    archive_out << CHNVP(Iydtdt);
    archive_out << CHNVP(y_cont);
    archive_out << CHNVP(ydt_cont);
    archive_out << CHNVP(ydtdt_cont);
}

void ChFseqNode::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFseqNode>();

    // stream in all member data:
    archive_in >> CHNVP(fx);
    archive_in >> CHNVP(duration);
    archive_in >> CHNVP(weight);
    archive_in >> CHNVP(t_start);
    archive_in >> CHNVP(t_end);
    archive_in >> CHNVP(Iy);
    archive_in >> CHNVP(Iydt);
    archive_in >> CHNVP(Iydtdt);
    archive_in >> CHNVP(y_cont);
    archive_in >> CHNVP(ydt_cont);
    archive_in >> CHNVP(ydtdt_cont);
}

// -------------------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionSequence)

ChFunctionSequence::ChFunctionSequence() : m_start(0), m_clamped(false) {}

ChFunctionSequence::ChFunctionSequence(const ChFunctionSequence& other) {
    m_start = other.m_start;
    m_functions = other.m_functions;
}

bool ChFunctionSequence::InsertFunct(std::shared_ptr<ChFunction> myfx,
                                     double duration,
                                     double weight,
                                     bool c0,
                                     bool c1,
                                     bool c2,
                                     int index) {
    ChFseqNode mfxsegment(myfx, duration);
    mfxsegment.y_cont = c0;
    mfxsegment.ydt_cont = c1;
    mfxsegment.ydtdt_cont = c2;
    mfxsegment.weight = weight;

    bool inserted = false;

    if (index == -1) {
        m_functions.push_back(mfxsegment);
        inserted = true;
    }
    if (!inserted) {
        std::list<ChFseqNode>::iterator iter;
        size_t i = 0;
        for (iter = m_functions.begin(); iter != m_functions.end(); ++iter, ++i) {
            if (i == index) {
                m_functions.insert(iter, mfxsegment);
                inserted = true;
                break;
            }
        }
    }
    if (!inserted) {
        m_functions.push_back(mfxsegment);
    }
    // update the continuity offsets and timings
    this->Setup();
    return inserted;
}

bool ChFunctionSequence::RemoveFunct(int index) {
    if (m_functions.size() == 0)
        return false;
    if ((index == -1) || (index > m_functions.size())) {
        m_functions.pop_back();
        return true;
    }
    if (index == 0) {
        m_functions.erase(m_functions.begin());
        return true;
    }
    std::list<ChFseqNode>::iterator iter;
    size_t i = 1;
    for (iter = m_functions.begin(); iter != m_functions.end(); ++iter, ++i) {
        if (i == index) {
            m_functions.erase(iter);
            this->Setup();
            return true;
        }
    }

    this->Setup();
    return false;
}

std::shared_ptr<ChFunction> ChFunctionSequence::GetFunction(int index) {
    ChFseqNode* mnode = GetNode(index);
    if (mnode)
        return mnode->fx;
    return std::shared_ptr<ChFunction>();
}

double ChFunctionSequence::GetWidth(int index) {
    ChFseqNode* mnode = GetNode(index);
    if (mnode)
        return mnode->duration;
    return 0.0;
}

ChFseqNode* ChFunctionSequence::GetNode(int index) {
    if (m_functions.size() == 0)
        return 0;
    if ((index == -1) || (index > m_functions.size())) {
        return &(*(m_functions.end()));
    }
    if (index == 0) {
        return &(*(m_functions.begin()));
    }
    size_t i = 1;
    for (auto iter = m_functions.begin(); iter != m_functions.end(); ++iter, ++i) {
        if (i == index) {
            return &(*iter);
        }
    }
    return 0;
}

void ChFunctionSequence::Setup() {
    double basetime = this->m_start;
    double lastIy = 0;
    double lastIy_dt = 0;
    double lastIy_dtdt = 0;

    std::list<ChFseqNode>::iterator iter_next;
    for (auto iter = m_functions.begin(); iter != m_functions.end(); ++iter) {
        iter->t_start = basetime;
        iter->t_end = basetime + iter->duration;
        iter->Iy = 0;
        iter->Iydt = 0;
        iter->Iydtdt = 0;

        if (auto mfillet = std::dynamic_pointer_cast<ChFunctionFillet3>(iter->fx)) {
            mfillet->SetStartVal(lastIy);
            mfillet->SetStartDer(lastIy_dt);

            iter_next = iter;
            ++iter_next;
            if (iter_next != m_functions.end()) {
                mfillet->SetEndVal(iter_next->fx->GetVal(0));
                mfillet->SetEndDer(iter_next->fx->GetDer(0));
            } else {
                mfillet->SetEndVal(0);
                mfillet->SetEndDer(0);
            }
            mfillet->SetWidth(iter->duration);
            mfillet->Setup();
            iter->Iy = iter->Iydt = iter->Iydtdt = 0;
        } else  // generic continuity conditions
        {
            if (iter->y_cont)
                iter->Iy = lastIy - iter->fx->GetVal(0);
            if (iter->ydt_cont)
                iter->Iydt = lastIy_dt - iter->fx->GetDer(0);
            if (iter->ydtdt_cont)
                iter->Iydtdt = lastIy_dtdt - iter->fx->GetDer2(0);
        }

        lastIy = iter->fx->GetVal(iter->duration) + iter->Iy + iter->Iydt * iter->duration +
                 iter->Iydtdt * iter->duration * iter->duration;
        lastIy_dt = iter->fx->GetDer(iter->duration) + iter->Iydt + iter->Iydtdt * iter->duration;
        lastIy_dtdt = iter->fx->GetDer2(iter->duration) + iter->Iydtdt;

        basetime += iter->duration;
    }
}

double ChFunctionSequence::GetVal(double x) const {
    double res = 0;
    double localtime = 0;

    if (m_clamped && x >= m_functions.back().t_end) {  // clamp to last node value after end
        auto last_node = &m_functions.back();
        localtime = m_functions.back().duration;
        res = last_node->fx->GetVal(localtime) + last_node->Iy + last_node->Iydt * localtime +
              last_node->Iydtdt * localtime * localtime;
    } else {  // find current node value
        for (auto iter = m_functions.begin(); iter != m_functions.end(); ++iter) {
            if ((x >= iter->t_start) && (x < iter->t_end)) {
                localtime = x - iter->t_start;
                res = iter->fx->GetVal(localtime) + iter->Iy + iter->Iydt * localtime +
                      iter->Iydtdt * localtime * localtime;
            }
        }
    }

    return res;
}

double ChFunctionSequence::GetDer(double x) const {
    double res = 0;
    double localtime;
    for (auto iter = m_functions.begin(); iter != m_functions.end(); ++iter) {
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            localtime = x - iter->t_start;
            res = iter->fx->GetDer(localtime) + iter->Iydt + iter->Iydtdt * localtime;
        }
    }
    return res;
}

double ChFunctionSequence::GetDer2(double x) const {
    double res = 0;
    double localtime;
    for (auto iter = m_functions.begin(); iter != m_functions.end(); ++iter) {
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            localtime = x - iter->t_start;
            res = iter->fx->GetDer2(localtime) + iter->Iydtdt;
        }
    }
    return res;
}

double ChFunctionSequence::GetWeight(double x) const {
    double res = 1.0;
    for (auto iter = m_functions.begin(); iter != m_functions.end(); ++iter) {
        if ((x >= iter->t_start) && (x < iter->t_end)) {
            res = iter->weight;
        }
    }
    return res;
}

void ChFunctionSequence::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionSequence>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_start);
    archive_out << CHNVP(m_functions);
    archive_out << CHNVP(m_clamped);
}

void ChFunctionSequence::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionSequence>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_start);
    archive_in >> CHNVP(m_functions);
    archive_in >> CHNVP(m_clamped);
}

}  // end namespace chrono
