//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChFunction_Base.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "ChFunction_Base.h"
#include "physics/ChGlobal.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChFunction> a_registration;

double ChFunction::Get_y_dN(double x, int derivate) {
    switch (derivate) {
        case 0:
            return Get_y(x);
        case 1:
            return Get_y_dx(x);
        case 2:
            return Get_y_dxdx(x);
        default:
            return Get_y(x);
    }
}

void ChFunction::Estimate_y_range(double xmin, double xmax, double& ymin, double& ymax, int derivate) {
    ymin = 10000;
    ymax = -10000;
    for (double mx = xmin; mx < xmax; mx += (xmax - xmin) / 100.0) {
        if (Get_y_dN(mx, derivate) < ymin)
            ymin = Get_y_dN(mx, derivate);
        if (Get_y_dN(mx, derivate) > ymax)
            ymax = Get_y_dN(mx, derivate);
    }
    if (fabs(ymax - ymin) < 10e-12) {
        ymin = -0.5;
        ymax = +1.0;
    }
    ymax += 0.12 * (ymax - ymin);
    ymin -= 0.12 * (ymax - ymin);
}

// some analysis functions
double ChFunction::Compute_max(double xmin, double xmax, double sampling_step, int derivate) {
    double mret = -1E30;
    for (double mx = xmin; mx <= xmax; mx += sampling_step) {
        if (this->Get_y_dN(mx, derivate) > mret)
            mret = this->Get_y_dN(mx, derivate);
    }
    return mret;
}

double ChFunction::Compute_min(double xmin, double xmax, double sampling_step, int derivate) {
    double mret = +1E30;
    for (double mx = xmin; mx <= xmax; mx += sampling_step) {
        if (this->Get_y_dN(mx, derivate) < mret)
            mret = this->Get_y_dN(mx, derivate);
    }
    return mret;
}

double ChFunction::Compute_mean(double xmin, double xmax, double sampling_step, int derivate) {
    double mret = 0;
    int numpts = 0;
    for (double mx = xmin; mx <= xmax; mx = mx + sampling_step) {
        numpts++;
        mret += this->Get_y_dN(mx, derivate);
    }
    return mret / ((double)numpts);
}

double ChFunction::Compute_sqrmean(double xmin, double xmax, double sampling_step, int derivate) {
    double mret = 0;
    int numpts = 0;
    for (double mx = xmin; mx <= xmax; mx = mx + sampling_step) {
        numpts++;
        mret += pow(this->Get_y_dN(mx, derivate), 2.);
    }
    return sqrt(mret / ((double)numpts));
}

double ChFunction::Compute_int(double xmin, double xmax, double sampling_step, int derivate) {
    double mret = 0;
    double ya = this->Get_y_dN(xmin, derivate);
    double yb = 0;
    for (double mx = xmin + sampling_step; mx <= xmax; mx += sampling_step) {
        yb = this->Get_y_dN(mx, derivate);
        mret += sampling_step * (ya + yb) * 0.5;  // trapezoidal quadrature
        ya = yb;
    }
    return mret;
}

////////////

int ChFunction::MakeOptVariableTree(ChList<chjs_propdata>* mtree) {
    const char** mvars = this->GetOptVariables();
    int i = 0;
    while (*(mvars + i) != 0) {
        chjs_propdata* mdata = new chjs_propdata;
        strncpy(mdata->propname, *(mvars + i), sizeof(mdata->propname)-1);
        strncpy(mdata->label, *(mvars + i), sizeof(mdata->label)-1);
        mdata->haschildren = FALSE;
        mtree->AddTail(mdata);
        i++;
    }
    /*
        // now dirty trick, because of 'C' variable is inherited by all functions,
        // but used by plain base class only..
        if (this->Get_Type()==FUNCT_CONST)
        {
            chjs_propdata* mdata = new chjs_propdata;
            strcpy(mdata->propname, "C");
            strcpy(mdata->label,    mdata->propname);
            mdata->haschildren = FALSE;
            mtree->AddTail(mdata);
            i++;
        }
    */
    return i;
}

static int _recurse_VariableTreeToFullNameVar(ChList<chjs_propdata>* mtree,
                                              ChList<chjs_fullnamevar>* mlist,
                                              char* maccumulator) {
    int i = 0;

    size_t mentrypos = strlen(maccumulator);

    ChNode<chjs_propdata>* mnode = mtree->GetHead();
    while (mnode) {
        if (strlen(maccumulator) + strlen(mnode->data->propname) < 120 - 1) {
            strcat(maccumulator, mnode->data->label);

            if (mnode->data->children.Count()) {
                strcat(maccumulator, ".");
                _recurse_VariableTreeToFullNameVar(&mnode->data->children, mlist, maccumulator);
            } else {
                chjs_fullnamevar* mfullname = new chjs_fullnamevar;
                strcpy(mfullname->propname, maccumulator);
                strcpy(mfullname->label, maccumulator);
                mfullname->active = TRUE;
                mfullname->script = NULL;
                mlist->AddTail(mfullname);
                i++;
            }

            maccumulator[mentrypos] = 0;
        }

        mnode = mnode->next;
    }
    return i;
}

int ChFunction::VariableTreeToFullNameVar(ChList<chjs_propdata>* mtree, ChList<chjs_fullnamevar>* mlist) {
    char accumulator[120];
    strcpy(accumulator, "context().");

    int i = _recurse_VariableTreeToFullNameVar(mtree, mlist, accumulator);

    return i;
}

int ChFunction::OptVariableCount() {
    ChList<chjs_propdata> mtree;
    ChList<chjs_fullnamevar> mlist;
    MakeOptVariableTree(&mtree);
    VariableTreeToFullNameVar(&mtree, &mlist);
    return mlist.Count();
}

////////////

void ChFunction::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);
}

/// Method to allow de serialization of transient data from archives.
void ChFunction::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();
}



int ChFunction::FilePostscriptPlot(ChFile_ps* m_file, int plotY, int plotDY, int plotDDY) {
    int mresol = 800;
    ChMatrixDynamic<> yf(mresol, 1);
    ChMatrixDynamic<> xf(mresol, 1);
    double mx, xmin, xmax;
    ChPageVect mp;
    // xmin
    mp = m_file->Get_G_p();
    mp = m_file->To_graph_from_page(mp);
    xmin = mp.x;
    // xmax
    mp = m_file->Get_G_p();
    mp.x = mp.x + m_file->Get_Gs_p().x;
    mp = m_file->To_graph_from_page(mp);
    xmax = mp.x;

    if (plotY) {
        mx = xmin;
        for (int j = 0; j < mresol; j++) {
            mp.x = mx;
            mp.y = this->Get_y(mx);
            xf.SetElement(j, 0, mp.x);
            yf.SetElement(j, 0, mp.y);
            mx += ((xmax - xmin) / ((double)mresol - 1.0));
        }
        m_file->DrawGraphXY(&yf, &xf);
    }
    if (plotDY) {
        mx = xmin;
        for (int j = 0; j < mresol; j++) {
            mp.x = mx;
            mp.y = this->Get_y_dx(mx);
            xf.SetElement(j, 0, mp.x);
            yf.SetElement(j, 0, mp.y);
            mx += ((xmax - xmin) / ((double)mresol - 1.0));
        }
        m_file->DrawGraphXY(&yf, &xf);
    }
    if (plotDDY) {
        mx = xmin;
        for (int j = 0; j < mresol; j++) {
            mp.x = mx;
            mp.y = this->Get_y_dxdx(mx);
            xf.SetElement(j, 0, mp.x);
            yf.SetElement(j, 0, mp.y);
            mx += ((xmax - xmin) / ((double)mresol - 1.0));
        }
        m_file->DrawGraphXY(&yf, &xf);
    }
    return 1;
}

int ChFunction::FileAsciiPairsSave(ChStreamOutAscii& m_file, double mxmin, double mxmax, int msamples) {
    if (msamples <= 1)
        throw(ChException("Warning! too short range or too long sampling period: no points can be saved"));
    if (msamples >= 100000)
        throw(ChException("Warning! Too many points should be saved"));
    if (mxmax <= mxmin)
        throw(ChException("Warning! Cannot save ChFunction if Xmax < Xmin"));

    m_file.SetNumFormat("%0.8f");

    double period = (mxmax - mxmin) / ((double)msamples - 1);

    double mX = mxmin;
    for (int cnt = 1; cnt <= msamples; cnt++) {
        m_file << mX;
        m_file << "    ";
        m_file << this->Get_y(mX);
        m_file.CR();
        mX += period;
    }
    return 1;
}

}  // END_OF_NAMESPACE____

// eof
