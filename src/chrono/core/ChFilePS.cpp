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

#include <cmath>

#include "chrono/core/ChFilePS.h"

namespace chrono {

const ChFile_ps_color ChFile_ps_color::WHITE(1, 1, 1);
const ChFile_ps_color ChFile_ps_color::BLACK(0, 0, 0);

const double ChFile_ps::PS_SCALE_CENTIMETERS = 28.3476;
const double ChFile_ps::PS_SCALE_INCHES = 72.0;

ChFile_ps_axis_setting::ChFile_ps_axis_setting() {
    InitializeDefaults();
}

void ChFile_ps_axis_setting::InitializeDefaults() {
    axis = true;
    axis_color.SetGray(.5);
    axis_width = 0.02;
    ticks = true;
    ticks_step = 0.1;
    ticks_width = 0.04;
    SetLabel((char*)" ");
    label_color.SetGray(.0);
    label_fontsize = 0.32;
    label_fontname = 0;
    numbers = true;
    numbers_color.SetGray(.0);
    numbers_fontsize = 0.20;
    min = 0;
    max = 1;
}

ChFile_ps_graph_setting::ChFile_ps_graph_setting() {
    InitializeDefaults();
}

ChFile_ps_graph_setting::ChFile_ps_graph_setting(ChFile_ps_graph_setting& source) {
    Xaxis = source.Xaxis;
    Yaxis = source.Yaxis;

    gridx = source.gridx;
    gridy = source.gridy;
    grid_color = source.grid_color;
    grid_width = source.grid_width;

    SetTitle(source.title);
    title_color = source.title_color;
    title_fontsize = source.title_fontsize;
    title_fontname = 0;

    picture_sizeX = source.picture_sizeX;
    picture_sizeY = source.picture_sizeY;

    def_line_width = source.def_line_width;
    def_line_color = source.def_line_color;
}

void ChFile_ps_graph_setting::InitializeDefaults() {
    Xaxis.InitializeDefaults();
    Yaxis.InitializeDefaults();

    Xaxis.SetLabel((char*)"t");
    Yaxis.SetLabel((char*)"Y");

    gridx = true;
    gridy = true;
    grid_color.Set(0.5, 0.6, 0.9);
    grid_width = 0.02;

    SetTitle((char*)"title");
    title_color.SetGray(.0);
    title_fontsize = 0.4;
    title_fontname = 0;

    picture_sizeX = 16.0;
    picture_sizeY = 6.0;

    def_line_color.Set(0.7, 0, 0);
    def_line_width = 0.035;
}

ChFile_ps::ChFile_ps(char m_name[], double x, double y, double w, double h, char* m_prolog_file)
    : ChStreamOutAsciiFile(m_name) {
    this->SetNumFormat("%g");

    strncpy(prolog_file, m_prolog_file, sizeof(prolog_file)-1);

    // init vars..
    unit_scale = PS_SCALE_CENTIMETERS;
    page_size.x() = w;
    page_size.y() = h;
    G_p.x() = 3;
    G_p.y() = 3;
    Gs_p.x() = 16;
    Gs_p.y() = 16;
    Gc_g.x() = 0;
    Gc_g.y() = 0;
    Gz.x() = 1;
    Gz.y() = 1;

    // Write header and bounding box
    *this << "%%!PS-Adobe-2.0\n";
    *this << "%%%%BoundingBox: ";
    *this << (int)(unit_scale * x) << " ";
    *this << (int)(unit_scale * y) << " ";
    *this << (int)(unit_scale * (x + w)) << " ";
    *this << (int)(unit_scale * (y + h)) << "\n";
    *this << "%%%%HiResBoundingBox: ";
    *this << (unit_scale * x) << " ";
    *this << (unit_scale * y) << " ";
    *this << (unit_scale * (x + w)) << " ";
    *this << (unit_scale * (y + h)) << "\n";

    // Write prolog
    int copied = 0;
    try {
        ChStreamInAsciiFile mprolog(prolog_file);
        while (!mprolog.End_of_stream()) {
            char mch;
            mprolog >> mch;
            *this << mch;
            copied++;
        }
    } catch (std::exception pollo) {
        if (!copied) {
            throw ChException("Could not include the 'prolog.ps'. Is the prolog.ps file properly installed?");
        }
    }

    // make first operations:
    *this << "mark \n%%Page: ? 1 \n%%PageBoundingBox: 36 36 574 755 \n%%PageOrientation: Portrait \nS GS \n";
    *this << unit_scale << " " << unit_scale << " SC\n";

    SetWidth(0.01);
    SetGray(0);
    SetFont((char*)"/Times-Roman", 1);
}

ChFile_ps::~ChFile_ps() {
    *this << "GR R	SP cleartomark end \n%%Trailer \n%%EOF\n";
}

// Transform positions from 'graph viewport space' to 'page space',
// or viceversa.
ChVector2<> ChFile_ps::To_page_from_graph(ChVector2<> mv_g) const {
    ChVector2<> v_p;
    ChVector2<> mGc_p;
    mGc_p.x() = G_p.x() + (Gs_p.x()) / 2;
    mGc_p.y() = G_p.y() + (Gs_p.y()) / 2;
    v_p.x() = mGc_p.x() + (mv_g.x() - Gc_g.x()) * Gz.x();
    v_p.y() = mGc_p.y() + (mv_g.y() - Gc_g.y()) * Gz.y();
    return v_p;
}
ChVector2<> ChFile_ps::To_graph_from_page(ChVector2<> mv_p) const {
    ChVector2<> v_g;
    ChVector2<> mGc_p;
    mGc_p.x() = G_p.x() + (Gs_p.x()) / 2;
    mGc_p.y() = G_p.y() + (Gs_p.y()) / 2;
    v_g.x() = (mv_p.x() - mGc_p.x()) * (1 / Gz.x()) + Gc_g.x();
    v_g.y() = (mv_p.y() - mGc_p.y()) * (1 / Gz.y()) + Gc_g.y();
    return v_g;
}

ChVector2<> ChFile_ps::TransPt(ChVector2<> mfrom, Space space) const {
    if ((mfrom.x() > 1.e+20) || (mfrom.x() < -1.e+20))
        mfrom.x() = 0;
    if ((mfrom.y() > 1.e+20) || (mfrom.y() < -1.e+20))
        mfrom.y() = 0;

    switch (space) {
        case Space::PAGE:
            return mfrom;
        case Space::GRAPH:
            return To_page_from_graph(mfrom);
        default:
            return mfrom;
    }
}

void ChFile_ps::Set_ZoomPan_by_fit(double Xgmin, double Xgmax, double Ygmin, double Ygmax) {
    Gz.x() = Gs_p.x() / (Xgmax - Xgmin);
    Gz.y() = Gs_p.y() / (Ygmax - Ygmin);
    Gc_g.x() = Xgmin + (Xgmax - Xgmin) / 2;
    Gc_g.y() = Ygmin + (Ygmax - Ygmin) / 2;
}

void ChFile_ps::SetGray(double mf) {
    *this << mf << " SG\n";
}
void ChFile_ps::SetWidth(double mf) {
    *this << mf << " LW\n";
}
void ChFile_ps::SetLinecap(double mf) {
    *this << mf << " LC\n";
}
void ChFile_ps::SetRGB(double r, double g, double b) {
    *this << r << " " << g << " " << b << " "
          << " SRGB\n";
}
void ChFile_ps::SetFont(char* name, double size) {
    *this << name;
    *this << " findfont \n";
    *this << size;
    *this << " scalefont \n";
    *this << "setfont \n";
}

void ChFile_ps::CustomPsCommand(char* mcommand) {
    *this << mcommand << "\n";
}
void ChFile_ps::MoveTo(ChVector2<> mp) {
    *this << mp.x() << " ";
    *this << mp.y() << " ";
    *this << "MT\n";
}
void ChFile_ps::StartLine() {
    *this << "NP\n";
}
void ChFile_ps::AddLinePoint(ChVector2<> mp) {
    *this << mp.x() << " ";
    *this << mp.y() << " ";
    *this << "LT\n";
}
void ChFile_ps::CloseLine() {
    *this << "CP\n";
}
void ChFile_ps::PaintStroke() {
    *this << "SK\n";
}
void ChFile_ps::PaintFill() {
    *this << "FL\n";
}
void ChFile_ps::Clip() {
    *this << "CL\n";
}
void ChFile_ps::GrSave() {
    *this << "GS\n";
}
void ChFile_ps::GrRestore() {
    *this << "GR\n";
}

// Hi level PS draw functions:

void ChFile_ps::DrawPoint(ChVector2<> mfrom, Space space) {
    GrSave();
    SetLinecap(1);
    StartLine();
    MoveTo(TransPt(mfrom, space));
    AddLinePoint(TransPt(mfrom, space));
    PaintStroke();
    GrRestore();
}

void ChFile_ps::DrawLine(ChVector2<> mfrom, ChVector2<> mto, Space space) {
    GrSave();
    StartLine();
    MoveTo(TransPt(mfrom, space));
    AddLinePoint(TransPt(mto, space));
    PaintStroke();
    GrRestore();
}

void ChFile_ps::DrawRectangle(ChVector2<> mfrom, ChVector2<> mwh, Space space, bool filled) {
    ChVector2<> mp1, mp2, mp3;
    mp1.x() = mfrom.x() + mwh.x();
    mp1.y() = mfrom.y();
    mp2.x() = mfrom.x() + mwh.x();
    mp2.y() = mfrom.y() + mwh.y();
    mp3.x() = mfrom.x();
    mp3.y() = mfrom.y() + mwh.y();
    GrSave();
    StartLine();
    MoveTo(TransPt(mfrom, space));
    AddLinePoint(TransPt(mp1, space));
    AddLinePoint(TransPt(mp2, space));
    AddLinePoint(TransPt(mp3, space));
    CloseLine();
    if (filled)
        PaintFill();
    else
        PaintStroke();
    GrRestore();
}

void ChFile_ps::ClipRectangle(ChVector2<> mfrom, ChVector2<> mwh, Space space) {
    ChVector2<> mp1, mp2, mp3;
    mp1.x() = mfrom.x() + mwh.x();
    mp1.y() = mfrom.y();
    mp2.x() = mfrom.x() + mwh.x();
    mp2.y() = mfrom.y() + mwh.y();
    mp3.x() = mfrom.x();
    mp3.y() = mfrom.y() + mwh.y();
    StartLine();
    MoveTo(TransPt(mfrom, space));
    AddLinePoint(TransPt(mp1, space));
    AddLinePoint(TransPt(mp2, space));
    AddLinePoint(TransPt(mp3, space));
    CloseLine();
    Clip();
}

void ChFile_ps::ClipToGraph() {
    ClipRectangle(Get_G_p(), Get_Gs_p(), Space::PAGE);
}

void ChFile_ps::DrawText(ChVector2<> mfrom, char* string, Space space, Justification justified) {
    GrSave();
    MoveTo(TransPt(mfrom, space));
    *this << "(";
    *this << string;
    switch (justified) {
        case Justification::LEFT:
            *this << ") show \n";
            break;
        case Justification::RIGHT:
            *this << ") dup stringwidth pop neg 0 rmoveto show \n";
            break;
        case Justification::CENTER:
            *this << ") dup stringwidth pop 2 div neg 0 rmoveto show \n";
            break;
        default:
            *this << ") show \n";
    }
    GrRestore();
}

void ChFile_ps::DrawText(ChVector2<> mfrom, double number, Space space, Justification justified) {
    char mbuff[20];
    sprintf(mbuff, this->number_format, number);
    DrawText(mfrom, mbuff, space, justified);
}

void ChFile_ps::DrawGraphAxes(ChFile_ps_graph_setting* msetting) {
    ChVector2<> cpt, cp1, cp2, lole_g, upri_g, org_g, org_p;

    // save old gfx mode
    GrSave();

    // some coords
    lole_g = To_graph_from_page(G_p);
    cpt.x() = G_p.x() + Gs_p.x();
    cpt.y() = G_p.y() + Gs_p.y();
    upri_g = To_graph_from_page(cpt);
    org_g.x() = org_g.y() = 0;
    org_p = To_page_from_graph(org_g);

    // Set clip region
    GrSave();
    ClipToGraph();  // ClipRectangle(G_p,Gs_p,PS_SPACE_PAGE);

    // X AXIS ticks, grid numbers
    //

    SetFont((char*)"/Helvetica", msetting->Xaxis.numbers_fontsize);

    double stpx = ceil(lole_g.x() / msetting->Xaxis.ticks_step) * msetting->Xaxis.ticks_step;
    for (double w = stpx; w < upri_g.x(); w += msetting->Xaxis.ticks_step) {
        if ((upri_g.x() - lole_g.x()) / msetting->Xaxis.ticks_step > 200.0)
            break;
        cpt.x() = w;
        cpt.y() = 0;
        if (msetting->gridx) {
            cp1 = cpt;
            cp2 = cp1;
            cp1.y() = lole_g.y();
            cp2.y() = upri_g.y();
            cp1 = To_page_from_graph(cp1);
            cp2 = To_page_from_graph(cp2);
            SetWidth(msetting->grid_width);
            SetRGB(msetting->grid_color);
            DrawLine(cp1, cp2, Space::PAGE);
        }
        if (msetting->Xaxis.ticks) {
            cp1 = To_page_from_graph(cpt);
            cp2.x() = cp1.x();
            cp2.y() = cp1.y() + 0.10;
            SetWidth(msetting->Xaxis.ticks_width);
            SetRGB(msetting->Xaxis.axis_color);
            DrawLine(cp1, cp2, Space::PAGE);
        }
        if (msetting->Xaxis.numbers) {
            char numstr[20];
            SetRGB(msetting->Xaxis.numbers_color);
            cp1 = To_page_from_graph(cpt);
            cp1.y() = G_p.y() + 0.16;
            cp1.x() = cp1.x() + 0.1;
            double tw = w;
            if (fabs(w) < 1.e-15)
                tw = 0;
            sprintf(numstr, this->number_format, tw);
            DrawText(cp1, numstr, Space::PAGE);
        }
    }

    // Y AXIS grid, numbers, ticks
    //

    SetFont((char*)"/Helvetica", msetting->Yaxis.numbers_fontsize);

    double stpy = ceil(lole_g.y() / msetting->Yaxis.ticks_step) * msetting->Yaxis.ticks_step;
    for (double h = stpy; h < upri_g.y(); h += msetting->Yaxis.ticks_step) {
        if ((upri_g.y() - lole_g.y()) / msetting->Yaxis.ticks_step > 200.0)
            break;
        cpt.x() = 0;
        cpt.y() = h;
        if (msetting->gridy) {
            cp1 = cpt;
            cp2 = cp1;
            cp1.x() = lole_g.x();
            cp2.x() = upri_g.x();
            cp1 = To_page_from_graph(cp1);
            cp2 = To_page_from_graph(cp2);
            SetWidth(msetting->grid_width);
            SetRGB(msetting->grid_color);
            DrawLine(cp1, cp2, Space::PAGE);
        }
        if (msetting->Yaxis.ticks) {
            cp1 = To_page_from_graph(cpt);
            cp2.x() = cp1.x() + 0.10;
            cp2.y() = cp1.y();
            SetWidth(msetting->Yaxis.ticks_width);
            SetRGB(msetting->Yaxis.axis_color);
            DrawLine(cp1, cp2, Space::PAGE);
        }
        if (msetting->Yaxis.numbers) {
            char numstr[20];
            SetRGB(msetting->Yaxis.numbers_color);
            cp1 = To_page_from_graph(cpt);
            cp1.x() = G_p.x() + 0.16;
            cp1.y() = cp1.y() + 0.1;
            double th = h;
            if (fabs(th) < 1.e-15)
                th = 0;
            sprintf(numstr, this->number_format, th);
            DrawText(cp1, numstr, Space::PAGE);
        }
    }

    // draw x axis line
    SetWidth(msetting->Xaxis.axis_width);
    SetRGB(msetting->Xaxis.axis_color);
    cp1 = org_g;
    cp2 = org_g;
    cp1.x() = lole_g.x();
    cp2.x() = upri_g.x();
    DrawLine(cp1, cp2, Space::GRAPH);

    // draw y axis line
    SetWidth(msetting->Yaxis.axis_width);
    SetRGB(msetting->Yaxis.axis_color);
    cp1 = org_g;
    cp2 = org_g;
    cp1.y() = lole_g.y();
    cp2.y() = upri_g.y();
    DrawLine(cp1, cp2, Space::GRAPH);

    // draw xy labels
    //if (msetting->Xaxis.label) { // pointers to static arrays are always TRUE
    cp1.x() = G_p.x() + Gs_p.x() - 0.4;
    cp1.y() = G_p.y() + 0.4;
    SetRGB(msetting->Xaxis.label_color);
    SetFont(ch_font_labels[msetting->Xaxis.label_fontname], msetting->Xaxis.label_fontsize);
    DrawText(cp1, msetting->Xaxis.label, Space::PAGE, Justification::RIGHT);
    //}
    //if (msetting->Yaxis.label) { // pointers to static arrays are always TRUE
    cp1.x() = G_p.x() + 0.7;
    cp1.y() = G_p.y() + Gs_p.y() - 0.3;
    SetRGB(msetting->Yaxis.label_color);
    SetFont(ch_font_labels[msetting->Yaxis.label_fontname], msetting->Yaxis.label_fontsize);
    DrawText(cp1, msetting->Yaxis.label, Space::PAGE);
    //}

    // restore old gfx mode -without the clipping region
    GrRestore();

    // draw enclosing frame
    SetGray(0.0);
    DrawRectangle(G_p, Gs_p, Space::PAGE, false);

    // draw title
    //if (msetting->title) // pointers to static arrays are always TRUE
    if (*msetting->title != 0) {
        SetFont(ch_font_labels[msetting->title_fontname], msetting->title_fontsize);
        SetRGB(msetting->title_color);
        cpt.x() = G_p.x() + 0.0;
        cpt.y() = G_p.y() + Gs_p.y() + 0.4;
        DrawText(cpt, msetting->title, Space::PAGE);
    }

    // return to original gfx mode
    GrRestore();
}

void ChFile_ps::DrawGraphXY(ChMatrix<>* Yvalues, ChMatrix<>* Xvalues) {
    int points;
    ChVector2<> mp;

    // Set clip region
    GrSave();
    ClipToGraph();

    points = Yvalues->GetRows();
    if (Xvalues->GetRows() < points)
        points = Xvalues->GetRows();
    if (!points)
        return;

    StartLine();
    for (int i = 0; i < points; i++) {
        mp.x() = Xvalues->GetElement(i, 0);
        mp.y() = Yvalues->GetElement(i, 0);
        mp = To_page_from_graph(mp);
        if (i == 0)
            MoveTo(mp);
        else
            AddLinePoint(mp);
    }
    PaintStroke();

    GrRestore();
}

void ChFile_ps::DrawGraphXY(ChMatrix<>* Yvalues, double Xfrom, double Xstep) {
    int points;
    ChVector2<> mp;
    double mx;

    // clip region
    GrSave();
    ClipToGraph();

    points = Yvalues->GetRows();
    if (!points)
        return;

    StartLine();
    mx = Xfrom;
    for (int i = 0; i < points; i++) {
        mp.x() = mx;
        mp.y() = Yvalues->GetElement(i, 0);
        mp = To_page_from_graph(mp);
        if (i == 0)
            MoveTo(mp);
        else
            AddLinePoint(mp);
        mx += Xstep;
    }
    PaintStroke();

    GrRestore();
}

void ChFile_ps::DrawGraphLabel(double dx,
                               double dy,
                               double fontsize,
                               char* label,
                               int dolinesample,
                               bool background,
                               double backwidth,
                               ChFile_ps_color bkgndcolor) {
    if (fontsize == 0)
        fontsize = 0.3;
    ChVector2<> mp1, mp2, mp3;
    mp3.y() = G_p.y() + dy;
    mp1.y() = mp3.y() + 0.0;
    mp2.y() = mp1.y();
    mp1.x() = G_p.x() + dx;
    mp2.x() = G_p.x() + dx + 0.6;
    ChVector2<> mpa, mpb;
    mpa = mp1;
    mpa.y() -= fontsize * 0.7;
    mpb.x() = backwidth;
    mpb.y() = fontsize * 1.4;

    GrSave();
    ClipToGraph();
    SetFont((char*)"/Times-Italic", fontsize);

    if (background) {
        GrSave();
        SetRGB(bkgndcolor);
        DrawRectangle(mpa, mpb, Space::PAGE, true);
        GrRestore();
    }
    if (dolinesample) {
        mp1.x() += 0.08;
        DrawLine(mp1, mp2, Space::PAGE);
        mp3.x() = mp2.x() + 0.3;
    } else {
        mp3.x() = mp1.x();
    }
    mp3.y() = mp1.y() - fontsize * 0.0;
    SetGray(0.0);
    DrawText(mp3, label, Space::PAGE);

    GrRestore();
}

//
// External utility functions
//

ChVector2<> pv_set(double x, double y) {
    ChVector2<> mpv;
    mpv.x() = x;
    mpv.y() = y;
    return mpv;
}

ChVector2<> pv_set(ChVector<> mv) {
    ChVector2<> mpv;
    mpv.x() = mv.x();
    mpv.y() = mv.y();
    return mpv;
}

}  // end namespace chrono
