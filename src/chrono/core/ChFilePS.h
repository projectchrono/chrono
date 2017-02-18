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

#ifndef CHFILEPS_H
#define CHFILEPS_H

#include "chrono/core/ChStream.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector2.h"

namespace chrono {

/// Class for RGB color for a PostScript
/// item (line, circle, etc)

class ChApi ChFile_ps_color {
  public:
    double r;
    double g;
    double b;

    ChFile_ps_color() { r = g = b = 0; }
    ChFile_ps_color(double mr, double mg, double mb) {
        r = mr;
        g = mg;
        b = mb;
    };

    void Set(double mr, double mg, double mb) {
        r = mr;
        g = mg;
        b = mb;
    };
    void SetGray(double mgr) {
        r = mgr;
        g = mgr;
        b = mgr;
    };

    static const ChFile_ps_color WHITE;
    static const ChFile_ps_color BLACK;
};

static char* ch_font_labels[] = {(char*)"/Times-Roman",
                                 (char*)"/Times-Italic",
                                 (char*)"/Times-Bold",
                                 (char*)"/Times-BoldItalic",
                                 (char*)"/Helvetica",
                                 (char*)"/Helvetica-Oblique",
                                 (char*)"/Helvetica-Bold",
                                 (char*)"/Helvetica-BoldOblique",
                                 0};

/// Class for settings of an axis (x or y, or t) of a 2D
/// plotting of a function on a EPS file.

class ChApi ChFile_ps_axis_setting {
  public:
    bool axis;
    ChFile_ps_color axis_color;
    double axis_width;
    bool ticks;
    double ticks_step;
    double ticks_width;
    static const int PS_STRLEN_LABEL = 100;
    char label[PS_STRLEN_LABEL];
    ChFile_ps_color label_color;
    double label_fontsize;
    int label_fontname;
    bool numbers;
    ChFile_ps_color numbers_color;
    double numbers_fontsize;
    double min;
    double max;

    // Members:
    ChFile_ps_axis_setting();

    void InitializeDefaults();
    bool SetLabel(char* ml) {
        if (strlen(ml) < PS_STRLEN_LABEL)
            strcpy(label, ml);
        else
            return false;
        return true;
    }
};

/// Class for generic settings of a 2D
/// plotting of a function on a EPS file.

class ChApi ChFile_ps_graph_setting {
  public:
    ChFile_ps_axis_setting Xaxis;
    ChFile_ps_axis_setting Yaxis;

    bool gridx;
    bool gridy;
    double grid_width;
    ChFile_ps_color grid_color;
    static const int PS_STRLEN_TITLE = 100;
    char title[PS_STRLEN_TITLE];
    ChFile_ps_color title_color;
    double title_fontsize;
    int title_fontname;

    double picture_sizeX;
    double picture_sizeY;

    ChFile_ps_color def_line_color;
    double def_line_width;

    // Members:
    ChFile_ps_graph_setting();
    ChFile_ps_graph_setting(ChFile_ps_graph_setting& source);

    void InitializeDefaults();
    bool SetTitle(char* ml) {
        if (strlen(ml) < PS_STRLEN_TITLE)
            strcpy(this->title, ml);
        else
            return false;
        return true;
    }
};

/// Class for postScript(TM) output.
/// Defines special file class ChFile_ps for EPS
/// or PS output, i.e. Encapsulated PostScript or
/// PostScript (TM) files incuding vectorial plots,
/// graphs, lines, fonts, etc.

class ChApi ChFile_ps : public ChStreamOutAsciiFile {
  protected:
    double unit_scale;     // (72)/(current length unit, in inches)
    ChVector2<> page_size;  // max width/height, or initial w/h of bbox for eps

    // graph-viewport variables, for transformation 2d graph space <-> page space
    ChVector2<> G_p;         // viewport position of left-lower corner, in page space
    ChVector2<> Gs_p;        // viewport width/height, in page space
    ChVector2<> Gc_g;        // viewport center, in world or graph space;
    ChVector2<> Gz;          // viewport x,y zoom factors, as (page unit)/(2dgraph units)
    char prolog_file[150];  // path and name of Chrono eps prolog file (default "prolog.ps")

  public:
    enum class Justification {
        LEFT,
        RIGHT,
        CENTER,
    };

    enum class Space {
        PAGE,
        GRAPH
    };

    static const double PS_SCALE_CENTIMETERS;
    static const double PS_SCALE_INCHES;

    //
    // CONSTRUCTORS
    //

    // ChFile_ps(char m_name[]) : ChStreamOutAsciiFile(m_name);
    /// Constructor, with optional position and size of bounding box
    ChFile_ps(char* m_name,
              double x = 1,
              double y = 1,
              double w = 20,
              double h = 29,
              char* m_prolog_file = (char*)"prolog.ps");

    /// Destructor. Write trailer to PS file and close it.
    ~ChFile_ps();

    //
    // FUNCTIONS
    //

    // Functions for setting the viewport of grpahs for graph plotting

    /// Get viewport position of left-lower graph corner, in page space
    const ChVector2<>& Get_G_p() const { return G_p; };
    /// Get viewport width/height of graph, in page space
    const ChVector2<>& Get_Gs_p() const { return Gs_p; };
    /// Get viewport center, in 2d graph space;
    const ChVector2<>& Get_Gc_g() const { return Gc_g; };
    /// Get viewport graph zoom factor:
    const ChVector2<>& Get_Gz() const { return Gz; };
    /// Set viewport position of left-lower graph corner, in page space
    void Set_G_p(ChVector2<> mv) { G_p = mv; };
    /// Set viewport width/height of graph, in page space
    void Set_Gs_p(ChVector2<> mv) { Gs_p = mv; };
    /// Set viewport center, in 2d graph space;
    void Set_Gc_g(ChVector2<> mv) { Gc_g = mv; };
    /// Set viewport graph zoom factor:
    void Set_Gz(double mz) {
        Gz.x() = mz;
        Gz.y() = mz;
    }
    /// Set viewport graph zoom factor:
    void Set_Gz(ChVector2<> mz) { Gz = mz; }
    /// Set viewport zoom and pan (center) given the max/min
    /// extension in graph space
    void Set_ZoomPan_by_fit(double Xgmin, double Xgmax, double Ygmin, double Ygmax);

    /// Transform position from 'page space' to 'graph viewport space'
    ChVector2<> To_page_from_graph(ChVector2<> mv_g) const;
    /// Transform position from 'graph viewport space' to 'page space',
    ChVector2<> To_graph_from_page(ChVector2<> mv_p) const;

    ChVector2<> TransPt(ChVector2<> mfrom, Space space) const;

    // --- Functions which record graphical operations on file ------

    // Set graphical mode

    /// Sets the gray level of next drawn item, 0..1
    void SetGray(double mf);
    /// Sets the width of next drawn line
    void SetWidth(double mf);
    /// Sets the linecap
    void SetLinecap(double mf);
    /// Sets the RGB color of next drawn item
    void SetRGB(double r, double g, double b);
    /// Sets the color of next drawn item
    void SetRGB(ChFile_ps_color mc) { SetRGB(mc.r, mc.g, mc.b); };
    /// Sets the font of next drawn text (note: must use
    /// PS embedded fonts like "Times-Roman", etc.)
    void SetFont(char* name, double size);

    // Low level draw functions

    void CustomPsCommand(char* command);
    void MoveTo(ChVector2<> mp);  // move the cursor
    void StartLine();            // always sequence StartLine()..AddLinePoint()..PaintStroke()
    void AddLinePoint(ChVector2<> mp);
    void CloseLine();
    void PaintStroke();  // after the line has been set, this draws it!
    void PaintFill();    // same as before, but fills the enclosed area.
    void Clip();         // same as before, but defines clip region.
    void GrSave();       // save graph variable status (current clip region, linewidth, colours, etc.)
    void GrRestore();    // restore graph variable status (current clip region, linewidth, colours, etc.)

    // Hi level draw functions: if space = PS_SPACE_GRAPH the coords are considered in 2d
    // graph space and then projected and clipped onto it, otherwise are in page space.

    /// Draws a single "dot" point,
    void DrawPoint(ChVector2<> mfrom, Space space);
    /// Draws line from point to point,
    void DrawLine(ChVector2<> mfrom, ChVector2<> mto, Space space);
    /// Draws rectangle from point to point
    void DrawRectangle(ChVector2<> mfrom, ChVector2<> mwh, Space space, bool filled);
    /// Sets clip rectangle draw region, from point to point (remember GrSave() and GrRestore() before and later..)
    void ClipRectangle(ChVector2<> mfrom, ChVector2<> mwh, Space space);
    /// Sets clip rectangle as graph region (remember GrSave() and GrRestore() before and later..)
    void ClipToGraph();

    /// Draw text at given position
    void DrawText(ChVector2<> mfrom, char* string, Space space, Justification justified = Justification::LEFT);
    /// Draw number at given position
    void DrawText(ChVector2<> mfrom, double number, Space space, Justification justified = Justification::LEFT);

    /// Draw the x/y axes for the graph viewport
    void DrawGraphAxes(ChFile_ps_graph_setting* msetting);

    /// Draw a line into graph viewport, where the xy coords are stored in two
    /// separate arrays:
    void DrawGraphXY(ChMatrix<>* Yvalues, ChMatrix<>* Xvalues);

    /// Draw a line into graph viewport, where the y coords are into an array
    /// and x coords run form Xfrom... with steps Xsteps-
    void DrawGraphXY(ChMatrix<>* Yvalues, double Xfrom, double Xstep);

    /// Draws a text label (with "fontsize" size) with postion relative to low-left corner
    /// of graph. If "dolinesample= true", also draws a little horizontal sample of line
    /// with current width/colour on the left of the label.
    void DrawGraphLabel(double dx,
                        double dy,
                        double fontsize,
                        char* label,
                        int dolinesample,
                        bool background = true,
                        double backwidth = 3.0,
                        ChFile_ps_color bkgndcolor = ChFile_ps_color::WHITE);
};

extern ChApi ChVector2<> pv_set(double x, double y);
extern ChApi ChVector2<> pv_set(ChVector<> mv);


}  // end namespace chrono

#endif
