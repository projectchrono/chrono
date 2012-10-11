#ifndef CHFILEPS_H
#define CHFILEPS_H

//////////////////////////////////////////////////
//  
//   ChFilePS.h
//
//   Defines special file class ChFile_ps for EPS
//   or PS output, i.e. Encapsulated PostScript or 
//   PostScript (TM).
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChStream.h"
#include "core/ChMatrix.h"


namespace chrono 
{


/// Class for x,y coordinate 
/// on a PostScript page

class ChApi ChPageVect {
public:
	double x;
	double y; 
};
 
/// Class for RGB color for a PostScript
/// item (line, circle, etc)

class ChApi ChFile_ps_color {
public:
	double r;
	double g;
	double b;

	ChFile_ps_color() {r=g=b=0;}
	ChFile_ps_color(double mr, double mg, double mb) {r=mr; g=mg; b=mb;};

	void Set (double mr, double mg, double mb) {r=mr; g=mg; b=mb;};
	void SetGray(double mgr) {r=mgr; g=mgr; b=mgr;};
};

const ChFile_ps_color PS_COLOR_WHITE(1,1,1);
const ChFile_ps_color PS_COLOR_BLACK(0,0,0);

#define PS_STRLEN_LABEL 100

static char *ch_font_labels[] = {
		(char*)"/Times-Roman",
		(char*)"/Times-Italic",
		(char*)"/Times-Bold",
		(char*)"/Times-BoldItalic",
		(char*)"/Helvetica",
		(char*)"/Helvetica-Oblique",
		(char*)"/Helvetica-Bold",
		(char*)"/Helvetica-BoldOblique",
    0
};

/// Class for settings of an axis (x or y, or t) of a 2D 
/// plotting of a function on a EPS file.

class ChApi ChFile_ps_axis_setting {
public:
	bool			axis;
	ChFile_ps_color axis_color;
	double			axis_width;
	bool			ticks;
	double			ticks_step;
	double			ticks_width;
	char			label[PS_STRLEN_LABEL];
	ChFile_ps_color label_color;
	double		    label_fontsize;
	int				label_fontname;
	bool			numbers;
	ChFile_ps_color numbers_color;
	double		    numbers_fontsize;
	double			min;
	double			max;

	// Members:
	ChFile_ps_axis_setting();
	
	void InitializeDefaults();
	bool SetLabel(char* ml) {if (strlen(ml)<PS_STRLEN_LABEL) strcpy(label,ml); else return false; return true;}
};

/// Class for generic settings of a 2D 
/// plotting of a function on a EPS file.

class ChApi ChFile_ps_graph_setting {
public:
	ChFile_ps_axis_setting Xaxis;
	ChFile_ps_axis_setting Yaxis;

	bool			gridx;
	bool			gridy;
	double			grid_width;
	ChFile_ps_color grid_color;

	char			title[PS_STRLEN_LABEL];
	ChFile_ps_color title_color;
	double		    title_fontsize;
	int				title_fontname;

	double			picture_sizeX;
	double			picture_sizeY;

	ChFile_ps_color	def_line_color;
	double			def_line_width;

	// Members:
	ChFile_ps_graph_setting();
	ChFile_ps_graph_setting(ChFile_ps_graph_setting& source);

	void InitializeDefaults();
	bool SetTitle(char* ml) {if (strlen(ml)<PS_STRLEN_LABEL) strcpy(this->title,ml); else return false; return true;}
};


/// Class for postScript(TM) output. 
/// Defines special file class ChFile_ps for EPS
/// or PS output, i.e. Encapsulated PostScript or 
/// PostScript (TM) files incuding vectorial plots,
/// graphs, lines, fonts, etc.

class ChApi ChFile_ps : public ChStreamOutAsciiFile 
{
protected:

				//
	  			// DATA
				//

	double unit_scale;			// (72)/(current lenght unit, in inches)
	ChPageVect page_size;		// max width/height, or initial w/h of bbox for eps

		// graph-viewport variables, for transformation 2d graph space <-> page space
	ChPageVect G_p;				// viewport position of left-lower corner, in page space
	ChPageVect Gs_p;			// viewport width/height, in page space
	ChPageVect Gc_g;			// viewport center, in world or graph space;
	ChPageVect Gz;				// viewport x,y zoom factors, as (page unit)/(2dgraph units)
	char prolog_file[150];		// path and name of Chrono eps prolog file (default "prolog.ps")

public:
				//
	  			// CONSTRUCTORS
				//

	//ChFile_ps(char m_name[]) : ChStreamOutAsciiFile(m_name);	
					/// Constructor, with optional position and size of bounding box
	ChFile_ps ( char*  m_name, double x = 1, double y = 1, double w = 20, double h = 29, char*  m_prolog_file = (char *)"prolog.ps");

					/// Destructor. Write trailer to PS file and close it.
	~ChFile_ps();
				
				//
	  			// FUNCTIONS
				//

		// Functions for setting the viewport of grpahs for graph plotting

				/// Get viewport position of left-lower graph corner, in page space
	ChPageVect Get_G_p() {return G_p;};
				/// Get viewport width/height of graph, in page space
	ChPageVect Get_Gs_p() {return Gs_p;};
				/// Get viewport center, in 2d graph space;
	ChPageVect Get_Gc_g() {return Gc_g;};
				/// Get viewport graph zoom factor:
	ChPageVect Get_Gz() {return Gz;};
				/// Set viewport position of left-lower graph corner, in page space
	void Set_G_p (ChPageVect mv) {G_p = mv;};
				/// Set viewport width/height of graph, in page space
	void Set_Gs_p (ChPageVect mv) {Gs_p = mv;};
				/// Set viewport center, in 2d graph space;
	void Set_Gc_g (ChPageVect mv) {Gc_g = mv;};
				/// Set viewport graph zoom factor:
	void Set_Gz (double mz) {Gz.x = mz; Gz.y = mz;}
				/// Set viewport graph zoom factor:
	void Set_Gz (ChPageVect mz) {Gz = mz;}
				/// Set viewport zoom and pan (center) given the max/min 
				/// extension in graph space
	void Set_ZoomPan_by_fit(double Xgmin, double Xgmax, double Ygmin, double Ygmax);
		
				/// Trasform position from 'page space' to 'graph viewport space'
	ChPageVect To_page_from_graph (ChPageVect mv_g);
				/// Trasform position from 'graph viewport space' to 'page space',
	ChPageVect To_graph_from_page (ChPageVect mv_p);

	ChPageVect TransPt (ChPageVect mfrom, int space);


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
	void SetRGB(ChFile_ps_color mc) {SetRGB(mc.r, mc.g, mc.b);};
				/// Sets the font of next drawn text (note: must use 
				/// PS embedded fonts like "Times-Roman", etc.)
	void SetFont(char* name, double size);

		// Low level draw functions

	void CustomPsCommand (char* command);
	void MoveTo(ChPageVect mp);	// move the cursor
	void StartLine();		// always sequence StartLine()..AddLinePoint()..PaintStroke()
	void AddLinePoint(ChPageVect mp);
	void CloseLine();
	void PaintStroke();	// after the line has been set, this draws it!
	void PaintFill();	// same as before, but fills the enclosed area.
	void Clip();		// same as before, but defines clip region.
	void GrSave();		// save graph variable status (current clip region, linewidth, colours, etc.)
	void GrRestore();	// restore graph variable status (current clip region, linewidth, colours, etc.)

		// Hi level draw functions: if space = PS_SPACE_GRAPH the coords are considered in 2d
		// graph space and then projected and clipped onto it, otherwise are in page space.

				/// Draws a single "dot" point, 
	void DrawPoint(ChPageVect mfrom, int space);
				/// Draws line from point to point, 
	void DrawLine(ChPageVect mfrom, ChPageVect mto, int space);
				/// Draws rectangle from point to point
	void DrawRectangle(ChPageVect mfrom, ChPageVect mwh, int space, int filled);
				/// Sets clip rectangle draw region, from point to point (remember GrSave() and GrRestore() before and later..) 
	void ClipRectangle(ChPageVect mfrom, ChPageVect mwh, int space);
				/// Sets clip rectangle as graph region (remember GrSave() and GrRestore() before and later..) 
	void ClipToGraph();
				
	enum {
		PS_LEFT_JUSTIFIED = 0,
		PS_RIGHT_JUSTIFIED,
		PS_CENTER_JUSTIFIED,
	};	
				/// Draw text at given position
	void DrawText(ChPageVect mfrom, char* string, int space, int justified=PS_LEFT_JUSTIFIED);
				/// Draw number at given position
	void DrawText(ChPageVect mfrom, double number, int space, int justified=PS_LEFT_JUSTIFIED);

				/// Draw the x/y axes for the graph viewport
	void DrawGraphAxes(ChFile_ps_graph_setting* msetting);

				/// Draw a line into graph viewport, where the xy coords are stored in two
				/// separate arrays:
	void DrawGraphXY (ChMatrix<>* Yvalues, ChMatrix<>* Xvalues);

				/// Draw a line into graph viewport, where the y coords are into an array 
				/// and x coords run form Xfrom... with steps Xsteps- 
	void DrawGraphXY (ChMatrix<>* Yvalues, double Xfrom, double Xstep);

				/// Draws a text label (with "fontsize" size) with postion relative to low-left corner
				/// of graph. If "dolinesample= true", also draws a little horizontal sample of line
				/// with current width/colour on the left of the label.
	void DrawGraphLabel(double dx, double dy, double fontsize, char* label, int dolinesample,
					  bool background=true, double backwidth=3.0, ChFile_ps_color bkgndcolor=PS_COLOR_WHITE);
};
 

extern ChApi ChPageVect pv_set(double x, double y);
extern ChApi ChPageVect pv_set(Vector mv);

#define PS_SPACE_PAGE  0
#define PS_SPACE_GRAPH 1

#define PS_SCALE_CENTIMETERS	28.3476	
#define PS_SCALE_INCHES			72



} // END_OF_NAMESPACE____


#endif
