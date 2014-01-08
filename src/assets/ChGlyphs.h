//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHGLYPHS_H
#define CHGLYPHS_H


#include "assets/ChVisualization.h"
#include "assets/ChColor.h"
#include "core/ChMatrix.h"

namespace chrono
{

/// Class for referencing a set of 'glyps', that are simple symbols
/// such as arrows or points to be drawn for showing vector directions etc.
/// Remember that depending on the type of visualization system 
/// (POVray, Irrlicht,etc.) this asset might not be supported.

class ChApi ChGlyphs : public ChVisualization {

public:
	enum eCh_GlyphType{
						 GLYPH_POINT = 0,
						 GLYPH_VECTOR
				};

public:
				//
	  			// DATA
				//
	std::vector< ChVector<double> > points;
	std::vector< ChColor >			colors;
	
		// optional attrs
	std::vector< ChVector<double> > vectors;

protected:

	eCh_GlyphType draw_mode;

	double size;

	bool zbuffer_hide;

public:
				//
	  			// CONSTRUCTORS
				//

	ChGlyphs ()  
			{ 
				draw_mode = GLYPH_POINT;
				size = 0.002;
				zbuffer_hide = true;
			};

	virtual ~ChGlyphs () {};

				//
	  			// FUNCTIONS
				//


			/// Get the way that glyphs must be rendered
	eCh_GlyphType GetDrawMode() {return draw_mode;}

			/// Set the way that glyphs must be rendered
	void SetDrawMode(eCh_GlyphType mmode) 
	{
		draw_mode = mmode;
	}

			
			/// Resize the vectors of data so that memory management is faster
			/// than calling SetGlyphPoint() etc. by starting with zero size vectors 
			/// that will be inflated when needed..
	void Reserve(unsigned int n_glyphs);

			/// Get the number of glyphs
	unsigned int GetNumberOfGlyphs() { return points.size(); }

			/// Get the 'size' (thickness of symbol, depending on the rendering
			/// system) of the glyph symbols
	double GetGlyphsSize() {return size;}

			/// Set the 'size' (thickness of symbol, depending on the rendering
			/// system) of the glyph symbols
	void SetGlyphsSize(double msize) {size = msize;}


			// Set the Z buffer enable/disable (for those rendering systems that can do this)
			// If hide= false, symbols will appear even if hidden by other geometries. Default true.
	void SetZbufferHide(bool mhide) {this->zbuffer_hide = mhide;}
	bool GetZbufferHide() {return this->zbuffer_hide;}



			/// Fast method to set a glyph for GLYPH_POINT draw mode.
			/// If the id is more than the reserved amount of glyphs (see Reserve() ) the vectors are inflated.
	void SetGlyphPoint (unsigned int id, ChVector<> mpoint, ChColor mcolor = ChColor(1,0,0) );

			/// Fast method to set a glyph for GLYPH_VECTOR draw mode.
			/// If the id is more than the reserved amount of glyphs (see Reserve() ) the vectors are inflated.
	void SetGlyphVector(unsigned int id, ChVector<> mpoint, ChVector<> mvector, ChColor mcolor = ChColor(1,0,0) );


};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
