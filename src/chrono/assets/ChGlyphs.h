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

#include "chrono/assets/ChVisualization.h"
#include "chrono/assets/ChColor.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

/// Class for referencing a set of 'glyps', that are simple symbols
/// such as arrows or points to be drawn for showing vector directions etc.
/// Remember that depending on the type of visualization system
/// (POVray, Irrlicht,etc.) this asset might not be supported.

class ChApi ChGlyphs : public ChVisualization {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChGlyphs)

  public:
    enum eCh_GlyphType { 
        GLYPH_POINT = 0, 
        GLYPH_VECTOR, 
        GLYPH_COORDSYS };

    CH_ENUM_MAPPER_BEGIN(eCh_GlyphType);
     CH_ENUM_VAL(GLYPH_POINT);
     CH_ENUM_VAL(GLYPH_VECTOR);
     CH_ENUM_VAL(GLYPH_COORDSYS); 
    CH_ENUM_MAPPER_END(eCh_GlyphType);

  public:
    //
    // DATA
    //
    std::vector<ChVector<double> > points;
    std::vector<ChColor> colors;

    // optional attrs
    std::vector<ChVector<double> > vectors;
    std::vector<ChQuaternion<double> > rotations;

  protected:
    eCh_GlyphType draw_mode;

    double size;

    bool zbuffer_hide;

  public:
    //
    // CONSTRUCTORS
    //

    ChGlyphs() {
        draw_mode = GLYPH_POINT;
        size = 0.002;
        zbuffer_hide = true;
    };

    virtual ~ChGlyphs(){};

    //
    // FUNCTIONS
    //

    /// Get the way that glyphs must be rendered
    eCh_GlyphType GetDrawMode() { return draw_mode; }

    /// Set the way that glyphs must be rendered
    void SetDrawMode(eCh_GlyphType mmode) { draw_mode = mmode; }

    /// Resize the vectors of data so that memory management is faster
    /// than calling SetGlyphPoint() etc. by starting with zero size vectors
    /// that will be inflated when needed..
    void Reserve(unsigned int n_glyphs);

    /// Get the number of glyphs
    size_t GetNumberOfGlyphs() { return points.size(); }

    /// Get the 'size' (thickness of symbol, depending on the rendering
    /// system) of the glyph symbols
    double GetGlyphsSize() { return size; }

    /// Set the 'size' (thickness of symbol, depending on the rendering
    /// system) of the glyph symbols
    void SetGlyphsSize(double msize) { size = msize; }

    // Set the Z buffer enable/disable (for those rendering systems that can do this)
    // If hide= false, symbols will appear even if hidden by other geometries. Default true.
    void SetZbufferHide(bool mhide) { this->zbuffer_hide = mhide; }
    bool GetZbufferHide() { return this->zbuffer_hide; }

    /// Fast method to set a glyph for GLYPH_POINT draw mode.
    /// If the id is more than the reserved amount of glyphs (see Reserve() ) the vectors are inflated.
    void SetGlyphPoint(unsigned int id, ChVector<> mpoint, ChColor mcolor = ChColor(1, 0, 0));

    /// Fast method to set a glyph for GLYPH_VECTOR draw mode.
    /// If the id is more than the reserved amount of glyphs (see Reserve() ) the vectors are inflated.
    void SetGlyphVector(unsigned int id, ChVector<> mpoint, ChVector<> mvector, ChColor mcolor = ChColor(1, 0, 0));

    /// Fast method to set a glyph for GLYPH_COORDSYS draw mode.
    /// If the id is more than the reserved amount of glyphs (see Reserve() ) the csys are inflated.
    void SetGlyphCoordsys(unsigned int id, ChCoordsys<> mcoord);


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChGlyphs>();
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(points);
        marchive << CHNVP(colors);
        marchive << CHNVP(vectors);
        marchive << CHNVP(rotations);
        eCh_GlyphType_mapper mmapper;
        marchive << CHNVP(mmapper(draw_mode),"draw_mode");
        marchive << CHNVP(size);
        marchive << CHNVP(zbuffer_hide);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChGlyphs>();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(points);
        marchive >> CHNVP(colors);
        marchive >> CHNVP(vectors);
        marchive >> CHNVP(rotations);
        eCh_GlyphType_mapper mmapper;
        marchive >> CHNVP(mmapper(draw_mode),"draw_mode");
        marchive >> CHNVP(size);
        marchive >> CHNVP(zbuffer_hide);
    }
};

CH_CLASS_VERSION(ChGlyphs,0)

}  // end namespace chrono

#endif
