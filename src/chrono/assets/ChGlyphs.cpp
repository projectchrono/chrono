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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/assets/ChGlyphs.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChGlyphs)

ChGlyphs::ChGlyphs() {
    draw_mode = GLYPH_POINT;
    size = 0.002;
    zbuffer_hide = true;
}

void ChGlyphs::Reserve(unsigned int n_glyphs) {
    colors.resize(n_glyphs);
    points.resize(n_glyphs);

    switch (this->draw_mode) {
        case GLYPH_POINT:
            vectors.resize(0);
            rotations.resize(0);
            return;
        case GLYPH_VECTOR:
            vectors.resize(n_glyphs);
            rotations.resize(0);
            return;
        case GLYPH_COORDSYS:
            vectors.resize(0);
            rotations.resize(n_glyphs);
            return;
        default:
            vectors.resize(n_glyphs);
            rotations.resize(n_glyphs);
    }
}

// Fast method to set a glyph for GLYPH_POINT draw mode:
void ChGlyphs::SetGlyphPoint(unsigned int id, ChVector<> mpoint, ChColor mcolor) {
    if (points.size() <= id)
        points.resize(id + 1);
    points[id] = mpoint;

    if (colors.size() <= id)
        colors.resize(id + 1);
    colors[id] = mcolor;
}

// Fast method to set a glyph for GLYPH_VECTOR draw mode:
void ChGlyphs::SetGlyphVector(unsigned int id, ChVector<> mpoint, ChVector<> mvector, ChColor mcolor) {
    if (points.size() <= id)
        points.resize(id + 1);
    points[id] = mpoint;

    if (vectors.size() <= id)
        vectors.resize(id + 1);
    vectors[id] = mvector;

    if (colors.size() <= id)
        colors.resize(id + 1);
    colors[id] = mcolor;
}

// Fast method to set a glyph for GLYPH_COORDSYS draw mode.
// If the id is more than the reserved amount of glyphs (see Reserve() ) the csys are inflated.
void ChGlyphs::SetGlyphCoordsys(unsigned int id, ChCoordsys<> mcoord) {
    if (points.size() <= id)
        points.resize(id + 1);
    points[id] = mcoord.pos;

    if (rotations.size() <= id)
        rotations.resize(id + 1);
    rotations[id] = mcoord.rot;
}

void ChGlyphs::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChGlyphs>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(points);
    marchive << CHNVP(colors);
    marchive << CHNVP(vectors);
    marchive << CHNVP(rotations);
    eCh_GlyphType_mapper mmapper;
    marchive << CHNVP(mmapper(draw_mode), "draw_mode");
    marchive << CHNVP(size);
    marchive << CHNVP(zbuffer_hide);
}

void ChGlyphs::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChGlyphs>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(points);
    marchive >> CHNVP(colors);
    marchive >> CHNVP(vectors);
    marchive >> CHNVP(rotations);
    eCh_GlyphType_mapper mmapper;
    marchive >> CHNVP(mmapper(draw_mode), "draw_mode");
    marchive >> CHNVP(size);
    marchive >> CHNVP(zbuffer_hide);
}

}  // end namespace chrono
