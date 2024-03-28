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
    glyph_length_type = eCh_GlyphLength::CONSTANT;
    glyph_length_prop = "";
    glyph_scalelenght = 0.1;
    glyph_width_type = eCh_GlyphWidth::CONSTANT;
    glyph_width_prop = "";
    glyph_scalewidth = 0.002;
    glyph_basis_type = eCh_GlyphBasis::CONSTANT;
    glyph_basis_prop = "";
    glyph_basis_constant = QUNIT;
    glyph_eigenvalues_type = eCh_GlyphEigenvalues::CONSTANT;
    glyph_eigenvalues_prop = "";
    glyph_eigenvalue_constant = ChVector3d(1, 1, 1);
    glyph_color_type = eCh_GlyphColor::CONSTANT;
    glyph_color_prop = "";
    glyph_color_constant = ChColor(1, 0, 0);
    glyph_colormap_startscale = 0;
    glyph_colormap_endscale = 0;
    vector_tip = true;
    zbuffer_hide = true;
    this->vectors = 0;
    this->colors = 0;
    this->rotations = 0;
    this->eigenvalues = 0;
}

ChGlyphs::~ChGlyphs() {
    for (ChProperty* id : this->m_properties)
        delete (id);
}

void ChGlyphs::SetDrawMode(eCh_GlyphType mmode) {
    draw_mode = mmode;

    switch (draw_mode) {
        case GLYPH_POINT:
            break;
        case GLYPH_VECTOR:
            if (!this->vectors) {
                ChPropertyVector my_vectors;
                my_vectors.name = "V";
                this->AddProperty(my_vectors);
                this->vectors = &((ChPropertyVector*)(m_properties.back()))->data;
            }
            break;
        case GLYPH_COORDSYS:
            if (!this->rotations) {
                ChPropertyQuaternion my_quats;
                my_quats.name = "rot";
                this->AddProperty(my_quats);
                this->rotations = &((ChPropertyQuaternion*)(m_properties.back()))->data;
            }
            break;
        default:
            break;
    }
}

void ChGlyphs::Reserve(unsigned int n_glyphs) {
    points.resize(n_glyphs);

    for (ChProperty* prop : this->m_properties)
        prop->SetSize(n_glyphs);
}

// Fast method to set a glyph for GLYPH_POINT draw mode:
void ChGlyphs::SetGlyphPoint(unsigned int id, ChVector3d mpoint, ChColor mcolor) {
    if (this->draw_mode != GLYPH_POINT)
        SetDrawMode(GLYPH_POINT);

    if (points.size() <= id)
        points.resize(id + 1);
    points[id] = mpoint;

    if (!this->colors) {
        ChPropertyColor my_colors;
        my_colors.name = "color";
        this->AddProperty(my_colors);
        this->colors = &((ChPropertyColor*)(m_properties.back()))->data;
    }

    for (ChProperty* prop : this->m_properties) {
        if (prop->GetSize() <= id)
            prop->SetSize(id + 1);
    }

    (*this->colors)[id] = mcolor;
}

// Fast method to set a glyph for GLYPH_VECTOR draw mode:
void ChGlyphs::SetGlyphVector(unsigned int id, ChVector3d mpoint, ChVector3d mvector, ChColor mcolor) {
    if (this->draw_mode != GLYPH_VECTOR)
        SetDrawMode(GLYPH_VECTOR);

    if (points.size() <= id)
        points.resize(id + 1);
    points[id] = mpoint;

    if (!this->vectors) {
        ChPropertyVector my_vectors;
        my_vectors.name = "V";
        this->AddProperty(my_vectors);
        this->vectors = &((ChPropertyVector*)(m_properties.back()))->data;
    }
    if (!this->colors) {
        ChPropertyColor my_colors;
        my_colors.name = "color";
        this->AddProperty(my_colors);
        this->colors = &((ChPropertyColor*)(m_properties.back()))->data;
    }

    for (ChProperty* prop : this->m_properties) {
        if (prop->GetSize() <= id)
            prop->SetSize(id + 1);
    }

    (*this->vectors)[id] = mvector;
    (*this->colors)[id] = mcolor;
}

// Fast method to set a glyph for GLYPH_VECTOR draw mode, local basis
void ChGlyphs::SetGlyphVectorLocal(unsigned int id,
                                   ChVector3d mpoint,
                                   ChVector3d mvector,
                                   ChQuaternion<> mrot,
                                   ChColor mcolor) {
    if (this->draw_mode != GLYPH_VECTOR)
        SetDrawMode(GLYPH_VECTOR);

    if (points.size() <= id)
        points.resize(id + 1);
    points[id] = mpoint;

    if (!this->vectors) {
        ChPropertyVector my_vectors;
        my_vectors.name = "V";
        this->AddProperty(my_vectors);
        this->vectors = &((ChPropertyVector*)(m_properties.back()))->data;
    }
    if (!this->rotations) {
        ChPropertyQuaternion my_quats;
        my_quats.name = "rot";
        this->AddProperty(my_quats);
        this->rotations = &((ChPropertyQuaternion*)(m_properties.back()))->data;
    }
    if (!this->colors) {
        ChPropertyColor my_colors;
        my_colors.name = "color";
        this->AddProperty(my_colors);
        this->colors = &((ChPropertyColor*)(m_properties.back()))->data;
    }

    for (ChProperty* prop : this->m_properties) {
        if (prop->GetSize() <= id)
            prop->SetSize(id + 1);
    }

    (*this->vectors)[id] = mvector;
    (*this->rotations)[id] = mrot;
    (*this->colors)[id] = mcolor;
}

// Fast method to set a glyph for GLYPH_COORDSYS draw mode.
// If the id is more than the reserved amount of glyphs (see Reserve() ) the csys are inflated.
void ChGlyphs::SetGlyphCoordsys(unsigned int id, ChCoordsys<> mcoord) {
    if (this->draw_mode != GLYPH_COORDSYS)
        SetDrawMode(GLYPH_COORDSYS);

    if (points.size() <= id)
        points.resize(id + 1);
    points[id] = mcoord.pos;

    if (!this->rotations) {
        ChPropertyQuaternion my_quats;
        my_quats.name = "rot";
        this->AddProperty(my_quats);
        this->rotations = &((ChPropertyQuaternion*)(m_properties.back()))->data;
    }

    for (ChProperty* prop : this->m_properties) {
        if (prop->GetSize() <= id)
            prop->SetSize(id + 1);
    }
    (*this->rotations)[id] = mcoord.rot;
}

// Fast method to set a glyph for GLYPH_TENSOR draw mode.
// If the id is more than the reserved amount of glyphs (see Reserve() ) the csys are inflated.
void ChGlyphs::SetGlyphTensor(unsigned int id, ChVector3d mpoint, ChQuaternion<> mbasis, ChVector3d meigenvalues) {
    if (this->draw_mode != GLYPH_TENSOR) {
        SetDrawMode(GLYPH_TENSOR);
        glyph_eigenvalues_type = ChGlyphs::eCh_GlyphEigenvalues::PROPERTY;
        glyph_eigenvalues_prop = "eigenvalues";
    }

    if (points.size() <= id)
        points.resize(id + 1);
    points[id] = mpoint;

    if (!this->rotations) {
        ChPropertyQuaternion my_quats;
        my_quats.name = "rot";
        this->AddProperty(my_quats);
        this->rotations = &((ChPropertyQuaternion*)(m_properties.back()))->data;
    }
    if (!this->eigenvalues) {
        ChPropertyVector my_eigenvalues;
        my_eigenvalues.name = "eigenvalues";
        this->AddProperty(my_eigenvalues);
        this->eigenvalues = &((ChPropertyVector*)(m_properties.back()))->data;
    }

    for (ChProperty* prop : this->m_properties) {
        if (prop->GetSize() <= id)
            prop->SetSize(id + 1);
    }
    (*this->rotations)[id] = mbasis;
    (*this->eigenvalues)[id] = meigenvalues;
}

void ChGlyphs::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChGlyphs>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(points);
    eCh_GlyphType_mapper mmapper;
    archive_out << CHNVP(mmapper(draw_mode), "draw_mode");
    archive_out << CHNVP(zbuffer_hide);
    archive_out << CHNVP(m_properties);
}

void ChGlyphs::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChGlyphs>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(points);
    eCh_GlyphType_mapper mmapper;
    archive_in >> CHNVP(mmapper(draw_mode), "draw_mode");
    archive_in >> CHNVP(zbuffer_hide);
    archive_in >> CHNVP(m_properties);
}

}  // end namespace chrono
