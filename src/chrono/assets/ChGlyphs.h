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

#ifndef CHGLYPHS_H
#define CHGLYPHS_H

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/geometry/ChProperty.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a set of 'glyphs', that are simple symbols
/// such as arrows or points to be drawn for showing vector directions etc.
/// Remember that depending on the type of visualization system
/// (POVray, Irrlicht,etc.) this asset might not be supported.
class ChApi ChGlyphs : public ChVisualShape {
  public:
    enum eCh_GlyphType { 
        GLYPH_POINT = 0, 
        GLYPH_VECTOR,
        GLYPH_COORDSYS,
        GLYPH_TENSOR };

    /// Modes vector length, if GLYPH_VECTOR
    enum class eCh_GlyphLength {
        CONSTANT = 0,
        PROPERTY,
    };

    /// Modes for glyph width or size
    enum class eCh_GlyphWidth {
        CONSTANT = 0,
        PROPERTY,
    };

    /// Modes for type of glyph rotation (for GLYPH_COORDSYS mode)
    enum class eCh_GlyphBasis {
        CONSTANT = 0,
        PROPERTY,
    };

    /// Modes for type of glyph eigenvalues (for GLYPH_TENSOR mode)
    enum class eCh_GlyphEigenvalues {
        CONSTANT = 0,
        PROPERTY,
    };

    /// Modes for colorizing glyph 
    enum class eCh_GlyphColor {
        CONSTANT = 0,
        PROPERTY,
    };

    ChGlyphs();
    ~ChGlyphs();


    /// Get the way that glyphs must be rendered
    eCh_GlyphType GetDrawMode() const { return draw_mode; }

    /// Set the way that glyphs must be rendered.
    /// Also automatically add needed properties, if not yet done: 
    /// "V", an array of ChVector<>, for the GLYPH_VECTOR mode
    /// "rot", an array of ChQuaternion<>, for the GLYPH_COORDSYS mode
    /// "color", an array of ChColor, for all modes 
    void SetDrawMode(eCh_GlyphType mmode);

    /// Resize the vectors of data so that memory management is faster
    /// than calling SetGlyphPoint() etc. by starting with zero size vectors
    /// that will be inflated when needed..
    void Reserve(unsigned int n_glyphs);

    /// Get the number of glyphs
    size_t GetNumberOfGlyphs() const { return points.size(); }

    /// Get the 'size' (thickness of symbol, depending on the rendering
    /// system) of the glyph symbols
    double GetGlyphsSize() const { return glyph_scalewidth; }

    /// Set the 'size' (thickness of symbol, depending on the rendering
    /// system) of the glyph symbols
    void SetGlyphsSize(double msize) { glyph_scalewidth = msize; }

    // Set the Z buffer enable/disable (for those rendering systems that can do this)
    // If hide= false, symbols will appear even if hidden by other geometries. Default true.
    void SetZbufferHide(bool mhide) { this->zbuffer_hide = mhide; }
    bool GetZbufferHide() const { return this->zbuffer_hide; }

    /// Fast method to set a glyph for GLYPH_POINT draw mode.
    /// If the id is more than the reserved amount of glyphs (see Reserve() ) the vectors are inflated.
    void SetGlyphPoint(unsigned int id, ChVector<> mpoint, ChColor mcolor = ChColor(1, 0, 0));

    /// Fast method to set a glyph for GLYPH_VECTOR draw mode.
    /// If the id is more than the reserved amount of glyphs (see Reserve() ) the vectors are inflated.
    void SetGlyphVector(unsigned int id, ChVector<> mpoint, ChVector<> mvector, ChColor mcolor = ChColor(1, 0, 0));

    /// Fast method to set a glyph for GLYPH_VECTOR draw mode, with each vectors set in its own local (rotated) basis
    /// If the id is more than the reserved amount of glyphs (see Reserve() ) the vectors are inflated.
    void SetGlyphVectorLocal(unsigned int id, ChVector<> mpoint, ChVector<> mvector, ChQuaternion<> mrot, ChColor mcolor = ChColor(1, 0, 0));

    /// Fast method to set a glyph for GLYPH_COORDSYS draw mode.
    /// If the id is more than the reserved amount of glyphs (see Reserve() ) the csys are inflated.
    void SetGlyphCoordsys(unsigned int id, ChCoordsys<> mcoord);

    /// Fast method to set a glyph for GLYPH_TENSOR draw mode.
    /// If the id is more than the reserved amount of glyphs (see Reserve() ) the tensors are inflated.
    void SetGlyphTensor(unsigned int id, ChVector<> mpoint, ChQuaternion<> mbasis, ChVector<> eigenvalues);

    /// Gets the current array of per-point properties, if any.
    std::vector<geometry::ChProperty*> getProperties() { return m_properties; }

    /// Add a property as an array of data per-point. Deletion will be automatic at the end of mesh life.
    /// Warning: mprop.data.size() must be equal to points.size().  Cost: allocation and a data copy. 
    void AddProperty(geometry::ChProperty& mprop) { m_properties.push_back(mprop.clone());}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    /// @cond
    CH_ENUM_MAPPER_BEGIN(eCh_GlyphType);
    CH_ENUM_VAL(GLYPH_POINT);
    CH_ENUM_VAL(GLYPH_VECTOR);
    CH_ENUM_VAL(GLYPH_COORDSYS);
    CH_ENUM_VAL(GLYPH_TENSOR);
    CH_ENUM_MAPPER_END(eCh_GlyphType);
    /// @endcond

    std::vector<ChVector<double> > points;

    eCh_GlyphLength glyph_length_type;
    std::string glyph_length_prop; // selected property for length
    double glyph_scalelenght;
    eCh_GlyphWidth  glyph_width_type;
    std::string glyph_width_prop; // selected property for width
    double glyph_scalewidth;
    eCh_GlyphBasis  glyph_basis_type;
    std::string glyph_basis_prop; // selected property for basis
    ChQuaternion<> glyph_basis_constant;
    eCh_GlyphEigenvalues  glyph_eigenvalues_type;
    std::string glyph_eigenvalues_prop; // selected property for eigs
    ChVector<> glyph_eigenvalue_constant;
    eCh_GlyphColor  glyph_color_type;
    std::string glyph_color_prop; // selected property for color
    ChColor glyph_color_constant;
    double  glyph_colormap_startscale;
    double  glyph_colormap_endscale;
    bool vector_tip;

    // list of per-glyph properties. Some properties ("color", "V", "rot") might be automatically added when needed.
    std::vector<geometry::ChProperty*> m_properties;

    // shortcut pointers to data of properties in m_properties, for often needed data:
    std::vector<ChColor>* colors;
    std::vector<ChVector<double> >* vectors;
    std::vector<ChQuaternion<double> >* rotations;
    std::vector<ChVector<double> >* eigenvalues;

  private:
    eCh_GlyphType draw_mode;
    double size;
    bool zbuffer_hide;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChGlyphs, 0)

}  // end namespace chrono

#endif
