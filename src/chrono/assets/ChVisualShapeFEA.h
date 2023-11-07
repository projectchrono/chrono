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

#ifndef CH_VISUAL_SHAPE_FEA_H
#define CH_VISUAL_SHAPE_FEA_H

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

// Forward declarations
namespace fea {
class ChMesh;
class ChMeshSurface;
class ChContactSurface;
class ChNodeFEAxyz;
class ChNodeFEAxyzP;
class ChElementBase;
}  // namespace fea

/// FEA mesh visualization.
/// Adds to the containing visual model a trimesh and a glyphs visual shapes.
class ChApi ChVisualShapeFEA {
  public:
    /// Visualization mesh data type.  
    enum class DataType {
        NONE,
        SURFACE,
        CONTACTSURFACES,
        LOADSURFACES,
        NODE_DISP_NORM,
        NODE_DISP_X,
        NODE_DISP_Y,
        NODE_DISP_Z,
        NODE_SPEED_NORM,
        NODE_SPEED_X,
        NODE_SPEED_Y,
        NODE_SPEED_Z,
        NODE_ACCEL_NORM,
        NODE_ACCEL_X,
        NODE_ACCEL_Y,
        NODE_ACCEL_Z,
        ELEM_STRAIN_VONMISES,
        ELEM_STRESS_VONMISES,
        ELEM_STRAIN_HYDROSTATIC,
        ELEM_STRESS_HYDROSTATIC,
        ELEM_BEAM_MX,
        ELEM_BEAM_MY,
        ELEM_BEAM_MZ,
        ELEM_BEAM_TX,
        ELEM_BEAM_TY,
        ELEM_BEAM_TZ,
        NODE_P,
        ANCF_BEAM_AX,
        ANCF_BEAM_BD
    };

    /// Visualization glyph data type.
    enum class GlyphType {
        NONE,
        NODE_DOT_POS,
        NODE_CSYS,
        NODE_VECT_SPEED,
        NODE_VECT_ACCEL,
        ELEM_TENS_STRAIN,
        ELEM_TENS_STRESS,
        ELEM_VECT_DP  // gradient field for Poisson problems
    };

  public:
    ChVisualShapeFEA(std::shared_ptr<fea::ChMesh> fea_mesh);

    ~ChVisualShapeFEA() {}

    /// Access the referenced FEM mesh.
    fea::ChMesh& GetMesh() { return *FEMmesh; }

    /// Returns the current data type to be plotted (speeds, forces, etc.).
    DataType GetFEMdataType() { return fem_data_type; }

    /// Set the current data type to be plotted (speeds, forces, etc.).
    void SetFEMdataType(DataType mdata) { fem_data_type = mdata; }

    /// Returns the current data type to be drawn with glyphs.
    GlyphType GetFEMglyphType() { return fem_glyph; }

    /// Set the current data type to be drawn with glyphs.
    void SetFEMglyphType(GlyphType mdata) { fem_glyph = mdata; }

    /// Set min and max values of the plotted variable for the colorscale plots.
    void SetColorscaleMinMax(double min, double max) {
        colorscale_min = min;
        colorscale_max = max;
    }

    /// Set the scale for drawing the symbols for vectors, tensors, etc.
    void SetSymbolsScale(double mscale) { this->symbols_scale = mscale; }
    double GetSymbolsScale() { return this->symbols_scale; }

    /// Set the thickness of symbols used for drawing the vectors, tensors, etc.
    void SetSymbolsThickness(double mthick) { this->symbols_thickness = mthick; }
    double GetSymbolsThickness() { return this->symbols_thickness; }

    /// Set the resolution of beam triangulated drawing, along the beam direction (default: 8).
    void SetBeamResolution(int mres) { this->beam_resolution = mres; }
    int GetBeamResolution() { return this->beam_resolution; }

    /// Set the resolution of beam triangulated drawing, along the section (default: 10).
    /// For example, in case of a circular section, this is the number of points along the circle.
    void SetBeamResolutionSection(int mres) { this->beam_resolution_section = mres; }
    int GetBeamResolutionSection() { return this->beam_resolution_section; }

    /// Set the resolution of shell triangulated drawing (default: 2).
    /// This value represents the number of visualization mesh vertices on each FEM element edge.
    /// The default value of 2 results in a visualization mesh constructed using only the FEM nodes.
    void SetShellResolution(int mres) { this->shell_resolution = mres; }
    int GetShellResolution() { return this->shell_resolution; }

    /// Set shrinkage of elements during drawing.
    void SetShrinkElements(bool mshrink, double mfact) {
        shrink_elements = mshrink;
        shrink_factor = mfact;
    }

    /// Set as wireframe visualization.
    void SetWireframe(bool mwireframe) { this->wireframe = mwireframe; }

    /// Set backface cull speedup (default false).
    /// Must be set true for shells and in general where already double-sided twin triangles are used.
    void SetBackfaceCull(bool mbc) { this->backface_cull = mbc; }

    /// Set the Z buffer enable/disable, for visualization systems that support it (default: true).
    /// If hide = false, symbols will appear even if hidden by meshes/geometries.
    void SetZbufferHide(bool mhide) { this->zbuffer_hide = mhide; }

    /// Set color for SURFACE mode (also for wireframe lines).
    void SetDefaultMeshColor(ChColor mcolor) { this->meshcolor = mcolor; }

    /// Set color for GlyphType::NONE mode or for wireframe lines.
    void SetDefaultSymbolsColor(ChColor mcolor) { this->symbolscolor = mcolor; }

    /// Activate Gourad or Phong smoothing for faces of non-straight elements.
    /// Note: experimental feature.
    void SetSmoothFaces(bool msmooth) { this->smooth_faces = msmooth; }

    /// Draw the mesh in its underformed (reference) configuration.
    void SetDrawInUndeformedReference(bool mdu) { this->undeformed_reference = mdu; }

    /// Update the triangle visualization mesh so that it matches with the FEM mesh.
    void Update(ChPhysicsItem* updater, const ChFrame<>& frame);

  private:
    double ComputeScalarOutput(std::shared_ptr<fea::ChNodeFEAxyz> mnode,
                               int nodeID,
                               std::shared_ptr<fea::ChElementBase> melement);
    double ComputeScalarOutput(std::shared_ptr<fea::ChNodeFEAxyzP> mnode,
                               int nodeID,
                               std::shared_ptr<fea::ChElementBase> melement);
    ChColor ComputeFalseColor(double in);

    // Helper functions for updateing buffers of specific element types
    void UpdateBuffers_Tetrahedron(std::shared_ptr<fea::ChElementBase> element,
                                   geometry::ChTriangleMeshConnected& trianglemesh,
                                   unsigned int& i_verts,
                                   unsigned int& i_vnorms,
                                   unsigned int& i_vcols,
                                   unsigned int& i_triindex,
                                   bool& need_automatic_smoothing);
    void UpdateBuffers_Tetra_4_P(std::shared_ptr<fea::ChElementBase> element,
                                 geometry::ChTriangleMeshConnected& trianglemesh,
                                 unsigned int& i_verts,
                                 unsigned int& i_vnorms,
                                 unsigned int& i_vcols,
                                 unsigned int& i_triindex,
                                 bool& need_automatic_smoothing);
    void UpdateBuffers_Hex(std::shared_ptr<fea::ChElementBase> element,
                           geometry::ChTriangleMeshConnected& trianglemesh,
                           unsigned int& i_verts,
                           unsigned int& i_vnorms,
                           unsigned int& i_vcols,
                           unsigned int& i_triindex,
                           bool& need_automatic_smoothing);
    void UpdateBuffers_Beam(std::shared_ptr<fea::ChElementBase> element,
                            geometry::ChTriangleMeshConnected& trianglemesh,
                            unsigned int& i_verts,
                            unsigned int& i_vnorms,
                            unsigned int& i_vcols,
                            unsigned int& i_triindex,
                            bool& need_automatic_smoothing);
    void UpdateBuffers_Shell(std::shared_ptr<fea::ChElementBase> element,
                             geometry::ChTriangleMeshConnected& trianglemesh,
                             unsigned int& i_verts,
                             unsigned int& i_vnorms,
                             unsigned int& i_vcols,
                             unsigned int& i_triindex,
                             bool& need_automatic_smoothing);
    void UpdateBuffers_LoadSurface(std::shared_ptr<fea::ChMeshSurface> surface,
                                   geometry::ChTriangleMeshConnected& trianglemesh,
                                   unsigned int& i_verts,
                                   unsigned int& i_vnorms,
                                   unsigned int& i_vcols,
                                   unsigned int& i_triindex,
                                   bool& need_automatic_smoothing);
    void UpdateBuffers_ContactSurfaceMesh(std::shared_ptr<fea::ChContactSurface> surface,
                                          geometry::ChTriangleMeshConnected& trianglemesh,
                                          unsigned int& i_verts,
                                          unsigned int& i_vnorms,
                                          unsigned int& i_vcols,
                                          unsigned int& i_triindex,
                                          bool& need_automatic_smoothing);

    std::shared_ptr<fea::ChMesh> FEMmesh;

    DataType fem_data_type;
    GlyphType fem_glyph;

    double colorscale_min;
    double colorscale_max;

    double symbols_scale;
    double symbols_thickness;

    bool shrink_elements;
    double shrink_factor;

    bool wireframe;
    bool backface_cull;

    bool zbuffer_hide;

    bool smooth_faces;

    bool undeformed_reference;

    int beam_resolution;
    int beam_resolution_section;
    int shell_resolution;

    ChColor meshcolor;
    ChColor symbolscolor;

    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;
    std::shared_ptr<ChGlyphs> m_glyphs_shape;

    std::vector<int> normal_accumulators;

    friend class ChVisualModel;
};

/// @} chrono_assets

}  // end namespace chrono

#endif
