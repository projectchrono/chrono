// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_FORCE_NETWORK_VSG_H
#define CH_FORCE_NETWORK_VSG_H

#include <memory>

#include <vsg/all.h>

#include "chrono/core/ChVector2.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColormap.h"

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

class ChVisualSystemVSG;

/// @addtogroup vsg_module
/// @{

/// Renderer of a force network as a set of segments between arbitrary points.
/// Each segment is given by its two end points and a value. The value is mapped to a color, through a colormap or a
/// linear ramp between two colors, unless an explicit color is given for the segment. Segments are drawn as unlit lines
/// one pixel wide, or as lit cylinders whose radius is constant, mapped from the segment value, or given per segment.
/// All sizes are in world units.
///
/// This renderer is a component for VSG plugins. A plugin creates it, calls Initialize() from its OnBindAssets()
/// function, and passes the segments to draw with SetSegments(), typically from its OnRender() function. The plugin
/// extracts and filters the contact data; the renderer draws the segments it is given. The capacity (maximum number of
/// segments) is fixed at construction: segments beyond the capacity are dropped and counted.
class CH_VSG_API ChForceNetworkVSG {
  public:
    /// Rendering mode for network segments.
    enum class Mode {
        LINES,     ///< unlit lines, one pixel wide
        CYLINDERS  ///< lit cylinders, with radius in world units
    };

    /// Create a force-network renderer for at most the specified number of segments.
    ChForceNetworkVSG(unsigned int capacity);

    ~ChForceNetworkVSG();

    /// Set the rendering mode (default: CYLINDERS).
    void SetMode(Mode mode);

    /// Get the current rendering mode.
    Mode GetMode() const { return m_mode; }

    /// Map segment values to colors with the specified colormap (default: JET).
    void SetColormap(ChColormap::Type type);

    /// Map segment values to colors with a linear ramp between two colors.
    void SetColorRamp(const ChColor& color_min, const ChColor& color_max);

    /// Return true if segment values are mapped to colors with a colormap, false if with a two-color ramp.
    bool UsesColormap() const { return m_use_colormap; }

    /// Get the type of the colormap used to map segment values to colors.
    ChColormap::Type GetColormapType() const { return m_colormap_type; }

    /// Set the range of segment values mapped to colors and radii (default: [0, 1]).
    /// Values outside this range are clamped to it.
    void SetValueRange(double vmin, double vmax);

    /// Get the range of segment values mapped to colors and radii.
    ChVector2d GetValueRange() const { return ChVector2d(m_vmin, m_vmax); }

    /// Set a constant radius for segments drawn as cylinders (default: 0.001).
    void SetRadius(double radius);

    /// Map segment values to cylinder radii, from rmin at the low end of the value range to rmax at the high end.
    void SetRadiusRange(double rmin, double rmax);

    /// Set the number of divisions of the cylinder cross-section (default: 12).
    /// This function must be called before Initialize().
    void SetResolution(int num_divs);

    /// Create the VSG nodes and add them to the scene of the given visual system.
    /// The nodes must exist when the visual system compiles its scene. A plugin therefore calls this function from its
    /// OnBindAssets() function, before the visual system is initialized.
    void Initialize(ChVisualSystemVSG& vsys);

    /// Set the segments drawn from the next frame on.
    /// The arrays hold the data of `num_segments` segments: `points1` and `points2` the two end points (3 floats per
    /// segment), `values` one value per segment, `colors` (optional) an explicit RGB color per segment (3 floats per
    /// segment), and `radii` (optional) an explicit cylinder radius per segment. Explicit colors replace the colors
    /// mapped from the values and explicit radii replace the constant or mapped radii. The `values` array may be null if
    /// explicit colors are given and no radius range is set. Segments beyond the capacity are dropped and counted.
    void SetSegments(unsigned int num_segments, const float* points1, const float* points2, const float* values, const float* colors = nullptr, const float* radii = nullptr);

    /// Show or hide the force network (default: visible).
    void SetVisibility(bool visible);

    /// Return true if the force network is visible.
    bool IsVisible() const { return m_visible; }

    /// Get the maximum number of segments.
    unsigned int GetCapacity() const { return m_capacity; }

    /// Get the number of segments drawn, as set by the last call to SetSegments().
    unsigned int GetNumSegments() const { return m_num_segments; }

    /// Get the number of segments dropped by the last call to SetSegments() because they exceeded the capacity.
    unsigned int GetNumDropped() const { return m_num_dropped; }

    /// Get the total number of segments dropped because they exceeded the capacity, over all calls to SetSegments().
    unsigned long long GetTotalDropped() const { return m_total_dropped; }

  private:
    /// Return the color for the specified normalized value (in [0, 1]).
    vsg::vec4 MapColor(float t) const;

    /// Update the masks of the line and cylinder subgraphs from the mode and the visibility flag.
    void UpdateMasks();

    unsigned int m_capacity;  ///< maximum number of segments
    Mode m_mode;              ///< rendering mode
    bool m_visible;           ///< network visibility

    bool m_use_colormap;                     ///< map values with the colormap (else with the two-color ramp)
    ChColormap::Type m_colormap_type;        ///< colormap type
    std::unique_ptr<ChColormap> m_colormap;  ///< colormap
    ChColor m_color_min;                     ///< ramp color at the low end of the value range
    ChColor m_color_max;                     ///< ramp color at the high end of the value range
    double m_vmin;                           ///< low end of the value range
    double m_vmax;                           ///< high end of the value range
    double m_radius;                         ///< constant cylinder radius
    bool m_map_radius;                       ///< map values to radii?
    double m_rmin;                           ///< cylinder radius at the low end of the value range
    double m_rmax;                           ///< cylinder radius at the high end of the value range
    int m_num_divs;                          ///< number of divisions of the cylinder cross-section

    unsigned int m_num_segments;         ///< number of segments drawn
    unsigned int m_num_dropped;          ///< number of segments dropped at the last call to SetSegments
    unsigned long long m_total_dropped;  ///< total number of segments dropped

    vsg::ref_ptr<vsg::Switch> m_root;              ///< root of the network subgraph (lines, cylinders)
    vsg::ref_ptr<vsg::InstanceNode> m_cylinders;   ///< cylinder instances
    vsg::ref_ptr<vsg::vec3Array> m_translations;   ///< cylinder centers
    vsg::ref_ptr<vsg::quatArray> m_rotations;      ///< cylinder orientations
    vsg::ref_ptr<vsg::vec3Array> m_scales;         ///< cylinder scales (radius, radius, length)
    vsg::ref_ptr<vsg::vec4Array> m_colors;         ///< cylinder colors
    vsg::ref_ptr<vsg::VertexDraw> m_lines;         ///< line list
    vsg::ref_ptr<vsg::vec3Array> m_line_vertices;  ///< line end points
    vsg::ref_ptr<vsg::vec3Array> m_line_colors;    ///< line colors (per end point)
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
