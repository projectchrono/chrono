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

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "chrono/assets/ChColormap.h"
#include "chrono/geometry/ChAABB.h"

#include "chrono_multicore/ChApiMulticore.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/utils/ChForceNetworkVSG.h"
#include "chrono_vsg/utils/ChInstancedShapesVSG.h"

namespace chrono {

/// @addtogroup multicore_visualization
/// @{

/// VSG-based run-time visualization plugin for Chrono::Multicore systems.
/// The plugin draws the granular bodies of a Multicore system as GPU instances (see vsg3d::ChInstancedShapesVSG) and
/// the contact force network (see vsg3d::ChForceNetworkVSG), writes images while the simulation runs, and shows a panel
/// with Multicore timers, contact counts, and solver statistics.
///
/// The force network includes all rigid-rigid contacts with a positive normal force, contacts with walls included.
/// Each contact is drawn as two half-segments, from each body center (or the center of the contact shape) to the
/// contact point, midway between the two contact points of the shapes. A half-segment is not drawn if it starts at a
/// large body (a body larger than the size limit of the automatic body selection, such as a wall) or if it is longer
/// than a maximum length (by default, the median body size). The network is colored by the normal force, the Coulomb
/// ratio (tangential force over its Coulomb limit), or the magnitude of the stored tangential displacement (SMC with
/// the MultiStep tangential displacement model). A force threshold and a region filter (slab and box) select the
/// contacts that are drawn; a slab gives section cuts.
///
/// The plugin must be attached to the VSG visual system before the visual system is initialized. It attaches the
/// Multicore system to the visual system if that was not done already.
class CH_MULTICORE_API ChMulticoreVisualizationVSG : public vsg3d::ChVisualSystemVSGPlugin {
  public:
    /// Quantity used to color the force network.
    enum class NetworkColoring {
        NORMAL_FORCE,            ///< normal contact force
        COULOMB_RATIO,           ///< tangential contact force over its Coulomb limit
        TANGENTIAL_DISPLACEMENT  ///< magnitude of the stored tangential displacement (SMC, MultiStep model)
    };

    /// Start point of the two half-segments of a contact.
    enum class NetworkAnchor {
        BODY_CENTER,  ///< center of mass of the body
        SHAPE_CENTER  ///< center of the contact shape (e.g., a component sphere of a clump)
    };

    ChMulticoreVisualizationVSG(ChSystemMulticore* sys);

    ~ChMulticoreVisualizationVSG();

    // ---- Granular bodies

    /// Enable or disable drawing of granular bodies as GPU instances (default: true).
    /// If disabled, all bodies keep the default binding of the visual system. This function must be called before the
    /// visual system is initialized.
    void EnableBodyInstancing(bool val) { m_use_instancing = val; }

    /// Set the size factor for the automatic selection of granular bodies (default: 10).
    /// Bodies whose size is at most this factor times the median body size are drawn as instances. The resulting size
    /// limit also identifies large bodies for the force network. This function must be called before the visual system
    /// is initialized.
    void SetBodySizeFactor(double factor);

    /// Set an explicit list of bodies drawn as instances, replacing the automatic selection.
    /// This function must be called before the visual system is initialized.
    void SetBodySelection(const std::vector<std::shared_ptr<ChBody>>& bodies) { m_shapes.SetBodies(bodies); }

    /// Set a predicate selecting the bodies drawn as instances, replacing the automatic selection.
    /// This function must be called before the visual system is initialized.
    void SetBodySelector(std::function<bool(const ChBody& body)> selector) { m_shapes.SetBodySelector(selector); }

    /// Set the source of the shapes drawn for each granular body (default: COLLISION).
    /// This function must be called before the visual system is initialized.
    void SetBodyShapeSource(vsg3d::ChInstancedShapesVSG::ShapeSource source);

    /// Set the number of divisions of a full circle used to tessellate round body shapes (default: 24).
    /// This function must be called before the visual system is initialized.
    void SetBodyResolution(int num_divs) { m_shapes.SetResolution(num_divs); }

    /// Set the color of granular bodies whose visual model defines no color (default: [0.75, 0.75, 0.75]).
    /// This function must be called before the visual system is initialized.
    void SetBodyDefaultColor(const ChColor& color) { m_shapes.SetDefaultColor(color); }

    /// Set a callback for the colors of granular bodies, replacing the colors from the visual models.
    void SetBodyColorCallback(std::shared_ptr<vsg3d::ChInstancedShapesVSG::ColorCallback> callback) { m_shapes.SetColorCallback(callback); }

    /// Set a callback for the visibility of granular bodies.
    void SetBodyVisibilityCallback(std::shared_ptr<vsg3d::ChInstancedShapesVSG::VisibilityCallback> callback) { m_body_visibility = callback; }

    /// Enable or disable drawing of inactive (fixed or sleeping) granular bodies (default: false).
    void SetInactiveBodyVisibility(bool visible) { m_shapes.SetInactiveBodyVisibility(visible); }

    /// Show or hide the granular bodies (default: visible).
    void SetBodyVisibility(bool visible) { m_shapes.SetVisibility(visible); }

    // ---- Force network

    /// Show or hide the force network (default: hidden).
    void SetNetworkVisibility(bool visible) { m_network_visible = visible; }

    /// Set the rendering mode of the force network (default: CYLINDERS).
    void SetNetworkMode(vsg3d::ChForceNetworkVSG::Mode mode) { m_network_mode = mode; }

    /// Set the quantity used to color the force network (default: NORMAL_FORCE).
    void SetNetworkColoring(NetworkColoring quantity) { m_coloring = quantity; }

    /// Set the quantity used to color the force network and the range of values mapped to colors.
    /// Without an explicit range, the range is [0, 1] for the Coulomb ratio and, for the other quantities, from 0 to the
    /// largest value at the current frame.
    void SetNetworkColoring(NetworkColoring quantity, double vmin, double vmax);

    /// Map network values to colors with the specified colormap (default: JET).
    void SetNetworkColormap(ChColormap::Type type);

    /// Map network values to colors with a linear ramp between two colors.
    void SetNetworkColorRamp(const ChColor& color_min, const ChColor& color_max);

    /// Set a constant radius for network segments drawn as cylinders (default: 0.1 times the median body size).
    void SetNetworkRadius(double radius);

    /// Scale the radius of network segments with the normal force, from rmin at fmin to rmax at fmax (clamped).
    void SetNetworkRadius(double rmin, double rmax, double fmin, double fmax);

    /// Set the force threshold of the network (default: 0).
    /// Only contacts with a normal force larger than the threshold are drawn. If `relative` is true, the threshold is a
    /// multiple of the mean normal force over all contacts with a positive normal force.
    void SetNetworkForceThreshold(double threshold, bool relative = false);

    /// Set the maximum length of a network half-segment (default: the median body size).
    void SetNetworkMaxHalfLength(double length) { m_max_half_length = length; }

    /// Set the start point of the network half-segments (default: BODY_CENTER).
    void SetNetworkAnchor(NetworkAnchor anchor) { m_anchor_type = anchor; }

    /// Set the maximum number of network half-segments (default: 12 times the number of bodies, at most 2e6).
    /// Half-segments beyond this capacity are dropped and counted. This function must be called before the visual
    /// system is initialized.
    void SetNetworkCapacity(unsigned int num_segments) { m_capacity = num_segments; }

    // ---- Region filter

    /// Restrict drawing to a slab of the specified half-thickness, centered at a point and normal to a direction.
    /// A contact is drawn if its contact point is in the slab. With SetRegionBodyFiltering(true), a granular body is
    /// drawn if its center is in the slab.
    void SetRegionSlab(const ChVector3d& point, const ChVector3d& normal, double half_thickness);

    /// Restrict drawing to an axis-aligned box.
    /// A contact is drawn if its contact point is in the box. With SetRegionBodyFiltering(true), a granular body is
    /// drawn if its center is in the box. A box and a slab can be combined.
    void SetRegionBox(const ChAABB& box);

    /// Remove the slab and box region filters.
    void ClearRegion();

    /// Apply the region filter to granular bodies as well as to the force network (default: false).
    void SetRegionBodyFiltering(bool val) { m_region_bodies = val; }

    // ---- Image output

    /// Set the output directory for images (default: ".").
    /// The directory is created if it does not exist.
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable or disable writing of images (default: false).
    /// Images are written to the output directory as img_00000.bmp, img_00001.bmp, and so on (consecutive numbers).
    /// The image of a frame is written during the next call to Render(), so the last frame of a run is written only if
    /// Render() is called once more.
    void SetImageOutput(bool val) { m_write_images = val; }

    /// Write the image of every n-th rendered frame (default: 1).
    void SetImageOutputInterval(unsigned int interval);

    /// Set the image format, given as a file extension (default: "bmp").
    /// The supported formats are those of vsgXchange (e.g., bmp, png, jpg, tga). Writing a PNG image is much slower
    /// than writing a BMP image.
    void SetImageOutputFormat(const std::string& extension) { m_image_format = extension; }

    /// Get the number of images written (requested) so far.
    unsigned int GetNumImages() const { return m_num_images; }

    // ---- Information

    /// Get the number of network half-segments drawn at the current frame.
    unsigned int GetNumNetworkSegments() const;

    /// Get the number of network half-segments dropped at the current frame because they exceeded the capacity.
    unsigned int GetNumNetworkSegmentsDropped() const;

    /// Get the number of bodies drawn as instances.
    unsigned int GetNumInstancedBodies() const { return static_cast<unsigned int>(m_shapes.GetBodies().size()); }

    /// Get the median body size, as computed at initialization.
    double GetMedianBodySize() const { return m_median_size; }

    virtual void OnAttach() override;
    virtual void OnInitialize() override;
    virtual void OnBindAssets() override;
    virtual void OnRender() override;

  private:
    /// Return true if the given point is in the region filter.
    bool InRegion(const ChVector3d& p) const;

    /// Update the force network from the current contact data.
    void UpdateNetwork();

    /// Return true if the body with the given index can anchor network half-segments (it is not a large body).
    bool IsAnchor(unsigned int body);

    ChSystemMulticore* m_sys;  ///< associated Multicore system

    // Granular bodies
    bool m_use_instancing;                                                               ///< draw granular bodies as instances?
    double m_size_factor;                                                                ///< size factor of the automatic selection
    vsg3d::ChInstancedShapesVSG::ShapeSource m_source;                                   ///< source of the body shapes
    vsg3d::ChInstancedShapesVSG m_shapes;                                                ///< instanced granular bodies
    std::shared_ptr<vsg3d::ChInstancedShapesVSG::VisibilityCallback> m_body_visibility;  ///< user body visibility callback
    double m_median_size;                                                                ///< median body size
    double m_size_limit;                                                                 ///< size limit for granular bodies
    std::vector<char> m_is_anchor;                                                       ///< per body: can anchor half-segments?

    // Force network
    std::unique_ptr<vsg3d::ChForceNetworkVSG> m_network;  ///< force network renderer
    unsigned int m_capacity;                              ///< maximum number of half-segments (0: automatic)
    bool m_network_visible;                               ///< draw the force network?
    vsg3d::ChForceNetworkVSG::Mode m_network_mode;        ///< rendering mode
    NetworkColoring m_coloring;                           ///< coloring quantity
    std::array<ChVector2d, 3> m_ranges;                   ///< ranges of the coloring quantities (automatic if empty)
    bool m_use_colormap;                                  ///< map values with the colormap (else with the ramp)
    ChColormap::Type m_colormap_type;                     ///< colormap
    ChColor m_color_min;                                  ///< ramp color at the low end of the range
    ChColor m_color_max;                                  ///< ramp color at the high end of the range
    double m_radius;                                      ///< constant radius (0: automatic)
    bool m_scale_radius;                                  ///< scale the radius with the normal force?
    double m_rmin, m_rmax;                                ///< radius range
    double m_fmin, m_fmax;                                ///< normal force range for the radius scaling
    double m_threshold;                                   ///< normal force threshold
    bool m_relative_threshold;                            ///< threshold relative to the mean normal force?
    double m_max_half_length;                             ///< maximum half-segment length (0: automatic)
    NetworkAnchor m_anchor_type;                          ///< start point of the half-segments
    ChVector2d m_current_range;                           ///< range of values mapped to colors at the current frame
    unsigned int m_num_contacts_drawn;                    ///< number of contacts drawn at the current frame

    // Region filter
    bool m_use_slab;               ///< slab filter active?
    ChVector3d m_slab_point;       ///< point on the slab mid-plane
    ChVector3d m_slab_normal;      ///< slab normal
    double m_slab_half_thickness;  ///< slab half-thickness
    bool m_use_box;                ///< box filter active?
    ChAABB m_box;                  ///< box
    bool m_region_bodies;          ///< filter the granular bodies too?

    // Image output
    bool m_write_images;            ///< write images?
    std::string m_image_dir;        ///< image output directory
    std::string m_image_format;     ///< image format (file extension)
    unsigned int m_image_interval;  ///< write every n-th frame
    unsigned int m_num_images;      ///< number of images written
    unsigned int m_num_frames;      ///< number of rendered frames

    // Work arrays for the force network
    std::vector<float> m_points1;
    std::vector<float> m_points2;
    std::vector<float> m_values;
    std::vector<float> m_radii;
    std::vector<double> m_fn;
    std::vector<double> m_ft;

    friend class MulticoreStatsVSG;
    friend class MulticoreBodyVisibility;
};

/// @} multicore_visualization

}  // namespace chrono
