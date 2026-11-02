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

#ifndef CH_INSTANCED_SHAPES_VSG_H
#define CH_INSTANCED_SHAPES_VSG_H

#include <functional>
#include <memory>
#include <unordered_set>
#include <vector>

#include <vsg/all.h>

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

class ChVisualSystemVSG;

/// @addtogroup vsg_module
/// @{

/// Renderer of granular bodies as GPU instances of shape templates.
/// Each selected body is decomposed into its collision shapes (or, optionally, its visual shapes) and each shape is
/// drawn as an instance of a template. Primitives that scale (sphere, ellipsoid, box, cylinder, cone) share one unit
/// template per type, with a per-instance scale. Shapes that do not scale (capsules, rounded boxes and cylinders,
/// triangle meshes, convex hulls) get one template per distinct geometry. The world transform of an instance is the
/// body reference frame times the shape frame, so bodies with several shapes (e.g., clumps of spheres) are drawn as
/// one instance per shape. All shapes of a body have the color of the body.
///
/// By default, the bodies drawn as instances are selected automatically by size: among the bodies that are active at
/// initialization and whose shapes are all supported, those whose size (largest dimension of the bounding box of their
/// shapes) is at most a factor (default: 10) times the median body size. Larger bodies (walls, containers, platens)
/// keep the default binding of the visual system. An explicit selection (a list of bodies or a predicate) replaces the
/// automatic one. The selected bodies are excluded from the default binding of the visual system.
///
/// Inactive bodies (fixed or sleeping) are hidden by default. Per-body color and visibility callbacks are supported.
///
/// This renderer is a component for VSG plugins. A plugin creates it, calls Initialize() from its OnBindAssets()
/// function, and calls Update() at every frame, typically from its OnRender() function.
class CH_VSG_API ChInstancedShapesVSG {
  public:
    /// Source of the shapes drawn for each body.
    enum class ShapeSource {
        COLLISION,  ///< shapes in the body collision model
        VISUAL      ///< shapes in the body visual model
    };

    /// Callback interface for per-body colors.
    class CH_VSG_API ColorCallback {
      public:
        virtual ~ColorCallback() {}

        /// Return the color of the specified body.
        virtual ChColor GetColor(const ChBody& body) const = 0;
    };

    /// Callback interface for per-body visibility.
    class CH_VSG_API VisibilityCallback {
      public:
        virtual ~VisibilityCallback() {}

        /// Return true if the specified body is visible.
        virtual bool IsVisible(const ChBody& body) const = 0;
    };

    ChInstancedShapesVSG();

    ~ChInstancedShapesVSG();

    /// Set the source of the shapes drawn for each body (default: COLLISION).
    /// This function must be called before Initialize().
    void SetShapeSource(ShapeSource source) { m_source = source; }

    /// Set the size factor for the automatic selection of bodies (default: 10).
    /// Bodies whose size is at most this factor times the median body size are drawn as instances. This function must
    /// be called before Initialize().
    void SetSizeFactor(double factor) { m_size_factor = factor; }

    /// Set an explicit list of bodies drawn as instances, replacing the automatic selection.
    /// This function must be called before Initialize().
    void SetBodies(const std::vector<std::shared_ptr<ChBody>>& bodies);

    /// Set a predicate selecting the bodies drawn as instances, replacing the automatic selection.
    /// This function must be called before Initialize().
    void SetBodySelector(std::function<bool(const ChBody& body)> selector) { m_selector = selector; }

    /// Set the number of divisions of a full circle used to tessellate round templates (default: 24).
    /// This function must be called before Initialize().
    void SetResolution(int num_divs);

    /// Set the material of the instanced shapes (default: ChVisualMaterial::Default()).
    /// The body colors replace the diffuse color of this material. This function must be called before Initialize().
    void SetMaterial(std::shared_ptr<ChVisualMaterial> material) { m_material = material; }

    /// Set the color of bodies whose visual model defines no color (default: [0.75, 0.75, 0.75]).
    /// Without a color callback, a body has the diffuse color of the first material in its visual model, if any, and
    /// this color otherwise. This function must be called before Initialize().
    void SetDefaultColor(const ChColor& color) { m_default_color = color; }

    /// Set a callback for the body colors, replacing the colors from the visual models.
    void SetColorCallback(std::shared_ptr<ColorCallback> callback) { m_color_callback = callback; }

    /// Set a callback for the body visibility.
    /// A body is drawn if the callback returns true and, unless SetInactiveBodyVisibility(true) was called, the body is
    /// active.
    void SetVisibilityCallback(std::shared_ptr<VisibilityCallback> callback) { m_visibility_callback = callback; }

    /// Enable or disable drawing of inactive (fixed or sleeping) bodies (default: false).
    void SetInactiveBodyVisibility(bool visible) { m_show_inactive = visible; }

    /// Show or hide all instanced bodies (default: visible).
    void SetVisibility(bool visible);

    /// Return true if the instanced bodies are visible.
    bool IsVisible() const { return m_visible; }

    /// Select the bodies drawn as instances, create the templates, and add the instance nodes to the visual system.
    /// The candidates are typically all bodies in the system. The selected bodies are excluded from the default
    /// binding of the visual system, so this function must be called before the visual system binds its assets; a
    /// plugin calls it from its OnBindAssets() function, before the visual system is initialized.
    void Initialize(ChVisualSystemVSG& vsys, const std::vector<std::shared_ptr<ChBody>>& candidates);

    /// Update the instances from the current body states, colors, and visibility.
    void Update();

    /// Get the bodies drawn as instances.
    const std::vector<std::shared_ptr<ChBody>>& GetBodies() const { return m_bodies; }

    /// Return true if the specified body is drawn as instances.
    bool IsInstanced(const ChBody& body) const { return m_body_set.count(&body) > 0; }

    /// Get the median size of the candidate bodies, as computed by Initialize().
    double GetMedianSize() const { return m_median_size; }

    /// Get the size limit for the automatic selection (the size factor times the median size).
    double GetSizeLimit() const { return m_size_factor * m_median_size; }

    /// Get the number of templates.
    unsigned int GetNumTemplates() const { return static_cast<unsigned int>(m_templates.size()); }

    /// Get the total number of shape instances (over all bodies drawn as instances).
    unsigned int GetNumInstances() const { return m_num_instances; }

    /// Get the number of shape instances drawn at the last update.
    unsigned int GetNumVisibleInstances() const { return m_num_visible; }

    /// Get the size of a body: the largest dimension of the bounding box of its shapes, in the body reference frame.
    /// Returns 0 if the body has no shapes from the specified source.
    static double GetBodySize(const ChBody& body, ShapeSource source);

  private:
    /// Shape instance of a body.
    struct Instance {
        unsigned int body;  ///< index of the body in the list of bodies drawn as instances
        ChFramed frame;     ///< shape frame, relative to the body reference frame
        bool identity;      ///< true if the shape frame is the identity
        vsg::vec3 scale;    ///< scale applied to the template
    };

    /// Template and its instances.
    struct Template {
        bool double_faced;                          ///< draw both faces of the template triangles?
        vsg::ref_ptr<vsg::InstanceNode> node;       ///< instance node
        vsg::ref_ptr<vsg::vec3Array> translations;  ///< instance positions
        vsg::ref_ptr<vsg::quatArray> rotations;     ///< instance orientations
        vsg::ref_ptr<vsg::vec3Array> scales;        ///< instance scales
        vsg::ref_ptr<vsg::vec4Array> colors;        ///< instance colors
        std::vector<Instance> instances;            ///< shape instances drawn with this template
    };

    /// Geometry of a template (vertices, normals, indices).
    struct Geometry {
        vsg::ref_ptr<vsg::vec3Array> vertices;
        vsg::ref_ptr<vsg::vec3Array> normals;
        vsg::ref_ptr<vsg::Data> indices;
    };

    /// Create the geometry of a triangle mesh, as a triangle soup with face normals.
    static Geometry CreateMeshGeometry(const ChTriangleMeshConnected& mesh);

    /// Shape of a body, mapped to a template.
    struct ShapeInfo {
        int kind;                          ///< template kind (unit primitive or distinct geometry)
        std::vector<double> params;        ///< dimensions of a distinct geometry
        const void* key;                   ///< object defining a distinct geometry (mesh or convex hull)
        std::function<Geometry()> create;  ///< function creating the template geometry
        ChFramed frame;                    ///< shape frame, relative to the body reference frame
        vsg::vec3 scale;                   ///< scale applied to the template
    };

    /// Decompose a body into shapes mapped to templates; return false if a shape is not supported.
    bool GetShapes(const ChBody& body, std::vector<ShapeInfo>& shapes) const;

    ShapeSource m_source;                                ///< source of the body shapes
    double m_size_factor;                                ///< size factor for the automatic selection
    std::vector<std::shared_ptr<ChBody>> m_selection;    ///< explicit selection
    std::function<bool(const ChBody& body)> m_selector;  ///< selection predicate
    int m_num_divs;                                      ///< tessellation resolution of round templates
    std::shared_ptr<ChVisualMaterial> m_material;        ///< material of the instanced shapes
    ChColor m_default_color;                             ///< color of bodies without visual material

    std::shared_ptr<ColorCallback> m_color_callback;            ///< per-body color callback
    std::shared_ptr<VisibilityCallback> m_visibility_callback;  ///< per-body visibility callback
    bool m_show_inactive;                                       ///< draw inactive bodies?
    bool m_visible;                                             ///< draw the instanced bodies?

    std::vector<std::shared_ptr<ChBody>> m_bodies;  ///< bodies drawn as instances
    std::unordered_set<const ChBody*> m_body_set;   ///< bodies drawn as instances, for lookup
    std::vector<vsg::vec4> m_body_colors;           ///< default body colors (from the visual models)
    double m_median_size;                           ///< median size of the candidate bodies

    std::vector<Template> m_templates;  ///< templates
    unsigned int m_num_instances;       ///< total number of shape instances
    unsigned int m_num_visible;         ///< number of shape instances drawn at the last update

    vsg::ref_ptr<vsg::Switch> m_root;  ///< root of the instance subgraph

    // Work arrays for the update
    std::vector<char> m_body_visible;
    std::vector<vsg::vec4> m_color_work;
    std::vector<ChFramed> m_frame_work;
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
