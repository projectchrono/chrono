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
// Authors: Kyle Sha
// =============================================================================
//
// Bridge from a Chrono ChSystem to a MetalRenderScene.
// Plain cross-platform C++: depends on Chrono and the standard library only, with no
// GPU or platform code, so the extraction can be read and tested without a Metal device.
//
// =============================================================================

#ifndef CH_METAL_SCENE_BUILDER_H
#define CH_METAL_SCENE_BUILDER_H

#include <map>
#include <memory>
#include <string>

#include "chrono_sensor/metal/ChMetalRenderTypes.h"

namespace chrono {
class ChSystem;
class ChBody;
class ChTriangleMeshConnected;
}  // namespace chrono

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

/// Translate a Chrono system's visual shapes into a MetalRenderScene.
///
/// Split into a full build and a per-frame refresh because the two have very different costs.
/// Build reads meshes from disk, resolves textures and populates the geometry cache; Refresh
/// only recomputes instance transforms and re-extracts the meshes that deform. A simulation
/// calls Build once and Refresh every frame.
class ChMetalSceneBuilder {
  public:
    explicit ChMetalSceneBuilder(ChSystem* sys) : m_sys(sys) {}

    /// Whether the number of visual shapes has changed since the last build.
    /// True when bodies or shapes were added or removed, which is the case Refresh cannot
    /// absorb: the instance list itself is a different length, so the scene needs rebuilding.
    bool TopologyChanged() const;

    /// Populate the scene's geometries, instances and texture paths from the Chrono system.
    /// Also records, per instance, how to update it later, which is what makes Refresh cheap.
    void Build(MetalRenderScene& scene);

    /// Recompute instance world transforms from the current body poses, and re-extract any
    /// deforming geometry in place. Cheap enough to call every frame.
    void Refresh(MetalRenderScene& scene);

  private:
    /// How to update one instance each frame.
    struct InstSrc {
        ChBody* body = nullptr;                         ///< the body it rides on; null for static, world-fixed geometry
        double sf[12];                                  ///< shape-in-body frame (rotation columns then position), fixed
        int geom = -1;                                  ///< index into the scene's geometry list
        bool dynamic = false;                           ///< whether the mesh itself has to be re-extracted
        std::shared_ptr<ChTriangleMeshConnected> mesh;  ///< source mesh, for a dynamic instance
    };

    /// Number of visual shapes currently in the system, the quantity TopologyChanged compares.
    int CountShapes() const;

    ChSystem* m_sys;                          ///< the system being translated
    std::vector<InstSrc> m_srcs;              ///< per-instance update sources, parallel to the scene's instances
    std::map<std::string, int> m_geom_cache;  ///< shared-geometry key -> index, so one mesh is read once
    mutable int m_last_shape_count = -1;      ///< shape count at the last build; -1 before the first one
};

/// @} sensor_metal

}  // namespace sensor
}  // namespace chrono

#endif
