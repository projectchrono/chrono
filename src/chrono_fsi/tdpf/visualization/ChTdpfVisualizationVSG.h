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

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/assets/ChColormap.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

namespace chrono {
namespace fsi {
namespace tdpf {

/// @addtogroup fsitdpf_visualization
/// @{

/// VSG-based run-time visualization system for TDPF-based FSI systems.
/// Used as a plugin to a Chrono::VSG visualization system.
class CH_FSI_API ChTdpfVisualizationVSG : public vsg3d::ChVisualSystemVSGPlugin {
  public:
    /// GPU shader modes supported by the TDPF wave mesh
    enum class ColorMode { NONE = 0, HEIGHT = 1, VELOCITY_MAG = 2, VELOCITY_X = 3, VELOCITY_Y = 4, VELOCITY_Z = 5 };

    ChTdpfVisualizationVSG(ChFsiSystemTDPF* sysFSI);
    ChTdpfVisualizationVSG(ChFsiFluidSystemTDPF* sysTDPF);

    ~ChTdpfVisualizationVSG();

    virtual void OnAttach() override;
    virtual void OnInitialize() override;
    virtual void OnBindAssets() override;
    virtual void OnRender() override;

    /// Get the type of the colormap currently in use.
    ChColormap::Type GetColormapType() const;

    /// Get the colormap object in current use.
    const ChColormap& GetColormap() const;

    /// Set output directory for saving frame snapshots (default: ".").
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable/disable writing of frame snapshots to file.
    void SetImageOutput(bool val) { m_write_images = val; }

    /// Enable/disable rendering of waves (default: true).
    void SetWaveMeshVisibility(bool val);
    
    void SetWaveMeshParams(const ChVector2d& center, const ChVector2d& size, double resolution = 100);
    void SetWaveMeshWireframe(bool wireframe);
    void SetWaveMeshColorMode(ColorMode mode, const ChVector2d& range);
    void SetWaveMeshColormap(ChColormap::Type type, float opacity = 1);

    /// Set wave mesh update frequency (default: 30 FPS).
    void SetWaveMeshUpdateFrequency(double freq);

    /// Add additional proxy body to supplemental system.
    /// Must be called before Initialize().
    /// The provided body is set fixed to ground and it is the caller's responsibility to update the position of
    /// this body before a call to Render().
    void AddProxyBody(std::shared_ptr<ChBody> body) {
        body->SetFixed(true);
        m_sysMBS->AddBody(body);
    }

    /// Return the internal Chrono system that holds visualization shapes.
    ChSystem* GetSystem() const { return m_sysMBS; }

  private:
    /// Deformable mesh for wave visualizaiton.
    struct WaveMesh {
        WaveMesh()
            : wireframe(false),
              center(ChVector2d(0, 0)),
              size(ChVector2d(100, 100)),
              resolution(100),
              colormode(ColorMode::NONE),
              range(ChVector2d(0, 0)),
              colormap_type(ChColormap::Type::BLUE) {}

        // Specification
        bool wireframe;                        ///< render wave mesh as wireframe?
        ChVector2d size;                       ///< wave mesh size (in X-Y plane)
        ChVector2d center;                     ///< wave mesh center (in X-Y plane)
        int resolution;                        ///< wave mesh resolution (in both directions)
        ColorMode colormode;                   ///< wave mesh coloring mode
        ChVector2d range;                      ///< min/max values for colored mode
        ChColormap::Type colormap_type;        ///< colormap type
        float opacity;                         ///< wave mesh opacity (1 for solid)
        std::unique_ptr<ChColormap> colormap;  ///< colormap for wave mesh false coloring

        // Implementation
        std::shared_ptr<ChTriangleMeshConnected> trimesh;  ///< reference to the Chrono triangle mesh
        vsg::ref_ptr<vsg::vec3Array> vertices;             ///< mesh vertices
        vsg::ref_ptr<vsg::vec3Array> normals;              ///< mesh normals
        vsg::ref_ptr<vsg::vec4Array> colors;               ///< mesh vertex colors
        bool mesh_soup;                                    ///< true if using separate triangles
        bool dynamic_vertices;                             ///< mesh vertices change
        bool dynamic_normals;                              ///< mesh normals change
        bool dynamic_colors;                               ///< mesh vertex colors change
    };

    void CreateWaveMesh();

    ChFsiSystemTDPF* m_sysFSI;        ///< associated FSI system
    ChFsiFluidSystemTDPF* m_sysTDPF;  ///< associated TDPF system
    ChSystem* m_sysMBS;               ///< internal Chrono system (holds proxies)

    double m_update_freq;  ///< rendering update rate
    double m_last_update;  ///< time of last update

    vsg::ref_ptr<vsg::Switch> m_wave_scene;  ///< VSG scene containing wave mesh
    WaveMesh m_wave_mesh;                    ///< wave trimesh
    bool m_waves_visible;                    ///< render waves?
    size_t m_wave_colorbar_id;               ///< ID of wave colorbar GUI

    bool m_write_images;      ///< if true, save snapshots
    std::string m_image_dir;  ///< directory for image files

    friend class FSITDPFStatsVSG;
};

/// @} fsitdpf_visualization

}  // namespace tdpf
}  // namespace fsi
}  // namespace chrono
