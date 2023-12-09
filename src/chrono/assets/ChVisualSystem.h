// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#ifndef CH_VISUAL_SYSTEM_H
#define CH_VISUAL_SYSTEM_H

#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Vertical direction
enum class CameraVerticalDir { Y, Z };

/// Base class for a Chrono run-time visualization system.
class ChApi ChVisualSystem {
  public:
    /// Supported run-time visualization systems.
    enum class Type {
        IRRLICHT,  ///< Irrlicht
        VSG,       ///< Vulkan Scene Graph
        OpenGL,    ///< OpenGL
        OptiX      ///< OptiX
    };

    virtual ~ChVisualSystem();

    /// Attach a Chrono system to this visualization system.
    virtual void AttachSystem(ChSystem* sys);

    /// Initialize the visualization system.
    /// This call must trigger a parsing of the associated Chrono systems to process all visual models.
    /// A derived class must ensure that this function is called only once (use the m_initialized flag).
    virtual void Initialize() = 0;

    /// Process all visual assets in the associated Chrono systems.
    /// This function is called by default for a Chrono system attached to this visualization system during
    /// initialization, but can also be called later if further modifications to visualization assets occur.
    virtual void BindAll() {}

    /// Process the visual assets for the specified physics item.
    /// This function must be called if a new physics item is added to the system or if changes to its visual model
    /// occur after the visualization system was attached to the Chrono system.
    virtual void BindItem(std::shared_ptr<ChPhysicsItem> item) {}

    /// Add a camera to the 3D scene.
    /// Return an ID which can be used later to modify camera location and/or target points.
    /// A concrete visualization system may or may not support multiuple cameras.
    virtual int AddCamera(const ChVector<>& pos, ChVector<> targ = VNULL) { return -1; }

    /// Add a grid with specified parameters in the x-y plane of the given frame.
    virtual void AddGrid(double x_step,                           ///< grid cell size in X direction
                         double y_step,                           ///< grid cell size in Y direction
                         int nx,                                  ///< number of cells in X direction
                         int ny,                                  ///< number of cells in Y direction
                         ChCoordsys<> pos = CSYSNORM,             ///< grid reference frame
                         ChColor col = ChColor(0.1f, 0.1f, 0.1f)  ///< grid line color
    ) {}

    /// Set the location of the specified camera.
    virtual void SetCameraPosition(int id, const ChVector<>& pos) {}

    /// Set the target (look-at) point of the specified camera.
    virtual void SetCameraTarget(int id, const ChVector<>& target) {}

    /// Set the location of the current (active) camera.
    virtual void SetCameraPosition(const ChVector<>& pos) {}

    /// Set the target (look-at) point of the current (active) camera.
    virtual void SetCameraTarget(const ChVector<>& target) {}

    /// Get the location of the current (active) camera.
    virtual ChVector<> GetCameraPosition() const { return VNULL; }

    /// Get the target (look-at) point of the current (active) camera.
    virtual ChVector<> GetCameraTarget() const { return VNULL; }

    /// Update the location and/or target points of the specified camera.
    void UpdateCamera(int id, const ChVector<>& pos, ChVector<> target);

    //// Update the location and/or target point of the current (active) camera.
    void UpdateCamera(const ChVector<>& pos, ChVector<> target);

    /// Add a visual model not associated with a physical item.
    /// Return an ID which can be used later to modify the position of this visual model.
    virtual int AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) { return -1; }

    /// Add a visual model not associated with a physical item.
    /// This version constructs a visual model consisting of the single specified shape
    /// Return an ID which can be used later to modify the position of this visual model.
    virtual int AddVisualModel(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) { return -1; }

    /// Update the position of the specified visualization-only model.
    virtual void UpdateVisualModel(int id, const ChFrame<>& frame) {}

    /// Run the visualization system.
    /// Returns `false` if the system must shut down.
    virtual bool Run() = 0;

    /// Terminate the visualization system.
    virtual void Quit() = 0;

    /// Perform any necessary operations at the beginning of each rendering frame.
    virtual void BeginScene() = 0;

    /// Draw all 3D shapes and GUI elements at the current frame.
    /// This function is typically called inside a loop such as
    /// <pre>
    ///    while(vis->Run()) {
    ///       ...
    ///       vis->Render();
    ///       ...
    ///    }
    /// </pre>
    virtual void Render() = 0;

    /// Render the specified reference frame.
    virtual void RenderFrame(const ChFrame<>& frame, double axis_length = 1) {}

    /// Render COG frames for all bodies in the system.
    virtual void RenderCOGFrames(double axis_length = 1) {}

    /// Perform any necessary operations ar the end of each rendering frame.
    virtual void EndScene() = 0;

    /// Return the simulation real-time factor (simlation time / simulated time).
    /// The default value returned by this base class is the RTF value from the first associated system (if any).
    virtual double GetSimulationRTF() const;

    /// Return the current simulated time.
    /// The default value returned by this base class is the time from the first associated system (if any).
    virtual double GetSimulationTime() const;

    /// Create a snapshot of the last rendered frame and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) {}

    /// Set output directory for saving frame snapshots (default: ".").
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable/disable writing of frame snapshots to file.
    void SetImageOutput(bool val) { m_write_images = val; }

    /// Enable modal analysis visualization.
    /// If supported, visualize an oscillatory motion of the n-th mode (if the associated system contains a
    /// ChModalAssembly).
    virtual void EnableModalAnalysis(bool val) {}

    /// Set the mode to be shown (only if some ChModalAssembly is found).
    virtual void SetModalModeNumber(int val) {}

    /// Set the amplitude of the shown mode (only if some ChModalAssembly is found).
    virtual void SetModalAmplitude(double val) {}

    /// Set the speed of the shown mode (only if some ChModalAssembly is found).
    virtual void SetModalSpeed(double val) {}

    /// Set the maximum number of modes selectable (only if some ChModalAssembly is found).
    virtual void SetModalModesMax(int maxModes) {}

    /// Get the list of associated Chrono systems.
    std::vector<ChSystem*> GetSystems() const { return m_systems; }

    /// Get the specified associated Chrono system.
    ChSystem& GetSystem(int i) const { return *m_systems[i]; }

  protected:
    ChVisualSystem();

    /// Perform any necessary setup operations at the beginning of a time step.
    /// Called by an associated ChSystem.
    virtual void OnSetup(ChSystem* sys) {}

    /// Perform any necessary update operations at the end of a time step.
    /// Called by an associated ChSystem.
    virtual void OnUpdate(ChSystem* sys) {}

    /// Remove all visualization objects from this visualization system.
    /// Called by an associated ChSystem.
    virtual void OnClear(ChSystem* sys) {}

    bool m_initialized;

    std::vector<ChSystem*> m_systems;  ///< associated Chrono system(s)

    bool m_write_images;      ///< if true, save snapshots
    std::string m_image_dir;  ///< directory for image files

    friend class ChSystem;
};

/// @} chrono_assets

}  // namespace chrono

#endif
