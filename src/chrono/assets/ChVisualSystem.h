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
#include "chrono/core/ChTimer.h"
#include "chrono/assets/ChVisualModel.h"
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
        OptiX,     ///< OptiX
        NONE
    };

    virtual ~ChVisualSystem();

    /// Enable/disable information terminal output during initialization (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Indicate whether the visual system was initialized or not.
    bool IsInitialized() const { return m_initialized; }

    /// Attach a Chrono system to this visualization system.
    virtual void AttachSystem(ChSystem* sys);

    /// Initialize the visualization system.
    /// This call must trigger a parsing of the associated Chrono systems to process all visual models.
    /// A derived class must ensure that this function is called only once (use the m_initialized flag).
    virtual void Initialize() {}

    /// Process all visual assets in the associated Chrono systems.
    /// This function is called by default for a Chrono system attached to this visualization system during
    /// initialization, but can also be called later if further modifications to visualization assets occur.
    virtual void BindAll() {}

    /// Process the visual assets for the specified physics item.
    /// This function must be called if a new physics item is added to the system or if changes to its visual model
    /// occur after the visualization system was attached to the Chrono system.
    virtual void BindItem(std::shared_ptr<ChPhysicsItem> item) {}

    /// Remove the visual assets for the specified physics item from this visualization system.
    virtual void UnbindItem(std::shared_ptr<ChPhysicsItem> item) {}

    /// Add a camera to the 3D scene.
    /// Return an ID which can be used later to modify camera location and/or target points.
    /// A concrete visualization system may or may not support multiuple cameras.
    virtual int AddCamera(const ChVector3d& pos, ChVector3d targ = VNULL) { return -1; }

    /// Add a grid with specified parameters in the x-y plane of the given frame.
    virtual void AddGrid(double x_step,                           ///< grid cell size in X direction
                         double y_step,                           ///< grid cell size in Y direction
                         int nx,                                  ///< number of cells in X direction
                         int ny,                                  ///< number of cells in Y direction
                         ChCoordsys<> pos = CSYSNORM,             ///< grid reference frame
                         ChColor col = ChColor(0.1f, 0.1f, 0.1f)  ///< grid line color
    ) {}

    /// Set the location of the specified camera.
    virtual void SetCameraPosition(int id, const ChVector3d& pos) {}

    /// Set the target (look-at) point of the specified camera.
    virtual void SetCameraTarget(int id, const ChVector3d& target) {}

    /// Set the location of the current (active) camera.
    virtual void SetCameraPosition(const ChVector3d& pos) {}

    /// Set the target (look-at) point of the current (active) camera.
    virtual void SetCameraTarget(const ChVector3d& target) {}

    /// Get the location of the current (active) camera.
    virtual ChVector3d GetCameraPosition() const { return VNULL; }

    /// Get the target (look-at) point of the current (active) camera.
    virtual ChVector3d GetCameraTarget() const { return VNULL; }

    /// Update the location and/or target points of the specified camera.
    void UpdateCamera(int id, const ChVector3d& pos, ChVector3d target);

    //// Update the location and/or target point of the current (active) camera.
    void UpdateCamera(const ChVector3d& pos, ChVector3d target);

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
    virtual bool Run() { return false; }

    /// Terminate the visualization system.
    virtual void Quit() {}

    /// Perform any necessary operations at the beginning of each rendering frame.
    virtual void BeginScene() {}

    /// Draw all 3D shapes and GUI elements at the current frame.
    /// This function is typically called inside a loop such as
    /// <pre>
    ///    while(vis->Run()) {
    ///       ...
    ///       vis->Render();
    ///       ...
    ///    }
    /// </pre>
    /// The default implementation only updates the overall RTF.
    virtual void Render();

    /// Render the specified reference frame.
    virtual void RenderFrame(const ChFrame<>& frame, double axis_length = 1) {}

    /// Render COG frames for all bodies in the system.
    virtual void RenderCOGFrames(double axis_length = 1) {}

    /// Perform any necessary operations ar the end of each rendering frame.
    virtual void EndScene() {}

    /// Get the list of associated Chrono systems.
    std::vector<ChSystem*> GetSystems() const { return m_systems; }

    /// Get the specified associated Chrono system.
    ChSystem& GetSystem(int i) const { return *m_systems[i]; }

    /// Return the current simulated time.
    /// The default value returned by this base class is the time from the first associated system (if any).
    double GetSimulationTime() const;

    /// Return the overall real time factor.
    /// This value represents the ratio between the wall clock time elapsed between two render frames and the duration
    /// by which simulation was advanced in this interval.
    double GetRTF() const { return m_rtf; }

    /// Return the simulation real-time factor (simulation time / simulated time) for the specified associated system.
    /// See ChSystem::GetRTF.
    double GetSimulationRTF(unsigned int i) const;

    /// Return the simulation real-time factor (simulation time / simulated time) for all associated system.
    /// See ChSystem::GetRTF.
    std::vector<double> GetSimulationRTFs() const;

    /// Get the number of bodies  (across all visualized systems).
    /// The reported number represents only active bodies, excluding sleeping or fixed.
    unsigned int GetNumBodies() const;

    /// Get the number of links  (across all visualized systems).
    /// The reported number represents only active bodies, excluding sleeping or fixed.
    unsigned int GetNumLinks() const;

    /// Get the number of meshes  (across all visualized systems).
    unsigned int GetNumMeshes() const;

    /// Get the number of shafts  (across all visualized systems).
    unsigned int GetNumShafts() const;

    /// Get the number of coordinates at the velocity level (across all visualized systems).
    unsigned int GetNumStates() const;

    /// Get the number of scalar constraints  (across all visualized systems).
    unsigned int GetNumConstraints() const;

    /// Gets the number of contacts.
    unsigned int GetNumContacts() const;

    /// Create a snapshot of the last rendered frame and save it to the provided file.
    /// The file extension determines the image format.
    virtual void WriteImageToFile(const std::string& filename) {}

    /// Set output directory for saving frame snapshots (default: ".").
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable/disable writing of frame snapshots to file.
    void SetImageOutput(bool val) { m_write_images = val; }

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

    bool m_verbose;      ///< terminal output
    bool m_initialized;  ///< visual system initialized
    ChTimer m_timer;     ///< timer for evaluating RTF
    double m_rtf;        ///< overall real time factor

    std::vector<ChSystem*> m_systems;  ///< associated Chrono system(s)

    bool m_write_images;      ///< if true, save snapshots
    std::string m_image_dir;  ///< directory for image files

    friend class ChSystem;
};

/// @} chrono_assets

}  // namespace chrono

#endif
