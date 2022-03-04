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

#ifndef CH_IRR_GUI_H
#define CH_IRR_GUI_H

#include <irrlicht.h>

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrTools.h"

namespace chrono {

// Forward references
class ChSystem;

namespace irrlicht {

// Forward references
class ChIrrEventReceiver;

/// Irrlicht GUI attached to a ChVisualSystemIrrlicht.
class ChApiIrr ChIrrGUI {
  public:
    ChIrrGUI(irr::IrrlichtDevice* device, ChSystem* sys);
    ~ChIrrGUI();

    /// Attach a custom event receiver to the application.
    void AddUserEventReceiver(irr::IEventReceiver* receiver);

    /// Perform setup when the associated Irrlicht visualization system is initialized.
    void Initialize();

    /// Perform operations after opening the Irrlicht scene for the current frame.
    void BeginScene();

    /// Render the GUI.
    void DrawAll();

    /// Perform operations before closing the Irrlicht scene for the current frame.
    void EndScene();

  private:
    void DrawCollisionShapes(irr::video::SColor color);

    void DumpSystemMatrices();

    irr::IrrlichtDevice* GetDevice() { return m_device; }
    irr::video::IVideoDriver* GetVideoDriver() { return m_device->getVideoDriver(); }
    irr::scene::ISceneManager* GetSceneManager() { return m_device->getSceneManager(); }
    irr::scene::ICameraSceneNode* GetActiveCamera() { return m_device->getSceneManager()->getActiveCamera(); }
    irr::gui::IGUIEnvironment* GetGUIEnvironment() { return m_device->getGUIEnvironment(); }

    /// Show the info panel in the 3D view
    void SetShowInfos(bool val) { show_infos = val; }
    bool GetShowInfos() { return show_infos; }

    void SetInfosTab(int ntab) { gad_tabbed->setActiveTab(ntab); }

    /// Show the realtime profiler in the 3D view
    void SetShowProfiler(bool val) { show_profiler = val; }
    bool GetShowProfiler() { return show_profiler; }

    /// Show the object explorer
    void SetShowExplorer(bool val) { show_explorer = val; }
    bool GetShowExplorer() { return show_explorer; }

    /// Set the label mode for contacts
    void SetContactsLabelMode(IrrContactsLabelMode mm) { gad_labelcontacts->setSelected((int)mm); }
    /// Set the draw mode for contacts
    void SetContactsDrawMode(IrrContactsDrawMode mm) { gad_drawcontacts->setSelected((int)mm); }
    /// Set the label mode for links
    void SetLinksLabelMode(IrrLinkLabelMode mm) { gad_labellinks->setSelected((int)mm); }
    /// Set the draw mode for links
    void SetLinksDrawMode(IrrLinkDrawMode mm) { gad_drawlinks->setSelected((int)mm); }
    /// Set if the AABB collision shapes will be plotted
    void SetPlotAABB(bool val) { gad_plot_aabb->setChecked(val); }
    /// Set if the COG frames will be plotted
    void SetPlotCOGFrames(bool val) { gad_plot_cogs->setChecked(val); }
    /// Set if the collision shapes will be plotted
    void SetPlotCollisionShapes(bool val) { gad_plot_collisionshapes->setChecked(val); }
    /// Set if the link frames will be plotted
    void SetPlotLinkFrames(bool val) { gad_plot_linkframes->setChecked(val); }
    /// Set if the COG frames will be plotted
    void SetPlotConvergence(bool val) { gad_plot_convergence->setChecked(val); }

    /// Set the scale for symbol drawing (link frames, COGs, etc.)
    void SetSymbolscale(double val);

    bool show_infos;
    bool show_profiler;
    bool show_explorer;

    double symbolscale;

    double camera_auto_rotate_speed;

    ChSystem* m_system;
    irr::IrrlichtDevice* m_device;

    ChIrrEventReceiver* m_receiver;                      ///< default event receiver
    std::vector<irr::IEventReceiver*> m_user_receivers;  ///< optional user-defined receivers

    std::shared_ptr<collision::ChCollisionSystem::VisualizationCallback> m_drawer;  ///< collision callback

    irr::gui::IGUITabControl* gad_tabbed;

    irr::gui::IGUIStaticText* gad_textFPS;
    irr::gui::IGUIComboBox* gad_drawcontacts;
    irr::gui::IGUIComboBox* gad_labelcontacts;
    irr::gui::IGUIComboBox* gad_drawlinks;
    irr::gui::IGUIComboBox* gad_labellinks;
    irr::gui::IGUICheckBox* gad_plot_aabb;
    irr::gui::IGUICheckBox* gad_plot_cogs;
    irr::gui::IGUICheckBox* gad_plot_collisionshapes;
    irr::gui::IGUICheckBox* gad_plot_linkframes;
    irr::gui::IGUICheckBox* gad_plot_convergence;

    irr::gui::IGUIEditBox* gad_symbolscale;
    irr::gui::IGUIStaticText* gad_symbolscale_info;
    irr::gui::IGUIStaticText* gad_textHelp;

    irr::gui::IGUITreeView* gad_treeview;

    friend class ChIrrEventReceiver;
    friend class ChVisualSystemIrrlicht;
};

}  // namespace irrlicht
}  // namespace chrono

#endif