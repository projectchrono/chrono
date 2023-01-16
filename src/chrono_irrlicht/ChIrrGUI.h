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
class ChVisualSystemIrrlicht;

/// @addtogroup irrlicht_module
/// @{

/// Irrlicht GUI attached to a ChVisualSystemIrrlicht.
class ChApiIrr ChIrrGUI {
  public:
    ChIrrGUI();
    ~ChIrrGUI();

    /// Perform operations after opening the Irrlicht scene for the current frame.
    void BeginScene();

    /// Render the GUI.
    void Render();

    /// Perform operations before closing the Irrlicht scene for the current frame.
    void EndScene();

  private:
    void Initialize(ChVisualSystemIrrlicht* vis);

    void DrawCollisionShapes(irr::video::SColor color);

    void DumpSystemMatrices();

    irr::IrrlichtDevice* GetDevice() { return m_device; }
    irr::video::IVideoDriver* GetVideoDriver() { return m_device->getVideoDriver(); }
    irr::scene::ISceneManager* GetSceneManager() { return m_device->getSceneManager(); }
    irr::scene::ICameraSceneNode* GetActiveCamera() { return m_device->getSceneManager()->getActiveCamera(); }
    irr::gui::IGUIEnvironment* GetGUIEnvironment() { return m_device->getGUIEnvironment(); }

    /// Attach a custom event receiver to the application.
    void AddUserEventReceiver(irr::IEventReceiver* receiver);

    /// Set the active tab on the info panel.
    void SetInfoTab(int ntab) { g_tabbed->setActiveTab(ntab); }

    /// Set the amplitude of the shown mode (only if some ChModalAssembly is found).
    void SetModalAmplitude(double val);

    /// Set the speed of the shown mode (only if some ChModalAssembly is found).
    void SetModalSpeed(double val);

    /// Set the label mode for contacts
    void SetContactsLabelMode(ContactsLabelMode mm) { g_labelcontacts->setSelected((int)mm); }
    /// Set the draw mode for contacts
    void SetContactsDrawMode(ContactsDrawMode mm) { g_drawcontacts->setSelected((int)mm); }
    /// Set the label mode for links
    void SetLinksLabelMode(LinkLabelMode mm) { g_labellinks->setSelected((int)mm); }
    /// Set the draw mode for links
    void SetLinksDrawMode(LinkDrawMode mm) { g_drawlinks->setSelected((int)mm); }
    /// Set if the AABB collision shapes will be plotted
    void SetPlotAABB(bool val) { g_plot_aabb->setChecked(val); }
    /// Set if the COG frames will be plotted
    void SetPlotCOGFrames(bool val) { g_plot_cogs->setChecked(val); }
    /// Set if the collision shapes will be plotted
    void SetPlotCollisionShapes(bool val) { g_plot_collisionshapes->setChecked(val); }
    /// Set if the link frames will be plotted
    void SetPlotLinkFrames(bool val) { g_plot_linkframes->setChecked(val); }
    /// Set if the COG frames will be plotted
    void SetPlotConvergence(bool val) { g_plot_convergence->setChecked(val); }

    /// Set the scale for symbol drawing (link frames, COGs, etc.)
    void SetSymbolscale(double val);

    bool initialized;

    bool show_infos;
    bool show_profiler;
    bool show_explorer;

    double symbolscale;

    double camera_auto_rotate_speed;

    bool modal_show;
    int modal_mode_n;
    double modal_amplitude;
    double modal_speed;
    double modal_phi;
    double modal_current_mode_n;
    double modal_current_freq;
    double modal_current_dampingfactor;

    ChVisualSystemIrrlicht* m_vis;
    ChSystem* m_system;
    irr::IrrlichtDevice* m_device;

    ChIrrEventReceiver* m_receiver;                      ///< default event receiver
    std::vector<irr::IEventReceiver*> m_user_receivers;  ///< optional user-defined receivers

    std::shared_ptr<collision::ChCollisionSystem::VisualizationCallback> m_drawer;  ///< collision callback

    irr::gui::IGUITabControl* g_tabbed;

    irr::gui::IGUIStaticText* g_textFPS;
    irr::gui::IGUIComboBox* g_drawcontacts;
    irr::gui::IGUIComboBox* g_labelcontacts;
    irr::gui::IGUIComboBox* g_drawlinks;
    irr::gui::IGUIComboBox* g_labellinks;
    irr::gui::IGUICheckBox* g_plot_aabb;
    irr::gui::IGUICheckBox* g_plot_cogs;
    irr::gui::IGUICheckBox* g_plot_collisionshapes;
    irr::gui::IGUICheckBox* g_plot_linkframes;
    irr::gui::IGUICheckBox* g_plot_convergence;

    irr::gui::IGUIEditBox* g_symbolscale;
    irr::gui::IGUIStaticText* g_textHelp;

    irr::gui::IGUIScrollBar* g_modal_mode_n;
    irr::gui::IGUIStaticText* g_modal_mode_n_info;
    irr::gui::IGUIEditBox* g_modal_amplitude;
    irr::gui::IGUIEditBox* g_modal_speed;

    irr::gui::IGUITreeView* g_treeview;

    friend class ChIrrEventReceiver;
    friend class ChVisualSystemIrrlicht;
};

/// @} irrlicht_module

}  // namespace irrlicht
}  // namespace chrono

#endif