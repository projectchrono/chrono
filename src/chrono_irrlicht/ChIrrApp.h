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

#ifndef CHIRRAPP_H
#define CHIRRAPP_H

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrAssetConverter.h"

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Class to add some GUI to Irrlicht+ChronoEngine applications. \n Such basic
/// GUI can be used to monitor solver timings, to change physical system
/// settings easily, and so on.
class ChApiIrr ChIrrApp : public ChIrrAppInterface {
  public:
    /// Create the application with Irrlicht context (3D view, device, etc.).
    ChIrrApp(ChSystem* psystem,
             const std::wstring& title = L"Chrono",
             const irr::core::dimension2d<irr::u32>& dimens = irr::core::dimension2d<irr::u32>(640, 480),
             VerticalDir vert = VerticalDir::Y,
             bool do_fullscreen = false,
             bool do_shadows = false,
             bool do_antialias = true,
             irr::video::E_DRIVER_TYPE mydriver = irr::video::EDT_DIRECT3D9,
             irr::ELOG_LEVEL log_level = irr::ELL_INFORMATION);

    /// Safely delete every Irrlicht item (including the Irrlicht scene nodes).
    virtual ~ChIrrApp();

    /// Gets the asset converter
    ChIrrAssetConverter* GetAssetConverter() { return mconverter; }

    /// Shortcut to add and bind a ChIrrNodeAsset to an item, if it has not been added previously.
    void AssetBind(std::shared_ptr<ChPhysicsItem> mitem);

    /// Shortcut to add and bind a ChIrrNodeAsset to all items in a ChSystem.
    /// If it has been already added, the existing ChIrrNodeAsset is used.
    /// NOTE. If you want a finer control on which item has an Irrlicht proxy,
    /// and which other does not need it, just use Bind(myitem) on a per-item basis.
    /// NOTE. This conversion should be done only if needed (ex. at the beginning
    /// of an animation), i.e. not too often, for performance reasons.
    void AssetBindAll();

    /// This function sets up the Irrlicht nodes corresponding to the geometric
    /// assets that are found in the ChPhysicsItem 'mitem'. For example, if one
    /// has added a ChSphereShape and a ChBoxShape to the assets of a ChBody, and
    /// a ChIrrNodeAsset too, this Update() function will prepare a ISceneNode in
    /// Irrlicht (precisely, a ChIrrNode node) and it will fill it with a
    /// spherical triangle mesh, and a box triangle mesh.
    /// NOTE. This must be done after the ChIrrNodeAsset has been created and
    /// bound, for example via Bind().
    /// NOTE. This conversion should be done only if needed (ex. at the beginning
    /// of an animation or when a shape changes), i.e. not too often, for
    /// performance reasons.
    void AssetUpdate(std::shared_ptr<ChPhysicsItem> mitem);

    /// For all items in a ChSystem, this function sets up the Irrlicht nodes
    /// corresponding to the geometric assets that have been added to the items.
    /// NOTE. This must be done after the ChIrrNodeAsset has been created and
    /// bound, for example via Bind().
    /// NOTE. This conversion should be done only if needed (ex. at the beginning
    /// of an animation), i.e. not too often, for performance reasons.
    void AssetUpdateAll();

    /// Shortcut to enable shadow maps for an item. Shadow maps in Irrlicht may
    /// slow visualization a bit. Also, one must remember to add shadow-enabled
    /// lights, using myapp.AddLightWithShadow(..)
    void AddShadow(std::shared_ptr<ChPhysicsItem> mitem);

    /// Shortcut to enable shadow maps for all items in scene. Shadow maps in
    /// Irrlicht may slow visualization a bit. Also, one must remember to add
    /// shadow-enabled lights, using myapp.AddLightWithShadow(..)
    void AddShadowAll();

  private:
    void _recurse_add_shadow(irr::scene::ISceneNode* mnode);

    ChIrrAssetConverter* mconverter;
};

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
