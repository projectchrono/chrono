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

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_parsers/urdf/ChParserURDF.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// VSG-based run-time visualization system for ChParserURDF.
/// Used as a plugin to a Chrono::VSG visualization system.
class ChApiParsers ChParserURDFVisualizationVSG : public vsg3d::ChVisualSystemVSGPlugin {
  public:
    ChParserURDFVisualizationVSG(ChParserURDF& urdf);
    ~ChParserURDFVisualizationVSG();

    virtual void OnAttach() override;
    virtual void OnInitialize() override;
    virtual void OnBindAssets() override;
    virtual void OnRender() override;

    /// Set output directory for saving frame snapshots (default: ".").
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable/disable writing of frame snapshots to file.
    void SetImageOutput(bool val) { m_write_images = val; }

    /// Add additional proxy body to supplemental system.
    /// Must be called before Initialize().
    /// The provided body is set fixed to ground and it is the caller's responsibility to update the position of
    /// this body before a call to Render().
    void AddProxyBody(std::shared_ptr<ChBody> body) {
        body->SetFixed(true);
        m_sys->AddBody(body);
    }

  private:
    ChParserURDF& m_urdf;  ///< associated URDF parser
    ChSystem* m_sys;       ///< Chrono system of the associated RoboSimian robot

    bool m_write_images;      ///< if true, save snapshots
    std::string m_image_dir;  ///< directory for image files

    friend class ChParserURDFStats;
};

/// @} parsers_module

}  // namespace parsers
}  // namespace chrono
