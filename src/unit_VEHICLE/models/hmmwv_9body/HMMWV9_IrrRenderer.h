// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Class for additional Irrlicht support for the HMMWV9 model. This class 
// implements a custom camera and provides support for rendering links and force
// elements. Its OnFrame() method must be invoked every time the Irrlicht scene
// is redrawn, after the call to ChIrrAppInterface::DrawAll().
//
// =============================================================================

#ifndef HMMWV9_IRR_RENDERER_H
#define HMMWV9_IRR_RENDERER_H

#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChIrrApp.h"

#include "HMMWV9_Vehicle.h"


class HMMWV9_IrrRenderer
{
public:
  HMMWV9_IrrRenderer(irr::ChIrrApp*            app,
                     const HMMWV9_Vehicle&     car,
                     const chrono::ChVector<>& cam_pos,
                     const chrono::ChVector<>& cam_targ,
                     const chrono::ChVector<>& cam_offset);

  ~HMMWV9_IrrRenderer() {}

  void OnFrame();

private:
  void createCamera(const chrono::ChVector<>& pos,
                    const chrono::ChVector<>& targ);
  void updateCamera();
  void renderSprings();
  void renderLinks();
  void renderGrid();

  irr::ChIrrAppInterface* m_app;
  const HMMWV9_Vehicle&   m_car;
  chrono::ChVector<>      m_cam_offset;
};


#endif
