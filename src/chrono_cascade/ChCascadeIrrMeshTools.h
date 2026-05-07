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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CH_CASCADE_IRR_MESHTOOLS_H
#define CH_CASCADE_IRR_MESHTOOLS_H

#include <irrlicht.h>

#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeMeshTools.h"
#include <TopoDS.hxx>

namespace chrono {
namespace cascade {

/// @addtogroup cascade_module
/// @{

/// Tools to convert OpenCASCADE shapes into 'Irrlicht' triangle meshes.
class ChCascadeIrrMeshTools {
  public:
    /// Function to convert an OpenCASCADE face into a Irrlicht mesh for visualization.
    static void FillIrrlichtMeshFromCascadeFace(irr::scene::IMesh* pMesh,
                                                const TopoDS_Face& F, irr::video::SColor clr = irr::video::SColor(255, 255, 255, 255));

    /// Function to convert an OpenCASCADE shape into a Irrlicht mesh, for visualization.
    static void FillIrrlichtMeshFromCascade(irr::scene::IMesh* pMesh,
                                            const TopoDS_Shape& mshape,
                                            double deflection = 1,
                                            bool relative_deflection = false,
                                            double angulardeflection = 0.5,
                                            irr::video::SColor clr = irr::video::SColor(255, 255, 255, 255));
};

/// @} cascade_module

}  // end namespace cascade
}  // end namespace chrono

#endif
