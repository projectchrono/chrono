//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHPATHSHAPE_H
#define CHPATHSHAPE_H

#include "assets/ChVisualization.h"
#include "geometry/ChCLinePath.h"
#include "geometry/ChCLineSegment.h"
#include "geometry/ChCLineArc.h"

namespace chrono {

/// Class for referencing a ChLinePath that can be
/// visualized in some way.

class ChApi ChPathShape : public ChVisualization {
  protected:
    //
    // DATA
    //
    ChSharedPtr<geometry::ChLinePath> gpath;

  public:
    //
    // CONSTRUCTORS
    //

    ChPathShape() {
        // default path
        gpath = ChSharedPtr<geometry::ChLinePath>(new geometry::ChLinePath);
    };
    ChPathShape(ChSharedPtr<geometry::ChLinePath>& mpath) : gpath(mpath){};

    virtual ~ChPathShape(){};

    //
    // FUNCTIONS
    //

    // Access the sphere geometry
    ChSharedPtr<geometry::ChLinePath> GetPathGeometry() { return gpath; }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
