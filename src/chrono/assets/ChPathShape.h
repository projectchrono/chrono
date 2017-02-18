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

#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChLineArc.h"

namespace chrono {

/// Class for referencing a ChLinePath that can be
/// visualized in some way.

class ChApi ChPathShape : public ChVisualization {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChPathShape)

  protected:
    //
    // DATA
    //
    std::shared_ptr<geometry::ChLinePath> gpath;

  public:
    //
    // CONSTRUCTORS
    //

    ChPathShape() {
        // default path
        gpath = std::make_shared<geometry::ChLinePath>();
    }

    ChPathShape(std::shared_ptr<geometry::ChLinePath>& mpath) : gpath(mpath) {}

    virtual ~ChPathShape() {}

    //
    // FUNCTIONS
    //

    // Access the sphere geometry
    std::shared_ptr<geometry::ChLinePath> GetPathGeometry() { return gpath; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChPathShape>();
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(gpath);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChPathShape>();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(gpath);
    }
};

CH_CLASS_VERSION(ChPathShape,0)

}  // end namespace chrono

#endif
