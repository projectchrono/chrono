//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHOBJSBOXSHAPE_H
#define CHOBJSBOXSHAPE_H

#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChBox.h"

namespace chrono {

/// Class for a box shape that can be visualized
/// in some way.

class ChApi ChBoxShape : public ChVisualization {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChBoxShape)

  protected:
    //
    // DATA
    //
    geometry::ChBox gbox;

  public:
    //
    // CONSTRUCTORS
    //

    ChBoxShape(){};
    ChBoxShape(geometry::ChBox& mbox) : gbox(mbox){};

    virtual ~ChBoxShape(){};

    //
    // FUNCTIONS
    //

    // Access the sphere geometry
    geometry::ChBox& GetBoxGeometry() { return gbox; }



    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChBoxShape>();
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(gbox);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChBoxShape>();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(gbox);
    }
};

CH_CLASS_VERSION(ChBoxShape,0)


}  // end namespace chrono

#endif
