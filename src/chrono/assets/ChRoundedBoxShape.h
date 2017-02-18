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

#ifndef CHROUNDEDBOXSHAPE_H
#define CHROUNDEDBOXSHAPE_H


#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChRoundedBox.h"

namespace chrono {

/// Class for referencing a rounded box shape that can be
/// visualized in some way.

class ChApi ChRoundedBoxShape : public ChVisualization {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChRoundedBoxShape)

  protected:
    //
    // DATA
    //
    geometry::ChRoundedBox groundedbox;

  public:
    //
    // CONSTRUCTORS
    //

    ChRoundedBoxShape() {}
    ChRoundedBoxShape(geometry::ChRoundedBox& mcap) : groundedbox(mcap) {}

    virtual ~ChRoundedBoxShape() {}

    //
    // FUNCTIONS
    //

    // Access the rounded box geometry
    geometry::ChRoundedBox& GetRoundedBoxGeometry() { return groundedbox; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChRoundedBoxShape>();
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(groundedbox);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChRoundedBoxShape>();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(groundedbox);
    }
};

CH_CLASS_VERSION(ChRoundedBoxShape,0)

}  // end namespace chrono

#endif
