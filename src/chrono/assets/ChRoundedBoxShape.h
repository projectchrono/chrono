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


#include "assets/ChVisualization.h"
#include "geometry/ChCRoundedBox.h"

namespace chrono {

/// Class for referencing a rounded box shape that can be
/// visualized in some way.

class ChApi ChRoundedBoxShape : public ChVisualization {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChRoundedBoxShape, ChVisualization);

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
        marchive.VersionWrite(1);
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(groundedbox);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(groundedbox);
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
