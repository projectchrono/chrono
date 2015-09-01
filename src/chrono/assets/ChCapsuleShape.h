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

#ifndef CHCAPSULESHAPE_H
#define CHCAPSULESHAPE_H


#include "assets/ChVisualization.h"
#include "geometry/ChCCapsule.h"

namespace chrono {

/// Class for referencing a capsule shape that can be
/// visualized in some way.

class ChApi ChCapsuleShape : public ChVisualization {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChCapsuleShape, ChVisualization);

  protected:
    //
    // DATA
    //
    geometry::ChCapsule gcapsule;

  public:
    //
    // CONSTRUCTORS
    //

    ChCapsuleShape() {}
    ChCapsuleShape(geometry::ChCapsule& mcap) : gcapsule(mcap) {}

    virtual ~ChCapsuleShape() {}

    //
    // FUNCTIONS
    //

    // Access the capsule geometry
    geometry::ChCapsule& GetCapsuleGeometry() { return gcapsule; }


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
        marchive << CHNVP(gcapsule);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(gcapsule);
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
