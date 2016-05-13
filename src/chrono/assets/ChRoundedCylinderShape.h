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

#ifndef CHROUNDEDCYLINDERSHAPE_H
#define CHROUNDEDCYLINDERSHAPE_H


#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChRoundedCylinder.h"

namespace chrono {

/// Class for referencing a rounded cylinder shape that can be
/// visualized in some way.

class ChApi ChRoundedCylinderShape : public ChVisualization {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChRoundedCylinderShape, ChVisualization);

  protected:
    //
    // DATA
    //
    geometry::ChRoundedCylinder groundedcyl;

  public:
    //
    // CONSTRUCTORS
    //

    ChRoundedCylinderShape() {}
    ChRoundedCylinderShape(geometry::ChRoundedCylinder& mcap) : groundedcyl(mcap) {}

    virtual ~ChRoundedCylinderShape() {}

    //
    // FUNCTIONS
    //

    // Access the rounded cylinder geometry
    geometry::ChRoundedCylinder& GetRoundedCylinderGeometry() { return groundedcyl; }

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
        marchive << CHNVP(groundedcyl);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(groundedcyl);
    }

};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
