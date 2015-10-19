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

#ifndef CHOBJELLIPSOIDSHAPE_H
#define CHOBJELLIPSOIDSHAPE_H


#include "assets/ChVisualization.h"
#include "geometry/ChCEllipsoid.h"

namespace chrono {

/// Class for referencing an ellipsoid shape that can be
/// visualized in some way.

class ChApi ChEllipsoidShape : public ChVisualization {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChEllipsoidShape, ChVisualization);

  protected:
    //
    // DATA
    //
    geometry::ChEllipsoid gellipsoid;

  public:
    //
    // CONSTRUCTORS
    //

    ChEllipsoidShape(){};
    ChEllipsoidShape(geometry::ChEllipsoid& mellipsoid) : gellipsoid(mellipsoid){};

    virtual ~ChEllipsoidShape(){};

    //
    // FUNCTIONS
    //

    // Access the sphere geometry
    geometry::ChEllipsoid& GetEllipsoidGeometry() { return gellipsoid; }


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
        marchive << CHNVP(gellipsoid);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(gellipsoid);
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
