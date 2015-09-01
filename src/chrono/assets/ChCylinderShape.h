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

#ifndef CHCYLINDERSHAPE_H
#define CHCYLINDERSHAPE_H


#include "assets/ChVisualization.h"
#include "geometry/ChCCylinder.h"

namespace chrono {

/// Class for referencing a cylinder shape that can be
/// visualized in some way.

class ChApi ChCylinderShape : public ChVisualization {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChCylinderShape, ChVisualization);

  protected:
    //
    // DATA
    //
    geometry::ChCylinder gcylinder;

  public:
    //
    // CONSTRUCTORS
    //

    ChCylinderShape(){};
    ChCylinderShape(geometry::ChCylinder& mcyl) : gcylinder(mcyl){};

    virtual ~ChCylinderShape(){};

    //
    // FUNCTIONS
    //

    // Access the sphere geometry
    geometry::ChCylinder& GetCylinderGeometry() { return gcylinder; }


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
        marchive << CHNVP(gcylinder);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(gcylinder);
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
