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

#ifndef CHOBJSPHERESHAPE_H
#define CHOBJSPHERESHAPE_H


#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChSphere.h"

namespace chrono {

/// Class for referencing a sphere shape that can be
/// visualized in some way.

class ChApi ChSphereShape : public ChVisualization {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChSphereShape)

  protected:
    //
    // DATA
    //
    geometry::ChSphere gsphere;

  public:
    //
    // CONSTRUCTORS
    //

    ChSphereShape(){};
    ChSphereShape(geometry::ChSphere& msphere) : gsphere(msphere){};

    virtual ~ChSphereShape(){};

    //
    // FUNCTIONS
    //

    // Access the sphere geometry
    geometry::ChSphere& GetSphereGeometry() { return gsphere; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChSphereShape>();
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(gsphere);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChSphereShape>();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(gsphere);
    }
};

CH_CLASS_VERSION(ChSphereShape,0)

}  // end namespace chrono

#endif
