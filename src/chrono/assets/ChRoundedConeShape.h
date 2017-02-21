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

#ifndef CHROUNDEDCONESHAPE_H
#define CHROUNDEDCONESHAPE_H


#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChRoundedCone.h"

namespace chrono {

/// Class for referencing a rounded cone shape that can be
/// visualized in some way.

class ChApi ChRoundedConeShape : public ChVisualization {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChRoundedConeShape)

  protected:
    //
    // DATA
    //
    geometry::ChRoundedCone groundedcone;

  public:
    //
    // CONSTRUCTORS
    //

    ChRoundedConeShape() {}
    ChRoundedConeShape(geometry::ChRoundedCone& mcap) : groundedcone(mcap) {}

    virtual ~ChRoundedConeShape() {}

    //
    // FUNCTIONS
    //

    // Access the rounded cone geometry
    geometry::ChRoundedCone& GetRoundedConeGeometry() { return groundedcone; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChRoundedConeShape>();
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(groundedcone);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChRoundedConeShape>();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(groundedcone);
    }
};

CH_CLASS_VERSION(ChRoundedConeShape,0)

}  // end namespace chrono

#endif
