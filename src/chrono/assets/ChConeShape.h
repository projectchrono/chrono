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

#ifndef CHOBJCONESHAPE_H
#define CHOBJCONESHAPE_H

#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChCone.h"

namespace chrono {

/// Class for referencing a cone shape that can be
/// visualized in some way.

class ChApi ChConeShape : public ChVisualization {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChConeShape)

  protected:
    //
    // DATA
    //
    geometry::ChCone gcone;

  public:
    //
    // CONSTRUCTORS
    //

    ChConeShape(){};
    ChConeShape(geometry::ChCone& mcone) : gcone(mcone){};

    virtual ~ChConeShape(){};

    //
    // FUNCTIONS
    //

    // Access the sphere geometry
    geometry::ChCone& GetConeGeometry() { return gcone; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChConeShape>();
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(gcone);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChConeShape>();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(gcone);
    }
};

CH_CLASS_VERSION(ChConeShape,0)

}  // end namespace chrono

#endif
