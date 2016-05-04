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

///////////////////////////////////////////////////
//
//   ChBoxShape.h
//
//   Class for defining a box as an asset shape
//   that can be visualized in some way.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "assets/ChVisualization.h"
#include "geometry/ChCBox.h"

namespace chrono {

/// Class for a box shape that can be visualized
/// in some way.

class ChApi ChBoxShape : public ChVisualization {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChBoxShape, ChVisualization);

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

    virtual void ArchiveOUT(ChArchiveOut& marchive) const override
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP_OUT(gbox);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override
    {
        // version number
        // int version =
        marchive.VersionRead();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP_IN(gbox);
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
