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
// Author: A.Tasora

#ifndef CHLINESHAPE_H
#define CHLINESHAPE_H

#include "assets/ChVisualization.h"
#include "geometry/ChCLine.h"
#include "geometry/ChCLineSegment.h"

namespace chrono {

/// Class for referencing a ChLine that can be
/// visualized in some way.

class ChApi ChLineShape : public ChVisualization {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChLineShape, ChVisualization);

  protected:
    //
    // DATA
    //
    std::shared_ptr<geometry::ChLine> gline;
    std::string name;

  public:
    //
    // CONSTRUCTORS
    //

    ChLineShape() {
        // default path
        gline = std::make_shared<geometry::ChLineSegment>();
    };

    ChLineShape(std::shared_ptr<geometry::ChLine>& mline) : gline(mline){};

    virtual ~ChLineShape(){};

    //
    // FUNCTIONS
    //

    // Access the line geometry
    std::shared_ptr<geometry::ChLine> GetLineGeometry() { return gline; }

    // Set the line geometry
    void SetLineGeometry(std::shared_ptr<geometry::ChLine> mline) { gline = mline; }

    const std::string& GetName() const { return name; }
    void SetName(const std::string& mname) { name = mname; }

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
        marchive << CHNVP(gline);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(gline);
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
