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

#ifndef CHOBJSHAPEFILE_H
#define CHOBJSHAPEFILE_H


#include "chrono/assets/ChVisualization.h"

namespace chrono {

/// Class for referencing a Wavefront/Alias .obj
/// file containing a shape that can be visualized
/// in some way.
/// The file is not load into this object: it
/// is simply a reference to the resource on the disk.

class ChApi ChObjShapeFile : public ChVisualization {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChObjShapeFile)

  protected:
    //
    // DATA
    //
    std::string filename;

  public:
    //
    // CONSTRUCTORS
    //

    ChObjShapeFile() : filename(""){};

    virtual ~ChObjShapeFile(){};

    //
    // FUNCTIONS
    //

    std::string GetFilename() const { return filename; }
    void SetFilename(const std::string ms) { filename = ms; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChObjShapeFile>();
        // serialize parent class
        ChVisualization::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(filename);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChObjShapeFile>();
        // deserialize parent class
        ChVisualization::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(filename);
    }
};

CH_CLASS_VERSION(ChObjShapeFile,0)

}  // end namespace chrono

#endif
