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

#ifndef CHTEXTURE_H
#define CHTEXTURE_H


#include "assets/ChAsset.h"

namespace chrono {

/// Base class for assets that define basic textures.
/// Assets can be attached to ChBody objects.
/// Different post processing modules can handle
/// textures in proper ways (ex for ray tracing, or
/// openGL visualization), or may also create specialized
/// classes of textures with more properties.

class ChApi ChTexture : public ChAsset {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChTexture, ChAsset);

  protected:
    //
    // DATA
    //
    std::string filename;

  public:
    //
    // CONSTRUCTORS
    //

    ChTexture() { filename = ""; };
    ChTexture(const char* mfilename) { filename = mfilename; };
    ChTexture(const std::string mfilename) { filename = mfilename; };

    virtual ~ChTexture(){};

    //
    // FUNCTIONS
    //

    // Get the texture filename. This information could be used by visualization postprocessing.
    const std::string& GetTextureFilename() const { return filename; }
    // Set the texture filename. This information could be used by visualization postprocessing.
    void SetTextureFilename(const std::string& mfile) { filename = mfile; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChAsset::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(filename);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChAsset::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(filename);
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
