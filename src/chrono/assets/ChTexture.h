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


#include "chrono/assets/ChAsset.h"

namespace chrono {

/// Base class for assets that define basic textures.
/// Assets can be attached to ChBody objects.
/// Different post processing modules can handle
/// textures in proper ways (ex for ray tracing, or
/// openGL visualization), or may also create specialized
/// classes of textures with more properties.

class ChApi ChTexture : public ChAsset {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChTexture)

  protected:
    //
    // DATA
    //
    std::string filename;
    float scale_x;
    float scale_y;

  public:
    //
    // CONSTRUCTORS
    //

    ChTexture() : scale_x(1), scale_y(1) { filename = ""; };
    ChTexture(const char* mfilename) : scale_x(1), scale_y(1) { filename = mfilename; };
    ChTexture(const std::string& mfilename) : scale_x(1), scale_y(1) { filename = mfilename; };

    virtual ~ChTexture(){};

    //
    // FUNCTIONS
    //

    // Get the texture filename. This information could be used by visualization postprocessing.
    const std::string& GetTextureFilename() const { return filename; }
    // Set the texture filename. This information could be used by visualization postprocessing.
    void SetTextureFilename(const std::string& mfile) { filename = mfile; }
    // Set the texture scale
    void SetTextureScale(float sx, float sy) { scale_x = sx; scale_y = sy; }
    // Get the texture scales (in X and Y directions)
    float GetTextureScaleX() const { return scale_x; }
    float GetTextureScaleY() const { return scale_y; }

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChTexture>();
        // serialize parent class
        ChAsset::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(filename);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChTexture>();
        // deserialize parent class
        ChAsset::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(filename);
    }
};

CH_CLASS_VERSION(ChTexture,0)

}  // end namespace chrono

#endif
