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

#ifndef CHCOLORASSET_H
#define CHCOLORASSET_H

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"

namespace chrono {

/// Base class for assets that carry basic informations about the surface color
/// for visualization assets. Here we assume that a ChColorAsset 'paints' all
/// the assets of the list (or in same ChAssetLevel). It can be used when
/// exporting to postprocessing or in OpenGL views. The postprocessor must be
/// able to recognize it and implement the proper translation.

class ChApi ChColorAsset : public ChAsset {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChColorAsset)

  public:
    ChColorAsset() : fading(0) {}
    ChColorAsset(const ChColor& c) : color(c), fading(0) {}
    ChColorAsset(float mR, float mG, float mB, float mA = 0) : color(mR, mG, mB, mA), fading(0) {}

    /// Get/Set the color of the surface. This information could be used by
    /// visualization postprocessing.
    const ChColor& GetColor() const { return color; }
    void SetColor(const ChColor& mc) { color = mc; }

    /// Get/Set the fading amount, 0 <= fading <= 1.
    /// If fading = 0, no transparency of surface,
    /// If fading = 1, surface is completely transparent.
    float GetFading() const { return fading; }
    void SetFading(const float mc) { fading = mc; }

  private:
    ChColor color;  // color of material
    float fading;   // transparency of material


  public:

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChColorAsset>();
        // serialize parent class
        ChAsset::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(color);
        marchive << CHNVP(fading);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChColorAsset>();
        // deserialize parent class
        ChAsset::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(color);
        marchive >> CHNVP(fading);
    }
};

CH_CLASS_VERSION(ChColorAsset,0)

}  // end namespace chrono

#endif
