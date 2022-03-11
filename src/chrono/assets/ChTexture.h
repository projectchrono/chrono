// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHTEXTURE_H
#define CHTEXTURE_H

#include <string>

#include "chrono/assets/ChAsset.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector2.h"

namespace chrono {

/// Class for defining a texture.
/// Encapsulates the texture filename and texture scale.
class ChApi ChTexture : public ChAsset {
  public:
    ChTexture();
    ChTexture(const char* filename);
    ChTexture(const std::string& filename, ChVector2<float> scale = ChVector2<float>(1, 1));

    ~ChTexture() {}

    /// Get the texture filename.
    const std::string& GetTextureFilename() const { return m_filename; }

    /// Set the texture filename.
    void SetTextureFilename(const std::string& filename) { m_filename = filename; }

    /// Set the texture scale (in X and Y directions).
    void SetTextureScale(float sx, float sy) {
        scale_x = sx;
        scale_y = sy;
        m_scale = ChVector2<float>(sx, sy);
    }

    /// Set the texture scale (in X and Y directions).
    void SetScale(const ChVector2<float>& scale) {
        scale_x = scale.x();
        scale_y = scale.y();
        m_scale = scale;
    }

    // Get the texture scales (in X and Y directions)
    float GetTextureScaleX() const { return scale_x; }
    float GetTextureScaleY() const { return scale_y; }

    /// Get the texture scale.
    const ChVector2<float>& GetScale() const { return m_scale; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    std::string m_filename;
    ChVector2<float> m_scale;
    float scale_x;
    float scale_y;
};

CH_CLASS_VERSION(ChTexture, 0)

}  // end namespace chrono

#endif
