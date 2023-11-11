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

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector2.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for defining a texture.
/// Encapsulates the texture filename and texture scale.
class ChApi ChTexture {
  public:
    ChTexture();
    ChTexture(const char* filename);
    ChTexture(const std::string& filename, float scale_x = 1, float scale_y = 1);

    ~ChTexture() {}

    /// Get the texture filename.
    const std::string& GetFilename() const { return m_filename; }

    /// Set the texture filename.
    void SetFilename(const std::string& filename) { m_filename = filename; }

    /// Set the texture scale (in X and Y directions).
    void SetScale(float sx, float sy);

    /// Set the texture scale (in X and Y directions).
    void SetScale(const ChVector2<float>& scale);

    /// Get the texture scales (in X and Y directions)
    float GetScaleX() const { return m_scale.x(); }
    float GetScaleY() const { return m_scale.y(); }

    /// Get the texture scales (in X and Y directions)
    const ChVector2<float>& GetScale() const { return m_scale; }

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& marchive);

  private:
    std::string m_filename;
    ChVector2<float> m_scale;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChTexture, 0)

}  // end namespace chrono

#endif
