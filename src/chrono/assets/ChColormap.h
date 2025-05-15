// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_COLORMAP_H
#define CH_COLORMAP_H

#include <vector>
#include <unordered_map>

#include "chrono/assets/ChColor.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Definition of a color map.
class ChApi ChColormap {
  public:
    /// Colormap type.
    /// Data obtained from https://www.kennethmoreland.com/color-advice/
    enum class Type { BLACK_BODY, BROWN, COPPER, FAST, INFERNO, JET, KINDLMANN, PLASMA, RED_BLUE, VIRIDIS };

    /// Names of colormap definition files.
    struct Files {
        std::string data_file;
        std::string img_file;
    };

    /// Construct a colormap of specified type.
    ChColormap(Type type = Type::JET);

    /// Load data for the specified colormap, replacing current data.
    void Load(Type type);

    /// Return a color corresponding to the provided value, assumed in [0,1].
    /// The output color is obtained by linear interpolation in the current colormap table.
    /// Out of range input values are clamped to [0,1].
    ChColor Get(double value) const;

    /// Return a color corresponding to the provided value, assumed in [vmin,vmax].
    /// The output color is obtained by linear interpolation in the current colormap table.
    /// Out of range input values are clamped to [vmin,vmax].
    ChColor Get(double value, double vmin, double vmax) const;

    /// Return the image filename for the specified colormap type.
    static const std::string& GetDataFilename(Type type) { return m_colormap_files.at(type).data_file; }

    /// Return the image filename for the specified colormap type.
    static const std::string& GetImageFilename(Type type) { return m_colormap_files.at(type).img_file; }

    /// Return the list of specification files for all available colormaps.
    static const std::unordered_map<Type, Files>& GetFilenames() { return m_colormap_files; }

    /// Method to allow serialization of transient data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

  private:
    typedef std::pair<double, ChColor> Entry;
    std::vector<Entry> m_map;

    static std::unordered_map<Type, Files> m_colormap_files;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChColormap, 0)

}  // end namespace chrono

#endif
