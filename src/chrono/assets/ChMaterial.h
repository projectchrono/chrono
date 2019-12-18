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

#ifndef CHMATERIAL_H
#define CHMATERIAL_H

#include <string>
#include <vector>

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Visualization material properties.
class ChApi ChMaterial {
  public:
    ChMaterial() {}
    ~ChMaterial() {}

    enum ChMaterialType { CH_MATERIAL_DIFFUSE, CH_MATERIAL_PHONG, CH_MATERIAL_CONDUCTOR, CH_MATERIAL_PLASTIC };

    bool IsVisible() const { return visible; }
    void SetVisible(bool mv) { visible = mv; }

    /// Get the color of the surface.
    /// This information could be used by visualization postprocessing.
    const ChColor& GetColor() const { return color; }

    /// Set the color of the surface.
    /// This information could be used by visualization postprocessing.
    void SetColor(const ChColor& mc) { color = mc; }

    /// Get the fading amount, 0..1.
    /// If =0, no transparency of surface, it =1 surface is completely transparent.
    float GetFading() const { return fading; }

    /// Set the fading amount, 0..1.
    /// If =0, no transparency of surface, it =1 surface is completely transparent.
    void SetFading(const float mc) { fading = mc; }

    /// Set the material type.
    void SetType(const ChMaterialType type) { material_type = type; }

    /// Set material properties.
    void SetOption(const std::string& type, const std::string& parameter, const std::string& value) {
        material_option temp;
        temp.type = type;
        temp.parameter = parameter;
        temp.value = value;
        options.push_back(temp);
    }

    ChColor color;                 ///< material color
    float fading;                  ///< transparency level
    ChMaterialType material_type;  ///< material type
    bool visible;                  ///< true if material is visible

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite<ChMaterial>();

        // serialize all member data:
        marchive << CHNVP(color);
        marchive << CHNVP(fading);
        ChMaterialType_mapper mmapper;
        marchive << CHNVP(mmapper(material_type), "material_type");
        marchive << CHNVP(options);
        marchive << CHNVP(visible);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead<ChMaterial>();

        // stream in all member data:
        marchive >> CHNVP(color);
        marchive >> CHNVP(fading);
        ChMaterialType_mapper mmapper;
        marchive >> CHNVP(mmapper(material_type), "material_type");
        marchive >> CHNVP(options);
        marchive >> CHNVP(visible);
    }

    /// @cond
    CH_ENUM_MAPPER_BEGIN(ChMaterialType);
    CH_ENUM_VAL(CH_MATERIAL_DIFFUSE);
    CH_ENUM_VAL(CH_MATERIAL_PHONG);
    CH_ENUM_VAL(CH_MATERIAL_CONDUCTOR);
    CH_ENUM_VAL(CH_MATERIAL_PLASTIC);
    CH_ENUM_MAPPER_END(ChMaterialType);
    /// @endcond

  private:
    struct material_option {
        std::string type, parameter, value;

        // SERIALIZATION

        /// Method to allow serialization of transient data to archives.
        virtual void ArchiveOUT(ChArchiveOut& marchive) {
            marchive.VersionWrite<material_option>();
            // serialize all member data:
            marchive << CHNVP(type);
            marchive << CHNVP(parameter);
            marchive << CHNVP(value);
        }

        /// Method to allow de-serialization of transient data from archives.
        virtual void ArchiveIN(ChArchiveIn& marchive) {
            int version = marchive.VersionRead<material_option>();
            // deserialize all member data:
            marchive >> CHNVP(type);
            marchive >> CHNVP(parameter);
            marchive >> CHNVP(value);
        }
    };

    std::vector<material_option> options;
};

CH_CLASS_VERSION(ChMaterial, 0)

}  // end namespace chrono

#endif
