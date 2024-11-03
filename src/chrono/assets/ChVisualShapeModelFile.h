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

#ifndef CH_VISUAL_SHAPE_MODEL_FILE_H
#define CH_VISUAL_SHAPE_MODEL_FILE_H

#include "chrono/assets/ChVisualShape.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a Wavefront OBJ file containing a shape that can be visualized in some way.
/// The file is not loaded into this object; this is simply a reference to the resource on disk.
class ChApi ChVisualShapeModelFile : public ChVisualShape {
  public:
    ChVisualShapeModelFile();
    ChVisualShapeModelFile(const std::string& fname);

    ~ChVisualShapeModelFile() {}

    const std::string& GetFilename() const { return filename; }
    void SetFilename(const std::string& fname) { filename = fname; }

    const ChVector3d& GetScale() const { return scale; }
    void SetScale(const ChVector3d& s) { scale = s; }
    void SetScale(double s) { scale = ChVector3d(s, s, s); }

    /// Get the shape bounding box.
    /// Currently, this calculates an updated bounding box only for OBJ and STL files.
    virtual ChAABB GetBoundingBox() const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::string filename;
    ChVector3d scale;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeModelFile, 0)

}  // end namespace chrono

#endif
