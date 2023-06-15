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

#ifndef CH_OBJFILE_SHAPE_H
#define CH_OBJFILE_SHAPE_H

#include "chrono/assets/ChVisualShape.h"

namespace chrono {

/// Class for referencing a Wavefront OBJ file containing a shape that can be visualized in some way.
/// The file is not loaded into this object; this is simply a reference to the resource on disk.
class ChApi ChModelFileShape : public ChVisualShape {
  public:
    ChModelFileShape();
    ChModelFileShape(const std::string& fname);

    ~ChModelFileShape() {}

    std::string GetFilename() const { return filename; }
    void SetFilename(const std::string& fname) { filename = fname; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    std::string filename;
};

CH_CLASS_VERSION(ChModelFileShape, 0)

}  // end namespace chrono

#endif
