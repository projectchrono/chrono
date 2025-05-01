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

#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeModelFile)

ChVisualShapeModelFile::ChVisualShapeModelFile() : filename(""), scale(1) {
    SetMutable(false);
}

ChVisualShapeModelFile::ChVisualShapeModelFile(const std::string& fname) : filename(fname), scale(1) {
    SetMutable(false);
}

ChAABB ChVisualShapeModelFile::GetBoundingBox() const {
    auto ext = filesystem::path(filename).extension();
    std::shared_ptr<ChTriangleMeshConnected> trimesh;
    if (ext == "obj" || ext == "OBJ")
        trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(filename, false);
    else if (ext == "stl" || ext == "STL")
        trimesh = ChTriangleMeshConnected::CreateFromSTLFile(filename, true);
    if (trimesh)
        return trimesh->GetBoundingBox();
    return ChAABB();
}

void ChVisualShapeModelFile::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeModelFile>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(filename);
    archive_out << CHNVP(scale);
}

void ChVisualShapeModelFile::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeModelFile>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(filename);
    archive_in >> CHNVP(scale);
}

}  // end namespace chrono
