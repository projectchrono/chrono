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
// Authors: Andrea Favali, Alessandro Tasora
// =============================================================================

#include "chrono/fea/ChMeshSurface.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChFaceTetra_4.h"

#include <unordered_set>
#include <map>
#include <array>
#include <algorithm>

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChMeshSurface)

void ChMeshSurface::AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set) {
    std::unordered_set<size_t> node_set_map;

    for (int i = 0; i < node_set.size(); ++i)
        node_set_map.insert((size_t)node_set[i].get());

    for (unsigned int ie = 0; ie < this->mmesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetra_4>(this->mmesh->GetElement(ie))) {
            bool n0 = (node_set_map.find((size_t)mtetra->GetNodeN(0).get()) != node_set_map.end());
            bool n1 = (node_set_map.find((size_t)mtetra->GetNodeN(1).get()) != node_set_map.end());
            bool n2 = (node_set_map.find((size_t)mtetra->GetNodeN(2).get()) != node_set_map.end());
            bool n3 = (node_set_map.find((size_t)mtetra->GetNodeN(3).get()) != node_set_map.end());

            if (n0 && n1 && n2) {
                auto mface = chrono_types::make_shared<ChFaceTetra_4>(mtetra, 3);
                this->AddFace(mface);
            }
            if (n1 && n2 && n3) {
                auto mface = chrono_types::make_shared<ChFaceTetra_4>(mtetra, 0);
                this->AddFace(mface);
            }
            if (n0 && n2 && n3) {
                auto mface = chrono_types::make_shared<ChFaceTetra_4>(mtetra, 1);
                this->AddFace(mface);
            }
            if (n0 && n1 && n3) {
                auto mface = chrono_types::make_shared<ChFaceTetra_4>(mtetra, 2);
                this->AddFace(mface);
            }
        }

        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF>(this->mmesh->GetElement(ie))) {
            this->AddFace(mshell);
        }
    }
    // GetLog() << "AddFacesFromNodeSet found " << this->faces.size() << " faces \n\n";
}

void ChMeshSurface::AddFacesFromBoundary() {
    /// Case1. Outer skin boundary of meshes of TETRAHEDRONS:
    ///
    std::multimap<std::array<ChNodeFEAxyz*, 3>, ChFaceTetra_4> face_map;

    for (unsigned int ie = 0; ie < this->mmesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetra_4>(mmesh->GetElement(ie))) {
            for (int nface = 0; nface < 4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map.insert({mface_key, mface});
            }
        }
    }
    for (unsigned int ie = 0; ie < this->mmesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetra_4>(mmesh->GetElement(ie))) {
            for (int nface = 0; nface < 4; ++nface) {
                ChFaceTetra_4 mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face.
                    // Instance it to be handled via shared ptr, and add it to list...
                    auto boundary_face = chrono_types::make_shared<ChFaceTetra_4>(mtetra, nface);
                    this->AddFace(boundary_face);
                }
            }
        }
    }
}

}  // end namespace fea
}  // end namespace chrono
