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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/fea/ChMeshSurface.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChTetrahedronFace.h"
#include "chrono/fea/ChHexahedronFace.h"

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
        auto element = mmesh->GetElement(ie);

        if (auto tet = std::dynamic_pointer_cast<ChElementTetrahedron>(element)) {
            unsigned char nodes = 0;  // if the i-th bit is 1, the corressponding hex node is in the set
            for (int in = 0; in < 4; in++) {
                if (node_set_map.find((size_t)tet->GetTetrahedronNode(in).get()) != node_set_map.end())
                    nodes |= 1 << in;
            }
            unsigned char masks[] = {
                0x0E,  // 1110 face 0
                0x0D,  // 1101 face 1
                0x0B,  // 1011 face 2
                0x07   // 0111 face 3
            };
            for (int j = 0; j < 4; j++) {
                if ((nodes & masks[j]) == masks[j])
                    AddFace(chrono_types::make_shared<ChTetrahedronFace>(tet, j));
            }
        }

        if (auto hex = std::dynamic_pointer_cast<ChElementHexahedron>(element)) {
            unsigned char nodes = 0;  // if the i-th bit is 1, the corressponding hex node is in the set
            for (int in = 0; in < 7; in++) {
                if (node_set_map.find((size_t)hex->GetHexahedronNode(in).get()) != node_set_map.end())
                    nodes |= 1 << in;
            }
            unsigned char masks[] = {
                0x0F,  // 00001111 face 0
                0x33,  // 00110011 face 1
                0x66,  // 01100110 face 2
                0xCC,  // 11001100 face 3
                0x99,  // 10011001 face 4
                0xF0   // 11110000 face 5
            };
            for (int j = 0; j < 6; j++) {
                if ((nodes & masks[j]) == masks[j])
                    AddFace(chrono_types::make_shared<ChHexahedronFace>(hex, j));
            }
        }

        if (auto shell = std::dynamic_pointer_cast<ChLoadableUV>(element)) {
            this->AddFace(shell);
        }
    }
}

void ChMeshSurface::AddFacesFromBoundary() {
    // Boundary faces of TETRAHEDRONS
    std::multimap<std::array<ChNodeFEAxyz*, 3>, ChTetrahedronFace> face_map_tet;

    for (unsigned int ie = 0; ie < this->mmesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetrahedron>(mmesh->GetElement(ie))) {
            for (int nface = 0; nface < 4; ++nface) {
                ChTetrahedronFace mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map_tet.insert({mface_key, mface});
            }
        }
    }
    for (unsigned int ie = 0; ie < this->mmesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetrahedron>(mmesh->GetElement(ie))) {
            for (int nface = 0; nface < 4; ++nface) {
                ChTetrahedronFace mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map_tet.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face.
                    // Instance it to be handled via shared ptr, and add it to list...
                    auto boundary_face = chrono_types::make_shared<ChTetrahedronFace>(mtetra, nface);
                    this->AddFace(boundary_face);
                }
            }
        }
    }

    // Boundary faces of HEXAHEDRONS
    std::multimap<std::array<ChNodeFEAxyz*, 4>, ChHexahedronFace> face_map_hex;

    for (unsigned int ie = 0; ie < mmesh->GetNelements(); ++ie) {
        if (auto mbrick = std::dynamic_pointer_cast<ChElementHexahedron>(mmesh->GetElement(ie))) {
            for (int nface = 0; nface < 6; ++nface) {
                ChHexahedronFace mface(mbrick, nface);
                std::array<ChNodeFEAxyz*, 4> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get(), mface.GetNodeN(3).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map_hex.insert({mface_key, mface});
            }
        }
    }
    for (unsigned int ie = 0; ie < mmesh->GetNelements(); ++ie) {
        if (auto mbrick = std::dynamic_pointer_cast<ChElementHexahedron>(mmesh->GetElement(ie))) {
            for (int nface = 0; nface < 6; ++nface) {
                ChHexahedronFace mface(mbrick, nface);
                std::array<ChNodeFEAxyz*, 4> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get(), mface.GetNodeN(3).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map_hex.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face.
                    // Instance it to be handled via shared ptr, and add it to list...
                    auto boundary_face = chrono_types::make_shared<ChHexahedronFace>(mbrick, nface);
                    this->AddFace(boundary_face);
                }
            }
        }
    }
}

}  // end namespace fea
}  // end namespace chrono
