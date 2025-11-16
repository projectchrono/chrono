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

#include "chrono/fea/ChDomainSurface.h"
#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChFieldElement.h"
//#include "chrono/fea/ChTetrahedronFace.h"
//#include "chrono/fea/ChHexahedronFace.h"

#include <unordered_set>
#include <map>
#include <array>
#include <algorithm>

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChDomainSurface)

void ChDomainSurface::AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase> >& node_set) {

    std::unordered_set<size_t> node_set_map;
    for (int i = 0; i < node_set.size(); ++i)
        node_set_map.insert((size_t)node_set[i].get());

    for (auto ie = this->mdomain->CreateIteratorOnElements(); !ie->is_end(); ie->next()) {
        auto element =ie->get_element();

        if (auto vol_el = std::dynamic_pointer_cast<ChFieldElementVolume>(element)) {
            for (int i_face = 0; i_face < vol_el->GetNumFaces(); ++i_face) {
                auto mface = vol_el->BuildFace(i_face, vol_el);
                bool is_face_selected = true;
                for (unsigned int i_node = 0; i_node < mface->GetNumNodes(); ++i_node) {
                    if (node_set_map.find((size_t)mface->GetNode(i_node).get()) == node_set_map.end()) {
                        is_face_selected = false;
                        break;
                    }
                }
                if (is_face_selected)
                    AddFace(mface);
            }
        }

        if (auto surf_el = std::dynamic_pointer_cast<ChFieldElementSurface>(element)) {
            bool is_face_selected = true;
            for (unsigned int i_node = 0; i_node < surf_el->GetNumNodes(); ++i_node) {
                if (node_set_map.find((size_t)surf_el->GetNode(i_node).get()) == node_set_map.end()) {
                    is_face_selected = false;
                    break;
                }
            }
            if (is_face_selected)
                AddFace(surf_el);
        }

    }
}

void ChDomainSurface::AddFacesFromBoundary() {

    // Boundary faces 
    std::multimap<std::array<ChNodeFEAbase*, 8>, std::shared_ptr<ChFieldElementSurface>> face_map_el;

    for (auto ie = this->mdomain->CreateIteratorOnElements(); !ie->is_end(); ie->next()) {
        auto element = ie->get_element();

        if (auto vol_el = std::dynamic_pointer_cast<ChFieldElementVolume>(element)) {
            for (int i_face = 0; i_face < vol_el->GetNumFaces(); ++i_face) {
                auto mface = vol_el->BuildFace(i_face, vol_el);
                std::array<ChNodeFEAbase*, 8> mface_key;
                unsigned int i_node = 0;
                for (; i_node < mface->GetNumNodes(); ++i_node) 
                    mface_key[i_node] = mface->GetNode(0).get();
                for (; i_node < mface_key.size(); ++i_node)
                    mface_key[i_node] = 0;
                std::sort(mface_key.begin(), mface_key.end());
                face_map_el.insert({ mface_key, mface });
            }
        }
    }

    for (auto it = face_map_el.begin(); it != face_map_el.end(); ) {
        auto key = it->first;
        size_t cnt = face_map_el.count(key);

        if (cnt == 1) {
            // Found a face that is not shared.. so it is a boundary face.
            // Instance it to be handled via shared ptr, and add it to list...
            this->AddFace(it->second);
        }

        // Advance iterator to the next distinct key
        it = face_map_el.upper_bound(key);
    }
}


}  // end namespace fea
}  // end namespace chrono
