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


#include "chrono/fea/ChFieldElement.h"
#include "ChBuilderVolume.h"
#include "chrono/fea/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/ChFieldElementTetrahedron4Face.h"

namespace chrono {
namespace fea {


void chrono::fea::ChBuilderVolumeBox::BuildVolume(const ChFrame<>& frame, int nlayers_x, int nlayers_y, int nlayers_z, double W_x, double W_y, double W_z) {
    this->nodes.Resize(nlayers_x + 1, nlayers_y + 1, nlayers_z + 1);
    this->elements.Resize(nlayers_x, nlayers_y, nlayers_z);

    for (int i_z = 0; i_z <= nlayers_z; ++i_z) {
        for (int i_y = 0; i_y <= nlayers_y; ++i_y) {
            for (int i_x = 0; i_x <= nlayers_x; ++i_x) {
                ChVector3d mypos((W_x / nlayers_x) * i_x, (W_y / nlayers_y) * i_y, (W_z / nlayers_z) * i_z);
                mypos = frame.TransformPointLocalToParent(mypos);
                auto mnode = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
                mnode->Set(mypos);
                nodes.at(i_x, i_y, i_z) = mnode;
                if (i_x > 0 && i_y > 0 && i_z > 0) {
                    auto hexa = chrono_types::make_shared<ChFieldElementHexahedron8>();
                    hexa->SetNodes({nodes.at(i_x - 1, i_y - 1, i_z - 1), nodes.at(i_x, i_y - 1, i_z - 1), nodes.at(i_x, i_y, i_z - 1), nodes.at(i_x - 1, i_y, i_z - 1),
                                    nodes.at(i_x - 1, i_y - 1, i_z), nodes.at(i_x, i_y - 1, i_z), nodes.at(i_x, i_y, i_z), nodes.at(i_x - 1, i_y, i_z)});
                    elements.at(i_x - 1, i_y - 1, i_z - 1) = hexa;

                    // fill 6 face containers
                    if (i_x == 1)
                        this->faces_x_lo.push_back(chrono_types::make_shared<ChFieldHexahedron8Face>(hexa, 0));
                    if (i_x == nlayers_x)
                        this->faces_x_hi.push_back(chrono_types::make_shared<ChFieldHexahedron8Face>(hexa, 1));
                    if (i_y == 1)
                        this->faces_y_lo.push_back(chrono_types::make_shared<ChFieldHexahedron8Face>(hexa, 2));
                    if (i_y == nlayers_y)
                        this->faces_y_hi.push_back(chrono_types::make_shared<ChFieldHexahedron8Face>(hexa, 3));
                    if (i_z == 1)
                        this->faces_z_lo.push_back(chrono_types::make_shared<ChFieldHexahedron8Face>(hexa, 4));
                    if (i_z == nlayers_z)
                        this->faces_z_hi.push_back(chrono_types::make_shared<ChFieldHexahedron8Face>(hexa, 5));
                } 
            }
        }
    }
}

void chrono::fea::ChBuilderVolumeBox::AddToDomain(std::shared_ptr<ChDomain> domain) {
    for (auto& created_element : this->elements.list())
        domain->AddElement(created_element);
    for (auto& created_node : this->nodes.list())
        for (int ifield = 0; ifield < domain->GetNumFields(); ++ifield)
            domain->GetField(ifield)->AddNode(created_node);
}


void chrono::fea::ChBuilderVolumeBoxTetra::BuildVolume(const ChFrame<>& frame, int nlayers_x, int nlayers_y, int nlayers_z, double W_x, double W_y, double W_z) {
    this->nodes.Resize(nlayers_x + 1, nlayers_y + 1, nlayers_z + 1);
    this->elements.Resize(nlayers_x, nlayers_y, nlayers_z);

    for (int i_z = 0; i_z <= nlayers_z; ++i_z) {
        for (int i_y = 0; i_y <= nlayers_y; ++i_y) {
            for (int i_x = 0; i_x <= nlayers_x; ++i_x) {
                ChVector3d mypos((W_x / nlayers_x) * i_x, (W_y / nlayers_y) * i_y, (W_z / nlayers_z) * i_z);
                mypos = frame.TransformPointLocalToParent(mypos);
                auto mnode = chrono_types::make_shared<ChNodeFEAfieldXYZ>();
                mnode->Set(mypos);
                nodes.at(i_x, i_y, i_z) = mnode;
                if (i_x > 0 && i_y > 0 && i_z > 0) {
                    auto tetra1 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra1->SetNodes(nodes.at(i_x - 1, i_y - 1, i_z - 1), nodes.at(i_x, i_y - 1, i_z - 1), nodes.at(i_x - 1, i_y - 1, i_z), nodes.at(i_x - 1, i_y, i_z - 1));
                    auto tetra2 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra2->SetNodes(nodes.at(i_x, i_y - 1, i_z), nodes.at(i_x - 1, i_y - 1, i_z), nodes.at(i_x, i_y - 1, i_z - 1), nodes.at(i_x, i_y, i_z));
                    auto tetra3 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra3->SetNodes(nodes.at(i_x - 1, i_y, i_z), nodes.at(i_x, i_y, i_z), nodes.at(i_x - 1, i_y, i_z - 1), nodes.at(i_x - 1, i_y - 1, i_z));
                    auto tetra4 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra4->SetNodes(nodes.at(i_x - 1, i_y, i_z - 1), nodes.at(i_x, i_y, i_z), nodes.at(i_x, i_y, i_z - 1), nodes.at(i_x, i_y - 1, i_z - 1));
                    auto tetra5 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra5->SetNodes(nodes.at(i_x, i_y, i_z), nodes.at(i_x - 1, i_y, i_z - 1), nodes.at(i_x - 1, i_y - 1, i_z), nodes.at(i_x, i_y - 1, i_z - 1));

                    elements.at(i_x - 1, i_y - 1, i_z - 1) = {tetra1, tetra2, tetra3, tetra4, tetra5};

                    // fill 6 face containers
                    if (i_x == 1) {
                        this->faces_x_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra1, 1));
                        this->faces_x_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra3, 1));
                    }
                    if (i_x == nlayers_x) {
                        this->faces_x_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra2, 1));
                        this->faces_x_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra4, 0));
                    }
                    if (i_y == 1) {
                        this->faces_y_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra2, 3));
                        this->faces_y_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra1, 3));
                    }
                    if (i_y == nlayers_y) {
                        this->faces_y_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra3, 3));
                        this->faces_y_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra4, 3));
                    }
                    if (i_z == 1) {
                        this->faces_z_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra1, 2));
                        this->faces_z_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra4, 1));
                    }
                    if (i_z == nlayers_z) {
                        this->faces_z_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra2, 2));
                        this->faces_z_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra3, 2));
                    }
                }
            }
        }
    }
}

void chrono::fea::ChBuilderVolumeBoxTetra::AddToDomain(std::shared_ptr<ChDomain> domain) {
    for (auto& created_elementc : this->elements.list())
        for (auto& created_element : created_elementc)
            domain->AddElement(created_element);
    for (auto& created_node : this->nodes.list())
        for (int ifield = 0; ifield < domain->GetNumFields(); ++ifield)
            domain->GetField(ifield)->AddNode(created_node);
}


}  // end namespace fea
}  // end namespace chrono
