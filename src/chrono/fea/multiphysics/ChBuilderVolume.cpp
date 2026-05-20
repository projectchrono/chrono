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


#include "chrono/fea/multiphysics/ChFieldElement.h"
#include "ChBuilderVolume.h"
#include "chrono/fea/multiphysics/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/multiphysics/ChFieldElementTetrahedron4Face.h"

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

void chrono::fea::ChBuilderVolumeBox::AddToModel(std::shared_ptr<ChFEModel> model) {
    for (auto& created_element : this->elements.list())
        model->AddElement(created_element);
    for (auto& created_node : this->nodes.list())
        for (int ifield = 0; ifield < model->GetNumFields(); ++ifield)
            model->GetField(ifield)->AddNode(created_node);
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
                    // Define the 8 corners of the current cube (local indexing)
                    auto A = nodes.at(i_x - 1, i_y - 1, i_z - 1);  // (0,0,0)
                    auto B = nodes.at(i_x, i_y - 1, i_z - 1);      // (1,0,0)
                    auto C = nodes.at(i_x - 1, i_y, i_z - 1);      // (0,1,0)
                    auto D = nodes.at(i_x, i_y, i_z - 1);          // (1,1,0)
                    auto E = nodes.at(i_x - 1, i_y - 1, i_z);      // (0,0,1)
                    auto F = nodes.at(i_x, i_y - 1, i_z);          // (1,0,1)
                    auto G = nodes.at(i_x - 1, i_y, i_z);          // (0,1,1)
                    auto H = nodes.at(i_x, i_y, i_z);              // (1,1,1)

                    // Kuhn decomposition: 6 tetrahedrons with orientation constraint
                    // Constraint: 4th node must be on negative side of plane (first 3 nodes clockwise)
                    // Verified via signed volume: dot(v3-v0, cross(v1-v0, v2-v0)) < 0

                    auto tetra1 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra1->SetNodes(A, D, B, H);  // Path: x→y→z

                    auto tetra2 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra2->SetNodes(A, B, F, H);  // Path: x→z→y

                    auto tetra3 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra3->SetNodes(A, C, D, H);  // Path: y→x→z

                    auto tetra4 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra4->SetNodes(A, G, C, H);  // Path: y→z→x

                    auto tetra5 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra5->SetNodes(A, F, E, H);  // Path: z→x→y

                    auto tetra6 = chrono_types::make_shared<ChFieldElementTetrahedron4>();
                    tetra6->SetNodes(A, E, G, H);  // Path: z→y→x

                    elements.at(i_x - 1, i_y - 1, i_z - 1) = {tetra1, tetra2, tetra3, tetra4, tetra5, tetra6};

                    // Track boundary faces (only when cube face coincides with box boundary)
                    // Face ID convention: id=0 opposite vertex 0, id=1 opposite vertex 1, etc.

                    // X-low boundary (x=0): cube face at local x=0 (i_x==1)
                    if (i_x == 1) {
                        // Tetra4 face 3: (A,G,C) on x=0 plane
                        // Tetra6 face 3: (A,E,G) on x=0 plane
                        this->faces_x_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra4, 3));
                        this->faces_x_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra6, 3));
                    }

                    // X-high boundary (x=W_x): cube face at local x=1 (i_x==nlayers_x)
                    if (i_x == nlayers_x) {
                        // Tetra1 face 0: (D,B,H) on x=1 plane
                        // Tetra2 face 0: (B,F,H) on x=1 plane
                        this->faces_x_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra1, 0));
                        this->faces_x_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra2, 0));
                    }

                    // Y-low boundary (y=0): cube face at local y=0 (i_y==1)
                    if (i_y == 1) {
                        // Tetra2 face 3: (A,B,F) on y=0 plane
                        // Tetra5 face 3: (A,F,E) on y=0 plane
                        this->faces_y_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra2, 3));
                        this->faces_y_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra5, 3));
                    }

                    // Y-high boundary (y=W_y): cube face at local y=1 (i_y==nlayers_y)
                    if (i_y == nlayers_y) {
                        // Tetra3 face 0: (C,D,H) on y=1 plane
                        // Tetra4 face 0: (G,C,H) on y=1 plane
                        this->faces_y_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra3, 0));
                        this->faces_y_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra4, 0));
                    }

                    // Z-low boundary (z=0): cube face at local z=0 (i_z==1)
                    if (i_z == 1) {
                        // Tetra1 face 3: (A,D,B) on z=0 plane
                        // Tetra3 face 3: (A,C,D) on z=0 plane
                        this->faces_z_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra1, 3));
                        this->faces_z_lo.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra3, 3));
                    }

                    // Z-high boundary (z=W_z): cube face at local z=1 (i_z==nlayers_z)
                    if (i_z == nlayers_z) {
                        // Tetra5 face 0: (F,E,H) on z=1 plane
                        // Tetra6 face 0: (E,G,H) on z=1 plane
                        this->faces_z_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra5, 0));
                        this->faces_z_hi.push_back(chrono_types::make_shared<ChFieldTetrahedron4Face>(tetra6, 0));
                    }
                }
            }
        }
    }
}

void chrono::fea::ChBuilderVolumeBoxTetra::AddToModel(std::shared_ptr<ChFEModel> model) {
    for (auto& created_elementc : this->elements.list())
        for (auto& created_element : created_elementc)
            model->AddElement(created_element);
    for (auto& created_node : this->nodes.list())
        for (int ifield = 0; ifield < model->GetNumFields(); ++ifield)
            model->GetField(ifield)->AddNode(created_node);
}


}  // end namespace fea
}  // end namespace chrono
