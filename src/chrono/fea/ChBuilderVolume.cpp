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

namespace chrono {
namespace fea {


void chrono::fea::ChBuilderVolumeBox::BuildVolume(const ChFrame<>& frame, int nlayers_x, int nlayers_y, int nlayers_z, double W_x, double W_y, double W_z)
{
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
                    auto hexa = chrono_types::make_shared <ChFieldElementHexahedron8>();
                    hexa->SetNodes({ nodes.at(i_x - 1, i_y - 1, i_z - 1),
                        nodes.at(i_x  , i_y - 1, i_z - 1),
                        nodes.at(i_x  , i_y  , i_z - 1),
                        nodes.at(i_x - 1, i_y  , i_z - 1),
                        nodes.at(i_x - 1, i_y - 1, i_z),
                        nodes.at(i_x  , i_y - 1, i_z),
                        nodes.at(i_x  , i_y  , i_z),
                        nodes.at(i_x - 1, i_y  , i_z)
                        });
                    elements.at(i_x - 1, i_y - 1, i_z - 1) = hexa;
                }
            }
        }
    }
}


void chrono::fea::ChBuilderVolumeBoxTetra::BuildVolume(const ChFrame<>& frame, int nlayers_x, int nlayers_y, int nlayers_z, double W_x, double W_y, double W_z)
{
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
                    auto tetra1 = chrono_types::make_shared <ChFieldElementTetrahedron4>();
                    tetra1->SetNodes(
                        nodes.at(i_x - 1, i_y - 1, i_z - 1),
                        nodes.at(i_x    , i_y - 1, i_z - 1),
                        nodes.at(i_x - 1, i_y - 1, i_z    ),
                        nodes.at(i_x - 1, i_y    , i_z - 1)
                        );
                    auto tetra2 = chrono_types::make_shared <ChFieldElementTetrahedron4>();
                    tetra2->SetNodes(
                        nodes.at(i_x    , i_y - 1, i_z    ),
                        nodes.at(i_x - 1, i_y - 1, i_z    ),
                        nodes.at(i_x    , i_y - 1, i_z - 1),
                        nodes.at(i_x    , i_y    , i_z    )
                    );
                    auto tetra3 = chrono_types::make_shared <ChFieldElementTetrahedron4>();
                    tetra3->SetNodes(
                        nodes.at(i_x - 1, i_y    , i_z    ),
                        nodes.at(i_x    , i_y    , i_z    ),
                        nodes.at(i_x - 1, i_y    , i_z - 1),
                        nodes.at(i_x - 1, i_y - 1, i_z    )
                    );
                    auto tetra4 = chrono_types::make_shared <ChFieldElementTetrahedron4>();
                    tetra4->SetNodes(
                        nodes.at(i_x - 1, i_y    , i_z - 1),
                        nodes.at(i_x    , i_y    , i_z    ),
                        nodes.at(i_x    , i_y    , i_z - 1),
                        nodes.at(i_x    , i_y - 1, i_z - 1)
                    );
                    auto tetra5 = chrono_types::make_shared <ChFieldElementTetrahedron4>();
                    tetra5->SetNodes(
                        nodes.at(i_x    , i_y    , i_z    ),
                        nodes.at(i_x - 1, i_y    , i_z - 1),
                        nodes.at(i_x - 1, i_y - 1, i_z    ),
                        nodes.at(i_x    , i_y - 1, i_z - 1)
                    );

                    elements.at(i_x - 1, i_y - 1, i_z - 1) = {tetra1, tetra2, tetra3, tetra4, tetra5};
                }
            }
        }
    }
}


}  // end namespace fea
}  // end namespace chrono
