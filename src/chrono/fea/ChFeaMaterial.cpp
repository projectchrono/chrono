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

#include <cmath>

#include "chrono/fea/ChFeaMaterial.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
/*
 ChFeaMaterialModel::ChFeaMaterialModel(const ChFeaMaterial& other) {
    //m_density = other.m_density;
}
*/

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFea3DContinuum)

void ChFea3DContinuum::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFeaMaterial>();
    // serialize parent class
    // serialize all member data:
    //archive_out << CHNVP(m_density);
}

void ChFea3DContinuum::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFeaMaterial>();
    // deserialize parent class
    // stream in all member data:
    ///archive_in >> CHNVP(m_density);
}

// -----------------------------------------------------------------------------

//***TEST***

void _test_function() {

    auto mnode1 = chrono_types::make_shared<ChFeaNodeXYZ>();
    auto mnode2 = chrono_types::make_shared<ChFeaNodeXYZ>();
    auto mnode3 = chrono_types::make_shared<ChFeaNodeXYZ>();
    auto mnode4 = chrono_types::make_shared<ChFeaNodeXYZ>();

    mnode1->Set(0, 0, 0);
    mnode2->Set(0, 0, 1);
    mnode3->Set(0, 1, 0);
    mnode4->Set(1, 0, 0);

    auto displacement_field = chrono_types::make_shared <ChFeaFieldDisplacement3D>();
    displacement_field->AddNode(mnode1);
    displacement_field->AddNode(mnode2);
    displacement_field->AddNode(mnode3);
    displacement_field->AddNode(mnode4);

    displacement_field->node_data[mnode1].SetFixed(true);
    displacement_field->node_data[mnode1].SetPos(*mnode1);
    displacement_field->node_data[mnode2].SetPos(*mnode2);
    displacement_field->node_data[mnode3].SetPos(*mnode3);
    displacement_field->node_data[mnode4].SetPos(ChVector3d(1, 0, 0));
    displacement_field->node_data[mnode4].SetLoad(ChVector3d(200,10,20));

    auto temperature_field = chrono_types::make_shared <ChFeaFieldTemperature>();
    temperature_field->AddNode(mnode1);
    temperature_field->AddNode(mnode2);
    temperature_field->AddNode(mnode3);
    temperature_field->AddNode(mnode4);
    temperature_field->node_data[mnode1].SetFixed(true);
    temperature_field->node_data[mnode4].State()[0] = 100;


}

}  // end namespace fea
}  // end namespace chrono
