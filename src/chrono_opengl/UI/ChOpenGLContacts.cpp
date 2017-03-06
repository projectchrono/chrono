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
// Authors: Hammad Mazhar
// =============================================================================
// Renders contact points as a point cloud
// =============================================================================

#include <iostream>

#include "chrono_opengl/UI/ChOpenGLContacts.h"
#include "chrono_opengl/ChOpenGLMaterials.h"

#ifdef CHRONO_PARALLEL
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChDataManager.h"
#endif

namespace chrono {
using namespace collision;
namespace opengl {
using namespace glm;

ChOpenGLContacts::ChOpenGLContacts() {}

bool ChOpenGLContacts::Initialize(ChOpenGLMaterial mat, ChOpenGLShader* shader) {
    if (GLReturnedError("Contacts::Initialize - on entry"))
        return false;
    contact_data.push_back(glm::vec3(0, 0, 0));
    contacts.Initialize(contact_data, mat, shader);
    contacts.SetPointSize(0.01f);
    return true;
}

void ChOpenGLContacts::UpdateChrono(ChSystem* system) {
    //  ChContactContainerDVI* container = (ChContactContainerDVI*)system->GetContactContainer();
    //  std::list<ChContactContainerDVI::ChContact_6_6*> list = container-GetContactList();
    //  int num_contacts = container->GetNcontacts();
    //  int counter = 0;
    //  contact_data.resize(num_contacts * 2);
    //
    //  for (std::list<ChContactContainerDVI::ChContact_6_6*>::const_iterator iterator = list.begin(), end = list.end();
    //  iterator != end; ++iterator) {
    //    ChVector<> p1 = (*iterator)->GetContactP1();
    //    ChVector<> p2 = (*iterator)->GetContactP2();
    //
    //    contact_data[counter] = glm::vec3(p1.x, p1.y, p1.z);
    //    contact_data[counter + num_contacts] = glm::vec3(p2.x, p2.y, p2.z);
    //    counter++;
    //  }
}
void ChOpenGLContacts::UpdateChronoParallel(ChSystemParallel* system) {
#ifdef CHRONO_PARALLEL
    ChParallelDataManager* data_manager = system->data_manager;
    int num_contacts = data_manager->num_rigid_contacts + data_manager->num_rigid_fluid_contacts +
                       data_manager->num_rigid_tet_contacts + data_manager->num_rigid_tet_node_contacts;

    // std::cout << "CONTACT RENDER: " << num_contacts << std::endl;

    if (num_contacts == 0) {
        return;
    }

    contact_data.resize(data_manager->num_rigid_contacts * 2 + data_manager->num_rigid_tet_contacts * 2 +
                        data_manager->num_rigid_tet_node_contacts + data_manager->num_rigid_fluid_contacts);

    //#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_rigid_contacts; i++) {
        real3 cpta = data_manager->host_data.cpta_rigid_rigid[i];
        real3 cptb = data_manager->host_data.cptb_rigid_rigid[i];

        contact_data[i] = glm::vec3(cpta.x, cpta.y, cpta.z);
        contact_data[i + data_manager->num_rigid_contacts] = glm::vec3(cptb.x, cptb.y, cptb.z);
    }
    int offset = data_manager->num_rigid_contacts * 2;
    int index = 0;
    if (data_manager->num_rigid_tet_contacts > 0) {
        for (int p = 0; p < data_manager->host_data.boundary_element_fea.size(); p++) {
            int start = data_manager->host_data.c_counts_rigid_tet[p];
            int end = data_manager->host_data.c_counts_rigid_tet[p + 1];
            for (int index = start; index < end; index++) {
                int i = index - start;  // index that goes from 0
                int rigid = data_manager->host_data.neighbor_rigid_tet[p * max_rigid_neighbors + i];
                int node = p;  // node body is in second index
                real3 cpta = data_manager->host_data.cpta_rigid_tet[p * max_rigid_neighbors + i];
                real3 cptb = data_manager->host_data.cptb_rigid_tet[p * max_rigid_neighbors + i];

                contact_data[index + offset] = glm::vec3(cpta.x, cpta.y, cpta.z);
                contact_data[index + offset + data_manager->num_rigid_tet_contacts] = glm::vec3(cptb.x, cptb.y, cptb.z);
                index++;
            }
        }
    }

    offset += (data_manager->num_rigid_tet_contacts) * 2;
    index = 0;
    for (int p = 0; p < (signed)data_manager->num_fea_nodes; p++) {
        int start = data_manager->host_data.c_counts_rigid_tet_node[p];
        int end = data_manager->host_data.c_counts_rigid_tet_node[p + 1];
        for (int index = start; index < end; index++) {
            int i = index - start;  // index that goes from 0
            int rigid = data_manager->host_data.neighbor_rigid_tet_node[p * max_rigid_neighbors + i];
            int node = p;  // node body is in second index
            real3 cpta = data_manager->host_data.cpta_rigid_tet_node[p * max_rigid_neighbors + i];

            contact_data[index + offset] = glm::vec3(cpta.x, cpta.y, cpta.z);
            index++;
        }
    }

    offset += (data_manager->num_rigid_tet_node_contacts);
    index = 0;
    for (int p = 0; p < (signed)data_manager->num_fluid_bodies; p++) {
        int start = data_manager->host_data.c_counts_rigid_fluid[p];
        int end = data_manager->host_data.c_counts_rigid_fluid[p + 1];
        for (int index = start; index < end; index++) {
            int i = index - start;
            real3 cpta = data_manager->host_data.cpta_rigid_fluid[p * max_rigid_neighbors + i];
            contact_data[index + offset] = glm::vec3(cpta.x, cpta.y, cpta.z);
            index++;
        }
    }
#endif
}

void ChOpenGLContacts::Update(ChSystem* physics_system) {
    contact_data.clear();
#ifdef CHRONO_PARALLEL
    if (ChSystemParallel* system_parallel = dynamic_cast<ChSystemParallel*>(physics_system)) {
        UpdateChronoParallel(system_parallel);
    } else
#endif
    {
        UpdateChrono(physics_system);
    }

    contacts.Update(contact_data);
}

void ChOpenGLContacts::TakeDown() {
    contacts.TakeDown();
    contact_data.clear();
}

void ChOpenGLContacts::Draw(const mat4& projection, const mat4& view) {
    glm::mat4 model(1);
    contacts.Draw(projection, view * model);
}
}
}
