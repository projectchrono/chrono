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
namespace chrono {
using namespace collision;
namespace opengl {
using namespace glm;

ChOpenGLContacts::ChOpenGLContacts() {
}

bool ChOpenGLContacts::Initialize(ChOpenGLMaterial mat, ChOpenGLShader* shader) {
  if (this->GLReturnedError("Contacts::Initialize - on entry"))
    return false;
  contact_data.push_back(vec3(0, 0, 0));
  contacts.Initialize(contact_data, mat, shader);
  contacts.SetPointSize(0.01);
  return true;
}

void ChOpenGLContacts::UpdateChrono(ChSystem* system) {
  ChContactContainer* container = (ChContactContainer*)system->GetContactContainer();
  std::list<ChContact*> list = container->GetContactList();
  int num_contacts = container->GetNcontacts();
  int counter = 0;
  contact_data.resize(num_contacts * 2);

  for (std::list<ChContact*>::const_iterator iterator = list.begin(), end = list.end(); iterator != end; ++iterator) {
    ChVector<> p1 = (*iterator)->GetContactP1();
    ChVector<> p2 = (*iterator)->GetContactP2();

    contact_data[counter] = glm::vec3(p1.x, p1.y, p1.z);
    contact_data[counter + num_contacts] = glm::vec3(p2.x, p2.y, p2.z);
    counter++;
  }
}
void ChOpenGLContacts::UpdateChronoParallel(ChSystemParallel* system) {
  ChParallelDataManager* data_manager = system->data_manager;
  int num_contacts = data_manager->num_rigid_contacts;
  if (num_contacts == 0) {
    return;
  }

  contact_data.resize(num_contacts * 2);

#pragma omp parallel for
  for (int i = 0; i < data_manager->num_rigid_contacts; i++) {
    real3 cpta = data_manager->host_data.cpta_rigid_rigid[i];
    real3 cptb = data_manager->host_data.cptb_rigid_rigid[i];

    contact_data[i] = glm::vec3(cpta.x, cpta.y, cpta.z);
    contact_data[i + data_manager->num_rigid_contacts] = glm::vec3(cptb.x, cptb.y, cptb.z);
  }
}

void ChOpenGLContacts::Update(ChSystem* physics_system) {
  contact_data.clear();
  if (ChSystemParallel* system_parallel = dynamic_cast<ChSystemParallel*>(physics_system)) {
    UpdateChronoParallel(system_parallel);
  } else {
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
