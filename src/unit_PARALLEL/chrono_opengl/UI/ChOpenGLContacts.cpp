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
// Class to render contact information
// Authors: Hammad Mazhar
// =============================================================================

#include <iostream>
#include "chrono_opengl/UI/ChOpenGLContacts.h"
using namespace glm;
using namespace chrono::opengl;

ChOpenGLContacts::ChOpenGLContacts() {}
void ChOpenGLContacts::Update() {}

void ChOpenGLContacts::TakeDown() {
  contacts.TakeDown();
  contact_data.clear()
}

void ChOpenGLContacts::Draw(const mat4& projection, const mat4& view) {}