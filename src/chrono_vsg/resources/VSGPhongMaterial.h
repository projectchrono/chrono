

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
// Authors: Rainer Gericke
// =============================================================================
// Header for an helper class defining materials for Phong shading
// =============================================================================

#ifndef VSG_PHONG_MATERIAL_H
#define VSG_PHONG_MATERIAL_H

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

struct CH_VSG_API PhongMaterial {
    vsg::vec4 ambient{0.0f, 0.0f, 0.0f, 1.0f};
    vsg::vec4 diffuse{1.0f, 1.0f, 1.0f, 1.0f};
    vsg::vec4 specular{0.0f, 0.0f, 0.0f, 1.0f};
    vsg::vec4 emissive{0.0f, 0.0f, 0.0f, 1.0f};
    float shininess{0.0f};
    float alphaMask{1.0};
    float alphaMaskCutoff{0.5};

    vsg::ref_ptr<vsg::Data> toData() {
        auto buffer = vsg::ubyteArray::create(sizeof(PhongMaterial));
        std::memcpy(buffer->data(), &ambient[0], sizeof(PhongMaterial));
        return buffer;
    }
};

}  // namespace vsg3d
}  // namespace chrono
#endif
