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

#ifndef VSG_PBR_MATERIAL_H
#define VSG_PBR_MATERIAL_H

#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>

namespace chrono {
namespace vsg3d {

struct PbrMaterial {
    vsg::vec4 baseColorFactor{1.0, 1.0, 1.0, 1.0};
    vsg::vec4 emissiveFactor{0.0, 0.0, 0.0, 1.0};
    vsg::vec4 diffuseFactor{1.0, 1.0, 1.0, 1.0};
    vsg::vec4 specularFactor{0.0, 0.0, 0.0, 1.0};
    float metallicFactor{1.0f};
    float roughnessFactor{1.0f};
    float alphaMask{1.0f};
    float alphaMaskCutoff{0.5f};

    vsg::ref_ptr<vsg::Data> toData() {
        auto buffer = vsg::ubyteArray::create(sizeof(PbrMaterial));
        std::memcpy(buffer->data(), &baseColorFactor.r, sizeof(PbrMaterial));
        return buffer;
    }
};

}  // namespace vsg3d
}  // namespace chrono
#endif
