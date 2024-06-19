// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Rainer Gericke
// =============================================================================

#ifndef CH_SHADER_UTILS_H
#define CH_SHADER_UTILS_H

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono/assets/ChVisualModel.h"

namespace chrono {
namespace vsg3d {

vsg::ref_ptr<vsg::ShaderSet> createLineShaderSet(vsg::ref_ptr<const vsg::Options> options);

vsg::ref_ptr<vsg::ShaderSet> createPbrShaderSet(vsg::ref_ptr<const vsg::Options> options,
                                                std::shared_ptr<ChVisualMaterial> material);

vsg::ref_ptr<vsg::StateGroup> createLineStateGroup(vsg::ref_ptr<const vsg::Options> options,
                                                   VkPrimitiveTopology topology);

vsg::ref_ptr<vsg::StateGroup> createPbrStateGroup(vsg::ref_ptr<const vsg::Options> options,
                                                  std::shared_ptr<ChVisualMaterial> material,
                                                  bool wireframe);

vsg::ref_ptr<vsg::PbrMaterialValue> createPbrMaterialFromChronoMaterial(std::shared_ptr<ChVisualMaterial> chronoMat);
vsg::ref_ptr<vsg::PhongMaterialValue> createPhongMaterialFromChronoMaterial(
    std::shared_ptr<ChVisualMaterial> chronoMat);

}  // namespace vsg3d
}  // namespace chrono

#endif
