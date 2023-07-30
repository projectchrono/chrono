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

#include "chrono_vsg/shapes/ShaderUtils.h"

namespace chrono {
namespace vsg3d {

vsg::ref_ptr<vsg::ShaderSet> createPbrShaderSet(vsg::ref_ptr<const vsg::Options> options,
                                                std::shared_ptr<ChVisualMaterial> material) {
    vsg::info("Local pbr_ShaderSet(", options, ")");

    auto vertexShader = vsg::read_cast<vsg::ShaderStage>("vsg/shaders/_tmp_.vert", options);
    auto fragmentShader = vsg::read_cast<vsg::ShaderStage>("vsg/shaders/_tmp__pbr.frag", options);

    if (!vertexShader || !fragmentShader) {
        vsg::error("pbr_ShaderSet(...) could not find shaders.");
        return {};
    }

    auto shaderSet = vsg::ShaderSet::create(vsg::ShaderStages{vertexShader, fragmentShader});

    // mandatory shader data
    shaderSet->addAttributeBinding("vsg_Vertex", "", 0, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));
    shaderSet->addAttributeBinding("vsg_Normal", "", 1, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));
    shaderSet->addAttributeBinding("vsg_TexCoord0", "", 2, VK_FORMAT_R32G32_SFLOAT, vsg::vec2Array::create(1));
    shaderSet->addAttributeBinding("vsg_Color", "", 3, VK_FORMAT_R32G32B32A32_SFLOAT, vsg::vec4Array::create(1));

    if (!material->GetKdTexture().empty()) {
        shaderSet->addUniformBinding(
            "diffuseMap", "VSG_DIFFUSE_MAP", 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            vsg::vec4Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32A32_SFLOAT}));
    }

    /* optional shader data

        shaderSet->addAttributeBinding("vsg_position", "VSG_INSTANCE_POSITIONS", 4, VK_FORMAT_R32G32B32_SFLOAT,
                                       vsg::vec3Array::create(1));
        shaderSet->addAttributeBinding("vsg_position_scaleDistance", "VSG_BILLBOARD", 4, VK_FORMAT_R32G32B32A32_SFLOAT,
                                       vsg::vec4Array::create(1));

        shaderSet->addUniformBinding("displacementMap", "VSG_DISPLACEMENT_MAP", 0, 6,
                                     VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT,
                                     vsg::floatArray2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT}));
        shaderSet->addUniformBinding("diffuseMap", "VSG_DIFFUSE_MAP", 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
       1, VK_SHADER_STAGE_FRAGMENT_BIT, vsg::ubvec4Array2D::create(1, 1,
       vsg::Data::Properties{VK_FORMAT_R8G8B8A8_UNORM})); shaderSet->addUniformBinding("mrMap",
       "VSG_METALLROUGHNESS_MAP", 0, 1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
                                     vsg::vec2Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32_SFLOAT}));
        shaderSet->addUniformBinding("normalMap", "VSG_NORMAL_MAP", 0, 2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                     VK_SHADER_STAGE_FRAGMENT_BIT,
                                     vsg::vec3Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32_SFLOAT}));
        shaderSet->addUniformBinding("aoMap", "VSG_LIGHTMAP_MAP", 0, 3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                     VK_SHADER_STAGE_FRAGMENT_BIT,
                                     vsg::floatArray2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT}));
        shaderSet->addUniformBinding("emissiveMap", "VSG_EMISSIVE_MAP", 0, 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
       1, VK_SHADER_STAGE_FRAGMENT_BIT, vsg::ubvec4Array2D::create(1, 1,
       vsg::Data::Properties{VK_FORMAT_R8G8B8A8_UNORM})); shaderSet->addUniformBinding("specularMap",
       "VSG_SPECULAR_MAP", 0, 5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
                                     vsg::ubvec4Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R8G8B8A8_UNORM}));
        shaderSet->addUniformBinding("lightData", "", 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                     VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array::create(64));
    */
    // always needed
    shaderSet->addUniformBinding("pbr", "", 0, 10, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
                                 vsg::PbrMaterialValue::create());
    shaderSet->addPushConstantRange("pc", "", VK_SHADER_STAGE_VERTEX_BIT, 0, 128);

    // additional defines
    shaderSet->optionalDefines = {"VSG_GREYSACLE_DIFFUSE_MAP", "VSG_TWO_SIDED_LIGHTING", "VSG_WORKFLOW_SPECGLOSS"};

    shaderSet->definesArrayStates.push_back(vsg::DefinesArrayState{
        {"VSG_INSTANCE_POSITIONS", "VSG_DISPLACEMENT_MAP"}, vsg::PositionAndDisplacementMapArrayState::create()});
    shaderSet->definesArrayStates.push_back(
        vsg::DefinesArrayState{{"VSG_INSTANCE_POSITIONS"}, vsg::PositionArrayState::create()});
    shaderSet->definesArrayStates.push_back(
        vsg::DefinesArrayState{{"VSG_DISPLACEMENT_MAP"}, vsg::DisplacementMapArrayState::create()});
    shaderSet->definesArrayStates.push_back(
        vsg::DefinesArrayState{{"VSG_BILLBOARD"}, vsg::BillboardArrayState::create()});

    shaderSet->customDescriptorSetBindings.push_back(vsg::ViewDependentStateBinding::create(1));

    return shaderSet;
}

vsg::ref_ptr<vsg::StateGroup> createPbrStateGroup(vsg::ref_ptr<const vsg::Options> options,
                                                  std::shared_ptr<ChVisualMaterial> material) {
    vsg::ref_ptr<vsg::SharedObjects> sharedObjects;

    if (!sharedObjects) {
        if (options)
            sharedObjects = options->sharedObjects;
        else
            sharedObjects = vsg::SharedObjects::create();
    }

    vsg::ref_ptr<vsg::ShaderSet> activeShaderSet = createPbrShaderSet(options, material);

    auto graphicsPipelineConfig = vsg::GraphicsPipelineConfigurator::create(activeShaderSet);

    auto& defines = graphicsPipelineConfig->shaderHints->defines;

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings;

    auto theScale = material->GetTextureScale();

    graphicsPipelineConfig->assignUniform("pbr", createPbrMaterialFromChronoMaterial(material));

    graphicsPipelineConfig->enableArray("vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, 12);
    graphicsPipelineConfig->enableArray("vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, 12);
    graphicsPipelineConfig->enableArray("vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, 8);
    graphicsPipelineConfig->enableArray("vsg_Color", VK_VERTEX_INPUT_RATE_INSTANCE, 16);

    if (!material->GetKdTexture().empty()) {
        auto image = vsg::read_cast<vsg::Data>(material->GetKdTexture(), options);
        if (image) {
            auto sampler = vsg::Sampler::create();
            sampler->maxLod =
                static_cast<uint32_t>(std::floor(std::log2(std::max(image->width(), image->height())))) + 1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;

            if (sharedObjects)
                sharedObjects->share(sampler);

            graphicsPipelineConfig->assignTexture("diffuseMap", image, sampler);
        } else {
            GetLog() << __func__ << ": could not read diffuse texture file <" << material->GetKdTexture() << ">!\n";
        }
    }

    bool use_blending = material->GetOpacity() < 1.0;
    graphicsPipelineConfig->colorBlendState->attachments = vsg::ColorBlendState::ColorBlendAttachments{
        {use_blending, VK_BLEND_FACTOR_SRC_ALPHA, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
         VK_BLEND_FACTOR_SRC_ALPHA, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_SUBTRACT,
         VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};

    // if required initialize GraphicsPipeline/Layout etc.
    if (sharedObjects)
        sharedObjects->share(graphicsPipelineConfig, [](auto gpc) { gpc->init(); });
    else
        graphicsPipelineConfig->init();

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    auto stateGroup = vsg::StateGroup::create();
    graphicsPipelineConfig->copyTo(stateGroup, sharedObjects);

    return stateGroup;
}

vsg::ref_ptr<vsg::PbrMaterialValue> createPbrMaterialFromChronoMaterial(std::shared_ptr<ChVisualMaterial> chronoMat) {
    auto pbrMat = vsg::PbrMaterialValue::create();
    float alpha = chronoMat->GetOpacity();
    float dim = 0.5f;
    pbrMat->value().baseColorFactor.set(dim * chronoMat->GetDiffuseColor().R, dim * chronoMat->GetDiffuseColor().G,
                                        dim * chronoMat->GetDiffuseColor().B, alpha);
    pbrMat->value().emissiveFactor.set(chronoMat->GetEmissiveColor().R, chronoMat->GetEmissiveColor().G,
                                       chronoMat->GetEmissiveColor().B, alpha);
    pbrMat->value().specularFactor.set(chronoMat->GetSpecularColor().R, chronoMat->GetSpecularColor().G,
                                       chronoMat->GetSpecularColor().B, alpha);
    pbrMat->value().roughnessFactor = chronoMat->GetRoughness();
    pbrMat->value().metallicFactor = chronoMat->GetMetallic();
    pbrMat->value().diffuseFactor.set(chronoMat->GetDiffuseColor().R, chronoMat->GetDiffuseColor().G,
                                      chronoMat->GetDiffuseColor().B, alpha);
    pbrMat->value().alphaMask = alpha;
    pbrMat->value().alphaMaskCutoff = 0.3f;

    return pbrMat;
}

}  // namespace vsg3d
}  // namespace chrono
