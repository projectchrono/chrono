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
#include "chrono_thirdparty/stb/stb_image.h"

namespace chrono {
namespace vsg3d {

vsg::ref_ptr<vsg::ShaderSet> createLineShaderSet(vsg::ref_ptr<const vsg::Options> options) {
    //vsg::info("Local LineShaderSet(", options, ")");

    auto vertexShader = vsg::read_cast<vsg::ShaderStage>("vsg/shaders/lineShader.vert", options);
    auto fragmentShader = vsg::read_cast<vsg::ShaderStage>("vsg/shaders/lineShader.frag", options);

    if (!vertexShader || !fragmentShader) {
        vsg::error("LineShaderSet(...) could not find shaders.");
        return {};
    }

    auto shaderSet = vsg::ShaderSet::create(vsg::ShaderStages{vertexShader, fragmentShader});

    shaderSet->addAttributeBinding("inPosition", "", 0, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));
    shaderSet->addAttributeBinding("inColor", "", 1, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));

    shaderSet->addPushConstantRange("pc", "", VK_SHADER_STAGE_VERTEX_BIT, 0, 128);

    return shaderSet;
}

vsg::ref_ptr<vsg::ShaderSet> createPbrShaderSet(vsg::ref_ptr<const vsg::Options> options,
                                                std::shared_ptr<ChVisualMaterial> material) {
    // vsg::info("Local pbr_ShaderSet(", options, ")");

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

    shaderSet->addUniformBinding("diffuseMap", "VSG_DIFFUSE_MAP", 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT,
                                 vsg::vec4Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32A32_SFLOAT}));

    shaderSet->addUniformBinding("mrMap", "VSG_METALLROUGHNESS_MAP", 0, 1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT,
                                 vsg::vec4Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32A32_SFLOAT}));

    shaderSet->addUniformBinding("normalMap", "VSG_NORMAL_MAP", 0, 2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT,
                                 vsg::vec3Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32_SFLOAT}));

    shaderSet->addUniformBinding("aoMap", "VSG_LIGHTMAP_MAP", 0, 3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT,
                                 vsg::vec3Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32_SFLOAT}));

    shaderSet->addUniformBinding("emissiveMap", "VSG_EMISSIVE_MAP", 0, 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT,
                                 vsg::vec4Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32A32_SFLOAT}));

    shaderSet->addUniformBinding("specularMap", "VSG_SPECULAR_MAP", 0, 5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT,
                                 vsg::vec4Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32A32_SFLOAT}));

    shaderSet->addUniformBinding("opacityMap", "VSG_OPACITY_MAP", 0, 7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT,
                                 vsg::vec4Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32G32B32A32_SFLOAT}));

    shaderSet->addUniformBinding("lightData", "", 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array::create(64));
    shaderSet->addUniformBinding("PbrData", "", 0, 10, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::PbrMaterialValue::create());
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

vsg::ref_ptr<vsg::StateGroup> createLineStateGroup(vsg::ref_ptr<const vsg::Options> options,
                                                   VkPrimitiveTopology topology) {
    vsg::ref_ptr<vsg::SharedObjects> sharedObjects;
    if (!sharedObjects) {
        if (options)
            sharedObjects = options->sharedObjects;
        else
            sharedObjects = vsg::SharedObjects::create();
    }

    vsg::ref_ptr<vsg::ShaderSet> activeShaderSet = createLineShaderSet(options);
    auto graphicsPipelineConfig = vsg::GraphicsPipelineConfigurator::create(activeShaderSet);

    auto& defines = graphicsPipelineConfig->shaderHints->defines;

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings;

    graphicsPipelineConfig->enableArray("inPosition", VK_VERTEX_INPUT_RATE_VERTEX, 12);
    graphicsPipelineConfig->enableArray("inColor", VK_VERTEX_INPUT_RATE_VERTEX, 12);
    graphicsPipelineConfig->inputAssemblyState->topology = topology;
    graphicsPipelineConfig->rasterizationState->lineWidth = 1;

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

vsg::ref_ptr<vsg::StateGroup> createPbrStateGroup(vsg::ref_ptr<const vsg::Options> options,
                                                  std::shared_ptr<ChVisualMaterial> material,
                                                  bool wireframe) {
    vsg::ref_ptr<vsg::SharedObjects> sharedObjects;

    bool use_blending = (material->GetOpacity() < 1.0) || (!material->GetOpacityTexture().empty());

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
    auto pbrMat = createPbrMaterialFromChronoMaterial(material);
    graphicsPipelineConfig->assignUniform("PbrData", pbrMat);

    graphicsPipelineConfig->enableArray("vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, 12);
    graphicsPipelineConfig->enableArray("vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, 12);
    graphicsPipelineConfig->enableArray("vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, 8);
    graphicsPipelineConfig->enableArray("vsg_Color", VK_VERTEX_INPUT_RATE_VERTEX, 16);

    if (wireframe) {
        graphicsPipelineConfig->rasterizationState->polygonMode = VK_POLYGON_MODE_LINE;
    }

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
            pbrMat->value().diffuseFactor.set(1.0, 1.0, 1.0, pbrMat->value().alphaMask);
            pbrMat->value().baseColorFactor.set(1.0, 1.0, 1.0, pbrMat->value().alphaMask);
        } else {
            GetLog() << __func__ << ": could not read diffuse texture file <" << material->GetKdTexture() << ">!\n";
        }
    }

    if (!material->GetKeTexture().empty()) {
        auto image = vsg::read_cast<vsg::Data>(material->GetKeTexture(), options);
        if (image) {
            auto sampler = vsg::Sampler::create();
            sampler->maxLod =
                static_cast<uint32_t>(std::floor(std::log2(std::max(image->width(), image->height())))) + 1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;

            if (sharedObjects)
                sharedObjects->share(sampler);

            graphicsPipelineConfig->assignTexture("emissiveMap", image, sampler);
        } else {
            GetLog() << __func__ << ": could not read emissive texture file <" << material->GetKeTexture() << ">!\n";
        }
    }

    if (!material->GetKsTexture().empty()) {
        auto image = vsg::read_cast<vsg::Data>(material->GetKsTexture(), options);
        if (image) {
            auto sampler = vsg::Sampler::create();
            sampler->maxLod =
                static_cast<uint32_t>(std::floor(std::log2(std::max(image->width(), image->height())))) + 1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;

            if (sharedObjects)
                sharedObjects->share(sampler);

            graphicsPipelineConfig->assignTexture("specularMap", image, sampler);
        } else {
            GetLog() << __func__ << ": could not read specular texture file <" << material->GetKsTexture() << ">!\n";
        }
    }

    if (!material->GetOpacityTexture().empty()) {
        auto image = vsg::read_cast<vsg::Data>(material->GetOpacityTexture(), options);
        if (image) {
            auto sampler = vsg::Sampler::create();
            sampler->maxLod =
                static_cast<uint32_t>(std::floor(std::log2(std::max(image->width(), image->height())))) + 1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;

            if (sharedObjects)
                sharedObjects->share(sampler);

            graphicsPipelineConfig->assignTexture("opacityMap", image, sampler);
        } else {
            GetLog() << __func__ << ": could not read opacity texture file <" << material->GetOpacityTexture()
                     << ">!\n";
        }
    }

    if (!material->GetNormalMapTexture().empty()) {
        auto image = vsg::read_cast<vsg::Data>(material->GetNormalMapTexture(), options);
        if (image) {
            auto sampler = vsg::Sampler::create();
            sampler->maxLod =
                static_cast<uint32_t>(std::floor(std::log2(std::max(image->width(), image->height())))) + 1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;

            if (sharedObjects)
                sharedObjects->share(sampler);

            graphicsPipelineConfig->assignTexture("normalMap", image, sampler);
        } else {
            GetLog() << __func__ << ": could not read normal map file <" << material->GetNormalMapTexture() << ">!\n";
        }
    }

    if (!material->GetAmbientOcclusionTexture().empty()) {
        auto image = vsg::read_cast<vsg::Data>(material->GetAmbientOcclusionTexture(), options);
        if (image) {
            auto sampler = vsg::Sampler::create();
            sampler->maxLod =
                static_cast<uint32_t>(std::floor(std::log2(std::max(image->width(), image->height())))) + 1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;

            if (sharedObjects)
                sharedObjects->share(sampler);

            graphicsPipelineConfig->assignTexture("aoMap", image, sampler);
        } else {
            GetLog() << __func__ << ": could not read ambient occlusion map file <"
                     << material->GetAmbientOcclusionTexture() << ">!\n";
        }
    }

    if (!material->GetMetallicTexture().empty() && !material->GetRoughnessTexture().empty()) {
        int wM, hM, nM;
        unsigned char* metalData = stbi_load(material->GetMetallicTexture().c_str(), &wM, &hM, &nM, 1);
        int wR, hR, nR;
        unsigned char* roughData = stbi_load(material->GetRoughnessTexture().c_str(), &wR, &hR, &nR, 1);
        if (metalData && roughData) {
            if ((wM != wR) || (hM != hR)) {
                vsg::error("Metalness and Roughness Textures must have the same size!");
                return {};
            }
        }

        auto texData = vsg::vec3Array2D::create(wR, hR, vsg::Data::Layout{VK_FORMAT_R32G32B32_SFLOAT});
        if (!texData) {
            vsg::error("Could not create texture data!");
            return {};
        }
        int k = 0;
        for (int j = 0; j < hR; j++) {
            for (int i = 0; i < wR; i++) {
                vsg::vec3 color(0.0f, 0.0f, 0.0f);
                float red = 0.0f;
                float green = 0.0f;
                float blue = 0.0f;
                if (roughData) {
                    green = float(roughData[k]) / 255.0f;
                }
                if (metalData) {
                    blue = float(metalData[k]) / 255.0f;
                }
                texData->set(i, j, vsg::vec3(0.0f, green, blue));
                k++;
            }
        }
        if (texData) {
            auto sampler = vsg::Sampler::create();
            sampler->maxLod =
                static_cast<uint32_t>(std::floor(std::log2(std::max(texData->width(), texData->height())))) + 1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;

            if (sharedObjects)
                sharedObjects->share(sampler);

            graphicsPipelineConfig->assignTexture("mrMap", texData, sampler);
        }
    }

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
    float dim = 1.0f;
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

vsg::ref_ptr<vsg::PhongMaterialValue> createPhongMaterialFromChronoMaterial(
    std::shared_ptr<chrono::ChVisualMaterial> chronoMat) {
    auto phongMat = vsg::PhongMaterialValue::create();
    float alpha = chronoMat->GetOpacity();

    phongMat->value().emissive.set(chronoMat->GetEmissiveColor().R, chronoMat->GetEmissiveColor().G,
                                   chronoMat->GetEmissiveColor().B, alpha);
    phongMat->value().specular.set(chronoMat->GetSpecularColor().R, chronoMat->GetSpecularColor().G,
                                   chronoMat->GetSpecularColor().B, alpha);
    phongMat->value().diffuse.set(chronoMat->GetDiffuseColor().R, chronoMat->GetDiffuseColor().G,
                                  chronoMat->GetDiffuseColor().B, alpha);
    phongMat->value().alphaMask = alpha;
    phongMat->value().alphaMaskCutoff = 0.3f;
    phongMat->value().ambient.set(chronoMat->GetAmbientColor().R, chronoMat->GetAmbientColor().G,
                                  chronoMat->GetAmbientColor().B, alpha);
    return phongMat;
}

}  // namespace vsg3d
}  // namespace chrono
