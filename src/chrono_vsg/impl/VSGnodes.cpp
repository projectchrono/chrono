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

#include <string>
#include <iostream>
#include <sstream>

#include "chrono/utils/ChConstants.h"
#include "chrono_vsg/impl/VSGnodes.h"

#include "chrono_vsg/impl/quadmapShaders.h"
#include "chrono_vsg/impl/cubemapShaders.h"
#include "chrono_vsg/impl/pcShaders.h"

#include "chrono_thirdparty/stb/stb_image_write.h"

using namespace std;

namespace chrono {
namespace vsg3d {

vsg::ref_ptr<vsg::Node> createSkysphere(const vsg::Path& filename,
                                        vsg::ref_ptr<vsg::Options> options,
                                        double azimuth_offset,
                                        bool yup) {
    auto root = vsg::StateGroup::create();

    auto textureData = vsg::read_cast<vsg::Data>(filename, options);
    if (!textureData) {
        std::cout << "Error: failed to load texture file : " << filename << std::endl;
        return {};
    }

    auto vertexShader = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", skysphere_vert);
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", skysphere_frag);
    const vsg::ShaderStages shaders{vertexShader, fragmentShader};

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection, view, and model matrices, actual push constant calls
                                              // automatically provided by the VSG's RecordTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}   // tex coord data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32_SFLOAT, 0}      // tex coord data
    };

    auto rasterState = vsg::RasterizationState::create();
    rasterState->cullMode = VK_CULL_MODE_BACK_BIT;

    auto depthState = vsg::DepthStencilState::create();
    depthState->depthTestEnable = VK_TRUE;
    depthState->depthWriteEnable = VK_FALSE;
    depthState->depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        rasterState,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        depthState};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    auto sampler = vsg::Sampler::create();
    auto texture = vsg::DescriptorImage::create(sampler, textureData, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{texture});
    auto bindDescriptorSet =
        vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, descriptorSet);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsPipeline, and binding of Descriptors
    // to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    scenegraph->add(bindGraphicsPipeline);
    scenegraph->add(bindDescriptorSet);

    // set up model transformation node
    auto transform = vsg::MatrixTransform::create();  // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    // create sphere vertices (radius = 1), texcoods, and indices
    int num_divs = 60;
    int num_theta = num_divs / 2;
    int num_phi = num_divs;

    double delta_theta = CH_PI / num_theta;
    double delta_phi = CH_2PI / num_phi;

    size_t nv = (num_phi + 1) * (num_theta + 1);
    auto vertices = vsg::vec3Array::create(nv);
    auto texcoords = vsg::vec2Array::create(nv);

    size_t nf = 2 * num_phi * num_theta;
    auto indices = vsg::ushortArray::create(3 * nf);

    int v = 0;  // current vertex counter
    for (int iPhi = 0; iPhi <= num_phi; iPhi++) {
        auto phi = iPhi * delta_phi;

        for (int iTheta = 0; iTheta <= num_theta; iTheta++) {
            auto theta = iTheta * delta_theta;

            double x = sin(theta) * cos(phi - azimuth_offset);
            double y = sin(theta) * sin(phi - azimuth_offset);
            double z = cos(theta);

            vertices->set(v, vsg::vec3(x, y, z));

            texcoords->set(v, vsg::vec2(phi / CH_2PI, theta / CH_PI));

            v++;
        }
    }

    size_t i = 0;  // current index counter
    for (int iPhi = 0; iPhi < num_phi; iPhi++) {
        for (int iTheta = 0; iTheta < num_theta; iTheta++) {
            int k1 = (num_theta + 1) * (iPhi + 0) + (iTheta + 0);
            int k2 = (num_theta + 1) * (iPhi + 0) + (iTheta + 1);
            int k3 = (num_theta + 1) * (iPhi + 1) + (iTheta + 1);
            int k4 = (num_theta + 1) * (iPhi + 1) + (iTheta + 0);

            indices->set(i + 0, k1);
            indices->set(i + 1, k3);
            indices->set(i + 2, k2);

            indices->set(i + 3, k1);
            indices->set(i + 4, k4);
            indices->set(i + 5, k3);

            i += 6;
        }
    }

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, texcoords}));
    drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::DrawIndexed::create(indices->size(), 1, 0, 0, 0));

    if (yup) {
        transform->matrix = vsg::rotate(vsg::PI * 0.5, -1.0, 0.0, 0.0);
    }
    // add drawCommands to transform
    transform->addChild(drawCommands);

    return scenegraph;
}

vsg::ref_ptr<vsg::Node> createSkybox(const vsg::Path& filename,
                                     vsg::ref_ptr<vsg::Options> options,
                                     double azimuth,
                                     bool yup) {
    auto data = vsg::read_cast<vsg::Data>(filename, options);
    if (!data) {
        std::cout << "Error: failed to load cubemap file : " << filename << std::endl;
        return {};
    }

    auto vertexShader = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", skybox_vert);
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", skybox_frag);
    const vsg::ShaderStages shaders{vertexShader, fragmentShader};

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection, view, and model matrices, actual push constant calls
                                              // automatically provided by the VSG's RecordTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}};

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}};

    auto rasterState = vsg::RasterizationState::create();
    rasterState->cullMode = VK_CULL_MODE_FRONT_BIT;

    auto depthState = vsg::DepthStencilState::create();
    depthState->depthTestEnable = VK_TRUE;
    depthState->depthWriteEnable = VK_FALSE;
    depthState->depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        rasterState,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        depthState};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto pipeline = vsg::GraphicsPipeline::create(pipelineLayout, shaders, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(pipeline);

    // create texture image and associated DescriptorSets and binding
    auto sampler = vsg::Sampler::create();

    auto texture = vsg::DescriptorImage::create(sampler, data, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{texture});
    auto bindDescriptorSet =
        vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, descriptorSet);

    auto root = vsg::StateGroup::create();
    root->add(bindGraphicsPipeline);
    root->add(bindDescriptorSet);

    auto vertices = vsg::vec3Array::create({// Back
                                            {-1.0f, -1.0f, -1.0f},
                                            {1.0f, -1.0f, -1.0f},
                                            {-1.0f, 1.0f, -1.0f},
                                            {1.0f, 1.0f, -1.0f},

                                            // Front
                                            {-1.0f, -1.0f, 1.0f},
                                            {1.0f, -1.0f, 1.0f},
                                            {-1.0f, 1.0f, 1.0f},
                                            {1.0f, 1.0f, 1.0f},

                                            // Left
                                            {-1.0f, -1.0f, -1.0f},
                                            {-1.0f, -1.0f, 1.0f},
                                            {-1.0f, 1.0f, -1.0f},
                                            {-1.0f, 1.0f, 1.0f},

                                            // Right
                                            {1.0f, -1.0f, -1.0f},
                                            {1.0f, -1.0f, 1.0f},
                                            {1.0f, 1.0f, -1.0f},
                                            {1.0f, 1.0f, 1.0f},

                                            // Bottom
                                            {-1.0f, -1.0f, -1.0f},
                                            {-1.0f, -1.0f, 1.0f},
                                            {1.0f, -1.0f, -1.0f},
                                            {1.0f, -1.0f, 1.0f},

                                            // Top
                                            {-1.0f, 1.0f, -1.0f},
                                            {-1.0f, 1.0f, 1.0f},
                                            {1.0f, 1.0f, -1.0f},
                                            {1.0f, 1.0f, 1.0}});

    auto indices = vsg::ushortArray::create({// Back
                                             0, 2, 1, 1, 2, 3,

                                             // Front
                                             6, 4, 5, 7, 6, 5,

                                             // Left
                                             10, 8, 9, 11, 10, 9,

                                             // Right
                                             14, 13, 12, 15, 13, 14,

                                             // Bottom
                                             17, 16, 19, 19, 16, 18,

                                             // Top
                                             23, 20, 21, 22, 20, 23});

    root->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices}));
    root->addChild(vsg::BindIndexBuffer::create(indices));
    root->addChild(vsg::DrawIndexed::create(indices->size(), 1, 0, 0, 0));

    auto xform = vsg::MatrixTransform::create();
    if (yup) {
        xform->matrix = vsg::rotate(-vsg::PI * 0.5, 0.0, 1.0, 0.0);
    } else {
        xform->matrix = vsg::rotate(vsg::PI * 0.5, 1.0, 0.0, 0.0);
    }
    xform->addChild(root);

    return xform;
}

}  // namespace vsg3d
}  // namespace chrono
