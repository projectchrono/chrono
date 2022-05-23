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
// Radu Serban, Rainer Gericke
// =============================================================================

#include "ShapeBuilder.h"

namespace chrono {
namespace vsg3d {
ShapeBuilder::ShapeBuilder(vsg::ref_ptr<vsg::Options> options) {
    m_options = options;
}

ShapeBuilder::~ShapeBuilder() {}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createBox(std::shared_ptr<ChVisualMaterial> material, vsg::dmat4 &tf_matrix) {
    auto scenegraph = vsg::Group::create();
    auto shaderSet = vsg::createPhongShaderSet(m_options);

    auto graphicsPipelineConfig = vsg::GraphicsPipelineConfig::create(shaderSet);

    // set up graphics pipeline
    vsg::Descriptors descriptors;

    // set up pass of material
    auto phongMat = vsg::PhongMaterialValue::create();
    float alpha = material->GetOpacity();
    phongMat->value().diffuse.set(material->GetDiffuseColor().R, material->GetDiffuseColor().G, material->GetDiffuseColor().B, alpha);
    phongMat->value().ambient.set(material->GetAmbientColor().R, material->GetAmbientColor().G, material->GetAmbientColor().B, alpha);
    phongMat->value().specular.set(material->GetSpecularColor().R, material->GetSpecularColor().G, material->GetSpecularColor().B, alpha);  // red specular highlight
    phongMat->value().alphaMask = alpha;
    phongMat->value().alphaMaskCutoff = 0.3f;

    // read texture image
    vsg::Path textureFile(material->GetKdTexture());
    if (textureFile) {
        auto textureData = vsg::read_cast<vsg::Data>(textureFile, m_options);
        if (!textureData) {
            std::cout << "Could not read texture file : " << textureFile << std::endl;
        }
        // enable texturing
        graphicsPipelineConfig->assignTexture(descriptors, "diffuseMap", textureData);
    }

    // set transparency, if needed
    vsg::ColorBlendState::ColorBlendAttachments colorBlendAttachments;
    VkPipelineColorBlendAttachmentState colorBlendAttachment = {};
    colorBlendAttachment.blendEnable = VK_FALSE;  // default
    colorBlendAttachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    if (phongMat->value().alphaMask < 1.0) {
        colorBlendAttachment.blendEnable = VK_TRUE;
        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
    }
    colorBlendAttachments.push_back(colorBlendAttachment);
    graphicsPipelineConfig->colorBlendState = vsg::ColorBlendState::create(colorBlendAttachments);
    graphicsPipelineConfig->assignUniform(descriptors, "material", phongMat);

    if (m_options->sharedObjects)
        m_options->sharedObjects->share(descriptors);

    // set up vertex and index arrays
    const float a = 0.5;
    /* auto vertices = vsg::vec3Array::create(
            {{-0.5f, -0.5f, 0.0f},
                    {0.5f, -0.5f, 0.0f},
                    {0.5f, 0.5f, 0.0f},
                    {-0.5f, 0.5f, 0.0f},
                    {-0.5f, -0.5f, -0.5f},
                    {0.5f, -0.5f, -0.5f},
                    {0.5f, 0.5f, -0.5f},
                    {-0.5f, 0.5f, -0.5f}});
*/
    auto vertices = vsg::vec3Array::create(
        {{-a, -a, -a}, {a, -a, -a},  {a, -a, a},  {-a, -a, a}, {a, a, -a},  {-a, a, -a}, {-a, a, a}, {a, a, a},
         {-a, a, -a},  {-a, -a, -a}, {-a, -a, a}, {-a, a, a},  {a, -a, -a}, {a, a, -a},  {a, a, a},  {a, -a, a},
         {a, -a, -a},  {-a, -a, -a}, {-a, a, -a}, {a, a, -a},  {-a, -a, a}, {a, -a, a},  {a, a, a},  {-a, a, a}});
    /*
    auto normals = vsg::vec3Array::create({{0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 1.0f}});
    */
    auto normals = vsg::vec3Array::create({{0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, 1, 0},  {0, 1, 0},
                                           {0, 1, 0},  {0, 1, 0},  {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0},
                                           {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {0, 0, -1}, {0, 0, -1},
                                           {0, 0, -1}, {0, 0, -1}, {0, 0, 1},  {0, 0, 1},  {0, 0, 1},  {0, 0, 1}});
    /*
    auto texcoords = vsg::vec2Array::create({{0.0f, 0.0f},
                                             {1.0f, 0.0f},
                                             {1.0f, 1.0f},
                                             {0.0f, 1.0f},
                                             {0.0f, 0.0f},
                                             {1.0f, 0.0f},
                                             {1.0f, 1.0f},
                                             {0.0f, 1.0f}});
    */
    auto texcoords = vsg::vec2Array::create(
        {{0.25, 0},     {0.5, 0},       {0.5, 0.3333},  {0.25, 0.3333}, {0.25, 0.6666}, {0.5, 0.6666},
         {0.5, 1.0},    {0.25, 1.0},    {0.0, 0.3333},  {0.25, 0.3333}, {0.25, 0.6666}, {0.0, 0.6666},
         {0.5, 0.3333}, {0.75, 0.3333}, {0.75, 0.6666}, {0.5, 0.6666},  {0.25, 0.3333}, {0.5, 0.3333},
         {0.5, 0.6666}, {0.25, 0.6666}, {0.75, 0.3333}, {1.0, 0.3333},  {1.0, 0.6666},  {0.75, 0.6666}});

    auto colors = vsg::vec4Value::create(vsg::vec4{1.0f, 1.0f, 1.0f, 1.0f});

    auto indices = vsg::ushortArray::create({0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
                                             12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23});

    vsg::DataList vertexArrays;

    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, vertices);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, normals);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, texcoords);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Color", VK_VERTEX_INPUT_RATE_INSTANCE, colors);

    if (m_options->sharedObjects)
        m_options->sharedObjects->share(vertexArrays);
    if (m_options->sharedObjects)
        m_options->sharedObjects->share(indices);

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(graphicsPipelineConfig->baseAttributeBinding, vertexArrays));
    drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::DrawIndexed::create(indices->size(), 1, 0, 0, 0));

    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(drawCommands->children);
        m_options->sharedObjects->share(drawCommands);
    }

    // register the ViewDescriptorSetLayout.
    vsg::ref_ptr<vsg::ViewDescriptorSetLayout> vdsl;
    if (m_options->sharedObjects)
        vdsl = m_options->sharedObjects->shared_default<vsg::ViewDescriptorSetLayout>();
    else
        vdsl = vsg::ViewDescriptorSetLayout::create();
    graphicsPipelineConfig->additionalDescrptorSetLayout = vdsl;

    // share the pipeline config and initialize if it's unique
    if (m_options->sharedObjects)
        m_options->sharedObjects->share(graphicsPipelineConfig, [](auto gpc) { gpc->init(); });
    else
        graphicsPipelineConfig->init();

    auto descriptorSet = vsg::DescriptorSet::create(graphicsPipelineConfig->descriptorSetLayout, descriptors);
    if (m_options->sharedObjects)
        m_options->sharedObjects->share(descriptorSet);

    auto bindDescriptorSet = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                            graphicsPipelineConfig->layout, 0, descriptorSet);
    if (m_options->sharedObjects)
        m_options->sharedObjects->share(bindDescriptorSet);

    auto bindViewDescriptorSets =
        vsg::BindViewDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipelineConfig->layout, 1);
    if (m_options->sharedObjects)
        m_options->sharedObjects->share(bindViewDescriptorSets);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    auto stateGroup = vsg::StateGroup::create();
    stateGroup->add(graphicsPipelineConfig->bindGraphicsPipeline);
    stateGroup->add(bindDescriptorSet);
    stateGroup->add(bindViewDescriptorSets);

    // set up model transformation node
    auto transform = vsg::MatrixTransform::create();
    transform->subgraphRequiresLocalFrustum = false;
    transform->matrix = tf_matrix;

    // add drawCommands to StateGroup
    stateGroup->addChild(drawCommands);
    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(stateGroup);
    }
    if (phongMat->value().alphaMask < 1.0) {
        vsg::ComputeBounds computeBounds;
        scenegraph->accept(computeBounds);
        auto depthSorted = vsg::DepthSorted::create();
        depthSorted->binNumber = 1;
        depthSorted->bound.set(0.0f, 0.0f, 0.0f, a);
        depthSorted->child = stateGroup;
        transform->addChild(depthSorted);
    } else {
        transform->addChild(stateGroup);
    }

    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(transform);
    }

    scenegraph->addChild(transform);

    return scenegraph;
}
}  // namespace vsg3d
}  // namespace chrono