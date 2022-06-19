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
#include "GetBoxShapeData.h"
#include "GetSphereShapeData.h"
#include "GetCylinderShapeData.h"
#include "GetCapsuleShapeData.h"

namespace chrono {
namespace vsg3d {
void ShapeBuilder::assignCompileTraversal(vsg::ref_ptr<vsg::CompileTraversal> ct) {
    compileTraversal = ct;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createShape(BasicShape theShape,
                                                   std::shared_ptr<ChPhysicsItem> physItem,
                                                   ChVisualModel::ShapeInstance shapeInstance,
                                                   std::shared_ptr<ChVisualMaterial> material,
                                                   vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                   bool drawMode) {
    auto scenegraph = vsg::Group::create();
    // store some information for easier update
    scenegraph->setValue("ItemPtr", physItem);
    scenegraph->setValue("ShapeInstancePtr", shapeInstance);
    scenegraph->setValue("TransformPtr", transform);

    auto shaderSet = vsg::createPhongShaderSet(m_options);
    if (drawMode) {
        auto rasterizationState = vsg::RasterizationState::create();
        rasterizationState->polygonMode = VK_POLYGON_MODE_LINE;
        shaderSet->defaultGraphicsPipelineStates.push_back(rasterizationState);
    }

    auto graphicsPipelineConfig = vsg::GraphicsPipelineConfig::create(shaderSet);

    // set up graphics pipeline
    vsg::Descriptors descriptors;

    // set up pass of material
    auto phongMat = vsg::PhongMaterialValue::create();
    float alpha = material->GetOpacity();
    phongMat->value().diffuse.set(material->GetDiffuseColor().R, material->GetDiffuseColor().G,
                                  material->GetDiffuseColor().B, alpha);
    phongMat->value().ambient.set(material->GetAmbientColor().R, material->GetAmbientColor().G,
                                  material->GetAmbientColor().B, alpha);
    phongMat->value().specular.set(material->GetSpecularColor().R, material->GetSpecularColor().G,
                                   material->GetSpecularColor().B, alpha);  // red specular highlight
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

    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::vec3Array> normals;
    vsg::ref_ptr<vsg::vec2Array> texcoords;
    vsg::ref_ptr<vsg::ushortArray> indices;
    float boundingSphereRadius;
    switch(theShape) {
        case BOX_SHAPE:
            GetBoxShapeData(vertices, normals, texcoords, indices, boundingSphereRadius);
            break;
        case SPHERE_SHAPE:
            GetSphereShapeData(vertices, normals, texcoords, indices, boundingSphereRadius);
            break;
        case CYLINDER_SHAPE:
            GetCylinderShapeData(vertices, normals, texcoords, indices, boundingSphereRadius);
            break;
        case CAPSULE_SHAPE:
            GetCapsuleShapeData(vertices, normals, texcoords, indices, boundingSphereRadius);
            break;
    }
    auto colors = vsg::vec4Value::create(vsg::vec4{1.0f, 1.0f, 1.0f, 1.0f});

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
    transform->subgraphRequiresLocalFrustum = false;

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
        depthSorted->bound.set(0.0f, 0.0f, 0.0f, boundingSphereRadius);
        depthSorted->child = stateGroup;
        transform->addChild(depthSorted);
    } else {
        transform->addChild(stateGroup);
    }

    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(transform);
    }

    scenegraph->addChild(transform);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);

    return scenegraph;
}

}  // namespace vsg3d
}  // namespace chrono
