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

#include "chrono_vsg/shapes/ShapeBuilder.h"
#include "chrono_vsg/shapes/ShaderUtils.h"
#include "chrono_vsg/shapes/GetBoxShapeData.h"
#include "chrono_vsg/shapes/GetDieShapeData.h"
#include "chrono_vsg/shapes/GetSphereShapeData.h"
#include "chrono_vsg/shapes/GetCylinderShapeData.h"
#include "chrono_vsg/shapes/GetCapsuleShapeData.h"
#include "chrono_vsg/shapes/GetConeShapeData.h"
#include "chrono_vsg/shapes/GetSurfaceShapeData.h"

#include "chrono_vsg/utils/ChConversionsVSG.h"

#include "chrono_vsg/resources/lineShader_vert.h"
#include "chrono_vsg/resources/lineShader_frag.h"

#include "chrono_thirdparty/stb/stb_image.h"

namespace chrono {
namespace vsg3d {

void ShapeBuilder::assignCompileTraversal(vsg::ref_ptr<vsg::CompileTraversal> ct) {
    compileTraversal = ct;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createPhongShape(BasicShape theShape,
                                                        std::shared_ptr<ChVisualMaterial> material,
                                                        vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                        bool wireframe,
                                                        std::shared_ptr<ChSurfaceShape> surface) {
    auto scenegraph = vsg::Group::create();

    if (compileTraversal)
        compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createPbrShape(BasicShape theShape,
                                                      std::shared_ptr<ChVisualMaterial> material,
                                                      vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      bool wireframe,
                                                      std::shared_ptr<ChSurfaceShape> surface) {
    auto scenegraph = vsg::Group::create();
    auto subgraph = createPbrStateGroup(m_options, material);
    transform->subgraphRequiresLocalFrustum = false;

    uint32_t instanceCount = 1;

    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::vec3Array> normals;
    vsg::ref_ptr<vsg::vec2Array> texcoords;
    vsg::ref_ptr<vsg::ushortArray> indices;
    vsg::ref_ptr<vsg::vec4Value> colors;

    switch (theShape) {
        case BOX_SHAPE:
            GetBoxShapeData(vertices, normals, texcoords, indices);
            break;
        case DIE_SHAPE:
            GetDieShapeData(vertices, normals, texcoords, indices);
            break;
        case SPHERE_SHAPE:
            GetSphereShapeData(vertices, normals, texcoords, indices);
            break;
        case CYLINDER_SHAPE:
            GetCylinderShapeData(vertices, normals, texcoords, indices);
            break;
        case CAPSULE_SHAPE:
            GetCapsuleShapeData(vertices, normals, texcoords, indices);
            break;
        case CONE_SHAPE:
            GetConeShapeData(vertices, normals, texcoords, indices);
            break;
        case SURFACE_SHAPE:
            GetSurfaceShapeData(surface, vertices, normals, texcoords, indices);
            break;
    }
    // apply texture scaling
    for (size_t i = 0; i < texcoords->size(); i++) {
        vsg::vec2 tx = texcoords->at(i);
        tx = vsg::vec2(tx.x * material->GetTextureScale().x(), tx.y * material->GetTextureScale().y());
        texcoords->set(i, tx);
    }

    // auto colors = vsg::vec4Value::create(vsg::vec4(1, 0, 1, 1)); instance color, good here for small objects
    // auto colors = vsg::vec4Array::create(vertices->size(),vsg::vec4(1,1,1,1)); vertex color, needed for false color
    // display
    // set color value from material information
    colors = vsg::vec4Value::create(vsg::vec4(material->GetDiffuseColor().R, material->GetDiffuseColor().G,
                                              material->GetDiffuseColor().B, material->GetOpacity()));

    // setup geometry
    auto vid = vsg::VertexIndexDraw::create();

    vsg::DataList arrays;
    arrays.push_back(vertices);
    if (normals)
        arrays.push_back(normals);
    if (texcoords)
        arrays.push_back(texcoords);
    if (colors)
        arrays.push_back(colors);
    vid->assignArrays(arrays);

    vid->assignIndices(indices);
    vid->indexCount = static_cast<uint32_t>(indices->size());
    vid->instanceCount = instanceCount;

    subgraph->addChild(vid);
    transform->addChild(subgraph);
    scenegraph->addChild(transform);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createTrimeshColShape(std::shared_ptr<ChTriangleMeshShape> tms,
                                                             vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                             bool wireframe) {
    auto scenegraph = vsg::Group::create();

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createTrimeshColAvgShape(std::shared_ptr<ChTriangleMeshShape> tms,
                                                                vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                                bool wireframe) {
    auto scenegraph = vsg::Group::create();

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createTrimeshPhongMatShape(std::shared_ptr<ChTriangleMeshShape> tms,
                                                                  vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                                  bool wireframe) {
    auto scenegraph = vsg::Group::create();

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createTrimeshPbrMatShape(std::shared_ptr<ChTriangleMeshShape> tms,
                                                                vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                                bool wireframe) {
    auto scenegraph = vsg::Group::create();

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createFrameSymbol(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                         float color_factor) {
    // Set red, green, and blue colors at specified darkness level
    ChColor R(1, 0, 0);
    ChColor G(0, 1, 0);
    ChColor B(0, 0, 1);

    auto hsvR = ChColor::RGB2HSV(R);
    hsvR[2] *= color_factor;
    R = ChColor::HSV2RGB(hsvR);

    auto hsvG = ChColor::RGB2HSV(G);
    hsvG[2] *= color_factor;
    G = ChColor::HSV2RGB(hsvG);

    auto hsvB = ChColor::RGB2HSV(B);
    hsvB[2] *= color_factor;
    B = ChColor::HSV2RGB(hsvB);

    //
    auto scenegraph = vsg::Group::create();

    vsg::ref_ptr<vsg::ShaderStage> vertexShader = lineShader_vert();
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = lineShader_frag();

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}   // colour data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}   // colour data
    };

    vsg::ref_ptr<vsg::InputAssemblyState> iaState = vsg::InputAssemblyState::create();
    iaState->topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

    vsg::ref_ptr<vsg::RasterizationState> raState = vsg::RasterizationState::create();
    raState->lineWidth = 1.0;  // only allowed value (also set as standard)

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        iaState,
        raState,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    scenegraph->addChild(bindGraphicsPipeline);

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    // calculate vertices
    const int numPoints = 6;
    auto vertices = vsg::vec3Array::create(numPoints);
    auto colors = vsg::vec3Array::create(numPoints);

    vertices->set(0, vsg::vec3(0, 0, 0));
    vertices->set(1, vsg::vec3(1, 0, 0));
    vertices->set(2, vsg::vec3(0, 0, 0));
    vertices->set(3, vsg::vec3(0, 1, 0));
    vertices->set(4, vsg::vec3(0, 0, 0));
    vertices->set(5, vsg::vec3(0, 0, 1));

    colors->set(0, vsg::vec3CH(R));
    colors->set(1, vsg::vec3CH(R));
    colors->set(2, vsg::vec3CH(G));
    colors->set(3, vsg::vec3CH(G));
    colors->set(4, vsg::vec3CH(B));
    colors->set(5, vsg::vec3CH(B));

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, colors}));
    drawCommands->addChild(vsg::Draw::create(vertices->size(), 1, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createLineShape(ChVisualModel::ShapeInstance shapeInstance,
                                                       std::shared_ptr<ChVisualMaterial> material,
                                                       vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                       std::shared_ptr<ChLineShape> ls) {
    auto scenegraph = vsg::Group::create();

    vsg::ref_ptr<vsg::ShaderStage> vertexShader = lineShader_vert();
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = lineShader_frag();

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}   // colour data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}   // colour data
    };

    vsg::ref_ptr<vsg::InputAssemblyState> iaState = vsg::InputAssemblyState::create();
    iaState->topology = VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;

    vsg::ref_ptr<vsg::RasterizationState> raState = vsg::RasterizationState::create();
    raState->lineWidth = 1.0;  // only allowed value (also set as standard)

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        iaState,
        raState,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    scenegraph->addChild(bindGraphicsPipeline);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create(); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    // calculate vertices
    int numPoints = ls->GetNumRenderPoints();
    double maxU = 1;
    if (auto mline_path = std::dynamic_pointer_cast<geometry::ChLinePath>(ls->GetLineGeometry()))
        maxU = mline_path->GetPathDuration();
    assert(numPoints > 2);
    // double ustep = 1.0 / double(numPoints - 1);
    auto vertices = vsg::vec3Array::create(numPoints);
    auto colors = vsg::vec3Array::create(numPoints);
    for (int i = 0; i < numPoints; i++) {
        // double u = ustep * (double(i));
        double u = maxU * ((double)i / (double)(numPoints - 1));  // abscissa
        ChVector<> pos;
        ls->GetLineGeometry()->Evaluate(pos, u);
        vertices->set(i, vsg::vec3CH(pos));
        auto cv =
            vsg::vec3(material->GetDiffuseColor().R, material->GetDiffuseColor().G, material->GetDiffuseColor().B);
        colors->set(i, cv);
    }
    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, colors}));
    drawCommands->addChild(vsg::Draw::create(vertices->size(), 1, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createPathShape(ChVisualModel::ShapeInstance shapeInstance,
                                                       std::shared_ptr<ChVisualMaterial> material,
                                                       vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                       std::shared_ptr<ChPathShape> ps) {
    auto scenegraph = vsg::Group::create();

    vsg::ref_ptr<vsg::ShaderStage> vertexShader = lineShader_vert();
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = lineShader_frag();

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}   // colour data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}   // colour data
    };

    vsg::ref_ptr<vsg::InputAssemblyState> iaState = vsg::InputAssemblyState::create();
    iaState->topology = VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;

    vsg::ref_ptr<vsg::RasterizationState> raState = vsg::RasterizationState::create();
    raState->lineWidth = 1.0;  // only allowed value (also set as standard)

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        iaState,
        raState,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    scenegraph->addChild(bindGraphicsPipeline);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create(); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    // calculate vertices
    int numPoints = ps->GetNumRenderPoints();
    assert(numPoints > 2);
    double maxU = ps->GetPathGeometry()->GetPathDuration();
    double ustep = maxU / double(numPoints - 1);
    auto vertices = vsg::vec3Array::create(numPoints);
    auto colors = vsg::vec3Array::create(numPoints);
    for (int i = 0; i < numPoints; i++) {
        double u = ustep * (double(i));
        ChVector<> pos;
        ps->GetPathGeometry()->Evaluate(pos, u);
        vertices->set(i, vsg::vec3CH(pos));
        auto cv =
            vsg::vec3(material->GetDiffuseColor().R, material->GetDiffuseColor().G, material->GetDiffuseColor().B);
        colors->set(i, cv);
    }
    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, colors}));
    drawCommands->addChild(vsg::Draw::create(vertices->size(), 1, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createSpringShape(std::shared_ptr<ChLinkBase> link,
                                                         ChVisualModel::ShapeInstance shapeInstance,
                                                         std::shared_ptr<ChVisualMaterial> material,
                                                         vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                         std::shared_ptr<ChSpringShape> ss) {
    auto scenegraph = vsg::Group::create();
    // store some information for easier update
    scenegraph->setValue("Link", link);
    scenegraph->setValue("ShapeInstance", shapeInstance);
    scenegraph->setValue("Transform", transform);

    vsg::ref_ptr<vsg::ShaderStage> vertexShader = lineShader_vert();
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = lineShader_frag();

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}   // colour data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}   // colour data
    };

    vsg::ref_ptr<vsg::InputAssemblyState> iaState = vsg::InputAssemblyState::create();
    iaState->topology = VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;

    vsg::ref_ptr<vsg::RasterizationState> raState = vsg::RasterizationState::create();
    raState->lineWidth = 1.0;  // only allowed value (also set as standard)

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        iaState,
        raState,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    scenegraph->addChild(bindGraphicsPipeline);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create(); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    // calculate vertices
    auto numPoints = ss->GetResolution();
    double turns = ss->GetTurns();
    assert(numPoints > 2);
    auto vertices = vsg::vec3Array::create(numPoints);
    auto colors = vsg::vec3Array::create(numPoints);
    double length = 1;
    vsg::vec3 p(0, -length / 2, 0);
    double phase = 0.0;
    double height = 0.0;
    for (int iu = 0; iu < numPoints; iu++) {
        phase = turns * CH_C_2PI * (double)iu / (double)numPoints;
        height = length * ((double)iu / (double)numPoints);
        vsg::vec3 pos;
        pos = p + vsg::vec3(cos(phase), height, sin(phase));
        vertices->set(iu, pos);
        auto cv =
            vsg::vec3(material->GetDiffuseColor().R, material->GetDiffuseColor().G, material->GetDiffuseColor().B);
        colors->set(iu, cv);
    }
    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, colors}));
    drawCommands->addChild(vsg::Draw::create(vertices->size(), 1, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createUnitSegment(std::shared_ptr<ChLinkBase> link,
                                                         ChVisualModel::ShapeInstance shapeInstance,
                                                         std::shared_ptr<ChVisualMaterial> material,
                                                         vsg::ref_ptr<vsg::MatrixTransform> transform) {
    auto scenegraph = vsg::Group::create();
    // store some information for easier update
    scenegraph->setValue("Link", link);
    scenegraph->setValue("ShapeInstance", shapeInstance);
    scenegraph->setValue("Transform", transform);

    vsg::ref_ptr<vsg::ShaderStage> vertexShader = lineShader_vert();
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = lineShader_frag();

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}   // colour data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}   // colour data
    };

    vsg::ref_ptr<vsg::InputAssemblyState> iaState = vsg::InputAssemblyState::create();
    iaState->topology = VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;

    vsg::ref_ptr<vsg::RasterizationState> raState = vsg::RasterizationState::create();
    raState->lineWidth = 1.0;  // only allowed value (also set as standard)

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        iaState,
        raState,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    scenegraph->addChild(bindGraphicsPipeline);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create(); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    // calculate vertices
    const int numPoints = 2;
    auto vertices = vsg::vec3Array::create(numPoints);
    auto colors = vsg::vec3Array::create(numPoints);
    double length = 1;
    vertices->set(0, vsg::vec3(0, -length / 2, 0));
    vertices->set(1, vsg::vec3(0, +length / 2, 0));
    auto cv = vsg::vec3(material->GetDiffuseColor().R, material->GetDiffuseColor().G, material->GetDiffuseColor().B);
    colors->set(0, cv);
    colors->set(1, cv);
    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, colors}));
    drawCommands->addChild(vsg::Draw::create(vertices->size(), 1, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createDecoGrid(double ustep,
                                                      double vstep,
                                                      int nu,
                                                      int nv,
                                                      ChCoordsys<> pos,
                                                      ChColor col) {
    auto scenegraph = vsg::Group::create();
    vsg::ref_ptr<vsg::ShaderStage> vertexShader = lineShader_vert();
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = lineShader_frag();

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // automatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}   // colour data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0}   // colour data
    };

    vsg::ref_ptr<vsg::InputAssemblyState> iaState = vsg::InputAssemblyState::create();
    iaState->topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

    vsg::ref_ptr<vsg::RasterizationState> raState = vsg::RasterizationState::create();
    raState->lineWidth = 1.0;  // only allowed value (also set as standard)

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        iaState,
        raState,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    scenegraph->addChild(bindGraphicsPipeline);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create(); // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    auto transform = vsg::MatrixTransform::create();
    auto p = pos.pos;
    auto r = pos.rot;
    double rotAngle;
    ChVector<> rotAxis;
    r.Q_to_AngAxis(rotAngle, rotAxis);
    transform->matrix =
        vsg::translate(p.x(), p.y(), p.z()) * vsg::rotate(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z());

    scenegraph->addChild(transform);
    // calculate vertices
    std::vector<ChVector<>> v;
    for (int iu = -nu / 2; iu <= nu / 2; iu++) {
        ChVector<> V1(iu * ustep, vstep * (nv / 2), 0);
        ChVector<> V2(iu * ustep, -vstep * (nv / 2), 0);
        v.push_back(V1);
        v.push_back(V2);
        // drawSegment(vis, pos.TransformLocalToParent(V1), pos.TransformLocalToParent(V2), col, use_Zbuffer);
    }

    for (int iv = -nv / 2; iv <= nv / 2; iv++) {
        ChVector<> V1(ustep * (nu / 2), iv * vstep, 0);
        ChVector<> V2(-ustep * (nu / 2), iv * vstep, 0);
        v.push_back(V1);
        v.push_back(V2);
        // drawSegment(vis, pos.TransformLocalToParent(V1), pos.TransformLocalToParent(V2), col, use_Zbuffer);
    }

    auto numPoints = v.size();
    auto vertices = vsg::vec3Array::create(numPoints);
    auto colors = vsg::vec3Array::create(numPoints);
    auto cv = vsg::vec3(col.R, col.G, col.B);
    colors->set(0, cv);
    for (size_t i = 0; i < numPoints; i++) {
        vertices->set(i, vsg::vec3CH(v[i]));
        colors->set(i, cv);
    }
    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, colors}));
    drawCommands->addChild(vsg::Draw::create(vertices->size(), 1, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);
    return scenegraph;
}

/// create a ShaderSet for Phong shaded rendering with tiled textures
vsg::ref_ptr<vsg::ShaderSet> ShapeBuilder::createTilingPhongShaderSet(vsg::ref_ptr<const vsg::Options> options) {
    if (options) {
        // check if a ShaderSet has already been assigned to the options object, if so return it
        if (auto itr = options->shaderSets.find("phong"); itr != options->shaderSets.end())
            return itr->second;
    }

    auto vertexShader = vsg::read_cast<vsg::ShaderStage>("vsg/shaders/chrono.vert", options);
    // if (!vertexShader)
    //     vertexShader = assimp_vert();  // fallback to shaders/assimp_vert.cpp
    auto fragmentShader = vsg::read_cast<vsg::ShaderStage>("vsg/shaders/chrono_phong.frag", options);
    // if (!fragmentShader)
    //     fragmentShader = assimp_phong_frag();

    auto shaderSet = vsg::ShaderSet::create(vsg::ShaderStages{vertexShader, fragmentShader});

    shaderSet->addAttributeBinding("vsg_Vertex", "", 0, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));
    shaderSet->addAttributeBinding("vsg_Normal", "", 1, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));
    shaderSet->addAttributeBinding("vsg_TexCoord0", "", 2, VK_FORMAT_R32G32_SFLOAT, vsg::vec2Array::create(1));
    shaderSet->addAttributeBinding("vsg_Color", "", 3, VK_FORMAT_R32G32B32A32_SFLOAT, vsg::vec4Array::create(1));
    shaderSet->addAttributeBinding("vsg_position", "VSG_INSTANCE_POSITIONS", 4, VK_FORMAT_R32G32B32_SFLOAT,
                                   vsg::vec3Array::create(1));

    shaderSet->addUniformBinding("displacementMap", "VSG_DISPLACEMENT_MAP", 0, 6,
                                 VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT,
                                 vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("diffuseMap", "VSG_DIFFUSE_MAP", 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("normalMap", "VSG_NORMAL_MAP", 0, 2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec3Array2D::create(1, 1));
    shaderSet->addUniformBinding("aoMap", "VSG_LIGHTMAP_MAP", 0, 3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("emissiveMap", "VSG_EMISSIVE_MAP", 0, 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("texrepeat", "", 0, 9, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                 VK_SHADER_STAGE_VERTEX_BIT, vsg::vec3Value::create());
    shaderSet->addUniformBinding("material", "", 0, 10, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::PhongMaterialValue::create());
    shaderSet->addUniformBinding("lightData", "", 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array::create(64));

    shaderSet->addPushConstantRange("pc", "", VK_SHADER_STAGE_VERTEX_BIT, 0, 128);

    shaderSet->optionalDefines = {"VSG_GREYSACLE_DIFFUSE_MAP", "VSG_TWO_SIDED_LIGHTING", "VSG_POINT_SPRITE"};

    shaderSet->definesArrayStates.push_back(vsg::DefinesArrayState{
        {"VSG_INSTANCE_POSITIONS", "VSG_DISPLACEMENT_MAP"}, vsg::PositionAndDisplacementMapArrayState::create()});
    shaderSet->definesArrayStates.push_back(
        vsg::DefinesArrayState{{"VSG_INSTANCE_POSITIONS"}, vsg::PositionArrayState::create()});
    shaderSet->definesArrayStates.push_back(
        vsg::DefinesArrayState{{"VSG_DISPLACEMENT_MAP"}, vsg::DisplacementMapArrayState::create()});

    return shaderSet;
}

/// create a ShaderSet for PBR shaded rendering with tiled textures
vsg::ref_ptr<vsg::ShaderSet> ShapeBuilder::createTilingPbrShaderSet(vsg::ref_ptr<const vsg::Options> options) {
    if (options) {
        // check if a ShaderSet has already been assigned to the options object, if so return it
        if (auto itr = options->shaderSets.find("pbr"); itr != options->shaderSets.end())
            return itr->second;
    }

    auto vertexShader = vsg::read_cast<vsg::ShaderStage>("vsg/shaders/chrono.vert", options);
    // if (!vertexShader)
    //     vertexShader = assimp_vert();  // fallback to shaders/assimp_vert.cpp
    auto fragmentShader = vsg::read_cast<vsg::ShaderStage>("vsg/shaders/chrono_pbr.frag", options);
    // if (!fragmentShader)
    //     fragmentShader = assimp_pbr_frag();

    auto shaderSet = vsg::ShaderSet::create(vsg::ShaderStages{vertexShader, fragmentShader});

    shaderSet->addAttributeBinding("vsg_Vertex", "", 0, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));
    shaderSet->addAttributeBinding("vsg_Normal", "", 1, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));
    shaderSet->addAttributeBinding("vsg_TexCoord0", "", 2, VK_FORMAT_R32G32_SFLOAT, vsg::vec2Array::create(1));
    shaderSet->addAttributeBinding("vsg_Color", "", 3, VK_FORMAT_R32G32B32A32_SFLOAT, vsg::vec4Array::create(1));
    shaderSet->addAttributeBinding("vsg_position", "VSG_INSTANCE_POSITIONS", 4, VK_FORMAT_R32G32B32_SFLOAT,
                                   vsg::vec3Array::create(1));

    shaderSet->addUniformBinding("displacementMap", "VSG_DISPLACEMENT_MAP", 0, 6,
                                 VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT,
                                 vsg::vec4Array2D::create(1, 1));

    shaderSet->addUniformBinding("diffuseMap", "VSG_DIFFUSE_MAP", 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("mrMap", "VSG_METALLROUGHNESS_MAP", 0, 1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("normalMap", "VSG_NORMAL_MAP", 0, 2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec3Array2D::create(1, 1));
    shaderSet->addUniformBinding("aoMap", "VSG_LIGHTMAP_MAP", 0, 3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("emissiveMap", "VSG_EMISSIVE_MAP", 0, 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("specularMap", "VSG_SPECULAR_MAP", 0, 5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("opacityMap", "VSG_OPACITY_MAP", 0, 7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array2D::create(1, 1));
    shaderSet->addUniformBinding("texrepeat", "", 0, 9, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                 VK_SHADER_STAGE_VERTEX_BIT, vsg::vec3Value::create());
    shaderSet->addUniformBinding("PbrData", "", 0, 10, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::PbrMaterialValue::create());
    shaderSet->addUniformBinding("LightData", "", 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec4Array::create(64));

    shaderSet->addPushConstantRange("pc", "", VK_SHADER_STAGE_VERTEX_BIT, 0, 128);

    shaderSet->optionalDefines = {"VSG_GREYSACLE_DIFFUSE_MAP", "VSG_TWO_SIDED_LIGHTING", "VSG_POINT_SPRITE"};

    shaderSet->definesArrayStates.push_back(vsg::DefinesArrayState{
        {"VSG_INSTANCE_POSITIONS", "VSG_DISPLACEMENT_MAP"}, vsg::PositionAndDisplacementMapArrayState::create()});
    shaderSet->definesArrayStates.push_back(
        vsg::DefinesArrayState{{"VSG_INSTANCE_POSITIONS"}, vsg::PositionArrayState::create()});
    shaderSet->definesArrayStates.push_back(
        vsg::DefinesArrayState{{"VSG_DISPLACEMENT_MAP"}, vsg::DisplacementMapArrayState::create()});

    return shaderSet;
}

bool ShapeBuilder::applyTexture(vsg::Path& path,
                                vsg::ref_ptr<vsg::GraphicsPipelineConfigurator> pipeConfig,
                                vsg::Descriptors& descriptors,
                                std::string& uniformName) {
    if (path) {
        auto texData = vsg::read_cast<vsg::Data>(path, m_options);
        if (!texData) {
            GetLog() << "Could not read texture file: " << path << "\n";
        } else {
            // enable texturing with mipmaps
            auto sampler = vsg::Sampler::create();
            sampler->maxLod =
                static_cast<uint32_t>(std::floor(std::log2(std::max(texData->width(), texData->height())))) + 1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;  // default yet, just an example how to set
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            pipeConfig->assignTexture(uniformName, texData, sampler);
            return true;
        }
    }
    return false;
}

bool ShapeBuilder::applyMetalRoughnessTexture(vsg::Path& metalPath,
                                              vsg::Path& roughPath,
                                              vsg::ref_ptr<vsg::GraphicsPipelineConfigurator> pipeConfig,
                                              vsg::Descriptors& descriptors,
                                              std::string& uniformName) {
    int wM, hM, nM;
    unsigned char* metalData = stbi_load(metalPath.string().c_str(), &wM, &hM, &nM, 1);
    int wR, hR, nR;
    unsigned char* roughData = stbi_load(roughPath.string().c_str(), &wR, &hR, &nR, 1);
    if (!metalData && !roughData) {
        // nothing to do
        return false;
    }
    if (metalData && roughData) {
        if ((wM != wR) || (hM != hR)) {
            GetLog() << "Metalness and Roughness Texture must have the same size!\n";
            return false;
        }
    }

    auto texData = vsg::vec3Array2D::create(wR, hR, vsg::Data::Layout{VK_FORMAT_R32G32B32_SFLOAT});
    if (!texData) {
        GetLog() << "Could not create texture data!\n";
        return false;
    }
    /*
     texData->set(0, 0, {0.0f, 0.0f, 1.0f});
     texData->set(1, 1, {0.0f, 0.0f, 1.0f});
     */
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
            texData->set(i, j, vsg::vec3(red, green, blue));
            k++;
        }
    }
    // enable texturing with mipmaps
    auto sampler = vsg::Sampler::create();
    sampler->maxLod = static_cast<uint32_t>(std::floor(std::log2(std::max(texData->width(), texData->height())))) + 1;
    sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;  // default yet, just an example how to set
    sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    pipeConfig->assignTexture(uniformName, texData, sampler);
    if (roughData)
        stbi_image_free(roughData);
    if (metalData)
        stbi_image_free(metalData);
    return true;
}

vsg::ref_ptr<vsg::PhongMaterialValue> ShapeBuilder::createPhongMaterialFromChronoMaterial(
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
