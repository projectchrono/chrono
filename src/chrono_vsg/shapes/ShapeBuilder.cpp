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

#include "ShapeBuilder.h"
#include "GetBoxShapeData.h"
#include "GetDiceShapeData.h"
#include "GetSphereShapeData.h"
#include "GetParticleShapeData.h"
#include "GetCylinderShapeData.h"
#include "GetCapsuleShapeData.h"
#include "GetConeShapeData.h"
#include "GetSurfaceShapeData.h"

#include "chrono_vsg/resources/lineShader_vert.h"
#include "chrono_vsg/resources/lineShader_frag.h"

namespace chrono {
namespace vsg3d {

void ShapeBuilder::assignCompileTraversal(vsg::ref_ptr<vsg::CompileTraversal> ct) {
    compileTraversal = ct;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createShape(BasicShape theShape,
                                                   std::shared_ptr<ChVisualMaterial> material,
                                                   vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                   bool drawMode,
                                                   std::shared_ptr<ChSurfaceShape> surface) {
    auto scenegraph = vsg::Group::create();

    vsg::ref_ptr<vsg::ShaderSet> shaderSet;

    auto repeatValues = vsg::vec3Value::create();
    repeatValues->set(vsg::vec3(material->GetKdTextureScale().x(), material->GetKdTextureScale().y(), 1.0f));
    shaderSet = createTilingPhongShaderSet(m_options);

    auto rasterizationState = vsg::RasterizationState::create();
    if (drawMode) {
        rasterizationState->polygonMode = VK_POLYGON_MODE_LINE;
    }
    shaderSet->defaultGraphicsPipelineStates.push_back(rasterizationState);
    auto graphicsPipelineConfig = vsg::GraphicsPipelineConfigurator::create(shaderSet);
    auto& defines = graphicsPipelineConfig->shaderHints->defines;

    // set up graphics pipeline
    vsg::Descriptors descriptors;

    // set up pass of material
    auto phongMat = vsg::PhongMaterialValue::create();
    float alpha = material->GetOpacity();
    phongMat->value().diffuse.set(material->GetDiffuseColor().R, material->GetDiffuseColor().G,
                                  material->GetDiffuseColor().B, alpha);
    /*
        phongMat->value().ambient.set(material->GetAmbientColor().R, material->GetAmbientColor().G,
                material->GetAmbientColor().B, alpha);
    */
    phongMat->value().ambient.set(1.0, 1.0, 1.0, alpha);  // ambient intensity set by ambient light source!
    phongMat->value().specular.set(material->GetSpecularColor().R, material->GetSpecularColor().G,
                                   material->GetSpecularColor().B, alpha);
    phongMat->value().emissive.set(material->GetEmissiveColor().R, material->GetEmissiveColor().G,
                                   material->GetEmissiveColor().B, alpha);
    phongMat->value().alphaMask = alpha;
    phongMat->value().alphaMaskCutoff = 0.3f;

    // read texture image for diffuse light
    vsg::Path diffuseTextureFile(material->GetKdTexture());
    if (diffuseTextureFile) {
        auto diffuseTextureData = vsg::read_cast<vsg::Data>(diffuseTextureFile, m_options);
        if (!diffuseTextureData) {
            std::cout << "Could not read texture file : " << diffuseTextureFile << std::endl;
        } else {
            // enable texturing with mipmaps
            auto sampler = vsg::Sampler::create();
            sampler->maxLod = static_cast<uint32_t>(std::floor(
                                  std::log2(std::max(diffuseTextureData->width(), diffuseTextureData->height())))) +
                              1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;  // default yet, just an example how to set
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            graphicsPipelineConfig->assignTexture(descriptors, "diffuseMap", diffuseTextureData, sampler);
            // vsg combines material color and texture color, better use only one of it
            phongMat->value().diffuse.set(1.0, 1.0, 1.0, alpha);
        }
    }
    // read texture image for diffuse light
    vsg::Path normalTextureFile(material->GetNormalMapTexture());
    if (normalTextureFile) {
        auto normalTextureData = vsg::read_cast<vsg::Data>(normalTextureFile, m_options);
        if (!normalTextureData) {
            std::cout << "Could not read texture file : " << normalTextureFile << std::endl;
        } else {
            // enable texturing with mipmaps
            auto sampler = vsg::Sampler::create();
            sampler->maxLod = static_cast<uint32_t>(std::floor(
                                  std::log2(std::max(normalTextureData->width(), normalTextureData->height())))) +
                              1;
            sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;  // default yet, just an example how to set
            sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            graphicsPipelineConfig->assignTexture(descriptors, "normalMap", normalTextureData, sampler);
            // vsg combines material color and texture color, better use only one of it
            // phongMat->value().diffuse.set(1.0, 1.0, 1.0, alpha);
        }
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
    graphicsPipelineConfig->assignUniform(descriptors, "texrepeat", repeatValues);
    graphicsPipelineConfig->assignUniform(descriptors, "material", phongMat);

    if (m_options->sharedObjects)
        m_options->sharedObjects->share(descriptors);

    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::vec3Array> normals;
    vsg::ref_ptr<vsg::vec2Array> texcoords;
    vsg::ref_ptr<vsg::ushortArray> indices;
    float boundingSphereRadius;
    switch (theShape) {
        case BOX_SHAPE:
            GetBoxShapeData(vertices, normals, texcoords, indices, boundingSphereRadius);
            break;
        case DIE_SHAPE:
            GetDiceShapeData(vertices, normals, texcoords, indices, boundingSphereRadius);
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
        case CONE_SHAPE:
            GetConeShapeData(vertices, normals, texcoords, indices, boundingSphereRadius);
            break;
        case SURFACE_SHAPE:
            GetSurfaceShapeData(surface, vertices, normals, texcoords, indices, boundingSphereRadius);
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
    graphicsPipelineConfig->additionalDescriptorSetLayout = vdsl;

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
    transform->addChild(stateGroup);

    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(transform);
    }

    scenegraph->addChild(transform);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createTrimeshColShape(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                             bool drawMode,
                                                             std::shared_ptr<ChTriangleMeshShape> tms) {
    auto scenegraph = vsg::Group::create();

    const auto& mesh = tms->GetMesh();

    const auto& vertices = mesh->getCoordsVertices();
    const auto& normals = mesh->getCoordsNormals();
    const auto& uvs = mesh->getCoordsUV();
    const auto& colors = mesh->getCoordsColors();

    const auto& v_indices = mesh->getIndicesVertexes();
    const auto& n_indices = mesh->getIndicesNormals();
    const auto& uv_indices = mesh->getIndicesUV();
    const auto& c_indices = mesh->getIndicesColors();

    unsigned int ntriangles = (unsigned int)v_indices.size();
    unsigned int nvertexes = ntriangles * 3;

    // Set the Irrlicht vertex and index buffers for the mesh buffer
    ChVector<> t[3];    // positions of triangle vertices
    ChVector<> n[3];    // normals at the triangle vertices
    ChVector2<> uv[3];  // UV coordinates at the triangle vertices
    ChColor col[3];     // color coordinates at the triangle vertices

    auto default_color = tms->GetColor();

    std::vector<ChVector<>> tmp_vertices;
    std::vector<ChVector<>> tmp_normals;
    std::vector<ChVector2<>> tmp_texcoords;
    std::vector<ChColor> tmp_colors;

    for (unsigned int itri = 0; itri < ntriangles; itri++) {
        for (int iv = 0; iv < 3; iv++)
            t[iv] = vertices[v_indices[itri][iv]];

        if (n_indices.size() == ntriangles) {
            for (int iv = 0; iv < 3; iv++)
                n[iv] = normals[n_indices[itri][iv]];
        } else {
            n[0] = Vcross(t[1] - t[0], t[2] - t[0]).GetNormalized();
            n[1] = n[0];
            n[2] = n[0];
        }

        if (uv_indices.size() == ntriangles) {
            for (int iv = 0; iv < 3; iv++)
                uv[iv] = uvs[uv_indices[itri][iv]];
        } else if (uv_indices.size() == 0 && uvs.size() == vertices.size()) {
            for (int iv = 0; iv < 3; iv++)
                uv[iv] = uvs[v_indices[itri][iv]];
        }

        if (c_indices.size() == ntriangles) {
            for (int iv = 0; iv < 3; iv++)
                col[iv] = colors[c_indices[itri][iv]];
        } else if (c_indices.size() == 0 && colors.size() == vertices.size()) {
            for (int iv = 0; iv < 3; iv++)
                col[iv] = colors[v_indices[itri][iv]];
        } else {
            for (int iv = 0; iv < 3; iv++)
                col[iv] = default_color;
        }

        for (int iv = 0; iv < 3; iv++) {
            tmp_vertices.push_back(t[iv]);
            tmp_normals.push_back(n[iv]);
            tmp_texcoords.push_back(uv[iv]);
            tmp_colors.push_back(col[iv]);
        }
    }
    // create and fill the vsg buffers
    size_t nVert = tmp_vertices.size();
    GetLog() << ">>>> nVertInput = " << vertices.size() << "  >>>>> nVertOutput = " << nVert << "\n";
    vsg::ref_ptr<vsg::vec3Array> vsg_vertices = vsg::vec3Array::create(nVert);
    vsg::ref_ptr<vsg::vec3Array> vsg_normals = vsg::vec3Array::create(nVert);
    vsg::ref_ptr<vsg::vec2Array> vsg_texcoords = vsg::vec2Array::create(nVert);
    vsg::ref_ptr<vsg::uintArray> vsg_indices = vsg::uintArray::create(nVert);
    vsg::ref_ptr<vsg::vec4Array> vsg_colors = vsg::vec4Array::create(nVert);
    for (size_t k = 0; k < nVert; k++) {
        vsg_vertices->set(k, vsg::vec3(tmp_vertices[k].x(), tmp_vertices[k].y(), tmp_vertices[k].z()));
        vsg_normals->set(k, vsg::vec3(tmp_normals[k].x(), tmp_normals[k].y(), tmp_normals[k].z()));
        // seems to work with v-coordinate flipped on VSG
        vsg_texcoords->set(k, vsg::vec2(tmp_texcoords[k].x(), 1.0f - tmp_texcoords[k].y()));
        vsg_colors->set(k, vsg::vec4(tmp_colors[k].R, tmp_colors[k].G, tmp_colors[k].B, 1.0f));
        vsg_indices->set(k, k);
    }

    vsg::ref_ptr<vsg::ShaderSet> shaderSet;

    shaderSet = createPhongShaderSet(m_options);

    auto rasterizationState = vsg::RasterizationState::create();
    if (drawMode) {
        rasterizationState->polygonMode = VK_POLYGON_MODE_LINE;
    }
    shaderSet->defaultGraphicsPipelineStates.push_back(rasterizationState);
    auto graphicsPipelineConfig = vsg::GraphicsPipelineConfigurator::create(shaderSet);
    auto& defines = graphicsPipelineConfig->shaderHints->defines;

    // set up graphics pipeline
    vsg::Descriptors descriptors;

    // set up pass of material
    auto phongMat = vsg::PhongMaterialValue::create();
    phongMat->value().ambient = vsg::vec4(0.2f, 0.2f, 0.2f, 1.0f);
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

    vsg::DataList vertexArrays;

    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, vsg_vertices);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, vsg_normals);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, vsg_texcoords);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Color", VK_VERTEX_INPUT_RATE_INSTANCE, vsg_colors);

    if (m_options->sharedObjects)
        m_options->sharedObjects->share(vertexArrays);
    if (m_options->sharedObjects)
        m_options->sharedObjects->share(vsg_indices);

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(graphicsPipelineConfig->baseAttributeBinding, vertexArrays));
    drawCommands->addChild(vsg::BindIndexBuffer::create(vsg_indices));
    drawCommands->addChild(vsg::DrawIndexed::create(vsg_indices->size(), 1, 0, 0, 0));

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
    graphicsPipelineConfig->additionalDescriptorSetLayout = vdsl;

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

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
    // Descriptors to decorate the whole graph
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
    transform->addChild(stateGroup);
    scenegraph->addChild(transform);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createTrimeshColShapeSCM(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                                bool drawMode,
                                                                std::shared_ptr<ChTriangleMeshShape> tms) {
    auto scenegraph = vsg::Group::create();

    const auto& mesh = tms->GetMesh();

    const auto& vertices = mesh->getCoordsVertices();
    const auto& normals = mesh->getCoordsNormals();
    const auto& uvs = mesh->getCoordsUV();
    const auto& colors = mesh->getCoordsColors();

    size_t nvertices = vertices.size();
    bool normals_ok = true;
    if (nvertices != normals.size()) {
        normals_ok = false;
    }
    bool texcoords_ok = true;
    if (nvertices != uvs.size()) {
        texcoords_ok = false;
    }
    bool colors_ok = true;
    if (nvertices != colors.size()) {
        colors_ok = false;
    }

    const auto& v_indices = mesh->getIndicesVertexes();
    unsigned int ntriangles = (unsigned int)v_indices.size();
    auto default_color = tms->GetColor();

    // create and fill the vsg buffers
    vsg::ref_ptr<vsg::vec3Array> vsg_vertices = vsg::vec3Array::create(nvertices);
    vsg::ref_ptr<vsg::vec3Array> vsg_normals = vsg::vec3Array::create(nvertices);
    vsg::ref_ptr<vsg::vec2Array> vsg_texcoords = vsg::vec2Array::create(nvertices);
    vsg::ref_ptr<vsg::uintArray> vsg_indices = vsg::uintArray::create(v_indices.size() * 3);
    vsg::ref_ptr<vsg::vec4Array> vsg_colors = vsg::vec4Array::create(nvertices);
    for (size_t k = 0; k < nvertices; k++) {
        vsg_vertices->set(k, vsg::vec3(vertices[k].x(), vertices[k].y(), vertices[k].z()));
        if (normals_ok) {
            vsg_normals->set(k, vsg::vec3(normals[k].x(), normals[k].y(), normals[k].z()));
        } else {
            vsg_normals->set(k, vsg::vec3(0.0, 0.0, 1.0));
        }
        // seems to work with v-coordinate flipped on VSG
        if (texcoords_ok) {
            vsg_texcoords->set(k, vsg::vec2(uvs[k].x(), uvs[k].y()));
        } else {
            vsg_texcoords->set(k, vsg::vec2(0.0, 0.0));
        }
        if (colors_ok) {
            vsg_colors->set(k, vsg::vec4(colors[k].R, colors[k].G, colors[k].B, 1.0f));
        } else {
            vsg_colors->set(k, vsg::vec4(default_color.R, default_color.G, default_color.B, 1.0f));
        }
    }
    size_t kk = 0;
    for (size_t k = 0; k < v_indices.size() * 3; k += 3) {
        vsg_indices->set(k, v_indices[kk][0]);
        vsg_indices->set(k + 1, v_indices[kk][1]);
        vsg_indices->set(k + 2, v_indices[kk++][2]);
    }
    vsg::ref_ptr<vsg::ShaderSet> shaderSet;

    shaderSet = createPhongShaderSet(m_options);

    auto rasterizationState = vsg::RasterizationState::create();
    if (drawMode) {
        rasterizationState->polygonMode = VK_POLYGON_MODE_LINE;
    }
    shaderSet->defaultGraphicsPipelineStates.push_back(rasterizationState);
    auto graphicsPipelineConfig = vsg::GraphicsPipelineConfigurator::create(shaderSet);
    auto& defines = graphicsPipelineConfig->shaderHints->defines;

    // set up graphics pipeline
    vsg::Descriptors descriptors;

    // set up pass of material
    auto phongMat = vsg::PhongMaterialValue::create();
    phongMat->value().ambient = vsg::vec4(0.2f, 0.2f, 0.2f, 1.0f);
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

    vsg::DataList vertexArrays;

    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, vsg_vertices);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, vsg_normals);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, vsg_texcoords);
    graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Color", VK_VERTEX_INPUT_RATE_VERTEX, vsg_colors);

    if (m_options->sharedObjects)
        m_options->sharedObjects->share(vertexArrays);
    if (m_options->sharedObjects)
        m_options->sharedObjects->share(vsg_indices);

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(graphicsPipelineConfig->baseAttributeBinding, vertexArrays));
    drawCommands->addChild(vsg::BindIndexBuffer::create(vsg_indices));
    drawCommands->addChild(vsg::DrawIndexed::create(vsg_indices->size(), 1, 0, 0, 0));

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
    graphicsPipelineConfig->additionalDescriptorSetLayout = vdsl;

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

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
    // Descriptors to decorate the whole graph
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
    transform->addChild(stateGroup);
    scenegraph->addChild(transform);
    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createTrimeshMatShape(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                             bool drawMode,
                                                             std::shared_ptr<ChTriangleMeshShape> tms) {
    auto scenegraph = vsg::Group::create();

    // set up model transformation node
    transform->subgraphRequiresLocalFrustum = false;
    scenegraph->addChild(transform);

    const auto& mesh = tms->GetMesh();
    const auto& materials = tms->GetMaterials();
    int nmaterials = (int)materials.size();

    const auto& vertices = mesh->getCoordsVertices();
    const auto& normals = mesh->getCoordsNormals();
    const auto& uvs = mesh->getCoordsUV();

    const auto& v_indices = mesh->getIndicesVertexes();
    const auto& n_indices = mesh->getIndicesNormals();
    const auto& uv_indices = mesh->getIndicesUV();
    const auto& m_indices = mesh->getIndicesMaterials();

    size_t ntriangles_all = (unsigned int)v_indices.size();

    // Count number of faces assigned to each material (buffer)
    std::vector<size_t> nfaces_per_buffer;
    if (m_indices.empty()) {
        assert(nmaterials == 1);
        nfaces_per_buffer.push_back(ntriangles_all);
    } else {
        for (size_t imat = 0; imat < nmaterials; imat++) {
            auto count = std::count(m_indices.begin(), m_indices.end(), imat);
            nfaces_per_buffer.push_back(count);
        }
    }

    for (size_t imat = 0; imat < nmaterials; imat++) {
        auto chronoMat = materials[imat];
        // translate to vsgMat here
        vsg::ref_ptr<vsg::ShaderSet> shaderSet;

        auto repeatValues = vsg::vec3Value::create();
        repeatValues->set(vsg::vec3(chronoMat->GetKdTextureScale().x(), chronoMat->GetKdTextureScale().y(), 1.0f));
        shaderSet = createTilingPhongShaderSet(m_options);

        auto rasterizationState = vsg::RasterizationState::create();
        if (drawMode) {
            rasterizationState->polygonMode = VK_POLYGON_MODE_LINE;
        }
        shaderSet->defaultGraphicsPipelineStates.push_back(rasterizationState);
        auto graphicsPipelineConfig = vsg::GraphicsPipelineConfigurator::create(shaderSet);
        auto& defines = graphicsPipelineConfig->shaderHints->defines;

        // two-sided polygons? -> cannot be used together with transparency!
        if (!tms->IsBackfaceCull() && chronoMat->GetOpacity() == 1.0) {
            graphicsPipelineConfig->rasterizationState->cullMode = VK_CULL_MODE_NONE;
            defines.insert("VSG_TWO_SIDED_LIGHTING");
        }

        // set up graphics pipeline
        vsg::Descriptors descriptors;

        // set up pass of material
        auto phongMat = vsg::PhongMaterialValue::create();
        float alpha = chronoMat->GetOpacity();
        phongMat->value().diffuse.set(chronoMat->GetDiffuseColor().R, chronoMat->GetDiffuseColor().G,
                                      chronoMat->GetDiffuseColor().B, alpha);
        phongMat->value().ambient.set(chronoMat->GetAmbientColor().R, chronoMat->GetAmbientColor().G,
                                      chronoMat->GetAmbientColor().B, alpha);
        phongMat->value().specular.set(chronoMat->GetSpecularColor().R, chronoMat->GetSpecularColor().G,
                                       chronoMat->GetSpecularColor().B, alpha);
        phongMat->value().emissive.set(chronoMat->GetEmissiveColor().R, chronoMat->GetEmissiveColor().G,
                                       chronoMat->GetEmissiveColor().B, alpha);
        phongMat->value().alphaMask = alpha;
        phongMat->value().alphaMaskCutoff = 0.3f;

        // read texture image for diffuse light
        vsg::Path diffuseTextureFile(chronoMat->GetKdTexture());
        if (diffuseTextureFile) {
            auto diffuseTextureData = vsg::read_cast<vsg::Data>(diffuseTextureFile, m_options);
            if (!diffuseTextureData) {
                std::cout << "Could not read texture file : " << diffuseTextureFile << std::endl;
            } else {
                // enable texturing with mipmaps
                auto sampler = vsg::Sampler::create();
                sampler->maxLod = static_cast<uint32_t>(std::floor(
                                      std::log2(std::max(diffuseTextureData->width(), diffuseTextureData->height())))) +
                                  1;
                sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;  // default yet, just an example how to set
                sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                graphicsPipelineConfig->assignTexture(descriptors, "diffuseMap", diffuseTextureData, sampler);
                // vsg combines material color and texture color, better use only one of it
                phongMat->value().diffuse.set(1.0, 1.0, 1.0, alpha);
            }
        }

        // read texture image for normal vectors
        vsg::Path normalTextureFile(chronoMat->GetNormalMapTexture());
        if (normalTextureFile) {
            auto normalTextureData = vsg::read_cast<vsg::Data>(normalTextureFile, m_options);
            if (!normalTextureData) {
                std::cout << "Could not read texture file : " << normalTextureFile << std::endl;
            } else {
                // enable texturing with mipmaps
                auto sampler = vsg::Sampler::create();
                sampler->maxLod = static_cast<uint32_t>(std::floor(
                                      std::log2(std::max(normalTextureData->width(), normalTextureData->height())))) +
                                  1;
                sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;  // default yet, just an example how to set
                sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                graphicsPipelineConfig->assignTexture(descriptors, "normalMap", normalTextureData, sampler);
            }
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
        graphicsPipelineConfig->assignUniform(descriptors, "texrepeat", repeatValues);
        graphicsPipelineConfig->assignUniform(descriptors, "material", phongMat);

        if (m_options->sharedObjects)
            m_options->sharedObjects->share(descriptors);

        std::vector<ChVector<>> tmp_vertices;
        std::vector<ChVector<>> tmp_normals;
        std::vector<ChVector2<>> tmp_texcoords;
        // Set the VSG vertex and index buffers for this material
        ChVector<> t[3];    // positions of triangle vertices
        ChVector<> n[3];    // normals at the triangle vertices
        ChVector2<> uv[3];  // UV coordinates at the triangle vertices
        size_t num_added_tri = 0;
        for (size_t itri = 0; itri < ntriangles_all; itri++) {
            if (!m_indices.empty() && m_indices[itri] != imat)
                continue;

            for (int iv = 0; iv < 3; iv++)
                t[iv] = vertices[v_indices[itri][iv]];

            if (n_indices.size() == ntriangles_all) {
                for (int iv = 0; iv < 3; iv++)
                    n[iv] = normals[n_indices[itri][iv]];
            } else {
                n[0] = Vcross(t[1] - t[0], t[2] - t[0]).GetNormalized();
                n[1] = n[0];
                n[2] = n[0];
            }

            if (uv_indices.size() == ntriangles_all) {
                for (int iv = 0; iv < 3; iv++)
                    uv[iv] = uvs[uv_indices[itri][iv]];
            } else {
                for (int iv = 0; iv < 3; iv++)
                    uv[iv] = 0.0;
            }
            for (int j = 0; j < 3; j++) {
                tmp_vertices.push_back(t[j]);
                tmp_normals.push_back(n[j]);
                tmp_texcoords.push_back(uv[j]);
            }
        }  // itri
        // create and fill the vsg buffers
        size_t nVert = tmp_vertices.size();
        vsg::ref_ptr<vsg::vec3Array> vsg_vertices = vsg::vec3Array::create(nVert);
        vsg::ref_ptr<vsg::vec3Array> vsg_normals = vsg::vec3Array::create(nVert);
        vsg::ref_ptr<vsg::vec2Array> vsg_texcoords = vsg::vec2Array::create(nVert);
        vsg::ref_ptr<vsg::uintArray> vsg_indices = vsg::uintArray::create(nVert);
        for (size_t k = 0; k < nVert; k++) {
            vsg_vertices->set(k, vsg::vec3(tmp_vertices[k].x(), tmp_vertices[k].y(), tmp_vertices[k].z()));
            vsg_normals->set(k, vsg::vec3(tmp_normals[k].x(), tmp_normals[k].y(), tmp_normals[k].z()));
            // seems to work with v-coordinate flipped on VSG
            vsg_texcoords->set(k, vsg::vec2(tmp_texcoords[k].x(), 1.0f - tmp_texcoords[k].y()));
            vsg_indices->set(k, k);
        }
        auto colors = vsg::vec4Value::create(vsg::vec4{1.0f, 1.0f, 1.0f, 1.0f});

        vsg::DataList vertexArrays;

        graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, vsg_vertices);
        graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, vsg_normals);
        graphicsPipelineConfig->assignArray(vertexArrays, "vsg_TexCoord0", VK_VERTEX_INPUT_RATE_VERTEX, vsg_texcoords);
        graphicsPipelineConfig->assignArray(vertexArrays, "vsg_Color", VK_VERTEX_INPUT_RATE_INSTANCE, colors);

        if (m_options->sharedObjects)
            m_options->sharedObjects->share(vertexArrays);
        if (m_options->sharedObjects)
            m_options->sharedObjects->share(vsg_indices);

        // setup geometry
        auto drawCommands = vsg::Commands::create();
        drawCommands->addChild(
            vsg::BindVertexBuffers::create(graphicsPipelineConfig->baseAttributeBinding, vertexArrays));
        drawCommands->addChild(vsg::BindIndexBuffer::create(vsg_indices));
        drawCommands->addChild(vsg::DrawIndexed::create(vsg_indices->size(), 1, 0, 0, 0));

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
        graphicsPipelineConfig->additionalDescriptorSetLayout = vdsl;

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

        // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
        // Descriptors to decorate the whole graph
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
        transform->addChild(stateGroup);
    }  // imat

    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(transform);
    }

    if (compileTraversal)
        compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createParticlePattern(std::shared_ptr<ChVisualMaterial> material,
                                                             bool drawMode) {
    auto scenegraph = vsg::Group::create();

    auto shaderSet = vsg::createPhongShaderSet(m_options);
    auto rasterizationState = vsg::RasterizationState::create();
    if (drawMode) {
        rasterizationState->polygonMode = VK_POLYGON_MODE_LINE;
    }
    shaderSet->defaultGraphicsPipelineStates.push_back(rasterizationState);
    auto graphicsPipelineConfig = vsg::GraphicsPipelineConfigurator::create(shaderSet);
    auto& defines = graphicsPipelineConfig->shaderHints->defines;

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
                                   material->GetSpecularColor().B, alpha);
    phongMat->value().emissive.set(material->GetEmissiveColor().R, material->GetEmissiveColor().G,
                                   material->GetEmissiveColor().B, alpha);
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

    GetParticleShapeData(vertices, normals, texcoords, indices, boundingSphereRadius);
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
    graphicsPipelineConfig->additionalDescriptorSetLayout = vdsl;

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

    // add drawCommands to StateGroup
    stateGroup->addChild(drawCommands);
    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(stateGroup);
    }
    scenegraph->addChild(stateGroup);

    if (compileTraversal)
        compileTraversal->compile(scenegraph);

    return scenegraph;
}

vsg::ref_ptr<vsg::Group> ShapeBuilder::createCoGSymbol(vsg::ref_ptr<vsg::MatrixTransform> transform) {
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
    double length = 1;
    vertices->set(0, vsg::vec3(0.0, 0.0, 0.0));
    vertices->set(1, vsg::vec3(1.0, 0.0, 0.0));
    colors->set(0, vsg::vec3(1.0, 0.0, 0.0));
    colors->set(1, vsg::vec3(1.0, 0.0, 0.0));
    vertices->set(2, vsg::vec3(0.0, 0.0, 0.0));
    vertices->set(3, vsg::vec3(0.0, 1.0, 0.0));
    colors->set(2, vsg::vec3(0.0, 1.0, 0.0));
    colors->set(3, vsg::vec3(0.0, 1.0, 0.0));
    vertices->set(4, vsg::vec3(0.0, 0.0, 0.0));
    vertices->set(5, vsg::vec3(0.0, 0.0, 1.0));
    colors->set(4, vsg::vec3(0.0, 0.0, 1.0));
    colors->set(5, vsg::vec3(0.0, 0.0, 1.0));

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
        vertices->set(i, vsg::vec3(pos.x(), pos.y(), pos.z()));
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
        vertices->set(i, vsg::vec3(pos.x(), pos.y(), pos.z()));
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
    int numPoints = ss->GetResolution();
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
    vsg::vec3 p1(0, length / 2, 0);
    vsg::vec3 p2(0, -length / 2, 0);
    vertices->set(0, p2);
    vertices->set(1, p1);
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

    const int numPoints = v.size();
    auto vertices = vsg::vec3Array::create(numPoints);
    auto colors = vsg::vec3Array::create(numPoints);
    auto cv = vsg::vec3(col.R, col.G, col.B);
    colors->set(0, cv);
    for (size_t i = 0; i < numPoints; i++) {
        vertices->set(i, vsg::vec3(v[i].x(), v[i].y(), v[i].z()));
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
                                 VK_SHADER_STAGE_FRAGMENT_BIT, vsg::vec3Value::create());
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

}  // namespace vsg3d
}  // namespace chrono
