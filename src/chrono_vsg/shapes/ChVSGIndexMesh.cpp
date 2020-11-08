#include "chrono_vsg/shapes/ChVSGIndexMesh.h"

#include "chrono_thirdparty/stb/stb_image.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono::vsg3d;

ChVSGIndexMesh::ChVSGIndexMesh(std::shared_ptr<ChBody> body,
                               std::shared_ptr<ChAsset> asset,
                               vsg::ref_ptr<vsg::MatrixTransform> transform)
    : m_matMode(MaterialMode::Unknown), m_bodyPtr(body), m_assetPtr(asset), m_transform(transform) {}

vsg::ref_ptr<vsg::ShaderStage> ChVSGIndexMesh::readVertexShader(std::string filePath) {
    return vsg::ShaderStage::read(VK_SHADER_STAGE_VERTEX_BIT, "main", filePath);
}

vsg::ref_ptr<vsg::ShaderStage> ChVSGIndexMesh::readFragmentShader(std::string filePath) {
    return vsg::ShaderStage::read(VK_SHADER_STAGE_FRAGMENT_BIT, "main", filePath);
}

vsg::ref_ptr<vsg::vec4Array2D> ChVSGIndexMesh::createRGBATexture(
    std::string filePath) {  // read texture image file with stb_image

    vsg::ref_ptr<vsg::vec4Array2D> image;
    filesystem::path testPath(filePath);

    if (testPath.exists() && testPath.is_file()) {
        GetLog() << "texture file '" << filePath << " exists.\n";
        int texWidth = -1, texHeight = -1, texChannels;
        float* pixels = stbi_loadf(filePath.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
        GetLog() << "Breite=" << texWidth << "Höhe=" << texHeight << "\n";
        image = vsg::vec4Array2D::create(texWidth, texHeight, vsg::vec4(0, 0, 0, 0),
                                         vsg::Data::Layout{VK_FORMAT_R32G32B32A32_SFLOAT});
        if (pixels && texWidth > 0 && texHeight > 0) {
            int k = 0;
            for (int j = 0; j < texHeight; j++) {
                for (int i = 0; i < texWidth; i++) {
                    float r = pixels[k++];
                    float g = pixels[k++];
                    float b = pixels[k++];
                    float a = pixels[k++];
                    vsg::vec4 col(r, g, b, a);
                    image->set(i, j, col);
                }
            }
            // release now obsolete pixel buffer
            stbi_image_free(pixels);
        }
    } else {
        GetLog() << "texture file '" << filePath << " not found.\n";
        image = vsg::vec4Array2D::create(2, 2, vsg::vec4(1, 1, 0, 0), vsg::Data::Layout{VK_FORMAT_R32G32B32A32_SFLOAT});
        image->set(0, 0, vsg::vec4(0, 0, 1, 0));
        image->set(1, 1, vsg::vec4(0, 0, 1, 0));
    }
    return image;
}

vsg::ref_ptr<vsg::Node> ChVSGIndexMesh::createVSGNode() {
    auto subgraph = vsg::StateGroup::create();
    subgraph->setValue("bodyPtr", m_bodyPtr);
    subgraph->setValue("assetPtr", m_assetPtr);
    subgraph->setValue("transform", m_transform);

    switch (m_matMode) {
        case MaterialMode::Unknown:
            break;
        case MaterialMode::Textured:
            genTexturedSubgraph(subgraph);
            break;
        case MaterialMode::SimplePhong:
            genSimplePhongSubgraph(subgraph);
            break;
        case MaterialMode::MappedPBR:
            GetLog() << "Mapped PBR does not work actually for unknown reasons. Don't rely on it!\n";
            genMappedPBRSubgraph(subgraph);
            break;
        case MaterialMode::PBR:
            GetLog() << "PBR does not work actually for unknown reasons. Don't rely on it!\n";
            genPBRSubgraph(subgraph);
            break;
    }

    return subgraph;
}

void ChVSGIndexMesh::genTexturedSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph) {
    vsg::ref_ptr<vsg::ShaderStage> vertexShader = readVertexShader(GetChronoDataFile("vsg/shaders/vert_Textured.spv"));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader =
        readFragmentShader(GetChronoDataFile("vsg/shaders/frag_Textured.spv"));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return;
    }

    // read texture image
    auto textureData = createRGBATexture(m_textureFilePath);
    if (!textureData) {
        std::cout << "Could not read texture file : " << m_textureFilePath << std::endl;
        return;
    }

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr},  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
        {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}   // tex coord data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32_SFLOAT, 0},     // tex coord data
    };

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        vsg::RasterizationState::create(),
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    auto texture = vsg::DescriptorImage::create(vsg::Sampler::create(), textureData, 0, 0,
                                                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto uniformValue = vsg::vec3Value::create(1.0f, 2.0f, 3.0f);
    auto uniform = vsg::DescriptorBuffer::create(uniformValue, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{texture, uniform});
    auto bindDescriptorSets = vsg::BindDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->layout,
                                                              0, vsg::DescriptorSets{descriptorSet});

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
    // Descriptors to decorate the whole graph auto scenegraph = vsg::StateGroup::create();
    subgraph->add(bindGraphicsPipeline);
    subgraph->add(bindDescriptorSets);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create();  // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    subgraph->addChild(m_transform);

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{m_vertices, m_texcoords}));
    drawCommands->addChild(vsg::BindIndexBuffer::create(m_indices));
    drawCommands->addChild(vsg::DrawIndexed::create(m_indices->size(), 1, 0, 0, 0));

    // add drawCommands to transform
    m_transform->addChild(drawCommands);
}

//=======================================================================================

void ChVSGIndexMesh::genSimplePhongSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph) {
    vsg::ref_ptr<vsg::ShaderStage> vertexShader =
        readVertexShader(GetChronoDataFile("vsg/shaders/vert_SimplePhong.spv"));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader =
        readFragmentShader(GetChronoDataFile("vsg/shaders/frag_SimplePhong.spv"));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return;
    }

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
        {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // color data
        VkVertexInputBindingDescription{2, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // normal data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},  // color data
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32B32_SFLOAT, 0},  // normal data
    };

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        vsg::RasterizationState::create(),
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    auto uniformValue1 = vsg::vec3Value::create(m_lightPosition);
    auto uniform1 = vsg::DescriptorBuffer::create(uniformValue1, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{uniform1});
    auto bindDescriptorSets = vsg::BindDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->layout,
                                                              0, vsg::DescriptorSets{descriptorSet});

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
    // Descriptors to decorate the whole graph auto scenegraph = vsg::StateGroup::create();
    subgraph->add(bindGraphicsPipeline);
    subgraph->add(bindDescriptorSets);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create();  // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    subgraph->addChild(m_transform);

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{m_vertices, m_colors, m_normals}));
    drawCommands->addChild(vsg::BindIndexBuffer::create(m_indices));
    drawCommands->addChild(vsg::DrawIndexed::create(m_indices->size(), 1, 0, 0, 0));

    // add drawCommands to transform
    m_transform->addChild(drawCommands);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ChVSGIndexMesh::genMappedPBRSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph) {
    vsg::ref_ptr<vsg::ShaderStage> vertexShader =
        readVertexShader(GetChronoDataFile("vsg/testshaders/vert_MappedPBR.spv"));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader =
        readFragmentShader(GetChronoDataFile("vsg/testshaders/frag_MappedPBR.spv"));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return;
    }

    // read texture images
    auto albedoData = createRGBATexture(m_albedoMapPath);
    if (!albedoData) {
        std::cout << "Could not read texture file : " << m_albedoMapPath << std::endl;
        return;
    }
    auto normalData = createRGBATexture(m_normalMapPath);
    if (!normalData) {
        std::cout << "Could not read texture file : " << m_normalMapPath << std::endl;
        return;
    }
    auto metallicData = createRGBATexture(m_metallicMapPath);
    if (!metallicData) {
        std::cout << "Could not read texture file : " << m_metallicMapPath << std::endl;
        return;
    }
    auto roughnessData = createRGBATexture(m_roughnessMapPath);
    if (!roughnessData) {
        std::cout << "Could not read texture file : " << m_roughnessMapPath << std::endl;
        return;
    }
    auto aoData = createRGBATexture(m_aoMapPath);
    if (!aoData) {
        std::cout << "Could not read texture file : " << m_aoMapPath << std::endl;
        return;
    }

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr},  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
        {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr},  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
        {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr},  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
        {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr},  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
        {4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr},  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // normal data
        VkVertexInputBindingDescription{2, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}   // tex coord data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},  // normal data
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32_SFLOAT, 0},     // tex coord data
    };

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        vsg::RasterizationState::create(),
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    auto albedo = vsg::DescriptorImage::create(vsg::Sampler::create(), albedoData, 0, 0,
                                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    // create texture image and associated DescriptorSets and binding
    auto normal = vsg::DescriptorImage::create(vsg::Sampler::create(), normalData, 1, 0,
                                               VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    // create texture image and associated DescriptorSets and binding
    auto metallic = vsg::DescriptorImage::create(vsg::Sampler::create(), metallicData, 2, 0,
                                                 VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    // create texture image and associated DescriptorSets and binding
    auto roughness = vsg::DescriptorImage::create(vsg::Sampler::create(), roughnessData, 3, 0,
                                                  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    // create texture image and associated DescriptorSets and binding
    auto ao =
        vsg::DescriptorImage::create(vsg::Sampler::create(), aoData, 4, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet =
        vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{albedo, normal, metallic, roughness, ao});
    auto bindDescriptorSets = vsg::BindDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->layout,
                                                              0, vsg::DescriptorSets{descriptorSet});

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
    // Descriptors to decorate the whole graph auto scenegraph = vsg::StateGroup::create();
    subgraph->add(bindGraphicsPipeline);
    subgraph->add(bindDescriptorSets);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create();  // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    subgraph->addChild(m_transform);

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{m_vertices, m_texcoords}));
    drawCommands->addChild(vsg::BindIndexBuffer::create(m_indices));
    drawCommands->addChild(vsg::DrawIndexed::create(m_indices->size(), 1, 0, 0, 0));

    // add drawCommands to transform
    m_transform->addChild(drawCommands);
}

//=======================================================================================

void ChVSGIndexMesh::genPBRSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph) {
    vsg::ref_ptr<vsg::ShaderStage> vertexShader = readVertexShader(GetChronoDataFile("vsg/shaders/vert_PBR.spv"));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = readFragmentShader(GetChronoDataFile("vsg/shaders/frag_PBR.spv"));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return;
    }

    // set up graphics pipeline
    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
        {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
         nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
    };

    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);

    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // normal data
        VkVertexInputBindingDescription{2, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // color data
        VkVertexInputBindingDescription{3, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX},      // metallic data
        VkVertexInputBindingDescription{4, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX},      // roughness data
        VkVertexInputBindingDescription{5, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX},      // ao data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},  // normal data
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32B32_SFLOAT, 0},  // color data
        VkVertexInputAttributeDescription{3, 3, VK_FORMAT_R32_SFLOAT, 0},        // metallic data
        VkVertexInputAttributeDescription{4, 4, VK_FORMAT_R32_SFLOAT, 0},        // roughness data
        VkVertexInputAttributeDescription{5, 5, VK_FORMAT_R32_SFLOAT, 0},        // ao data
    };

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        vsg::RasterizationState::create(),
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    auto uniformValue1 = vsg::vec3Value::create(m_lightPosition);
    auto uniform1 = vsg::DescriptorBuffer::create(uniformValue1, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{uniform1});
    auto bindDescriptorSets = vsg::BindDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->layout,
                                                              0, vsg::DescriptorSets{descriptorSet});

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
    // Descriptors to decorate the whole graph auto scenegraph = vsg::StateGroup::create();
    subgraph->add(bindGraphicsPipeline);
    subgraph->add(bindDescriptorSets);

    // set up model transformation node
    // auto transform = vsg::MatrixTransform::create();  // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    subgraph->addChild(m_transform);

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(
        0, vsg::DataList{m_vertices, m_normals, m_colors, m_metallics, m_roughnesses, m_aos}));
    drawCommands->addChild(vsg::BindIndexBuffer::create(m_indices));
    drawCommands->addChild(vsg::DrawIndexed::create(m_indices->size(), 1, 0, 0, 0));

    // add drawCommands to transform
    m_transform->addChild(drawCommands);
}
