#include "chrono_vsg/shapes/ChVSGIndexedMesh.h"

#include "chrono_thirdparty/stb/stb_image.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono::vsg3d;

ChVSGIndexedMesh::ChVSGIndexedMesh() {}

vsg::ref_ptr<vsg::ShaderStage> ChVSGIndexedMesh::readVertexShader(std::string filePath) {
    return vsg::ShaderStage::read(VK_SHADER_STAGE_VERTEX_BIT, "main", filePath);
}

vsg::ref_ptr<vsg::ShaderStage> ChVSGIndexedMesh::readFragmentShader(std::string filePath) {
    return vsg::ShaderStage::read(VK_SHADER_STAGE_FRAGMENT_BIT, "main", filePath);
}

vsg::ref_ptr<vsg::vec4Array2D> ChVSGIndexedMesh::createRGBATexture(
    std::string filePath) {  // read texture image file with stb_image

    vsg::ref_ptr<vsg::vec4Array2D> image;
    filesystem::path testPath(filePath);

    if (testPath.exists() && testPath.is_file()) {
        GetLog() << "texture file '" << filePath << " exists.\n";
        int texWidth = -1, texHeight = -1, texChannels;
        float* pixels = stbi_loadf(filePath.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

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

vsg::ref_ptr<vsg::Node> ChVSGIndexedMesh::createVSGNode(DrawMode drawMode,
                                                        vsg::ref_ptr<vsg::MatrixTransform> transform) {
    auto subgraph = vsg::StateGroup::create();
    switch (drawMode) {
        case DrawMode::Textured: {
            // set up search paths to SPIRV shaders and textures
            vsg::ref_ptr<vsg::ShaderStage> vertexShader =
                readVertexShader(GetChronoDataFile("vsg/shaders/vert_PushConstants.spv"));
            vsg::ref_ptr<vsg::ShaderStage> fragmentShader =
                readFragmentShader(GetChronoDataFile("vsg/shaders/frag_PushConstants.spv"));
            if (!vertexShader || !fragmentShader) {
                std::cout << "Could not create shaders." << std::endl;
                return {};
            }

            auto textureData = createRGBATexture(m_textureFilePath);
            // set up graphics pipeline
            vsg::DescriptorSetLayoutBindings descriptorBindings{
                {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
                 nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
            };

            auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);
            vsg::DescriptorSetLayouts descriptorSetLayouts{descriptorSetLayout};

            vsg::PushConstantRanges pushConstantRanges{
                {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                                      // autoaatically provided by the VSG's DispatchTraversal
            };

            auto pipelineLayout = vsg::PipelineLayout::create(descriptorSetLayouts, pushConstantRanges);

            vsg::VertexInputState::Bindings vertexBindingsDescriptions{
                VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
                VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // colour data
                VkVertexInputBindingDescription{2, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}   // tex coord data
            };

            vsg::VertexInputState::Attributes vertexAttributeDescriptions{
                VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
                VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},  // colour data
                VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32_SFLOAT, 0},     // tex coord data
            };

            vsg::GraphicsPipelineStates pipelineStates{
                vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
                vsg::InputAssemblyState::create(),
                vsg::RasterizationState::create(),
                vsg::MultisampleState::create(),
                vsg::ColorBlendState::create(),
                vsg::DepthStencilState::create()};

            auto graphicsPipeline = vsg::GraphicsPipeline::create(
                pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
            auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

            // create texture image and associated DescriptorSets and binding
            auto texture = vsg::DescriptorImage::create(vsg::Sampler::create(), textureData, 0, 0,
                                                        VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

            auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{texture});
            auto bindDescriptorSets =
                vsg::BindDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->getPipelineLayout(),
                                                0, vsg::DescriptorSets{descriptorSet});

            // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
            // Descriptors to decorate the whole graph
            auto subgraph = vsg::StateGroup::create();
            subgraph->add(bindGraphicsPipeline);
            subgraph->add(bindDescriptorSets);

            // setup geometry
            auto drawCommands = vsg::Commands::create();
            drawCommands->addChild(vsg::BindVertexBuffers::create(
                0, vsg::DataList{m_vertices, m_diffuseColor, m_texcoords}));  // shader doesn't support normals yet..
            drawCommands->addChild(vsg::BindIndexBuffer::create(m_indices));
            drawCommands->addChild(vsg::DrawIndexed::create(m_indices->size(), 1, 0, 0, 0));

            // add drawCommands to transform
            transform->addChild(drawCommands);

        } break;
        case DrawMode::Phong: {
            // set up search paths to SPIRV shaders and textures
            vsg::ref_ptr<vsg::ShaderStage> vertexShader =
                readVertexShader(GetChronoDataFile("vsg/shaders/vert_Phong.spv"));
            vsg::ref_ptr<vsg::ShaderStage> fragmentShader =
                readFragmentShader(GetChronoDataFile("vsg/shaders/frag_Phong.spv"));
            if (!vertexShader || !fragmentShader) {
                std::cout << "Could not create shaders." << std::endl;
                return {};
            }

            auto uniformValue = vsg::vec3Value::create(m_lightPosition);
            auto uniformBuffer = vsg::DescriptorBuffer::create(uniformValue, 0);
            vsg::DescriptorSetLayoutBindings lightSettingsDescriptorBindings{
                //{0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
                 VK_SHADER_STAGE_FRAGMENT_BIT,  // we only need it in the fragment shader program
                 nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
            };

            auto lightSettingsDescriptorSetLayout = vsg::DescriptorSetLayout::create(lightSettingsDescriptorBindings);

            auto uniformDscriptorSet =
                vsg::DescriptorSet::create(lightSettingsDescriptorSetLayout, vsg::Descriptors{uniformBuffer});

            vsg::DescriptorSetLayoutBindings descriptorBindings{
                {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT,
                 nullptr}  // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
            };

            auto descriptorSetLayout = vsg::DescriptorSetLayout::create(descriptorBindings);
            vsg::DescriptorSetLayouts descriptorSetLayouts{descriptorSetLayout};

            vsg::PushConstantRanges pushConstantRanges{
                {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                                      // autoatically provided by the VSG's DispatchTraversal
            };

            auto pipelineLayout = vsg::PipelineLayout::create(
                vsg::DescriptorSetLayouts{descriptorSetLayout, lightSettingsDescriptorSetLayout}, pushConstantRanges);
            auto uniformBindDescriptorSet =
                vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, uniformDscriptorSet);

            vsg::VertexInputState::Bindings vertexBindingsDescriptions{
                VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
                VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // normal data
                VkVertexInputBindingDescription{2, sizeof(vsg::vec3),
                                                VK_VERTEX_INPUT_RATE_VERTEX},  // ambient color data
                VkVertexInputBindingDescription{3, sizeof(vsg::vec3),
                                                VK_VERTEX_INPUT_RATE_VERTEX},  // diffuse color data
                VkVertexInputBindingDescription{4, sizeof(vsg::vec3),
                                                VK_VERTEX_INPUT_RATE_VERTEX},                    // specular color data
                VkVertexInputBindingDescription{5, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX},  // shininess data
                VkVertexInputBindingDescription{6, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX},  // opacity data
            };

            vsg::VertexInputState::Attributes vertexAttributeDescriptions{
                VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
                VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},  // normal data
                VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32B32_SFLOAT, 0},  // ambient color data
                VkVertexInputAttributeDescription{3, 3, VK_FORMAT_R32G32B32_SFLOAT, 0},  // diffuse color data
                VkVertexInputAttributeDescription{4, 4, VK_FORMAT_R32G32B32_SFLOAT, 0},  // specular color data
                VkVertexInputAttributeDescription{5, 5, VK_FORMAT_R32_SFLOAT, 0},        // shininess data
                VkVertexInputAttributeDescription{6, 6, VK_FORMAT_R32_SFLOAT, 0},        // opacity data
            };

            vsg::GraphicsPipelineStates pipelineStates{
                vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
                vsg::InputAssemblyState::create(),
                vsg::RasterizationState::create(),
                vsg::MultisampleState::create(),
                vsg::ColorBlendState::create(),
                vsg::DepthStencilState::create()};

            auto graphicsPipeline = vsg::GraphicsPipeline::create(
                pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
            auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

            // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
            // Descriptors to decorate the whole graph
            subgraph->add(bindGraphicsPipeline);
            // subgraph->add(bindDescriptorSets);

            // setup geometry
            auto drawCommands = vsg::Commands::create();
            drawCommands->addChild(
                vsg::BindVertexBuffers::create(0, vsg::DataList{m_vertices, m_normals, m_ambientColor, m_diffuseColor,
                                                                m_specularColor, m_shininess, m_opacity}));
            drawCommands->addChild(vsg::BindIndexBuffer::create(m_indices));
            drawCommands->addChild(vsg::DrawIndexed::create(m_indices->size(), 1, 0, 0, 0));

            // add drawCommands to transform
            transform->addChild(uniformBindDescriptorSet);
            transform->addChild(drawCommands);

        } break;
    }
    subgraph->addChild(transform);

    return subgraph;
}