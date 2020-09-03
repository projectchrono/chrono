#include "chrono_vsg/shapes/ChVSGCylinder.h"

#include "chrono_thirdparty/stb/stb_image.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono::vsg3d;

ChVSGCylinder::ChVSGCylinder() {}

vsg::ref_ptr<vsg::Node> ChVSGCylinder::createTexturedNode(vsg::vec4 color,
                                                          std::string texFilePath,
                                                          vsg::ref_ptr<vsg::MatrixTransform> transform) {
    // set up search paths to SPIRV shaders and textures
    vsg::ref_ptr<vsg::ShaderStage> vertexShader =
        readVertexShader(GetChronoDataFile("vsg/shaders/vert_PushConstants.spv"));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader =
        readFragmentShader(GetChronoDataFile("vsg/shaders/frag_PushConstants.spv"));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

    auto textureData = createRGBATexture(GetChronoDataFile(texFilePath));

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

    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    auto texture = vsg::DescriptorImage::create(vsg::Sampler::create(), textureData, 0, 0,
                                                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{texture});
    auto bindDescriptorSets = vsg::BindDescriptorSets::create(
        VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline->getPipelineLayout(), 0, vsg::DescriptorSets{descriptorSet});

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
    // Descriptors to decorate the whole graph
    auto subgraph = vsg::StateGroup::create();
    subgraph->add(bindGraphicsPipeline);
    subgraph->add(bindDescriptorSets);

    // set up vertex and index arrays
    auto vertices = vsg::vec3Array::create({
#include "cylinder_vertices.h"
    });

#if 0
    auto normals = vsg::vec3Array::create({
#include "cylinder_normals.h"
    });
#endif

    auto colors = vsg::vec3Array::create(vertices->size(), vsg::vec3(color.r, color.g, color.b));

    auto texcoords = vsg::vec2Array::create({
#include "cylinder_texcoords.h"
    });

    auto indices = vsg::ushortArray::create({
#include "cylinder_indices.h"
    });

    // VK_SHARING_MODE_EXCLUSIVE

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(
        0, vsg::DataList{vertices, colors, texcoords}));  // shader doesn't support normals yet..
    drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::DrawIndexed::create(indices->size(), 1, 0, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);
    subgraph->addChild(transform);

    return subgraph;
}

vsg::ref_ptr<vsg::Node> ChVSGCylinder::createPhongNode(vsg::vec3& lightPosition,
                                                       vsg::vec4 ambientColor,
                                                       vsg::vec4 diffuseColor,
                                                       vsg::vec4 specularColor,
                                                       float shininess,
                                                       float opacity,
                                                       vsg::ref_ptr<vsg::MatrixTransform> transform) {
    // set up search paths to SPIRV shaders and textures
    vsg::ref_ptr<vsg::ShaderStage> vertexShader = readVertexShader(GetChronoDataFile("vsg/shaders/vert_Phong.spv"));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = readFragmentShader(GetChronoDataFile("vsg/shaders/frag_Phong.spv"));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

    auto uniformValue = vsg::vec3Value::create(lightPosition);
    auto uniformBuffer = vsg::DescriptorBuffer::create(uniformValue, 0);
    vsg::DescriptorSetLayoutBindings lightSettingsDescriptorBindings{
        //{0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1,
         VK_SHADER_STAGE_FRAGMENT_BIT,  // we only need it in the fragment shader program
         nullptr}                       // { binding, descriptorTpe, descriptorCount, stageFlags, pImmutableSamplers}
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

    // auto pipelineLayout = vsg::PipelineLayout::create(descriptorSetLayouts, pushConstantRanges);

    auto pipelineLayout = vsg::PipelineLayout::create(
        vsg::DescriptorSetLayouts{descriptorSetLayout, lightSettingsDescriptorSetLayout}, pushConstantRanges);
    auto uniformBindDescriptorSet =
        vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, uniformDscriptorSet);

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // normal data
        VkVertexInputBindingDescription{2, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // ambient color data
        VkVertexInputBindingDescription{3, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // diffuse color data
        VkVertexInputBindingDescription{4, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // specular color data
        VkVertexInputBindingDescription{5, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX},      // shininess data
        VkVertexInputBindingDescription{6, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX},      // opacity data
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

    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of
    // Descriptors to decorate the whole graph
    auto subgraph = vsg::StateGroup::create();
    subgraph->add(bindGraphicsPipeline);
    // subgraph->add(bindDescriptorSets);

    // set up vertex and index arrays
    auto vertices = vsg::vec3Array::create({
#include "cylinder_vertices.h"
    });
    // Sphere Normals:
    auto normals = vsg::vec3Array::create({
#include "cylinder_normals.h"
    });

    auto ambientColors =
        vsg::vec3Array::create(vertices->size(), vsg::vec3(ambientColor.r, ambientColor.g, ambientColor.b));
    auto diffuseColors =
        vsg::vec3Array::create(vertices->size(), vsg::vec3(diffuseColor.r, diffuseColor.g, diffuseColor.b));
    auto specularColors =
        vsg::vec3Array::create(vertices->size(), vsg::vec3(specularColor.r, specularColor.g, specularColor.b));
    auto shininesses = vsg::floatArray::create(vertices->size(), shininess);
    auto opacities = vsg::floatArray::create(vertices->size(), opacity);

    auto indices = vsg::ushortArray::create({
#include "cylinder_indices.h"
    });

    // VK_SHARING_MODE_EXCLUSIVE

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(
        0, vsg::DataList{vertices, normals, ambientColors, diffuseColors, specularColors, shininesses, opacities}));
    drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::DrawIndexed::create(indices->size(), 1, 0, 0, 0));

    // add drawCommands to transform
    transform->addChild(uniformBindDescriptorSet);
    transform->addChild(drawCommands);
    subgraph->addChild(transform);

    return subgraph;
}
