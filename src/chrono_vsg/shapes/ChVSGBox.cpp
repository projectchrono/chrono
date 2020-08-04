#include "chrono_vsg/shapes/ChVSGBox.h"

#define STB_IMAGE_IMPLEMENTATION
#include "chrono_thirdparty/stb/stb_image.h"

using namespace chrono::vsg3d;

ChVSGBox::ChVSGBox() {}

void ChVSGBox::compile(vsg::ref_ptr<vsg::Node> subgraph) {
    // std::cout << "Builder::compile(" << subgraph << ") _compile = " << _compile << std::endl;
    if (_compile) {
        subgraph->accept(*_compile);
        _compile->context.record();
        _compile->context.waitForCompletion();
    }
}

vsg::ref_ptr<vsg::Node> ChVSGBox::createTexturedNode(vsg::vec4 color, vsg::ref_ptr<vsg::MatrixTransform> transform) {
    // set up search paths to SPIRV shaders and textures
    vsg::Paths searchPaths = vsg::getEnvPaths("VSG_FILE_PATH");

    vsg::ref_ptr<vsg::ShaderStage> vertexShader = vsg::ShaderStage::read(
        VK_SHADER_STAGE_VERTEX_BIT, "main", vsg::findFile("shaders/vert_PushConstants.spv", searchPaths));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = vsg::ShaderStage::read(
        VK_SHADER_STAGE_FRAGMENT_BIT, "main", vsg::findFile("shaders/frag_PushConstants.spv", searchPaths));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

    /* read texture image based on vsgb file
    vsg::Path textureFile("vsg/textures/Metal010.vsgb");
    auto textureData = vsg::read_cast<vsg::Data>(vsg::findFile(textureFile, searchPaths));

    if (!textureData) {
        std::cout << "Could not read texture file : " << textureFile << ", we replace it by an default image."
                  << std::endl;
        auto image = vsg::vec4Array2D::create(2, 2, color, vsg::Data::Layout{VK_FORMAT_R32G32B32A32_SFLOAT});
        image->set(0, 0, {0.0f, 0.0f, 1.0f, 1.0f});
        image->set(1, 1, {0.0f, 0.0f, 1.0f, 1.0f});
        textureData = image;
    } */
    // read texture image file with stb_image

    int texWidth, texHeight, texChannels;
    // float* pixels = stbi_loadf("/Volumes/Ramdisk/build-chrono/data/bluwhite.png", &texWidth, &texHeight,
    // &texChannels,
    //                           STBI_rgb_alpha);
    float* pixels = stbi_loadf(vsg::findFile("bluwhite.png", searchPaths).c_str(), &texWidth, &texHeight, &texChannels,
                               STBI_rgb_alpha);

    VkDeviceSize imageSize = texWidth * texHeight * 4;
    GetLog() << "Image size = " << texWidth << " * " << texWidth << " * " << texChannels << "\n";
    auto image = vsg::vec4Array2D::create(texWidth, texHeight, color, vsg::Data::Layout{VK_FORMAT_R32G32B32A32_SFLOAT});
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
    auto textureData = image;
    stbi_image_free(pixels);

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
        VkVertexInputBindingDescription{1, sizeof(vsg::vec4), VK_VERTEX_INPUT_RATE_VERTEX},  // colour data
        VkVertexInputBindingDescription{2, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}   // tex coord data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},     // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 0},  // colour data
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32_SFLOAT, 0},        // tex coord data
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

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    scenegraph->add(bindGraphicsPipeline);
    scenegraph->add(bindDescriptorSets);

    // set up vertex and index arrays
    auto vertices = vsg::vec3Array::create(
        {{1, -1, -1},  {1, 1, -1},  {1, 1, 1},  {1, -1, 1},  {-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1}, {-1, 1, 1},
         {-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1},  {-1, 1, -1},  {-1, 1, 1},  {1, 1, 1},
         {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1},  {1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}});

#if 0
    auto normals = vsg::vec3Array::create({{1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {1, 0, 0},  {-1, 0, 0}, {-1, 0, 0},
                                           {-1, 0, 0}, {-1, 0, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0},
                                           {0, 1, 0},  {0, 1, 0},  {0, 1, 0},  {0, 1, 0},  {0, 0, 1},  {0, 0, 1},
                                           {0, 0, 1},  {0, 0, 1},  {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}});
#endif

    auto colors = vsg::vec4Array::create(vertices->size(), color);

    auto texcoords = vsg::vec2Array::create({{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}, {0, 1},
                                             {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}, {0, 1},
                                             {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {1, 0}, {1, 1}, {0, 1}});

    auto indices = vsg::ushortArray::create({0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
                                             12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23});

    // VK_SHARING_MODE_EXCLUSIVE

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(
        0, vsg::DataList{vertices, colors, texcoords}));  // shader doesn't support normals yet..
    drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::DrawIndexed::create(indices->size(), 1, 0, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);
    scenegraph->addChild(transform);

    compile(scenegraph);

    return scenegraph;
}
