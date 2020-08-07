#include "chrono_vsg/shapes/ChVSGCylinder.h"

using namespace chrono::vsg3d;

ChVSGCylinder::ChVSGCylinder() {}

vsg::ref_ptr<vsg::Node> ChVSGCylinder::createTexturedNode(vsg::vec4 color,
                                                          vsg::ref_ptr<vsg::MatrixTransform> transform) {
    vsg::ref_ptr<vsg::ShaderStage> vertexShader = vsg::ShaderStage::read(
        VK_SHADER_STAGE_VERTEX_BIT, "main", GetChronoDataFile("vsg/shaders/vert_PushConstants.spv"));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader = vsg::ShaderStage::read(
        VK_SHADER_STAGE_FRAGMENT_BIT, "main", GetChronoDataFile("vsg/shaders/frag_PushConstants.spv"));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

    auto textureData = createRGBATexture(GetChronoDataFile("bluwhite.png"));

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
    auto vertices = vsg::vec3Array::create({{0, 0, 0.5},
                                            {0, 0, -0.5},
                                            {1, 0, -0.5},
                                            {0.965926, 0.258819, -0.5},
                                            {0.866025, 0.5, -0.5},
                                            {0.707107, 0.707107, -0.5},
                                            {0.5, 0.866025, -0.5},
                                            {0.258819, 0.965926, -0.5},
                                            {6.12323e-17, 1, -0.5},
                                            {-0.258819, 0.965926, -0.5},
                                            {-0.5, 0.866025, -0.5},
                                            {-0.707107, 0.707107, -0.5},
                                            {-0.866025, 0.5, -0.5},
                                            {-0.965926, 0.258819, -0.5},
                                            {-1, 1.22465e-16, -0.5},
                                            {-0.965926, -0.258819, -0.5},
                                            {-0.866025, -0.5, -0.5},
                                            {-0.707107, -0.707107, -0.5},
                                            {-0.5, -0.866025, -0.5},
                                            {-0.258819, -0.965926, -0.5},
                                            {-1.83697e-16, -1, -0.5},
                                            {0.258819, -0.965926, -0.5},
                                            {0.5, -0.866025, -0.5},
                                            {0.707107, -0.707107, -0.5},
                                            {0.866025, -0.5, -0.5},
                                            {0.965926, -0.258819, -0.5},
                                            {1, -2.44929e-16, -0.5},
                                            {1, 0, 0.5},
                                            {0.965926, 0.258819, 0.5},
                                            {0.866025, 0.5, 0.5},
                                            {0.707107, 0.707107, 0.5},
                                            {0.5, 0.866025, 0.5},
                                            {0.258819, 0.965926, 0.5},
                                            {6.12323e-17, 1, 0.5},
                                            {-0.258819, 0.965926, 0.5},
                                            {-0.5, 0.866025, 0.5},
                                            {-0.707107, 0.707107, 0.5},
                                            {-0.866025, 0.5, 0.5},
                                            {-0.965926, 0.258819, 0.5},
                                            {-1, 1.22465e-16, 0.5},
                                            {-0.965926, -0.258819, 0.5},
                                            {-0.866025, -0.5, 0.5},
                                            {-0.707107, -0.707107, 0.5},
                                            {-0.5, -0.866025, 0.5},
                                            {-0.258819, -0.965926, 0.5},
                                            {-1.83697e-16, -1, 0.5},
                                            {0.258819, -0.965926, 0.5},
                                            {0.5, -0.866025, 0.5},
                                            {0.707107, -0.707107, 0.5},
                                            {0.866025, -0.5, 0.5},
                                            {0.965926, -0.258819, 0.5},
                                            {1, -2.44929e-16, 0.5},
                                            {1, 0, -0.5},
                                            {0.965926, 0.258819, -0.5},
                                            {0.866025, 0.5, -0.5},
                                            {0.707107, 0.707107, -0.5},
                                            {0.5, 0.866025, -0.5},
                                            {0.258819, 0.965926, -0.5},
                                            {6.12323e-17, 1, -0.5},
                                            {-0.258819, 0.965926, -0.5},
                                            {-0.5, 0.866025, -0.5},
                                            {-0.707107, 0.707107, -0.5},
                                            {-0.866025, 0.5, -0.5},
                                            {-0.965926, 0.258819, -0.5},
                                            {-1, 1.22465e-16, -0.5},
                                            {-0.965926, -0.258819, -0.5},
                                            {-0.866025, -0.5, -0.5},
                                            {-0.707107, -0.707107, -0.5},
                                            {-0.5, -0.866025, -0.5},
                                            {-0.258819, -0.965926, -0.5},
                                            {-1.83697e-16, -1, -0.5},
                                            {0.258819, -0.965926, -0.5},
                                            {0.5, -0.866025, -0.5},
                                            {0.707107, -0.707107, -0.5},
                                            {0.866025, -0.5, -0.5},
                                            {0.965926, -0.258819, -0.5},
                                            {1, -2.44929e-16, -0.5},
                                            {1, 0, 0.5},
                                            {0.965926, 0.258819, 0.5},
                                            {0.866025, 0.5, 0.5},
                                            {0.707107, 0.707107, 0.5},
                                            {0.5, 0.866025, 0.5},
                                            {0.258819, 0.965926, 0.5},
                                            {6.12323e-17, 1, 0.5},
                                            {-0.258819, 0.965926, 0.5},
                                            {-0.5, 0.866025, 0.5},
                                            {-0.707107, 0.707107, 0.5},
                                            {-0.866025, 0.5, 0.5},
                                            {-0.965926, 0.258819, 0.5},
                                            {-1, 1.22465e-16, 0.5},
                                            {-0.965926, -0.258819, 0.5},
                                            {-0.866025, -0.5, 0.5},
                                            {-0.707107, -0.707107, 0.5},
                                            {-0.5, -0.866025, 0.5},
                                            {-0.258819, -0.965926, 0.5},
                                            {-1.83697e-16, -1, 0.5},
                                            {0.258819, -0.965926, 0.5},
                                            {0.5, -0.866025, 0.5},
                                            {0.707107, -0.707107, 0.5},
                                            {0.866025, -0.5, 0.5},
                                            {0.965926, -0.258819, 0.5},
                                            {1, -2.44929e-16, 0.5}});

    auto colors = vsg::vec4Array::create(vertices->size(), color);

    auto texcoords = vsg::vec2Array::create({{0.5, 0.5},
                                             {0.5, 0.5},
                                             {1, 0.5},
                                             {0.982963, 0.62941},
                                             {0.933013, 0.75},
                                             {0.853553, 0.853553},
                                             {0.75, 0.933013},
                                             {0.62941, 0.982963},
                                             {0.5, 1},
                                             {0.37059, 0.982963},
                                             {0.25, 0.933013},
                                             {0.146447, 0.853553},
                                             {0.0669873, 0.75},
                                             {0.0170371, 0.62941},
                                             {0, 0.5},
                                             {0.0170371, 0.37059},
                                             {0.0669873, 0.25},
                                             {0.146447, 0.146447},
                                             {0.25, 0.0669873},
                                             {0.37059, 0.0170371},
                                             {0.5, 0},
                                             {0.62941, 0.0170371},
                                             {0.75, 0.0669873},
                                             {0.853553, 0.146447},
                                             {0.933013, 0.25},
                                             {0.982963, 0.37059},
                                             {1, 0.5},
                                             {1, 0},
                                             {0.5, 0},
                                             {0.982963, 0},
                                             {0.62941, 0},
                                             {0.933013, 0},
                                             {0.75, 0},
                                             {0.853553, 0},
                                             {0.853553, 0},
                                             {0.75, 0},
                                             {0.933013, 0},
                                             {0.62941, 0},
                                             {0.982963, 0},
                                             {0.5, 0},
                                             {1, 0},
                                             {0.37059, 0},
                                             {0.982963, 0},
                                             {0.25, 0},
                                             {0.933013, 0},
                                             {0.146447, 0},
                                             {0.853553, 0},
                                             {0.0669873, 0},
                                             {0.75, 0},
                                             {0.0170371, 0},
                                             {0.62941, 0},
                                             {0, 0},
                                             {0.5, 1},
                                             {0.0170371, 1},
                                             {0.37059, 1},
                                             {0.0669873, 1},
                                             {0.25, 1},
                                             {0.146447, 1},
                                             {0.146447, 1},
                                             {0.25, 1},
                                             {0.0669873, 1},
                                             {0.37059, 1},
                                             {0.0170371, 1},
                                             {0.5, 1},
                                             {0, 1},
                                             {0.62941, 1},
                                             {0.0170371, 1},
                                             {0.75, 1},
                                             {0.0669873, 1},
                                             {0.853553, 1},
                                             {0.146447, 1},
                                             {0.933013, 1},
                                             {0.25, 1},
                                             {0.982963, 1},
                                             {0.37059, 1},
                                             {1, 1},
                                             {0.5, 1},
                                             {0, 0},
                                             {0.0416667, 0},
                                             {0.0833333, 0},
                                             {0.125, 0},
                                             {0.166667, 0},
                                             {0.208333, 0},
                                             {0.25, 0},
                                             {0.291667, 0},
                                             {0.333333, 0},
                                             {0.375, 0},
                                             {0.416667, 0},
                                             {0.458333, 0},
                                             {0.5, 0},
                                             {0.541667, 0},
                                             {0.583333, 0},
                                             {0.625, 0},
                                             {0.666667, 0},
                                             {0.708333, 0},
                                             {0.75, 0},
                                             {0.791667, 0},
                                             {0.833333, 0},
                                             {0.875, 0},
                                             {0.916667, 0},
                                             {0.958333, 0},
                                             {1, 0},
                                             {0, 0},
                                             {0.0416667, 0},
                                             {0.0833333, 0},
                                             {0.125, 0},
                                             {0.166667, 0},
                                             {0.208333, 0},
                                             {0.25, 0},
                                             {0.291667, 0},
                                             {0.333333, 0},
                                             {0.375, 0},
                                             {0.416667, 0},
                                             {0.458333, 0},
                                             {0.5, 0},
                                             {0.541667, 0},
                                             {0.583333, 0},
                                             {0.625, 0},
                                             {0.666667, 0},
                                             {0.708333, 0},
                                             {0.75, 0},
                                             {0.791667, 0},
                                             {0.833333, 0},
                                             {0.875, 0},
                                             {0.916667, 0},
                                             {0.958333, 0},
                                             {1, 0}});

    auto indices = vsg::ushortArray::create(
        {1, 3,  2,  1, 4,  3,  1, 5,  4,  1, 6,  5,  1, 7,  6,  1, 8,  7,  1, 9,  8,  1, 10, 9,  1, 11, 10, 1, 12, 11,
         1, 13, 12, 1, 14, 13, 1, 15, 14, 1, 16, 15, 1, 17, 16, 1, 18, 17, 1, 19, 18, 1, 20, 19, 1, 21, 20, 1, 22, 21,
         1, 23, 22, 1, 24, 23, 1, 25, 24, 1, 2,  25, 1, 2,  26, 0, 27, 28, 0, 28, 29, 0, 29, 30, 0, 30, 31, 0, 31, 32,
         0, 32, 33, 0, 33, 34, 0, 34, 35, 0, 35, 36, 0, 36, 37, 0, 37, 38, 0, 38, 39, 0, 39, 40, 0, 40, 41, 0, 41, 42,
         0, 42, 43, 0, 43, 44, 0, 44, 45, 0, 45, 46, 0, 46, 47, 0, 47, 48, 0, 48, 49, 0, 49, 50, 0, 50, 27, 0, 51, 27});

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
