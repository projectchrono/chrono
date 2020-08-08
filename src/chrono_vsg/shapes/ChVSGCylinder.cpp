#include "chrono_vsg/shapes/ChVSGCylinder.h"

using namespace chrono::vsg3d;

ChVSGCylinder::ChVSGCylinder() {}

vsg::ref_ptr<vsg::Node> ChVSGCylinder::createTexturedNode(vsg::vec4 color,
                                                          vsg::ref_ptr<vsg::MatrixTransform> transform) {
    vsg::ref_ptr<vsg::ShaderStage> vertexShader =
        readVertexShader(GetChronoDataFile("vsg/shaders/vert_PushConstants.spv"));
    vsg::ref_ptr<vsg::ShaderStage> fragmentShader =
        readFragmentShader(GetChronoDataFile("vsg/shaders/frag_PushConstants.spv"));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

    auto textureData = createRGBATexture(GetChronoDataFile("vsg/textures/Marble010.jpg"));

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
    auto vertices = vsg::vec3Array::create({{1, 0, -1},
                                            {1, 0, 1},
                                            {0.980785, -0.19509, -1},
                                            {0.980785, -0.19509, 1},
                                            {0.92388, -0.382683, -1},
                                            {0.92388, -0.382683, 1},
                                            {0.83147, -0.55557, -1},
                                            {0.83147, -0.55557, 1},
                                            {0.707107, -0.707107, -1},
                                            {0.707107, -0.707107, 1},
                                            {0.55557, -0.83147, -1},
                                            {0.55557, -0.83147, 1},
                                            {0.382683, -0.923879, -1},
                                            {0.382683, -0.923879, 1},
                                            {0.19509, -0.980785, -1},
                                            {0.19509, -0.980785, 1},
                                            {0, -1, -1},
                                            {0, -1, 1},
                                            {-0.19509, -0.980785, -1},
                                            {-0.19509, -0.980785, 1},
                                            {-0.382683, -0.923879, -1},
                                            {-0.382683, -0.923879, 1},
                                            {-0.55557, -0.83147, -1},
                                            {-0.55557, -0.83147, 1},
                                            {-0.707107, -0.707107, -1},
                                            {-0.707107, -0.707107, 1},
                                            {-0.83147, -0.55557, -1},
                                            {-0.83147, -0.55557, 1},
                                            {-0.92388, -0.382683, -1},
                                            {-0.92388, -0.382683, 1},
                                            {-0.980785, -0.19509, -1},
                                            {-0.980785, -0.19509, 1},
                                            {-1, 0, -1},
                                            {-1, 0, 1},
                                            {-0.980785, 0.195091, -1},
                                            {-0.980785, 0.195091, 1},
                                            {-0.923879, 0.382684, -1},
                                            {-0.923879, 0.382684, 1},
                                            {-0.831469, 0.555571, -1},
                                            {-0.831469, 0.555571, 1},
                                            {-0.707106, 0.707107, -1},
                                            {-0.707106, 0.707107, 1},
                                            {-0.55557, 0.83147, -1},
                                            {-0.55557, 0.83147, 1},
                                            {-0.382683, 0.92388, -1},
                                            {-0.382683, 0.92388, 1},
                                            {-0.195089, 0.980785, -1},
                                            {-0.195089, 0.980785, 1},
                                            {1e-06, 1, -1},
                                            {1e-06, 1, 1},
                                            {0.195091, 0.980785, -1},
                                            {0.195091, 0.980785, 1},
                                            {0.382684, 0.923879, -1},
                                            {0.382684, 0.923879, 1},
                                            {0.555571, 0.831469, -1},
                                            {0.555571, 0.831469, 1},
                                            {0.707108, 0.707106, -1},
                                            {0.707108, 0.707106, 1},
                                            {0.83147, 0.555569, -1},
                                            {0.83147, 0.555569, 1},
                                            {0.92388, 0.382682, -1},
                                            {0.92388, 0.382682, 1},
                                            {0.980786, 0.195089, -1},
                                            {0.980786, 0.195089, 1}});

    auto colors = vsg::vec4Array::create(vertices->size(), color);

    auto texcoords = vsg::vec2Array::create({{1, 1},
                                             {0.96875, 0.5},
                                             {1, 0.5},
                                             {0.96875, 1},
                                             {0.9375, 0.5},
                                             {0.9375, 1},
                                             {0.90625, 0.5},
                                             {0.90625, 1},
                                             {0.875, 0.5},
                                             {0.875, 1},
                                             {0.84375, 0.5},
                                             {0.84375, 1},
                                             {0.8125, 0.5},
                                             {0.8125, 1},
                                             {0.78125, 0.5},
                                             {0.78125, 1},
                                             {0.75, 0.5},
                                             {0.75, 1},
                                             {0.71875, 0.5},
                                             {0.71875, 1},
                                             {0.6875, 0.5},
                                             {0.6875, 1},
                                             {0.65625, 0.5},
                                             {0.65625, 1},
                                             {0.625, 0.5},
                                             {0.625, 1},
                                             {0.59375, 0.5},
                                             {0.59375, 1},
                                             {0.5625, 0.5},
                                             {0.5625, 1},
                                             {0.53125, 0.5},
                                             {0.53125, 1},
                                             {0.5, 0.5},
                                             {0.5, 1},
                                             {0.46875, 0.5},
                                             {0.46875, 1},
                                             {0.4375, 0.5},
                                             {0.4375, 1},
                                             {0.40625, 0.5},
                                             {0.40625, 1},
                                             {0.375, 0.5},
                                             {0.375, 1},
                                             {0.34375, 0.5},
                                             {0.34375, 1},
                                             {0.3125, 0.5},
                                             {0.3125, 1},
                                             {0.28125, 0.5},
                                             {0.28125, 1},
                                             {0.25, 0.5},
                                             {0.25, 1},
                                             {0.21875, 0.5},
                                             {0.21875, 1},
                                             {0.1875, 0.5},
                                             {0.1875, 1},
                                             {0.15625, 0.5},
                                             {0.15625, 1},
                                             {0.125, 0.5},
                                             {0.125, 1},
                                             {0.09375, 0.5},
                                             {0.09375, 1},
                                             {0.0625, 0.5},
                                             {0.028269, 0.341844},
                                             {0.158156, 0.028269},
                                             {0.471731, 0.158156},
                                             {0.0625, 1},
                                             {0.03125, 0.5},
                                             {0.03125, 1},
                                             {0, 0.5},
                                             {0.796822, 0.014612},
                                             {0.514611, 0.203179},
                                             {0.703179, 0.485389},
                                             {0.341844, 0.471731},
                                             {0.296822, 0.485388},
                                             {0.25, 0.49},
                                             {0.203179, 0.485389},
                                             {0.158156, 0.471731},
                                             {0.116663, 0.449553},
                                             {0.080295, 0.419706},
                                             {0.050447, 0.383337},
                                             {0.014612, 0.296822},
                                             {0.01, 0.25},
                                             {0.014611, 0.203179},
                                             {0.028269, 0.158156},
                                             {0.050447, 0.116663},
                                             {0.080294, 0.080294},
                                             {0.116663, 0.050447},
                                             {0.203178, 0.014612},
                                             {0.25, 0.01},
                                             {0.296822, 0.014612},
                                             {0.341844, 0.028269},
                                             {0.383337, 0.050447},
                                             {0.419706, 0.080294},
                                             {0.449553, 0.116663},
                                             {0.485388, 0.203178},
                                             {0.49, 0.25},
                                             {0.485388, 0.296822},
                                             {0.471731, 0.341844},
                                             {0.449553, 0.383337},
                                             {0.419706, 0.419706},
                                             {0.383337, 0.449553},
                                             {0, 1},
                                             {0.75, 0.49},
                                             {0.796822, 0.485388},
                                             {0.841844, 0.471731},
                                             {0.883337, 0.449553},
                                             {0.919706, 0.419706},
                                             {0.949553, 0.383337},
                                             {0.971731, 0.341844},
                                             {0.985388, 0.296822},
                                             {0.99, 0.25},
                                             {0.985388, 0.203178},
                                             {0.971731, 0.158156},
                                             {0.949553, 0.116663},
                                             {0.919706, 0.080294},
                                             {0.883337, 0.050447},
                                             {0.841844, 0.028269},
                                             {0.75, 0.01},
                                             {0.703178, 0.014612},
                                             {0.658156, 0.028269},
                                             {0.616663, 0.050447},
                                             {0.580294, 0.080294},
                                             {0.550447, 0.116663},
                                             {0.528269, 0.158156},
                                             {0.51, 0.25},
                                             {0.514612, 0.296822},
                                             {0.528269, 0.341844},
                                             {0.550447, 0.383337},
                                             {0.580295, 0.419706},
                                             {0.616663, 0.449553},
                                             {0.658156, 0.471731}});

    auto indices = vsg::ushortArray::create(
        {1,  2,  0,  3,  4,  2,  5,  6,  4,  7,  8,  6,  9,  10, 8,  11, 12, 10, 13, 14, 12, 15, 16, 14, 17, 18, 16,
         19, 20, 18, 21, 22, 20, 23, 24, 22, 25, 26, 24, 27, 28, 26, 29, 30, 28, 31, 32, 30, 33, 34, 32, 35, 36, 34,
         37, 38, 36, 39, 40, 38, 41, 42, 40, 43, 44, 42, 45, 46, 44, 47, 48, 46, 49, 50, 48, 51, 52, 50, 53, 54, 52,
         55, 56, 54, 57, 58, 56, 59, 60, 58, 53, 37, 21, 61, 62, 60, 63, 0,  62, 30, 46, 62, 1,  3,  2,  3,  5,  4,
         5,  7,  6,  7,  9,  8,  9,  11, 10, 11, 13, 12, 13, 15, 14, 15, 17, 16, 17, 19, 18, 19, 21, 20, 21, 23, 22,
         23, 25, 24, 25, 27, 26, 27, 29, 28, 29, 31, 30, 31, 33, 32, 33, 35, 34, 35, 37, 36, 37, 39, 38, 39, 41, 40,
         41, 43, 42, 43, 45, 44, 45, 47, 46, 47, 49, 48, 49, 51, 50, 51, 53, 52, 53, 55, 54, 55, 57, 56, 57, 59, 58,
         59, 61, 60, 5,  3,  1,  1,  63, 5,  63, 61, 5,  61, 59, 57, 57, 55, 61, 55, 53, 61, 53, 51, 49, 49, 47, 53,
         47, 45, 53, 45, 43, 37, 43, 41, 37, 41, 39, 37, 37, 35, 33, 33, 31, 29, 29, 27, 25, 25, 23, 21, 21, 19, 17,
         17, 15, 21, 15, 13, 21, 13, 11, 5,  11, 9,  5,  9,  7,  5,  37, 33, 21, 33, 29, 21, 29, 25, 21, 5,  61, 53,
         53, 45, 37, 21, 13, 5,  5,  53, 21, 61, 63, 62, 63, 1,  0,  62, 0,  2,  2,  4,  6,  6,  8,  10, 10, 12, 6,
         12, 14, 6,  14, 16, 18, 18, 20, 14, 20, 22, 14, 22, 24, 26, 26, 28, 30, 30, 32, 34, 34, 36, 30, 36, 38, 30,
         38, 40, 42, 42, 44, 46, 46, 48, 50, 50, 52, 46, 52, 54, 46, 54, 56, 58, 58, 60, 62, 62, 2,  6,  22, 26, 30,
         38, 42, 46, 54, 58, 46, 58, 62, 46, 62, 6,  14, 14, 22, 30, 30, 38, 46, 62, 14, 30});

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
