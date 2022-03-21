#include "createQuad.h"

#include "chrono_vsg/resources/pushConstantShaders.h"

namespace chrono {
namespace vsg3d {

vsg::ref_ptr<vsg::Node> createQuad(const vsg::vec3& origin,
                                   const vsg::vec3& horizontal,
                                   const vsg::vec3& vertical,
                                   vsg::ref_ptr<vsg::Data> sourceData) {
    struct ConvertToRGBA : public vsg::Visitor {
        vsg::ref_ptr<vsg::Data> textureData;

        void apply(vsg::Data& data) override { textureData = &data; }

        void apply(vsg::uintArray2D& fa) override {
            // treat as a 24bit depth buffer
            float div = 1.0f / static_cast<float>(1 << 24);

            auto rgba =
                vsg::vec4Array2D::create(fa.width(), fa.height(), vsg::Data::Layout{VK_FORMAT_R32G32B32A32_SFLOAT});
            auto dest_itr = rgba->begin();
            for (auto& v : fa) {
                float m = static_cast<float>(v) * div;
                (*dest_itr++).set(m, m, m, 1.0);
            }
            textureData = rgba;
        }

        void apply(vsg::floatArray2D& fa) override {
            auto rgba =
                vsg::vec4Array2D::create(fa.width(), fa.height(), vsg::Data::Layout{VK_FORMAT_R32G32B32A32_SFLOAT});
            auto dest_itr = rgba->begin();
            for (auto& v : fa) {
                (*dest_itr++).set(v, v, v, 1.0);
            }
            textureData = rgba;
        }

        vsg::ref_ptr<vsg::Data> convert(vsg::ref_ptr<vsg::Data> data) {
            if (data)
                data->accept(*this);
            else {
                auto image = vsg::vec4Array2D::create(1, 1, vsg::Data::Layout{VK_FORMAT_R32G32B32A32_SFLOAT});
                image->set(0, 0, vsg::vec4(0.5f, 1.0f, 0.5f, 1.0f));
                textureData = image;
            }

            return textureData;
        }

    } convertToRGBA;

    auto textureData = convertToRGBA.convert(sourceData);
    if (!textureData)
        return {};

    // load shaders from memory
    auto vertexShader = vert_PushConstants();
    auto fragmentShader = frag_PushConstants();
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return {};
    }

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

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    auto graphicsPipeline =
        vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vertexShader, fragmentShader}, pipelineStates);
    auto bindGraphicsPipeline = vsg::BindGraphicsPipeline::create(graphicsPipeline);

    // create texture image and associated DescriptorSets and binding
    auto texture = vsg::DescriptorImage::create(vsg::Sampler::create(), textureData, 0, 0,
                                                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    auto descriptorSet = vsg::DescriptorSet::create(descriptorSetLayout, vsg::Descriptors{texture});
    auto bindDescriptorSets = vsg::BindDescriptorSets::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0,
                                                              vsg::DescriptorSets{descriptorSet});

    // create StateGroup as the root of the scene/command graph to hold the GraphicsProgram, and binding of Descriptors
    // to decorate the whole graph
    auto scenegraph = vsg::StateGroup::create();
    scenegraph->add(bindGraphicsPipeline);
    scenegraph->add(bindDescriptorSets);

    // set up model transformation node
    auto transform = vsg::MatrixTransform::create();  // VK_SHADER_STAGE_VERTEX_BIT

    // add transform to root of the scene graph
    scenegraph->addChild(transform);

    // set up vertex and index arrays
    auto vertices =
        vsg::vec3Array::create({origin, origin + horizontal, origin + horizontal + vertical,
                                origin + vertical});  // VK_FORMAT_R32G32B32_SFLOAT, VK_VERTEX_INPUT_RATE_INSTANCE,
                                                      // VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_SHARING_MODE_EXCLUSIVE

    auto colors =
        vsg::vec3Array::create({{1.0f, 1.0f, 1.0f},
                                {1.0f, 1.0f, 1.0f},
                                {1.0f, 1.0f, 1.0f},
                                {1.0f, 1.0f, 1.0f}});  // VK_FORMAT_R32G32B32_SFLOAT, VK_VERTEX_INPUT_RATE_VERTEX,
                                                       // VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_SHARING_MODE_EXCLUSIVE

    bool top_left = textureData->getLayout().origin == vsg::TOP_LEFT;  // in Vulkan the origin is by default top left.
    float left = 0.0f;
    float right = 1.0f;
    float top = top_left ? 0.0f : 1.0f;
    float bottom = top_left ? 1.0f : 0.0f;
    auto texcoords =
        vsg::vec2Array::create({{left, bottom},
                                {right, bottom},
                                {right, top},
                                {left, top}});  // VK_FORMAT_R32G32_SFLOAT, VK_VERTEX_INPUT_RATE_VERTEX,
                                                // VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_SHARING_MODE_EXCLUSIVE

    auto indices =
        vsg::ushortArray::create({0, 1, 2, 2, 3, 0});  // VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_SHARING_MODE_EXCLUSIVE

    // setup geometry
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices, colors, texcoords}));
    drawCommands->addChild(vsg::BindIndexBuffer::create(indices));
    drawCommands->addChild(vsg::DrawIndexed::create(6, 1, 0, 0, 0));

    // add drawCommands to transform
    transform->addChild(drawCommands);

    return scenegraph;
}
}  // namespace vsg3d
}  // namespace chrono
