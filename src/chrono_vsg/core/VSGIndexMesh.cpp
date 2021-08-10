#include "chrono_vsg/core/VSGIndexMesh.h"
#include "chrono_vsg/core/vsg_vertex_shader.h"
#include "chrono_vsg/core/vsg_phong_fragment_shader.h"
#include "chrono_vsg/core/vsg_pbr_fragment_shader.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono::vsg3d;

static std::string processGLSLShaderSource(const std::string& source, const std::vector<std::string>& defines) {
    // trim leading spaces/tabs
    auto trimLeading = [](std::string& str) {
        size_t startpos = str.find_first_not_of(" \t");
        if (std::string::npos != startpos) {
            str = str.substr(startpos);
        }
    };

    // trim trailing spaces/tabs/newlines
    auto trimTrailing = [](std::string& str) {
        size_t endpos = str.find_last_not_of(" \t\n");
        if (endpos != std::string::npos) {
            str = str.substr(0, endpos + 1);
        }
    };

    // sanitise line by triming leading and trailing characters
    auto sanitise = [&trimLeading, &trimTrailing](std::string& str) {
        trimLeading(str);
        trimTrailing(str);
    };

    // return true if str starts with match string
    auto startsWith = [](const std::string& str, const std::string& match) {
        return str.compare(0, match.length(), match) == 0;
    };

    // returns the string between the start and end character
    auto stringBetween = [](const std::string& str, const char& startChar, const char& endChar) {
        auto start = str.find_first_of(startChar);
        if (start == std::string::npos)
            return std::string();

        auto end = str.find_first_of(endChar, start);
        if (end == std::string::npos)
            return std::string();

        if ((end - start) - 1 == 0)
            return std::string();

        return str.substr(start + 1, (end - start) - 1);
    };

    auto split = [](const std::string& str, const char& seperator) {
        std::vector<std::string> elements;

        std::string::size_type prev_pos = 0, pos = 0;

        while ((pos = str.find(seperator, pos)) != std::string::npos) {
            auto substring = str.substr(prev_pos, pos - prev_pos);
            elements.push_back(substring);
            prev_pos = ++pos;
        }

        elements.push_back(str.substr(prev_pos, pos - prev_pos));

        return elements;
    };

    auto addLine = [](std::ostringstream& ss, const std::string& line) { ss << line << "\n"; };

    std::istringstream iss(source);
    std::ostringstream headerstream;
    std::ostringstream sourcestream;

    const std::string versionmatch = "#version";
    const std::string importdefinesmatch = "#pragma import_defines";

    std::vector<std::string> finaldefines;

    for (std::string line; std::getline(iss, line);) {
        std::string sanitisedline = line;
        sanitise(sanitisedline);

        // is it the version
        if (startsWith(sanitisedline, versionmatch)) {
            addLine(headerstream, line);
        }
        // is it the defines import
        else if (startsWith(sanitisedline, importdefinesmatch)) {
            // get the import defines between ()
            auto csv = stringBetween(sanitisedline, '(', ')');
            auto importedDefines = split(csv, ',');

            addLine(headerstream, line);

            // loop the imported defines and see if it's also requested in defines, if so insert a define line
            for (auto importedDef : importedDefines) {
                auto sanitisedImportDef = importedDef;
                sanitise(sanitisedImportDef);

                auto finditr = std::find(defines.begin(), defines.end(), sanitisedImportDef);
                if (finditr != defines.end()) {
                    addLine(headerstream, "#define " + sanitisedImportDef);
                }
            }
        } else {
            // standard source line
            addLine(sourcestream, line);
        }
    }

    return headerstream.str() + sourcestream.str();
}

VSGIndexMesh::VSGIndexMesh(std::shared_ptr<ChBody> body,
                           std::shared_ptr<ChAsset> geometryAsset,
                           vsg::ref_ptr<vsg::MatrixTransform> transform,
                           bool polygonFilled)
    : m_matMode(MaterialMode::Unknown),
      m_bodyPtr(body),
      m_assetPtr(geometryAsset),
      m_transform(transform),
      m_polygonFilled(polygonFilled) {}

vsg::ref_ptr<vsg::Node> VSGIndexMesh::createVSGNode() {
    auto subgraph = vsg::StateGroup::create();
    subgraph->setValue("bodyPtr", m_bodyPtr);
    subgraph->setValue("assetPtr", m_assetPtr);
    subgraph->setValue("transform", m_transform);

    switch (m_matMode) {
        case MaterialMode::Unknown:
            break;
        case MaterialMode::Phong:
            genPhongSubgraph(subgraph);
            break;
        case MaterialMode::PBR:
            genPBRSubgraph(subgraph);
            break;
    }
    return subgraph;
}

vsg::ref_ptr<vsg::GraphicsPipeline> VSGIndexMesh::createPipeline(
    vsg::ref_ptr<vsg::ShaderStage> vs,
    vsg::ref_ptr<vsg::ShaderStage> fs,
    vsg::ref_ptr<vsg::DescriptorSetLayout> descriptorSetLayout,
    bool doubleSided,
    bool enableBlend) {
    vsg::PushConstantRanges pushConstantRanges{
        {VK_SHADER_STAGE_VERTEX_BIT, 0, 128}  // projection view, and model matrices, actual push constant calls
                                              // autoaatically provided by the VSG's DispatchTraversal
    };

    vsg::VertexInputState::Bindings vertexBindingsDescriptions{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // vertex data
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},  // normal data
        VkVertexInputBindingDescription{2, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}   // texcoord data
    };

    vsg::VertexInputState::Attributes vertexAttributeDescriptions{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},  // vertex data
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},  // normal data
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32_SFLOAT, 0}      // texcoord data
    };

    auto rasterState = vsg::RasterizationState::create();
    rasterState->cullMode = doubleSided ? VK_CULL_MODE_NONE : VK_CULL_MODE_BACK_BIT;

    auto colorBlendState = vsg::ColorBlendState::create();
    colorBlendState->attachments = vsg::ColorBlendState::ColorBlendAttachments{
        {enableBlend, VK_BLEND_FACTOR_SRC_ALPHA, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
         VK_BLEND_FACTOR_SRC_ALPHA, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_SUBTRACT,
         VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};

    vsg::GraphicsPipelineStates pipelineStates{
        vsg::VertexInputState::create(vertexBindingsDescriptions, vertexAttributeDescriptions),
        vsg::InputAssemblyState::create(),
        rasterState,
        vsg::MultisampleState::create(),
        colorBlendState,
        vsg::DepthStencilState::create()};

    auto pipelineLayout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout}, pushConstantRanges);
    return vsg::GraphicsPipeline::create(pipelineLayout, vsg::ShaderStages{vs, fs}, pipelineStates);
}

void VSGIndexMesh::genPhongSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph) {
    auto vertexShader =
        vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", processGLSLShaderSource(vsg_vertex_shader, {}));
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main",
                                                   processGLSLShaderSource(vsg_phong_fragment_shader, {}));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return;
    }

    vsg::DescriptorSetLayoutBindings descriptorBindings{
        {10, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};

    auto defaultPipeline =
        createPipeline(vertexShader, fragmentShader, vsg::DescriptorSetLayout::create(descriptorBindings), false, true);

    // create texture image and associated DescriptorSets and binding
    PhongMaterial mat;
    auto material = vsg::DescriptorBuffer::create(mat.toData(), 10);

    auto descriptorSet =
        vsg::DescriptorSet::create(defaultPipeline->layout->setLayouts.front(), vsg::Descriptors{material});
    auto defaultState =
        vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, defaultPipeline->layout, 0, descriptorSet);

    subgraph->add(vsg::BindGraphicsPipeline::create(defaultPipeline));
    subgraph->add(defaultState);
    auto stategroup = vsg::StateGroup::create();
    stategroup->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{m_vertices, m_normals, m_texcoords}));
    stategroup->addChild(vsg::BindIndexBuffer::create(m_indices));
    stategroup->addChild(vsg::DrawIndexed::create(static_cast<uint32_t>(m_indices->size()), 1, 0, 0, 0));
    m_transform->addChild(stategroup);
    subgraph->addChild(m_transform);
}

void VSGIndexMesh::genPBRSubgraph(vsg::ref_ptr<vsg::StateGroup> subgraph) {
    auto vertexShader =
        vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", processGLSLShaderSource(vsg_vertex_shader, {}));
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main",
                                                   processGLSLShaderSource(vsg_pbr_fragment_shader, {}));
    if (!vertexShader || !fragmentShader) {
        std::cout << "Could not create shaders." << std::endl;
        return;
    }
}
