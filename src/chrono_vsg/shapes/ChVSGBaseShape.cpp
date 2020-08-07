#include "chrono_vsg/shapes/ChVSGBaseShape.h"

#define STB_IMAGE_IMPLEMENTATION
#include "chrono_thirdparty/stb/stb_image.h"

using namespace chrono::vsg3d;

ChVSGBaseShape::ChVSGBaseShape() {}

void ChVSGBaseShape::compile(vsg::ref_ptr<vsg::Node> subgraph) {
    // std::cout << "Builder::compile(" << subgraph << ") _compile = " << _compile << std::endl;
    if (_compile) {
        subgraph->accept(*_compile);
        _compile->context.record();
        _compile->context.waitForCompletion();
    }
}

vsg::ref_ptr<vsg::ShaderStage> ChVSGBaseShape::readVertexShader(std::string filePath) {
    return vsg::ShaderStage::read(VK_SHADER_STAGE_VERTEX_BIT, "main", filePath);
}

vsg::ref_ptr<vsg::ShaderStage> ChVSGBaseShape::readFragmentShader(std::string filePath) {
    return vsg::ShaderStage::read(VK_SHADER_STAGE_FRAGMENT_BIT, "main", filePath);
}

vsg::ref_ptr<vsg::vec4Array2D> ChVSGBaseShape::createRGBATexture(std::string filePath) {
    // read texture image file with stb_image
    int texWidth, texHeight, texChannels;
    float* pixels = stbi_loadf(filePath.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

    vsg::ref_ptr<vsg::vec4Array2D> image = vsg::vec4Array2D::create(texWidth, texHeight, vsg::vec4(0, 0, 0, 0),
                                                                    vsg::Data::Layout{VK_FORMAT_R32G32B32A32_SFLOAT});
    if (pixels) {
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
    } else {
        GetLog() << "ChVSGBaseShape::createRGBATexture() - could not read texture file = " << filePath << "\n";
    }

    return image;
}