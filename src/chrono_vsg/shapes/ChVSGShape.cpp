#include "chrono_vsg/shapes/ChVSGShape.h"

#include "chrono_thirdparty/stb/stb_image.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono::vsg3d;

ChVSGShape::ChVSGShape() {}

vsg::ref_ptr<vsg::ShaderStage> ChVSGShape::readVertexShader(std::string filePath) {
    return vsg::ShaderStage::read(VK_SHADER_STAGE_VERTEX_BIT, "main", filePath);
}

vsg::ref_ptr<vsg::ShaderStage> ChVSGShape::readFragmentShader(std::string filePath) {
    return vsg::ShaderStage::read(VK_SHADER_STAGE_FRAGMENT_BIT, "main", filePath);
}

vsg::ref_ptr<vsg::vec4Array2D> ChVSGShape::createRGBATexture(
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
