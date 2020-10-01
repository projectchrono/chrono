#include "ChVSGPbrMaterial.h"

using namespace chrono::vsg3d;

ChVSGPbrMaterial::ChVSGPbrMaterial(PbrPresets mat) {
    switch (mat) {
        case PbrPresets::TestMat:
            albedo = vsg::vec3(0.07568, 0.61424, 0.07568);
            metallic = 0.3;
            roughness = 0.6;
            ao = 0.04;
            break;
    }
}
