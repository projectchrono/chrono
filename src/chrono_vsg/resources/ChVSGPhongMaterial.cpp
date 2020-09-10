#include "ChVSGPhongMaterial.h"

using namespace chrono::vsg3d;

ChVSGPhongMaterial::ChVSGPhongMaterial(PhongPresets mat) {
    switch (mat) {
        case PhongPresets::Emerald:
            ambientColor = vsg::vec3(0.0215, 0.1745, 0.0215);
            diffuseColor = vsg::vec3(0.07568, 0.61424, 0.07568);
            specularColor = vsg::vec3(0.633, 0.727811, 0.633);
            shininess = 0.6 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::Jade:
            ambientColor = vsg::vec3(0.135, 0.2225, 0.1575);
            diffuseColor = vsg::vec3(0.54, 0.89, 0.63);
            specularColor = vsg::vec3(0.316228, 0.316228, 0.316228);
            shininess = 0.1 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::Obsidian:
            ambientColor = vsg::vec3(0.05375, 0.05, 0.06625);
            diffuseColor = vsg::vec3(0.18275, 0.17, 0.22525);
            specularColor = vsg::vec3(0.332741, 0.328634, 0.346435);
            shininess = 0.3 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::Pearl:
            ambientColor = vsg::vec3(0.25, 0.20725, 0.20725);
            diffuseColor = vsg::vec3(1.0, 0.829, 0.829);
            specularColor = vsg::vec3(0.296648, 0.296648, 0.296648);
            shininess = 0.088 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::Ruby:
            ambientColor = vsg::vec3(0.1745, 0.01175, 0.01175);
            diffuseColor = vsg::vec3(0.61424, 0.04136, 0.04136);
            specularColor = vsg::vec3(0.727811, 0.626959, 0.626959);
            shininess = 0.6 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::Turquoise:
            ambientColor = vsg::vec3(0.1, 0.18725, 0.1745);
            diffuseColor = vsg::vec3(0.396, 0.74151, 0.69102);
            specularColor = vsg::vec3(0.297254, 0.30829, 0.306678);
            shininess = 0.1 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::Brass:
            ambientColor = vsg::vec3(0.329412, 0.223529, 0.027451);
            diffuseColor = vsg::vec3(0.780392, 0.568627, 0.113725);
            specularColor = vsg::vec3(0.992157, 0.941176, 0.807843);
            shininess = 0.21794872 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::Bronze:
            ambientColor = vsg::vec3(0.2125, 0.1275, 0.054);
            diffuseColor = vsg::vec3(0.714, 0.4284, 0.18144);
            specularColor = vsg::vec3(0.393548, 0.271906, 0.166721);
            shininess = 0.2 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::PolishedBronze:
            ambientColor = vsg::vec3(0.25, 0.148, 0.06475);
            diffuseColor = vsg::vec3(0.4, 0.2368, 0.1036);
            specularColor = vsg::vec3(0.774597, 0.458561, 0.200621);
            shininess = 76.8;
            opacity = 1.0;
            break;
        case PhongPresets::Chrome:
            ambientColor = vsg::vec3(0.25, 0.25, 0.25);
            diffuseColor = vsg::vec3(0.4, 0.4, 0.4);
            specularColor = vsg::vec3(0.774597, 0.774597, 0.774597);
            shininess = 0.6 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::Copper:
            ambientColor = vsg::vec3(0.19125, 0.0735, 0.0225);
            diffuseColor = vsg::vec3(0.7038, 0.27048, 0.0828);
            specularColor = vsg::vec3(0.256777, 0.137622, 0.086014);
            shininess = 0.1 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::PolishedCopper:
            ambientColor = vsg::vec3(0.2295, 0.08825, 0.0275);
            diffuseColor = vsg::vec3(0.5508, 0.2118, 0.066);
            specularColor = vsg::vec3(0.580594, 0.223257, 0.0695701);
            shininess = 51.2;
            opacity = 1.0;
            break;
        case PhongPresets::Gold:
            ambientColor = vsg::vec3(0.24725, 0.1995, 0.0745);
            diffuseColor = vsg::vec3(0.75164, 0.60648, 0.22648);
            specularColor = vsg::vec3(0.628281, 0.555802, 0.366065);
            shininess = 0.4 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::PolishedGold:
            ambientColor = vsg::vec3(0.24725, 0.2245, 0.0645);
            diffuseColor = vsg::vec3(0.34615, 0.3143, 0.0903);
            specularColor = vsg::vec3(0.797357, 0.723991, 0.208006);
            shininess = 83.2;
            opacity = 1.0;
            break;
        case PhongPresets::Pewter:
            ambientColor = vsg::vec3(0.105882, 0.058824, 0.113725);
            diffuseColor = vsg::vec3(0.427451, 0.470588, 0.541176);
            specularColor = vsg::vec3(0.333333, 0.333333, 0.521569);
            shininess = 9.84615;
            opacity = 1.0;
            break;
        case PhongPresets::Silver:
            ambientColor = vsg::vec3(0.19225, 0.19225, 0.19225);
            diffuseColor = vsg::vec3(0.50754, 0.50754, 0.50754);
            specularColor = vsg::vec3(0.508273, 0.508273, 0.508273);
            shininess = 0.4 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::PolishedSilver:
            ambientColor = vsg::vec3(0.23125, 0.23125, 0.23125);
            diffuseColor = vsg::vec3(0.2775, 0.2775, 0.2775);
            specularColor = vsg::vec3(0.773911, 0.773911, 0.773911);
            shininess = 89.6;
            opacity = 1.0;
            break;
        case PhongPresets::BlackPlastic:
            ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            diffuseColor = vsg::vec3(0.01, 0.01, 0.01);
            specularColor = vsg::vec3(0.50, 0.50, 0.50);
            shininess = 0.25 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::CyanPlastic:
            ambientColor = vsg::vec3(0.0, 0.1, 0.06);
            diffuseColor = vsg::vec3(0.0, 0.50980392, 0.50980392);
            specularColor = vsg::vec3(0.50196078, 0.50196078, 0.50196078);
            shininess = 0.25 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::GreenPlastic:
            ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            diffuseColor = vsg::vec3(0.1, 0.35, 0.1);
            specularColor = vsg::vec3(0.45, 0.55, 0.45);
            shininess = 0.25 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::RedPlastic:
            ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            diffuseColor = vsg::vec3(0.5, 0.0, 0.0);
            specularColor = vsg::vec3(0.7, 0.6, 0.6);
            shininess = 0.25 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::WhitePlastic:
            ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            diffuseColor = vsg::vec3(0.55, 0.55, 0.55);
            specularColor = vsg::vec3(0.70, 0.70, 0.70);
            shininess = 0.25 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::YellowPlastic:
            ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            diffuseColor = vsg::vec3(0.5, 0.5, 0.0);
            specularColor = vsg::vec3(0.60, 0.60, 0.50);
            shininess = 0.25 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::BluePlastic:
            ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            diffuseColor = vsg::vec3(0.0, 0.0, 0.5);
            specularColor = vsg::vec3(0.50, 0.50, 0.60);
            shininess = 0.25 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::BlackRubber:
            ambientColor = vsg::vec3(0.02, 0.02, 0.02);
            diffuseColor = vsg::vec3(0.01, 0.01, 0.01);
            specularColor = vsg::vec3(0.4, 0.4, 0.4);
            shininess = 0.078125 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::CyanRubber:
            ambientColor = vsg::vec3(0.0, 0.05, 0.05);
            diffuseColor = vsg::vec3(0.4, 0.5, 0.5);
            specularColor = vsg::vec3(0.04, 0.7, 0.7);
            shininess = 0.078125 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::GreenRubber:
            ambientColor = vsg::vec3(0.0, 0.05, 0.0);
            diffuseColor = vsg::vec3(0.4, 0.5, 0.4);
            specularColor = vsg::vec3(0.04, 0.7, 0.04);
            shininess = 0.078125 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::RedRubber:
            ambientColor = vsg::vec3(0.05, 0.0, 0.0);
            diffuseColor = vsg::vec3(0.5, 0.4, 0.4);
            specularColor = vsg::vec3(0.7, 0.04, 0.04);
            shininess = 0.078125 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::WhiteRubber:
            ambientColor = vsg::vec3(0.05, 0.05, 0.05);
            diffuseColor = vsg::vec3(0.5, 0.5, 0.5);
            specularColor = vsg::vec3(0.7, 0.7, 0.7);
            shininess = 0.078125 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::YellowRubber:
            ambientColor = vsg::vec3(0.05, 0.05, 0.0);
            diffuseColor = vsg::vec3(0.5, 0.5, 0.4);
            specularColor = vsg::vec3(0.7, 0.7, 0.04);
            shininess = 0.078125 * 128.0;
            opacity = 1.0;
            break;
        case PhongPresets::BlueRubber:
            ambientColor = vsg::vec3(0.0, 0.0, 0.05);
            diffuseColor = vsg::vec3(0.4, 0.4, 0.5);
            specularColor = vsg::vec3(0.4, 0.04, 0.07);
            shininess = 0.078125 * 128.0;
            opacity = 1.0;
            break;
    }
}
