#include "ChVSGPhongMaterial.h"

using namespace chrono::vsg3d;

ChVSGPhongMaterial::ChVSGPhongMaterial(VSG_PHONG_MATERIAL mat) {
    switch (mat) {
        case VSG_PHONG_MAT_EMERALD:
            m_ambientColor = vsg::vec3(0.0215, 0.1745, 0.0215);
            m_diffuseColor = vsg::vec3(0.07568, 0.61424, 0.07568);
            m_specularColor = vsg::vec3(0.633, 0.727811, 0.633);
            m_shininess = 0.6 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_JADE:
            m_ambientColor = vsg::vec3(0.135, 0.2225, 0.1575);
            m_diffuseColor = vsg::vec3(0.54, 0.89, 0.63);
            m_specularColor = vsg::vec3(0.316228, 0.316228, 0.316228);
            m_shininess = 0.1 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_OBSIDIAN:
            m_ambientColor = vsg::vec3(0.05375, 0.05, 0.06625);
            m_diffuseColor = vsg::vec3(0.18275, 0.17, 0.22525);
            m_specularColor = vsg::vec3(0.332741, 0.328634, 0.346435);
            m_shininess = 0.3 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_PEARL:
            m_ambientColor = vsg::vec3(0.25, 0.20725, 0.20725);
            m_diffuseColor = vsg::vec3(1.0, 0.829, 0.829);
            m_specularColor = vsg::vec3(0.296648, 0.296648, 0.296648);
            m_shininess = 0.088 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_RUBY:
            m_ambientColor = vsg::vec3(0.1745, 0.01175, 0.01175);
            m_diffuseColor = vsg::vec3(0.61424, 0.04136, 0.04136);
            m_specularColor = vsg::vec3(0.727811, 0.626959, 0.626959);
            m_shininess = 0.6 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_TURQUOISE:
            m_ambientColor = vsg::vec3(0.1, 0.18725, 0.1745);
            m_diffuseColor = vsg::vec3(0.396, 0.74151, 0.69102);
            m_specularColor = vsg::vec3(0.297254, 0.30829, 0.306678);
            m_shininess = 0.1 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_BRASS:
            m_ambientColor = vsg::vec3(0.329412, 0.223529, 0.027451);
            m_diffuseColor = vsg::vec3(0.780392, 0.568627, 0.113725);
            m_specularColor = vsg::vec3(0.992157, 0.941176, 0.807843);
            m_shininess = 0.21794872 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_BRONZE:
            m_ambientColor = vsg::vec3(0.2125, 0.1275, 0.054);
            m_diffuseColor = vsg::vec3(0.714, 0.4284, 0.18144);
            m_specularColor = vsg::vec3(0.393548, 0.271906, 0.166721);
            m_shininess = 0.2 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_POLISHED_BRONZE:
            m_ambientColor = vsg::vec3(0.25, 0.148, 0.06475);
            m_diffuseColor = vsg::vec3(0.4, 0.2368, 0.1036);
            m_specularColor = vsg::vec3(0.774597, 0.458561, 0.200621);
            m_shininess = 76.8;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_CHROME:
            m_ambientColor = vsg::vec3(0.25, 0.25, 0.25);
            m_diffuseColor = vsg::vec3(0.4, 0.4, 0.4);
            m_specularColor = vsg::vec3(0.774597, 0.774597, 0.774597);
            m_shininess = 0.6 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_COPPER:
            m_ambientColor = vsg::vec3(0.19125, 0.0735, 0.0225);
            m_diffuseColor = vsg::vec3(0.7038, 0.27048, 0.0828);
            m_specularColor = vsg::vec3(0.256777, 0.137622, 0.086014);
            m_shininess = 0.1 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_POLISHED_COPPER:
            m_ambientColor = vsg::vec3(0.2295, 0.08825, 0.0275);
            m_diffuseColor = vsg::vec3(0.5508, 0.2118, 0.066);
            m_specularColor = vsg::vec3(0.580594, 0.223257, 0.0695701);
            m_shininess = 51.2;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_GOLD:
            m_ambientColor = vsg::vec3(0.24725, 0.1995, 0.0745);
            m_diffuseColor = vsg::vec3(0.75164, 0.60648, 0.22648);
            m_specularColor = vsg::vec3(0.628281, 0.555802, 0.366065);
            m_shininess = 0.4 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_POLISHED_GOLD:
            m_ambientColor = vsg::vec3(0.24725, 0.2245, 0.0645);
            m_diffuseColor = vsg::vec3(0.34615, 0.3143, 0.0903);
            m_specularColor = vsg::vec3(0.797357, 0.723991, 0.208006);
            m_shininess = 83.2;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_PEWTER:
            m_ambientColor = vsg::vec3(0.105882, 0.058824, 0.113725);
            m_diffuseColor = vsg::vec3(0.427451, 0.470588, 0.541176);
            m_specularColor = vsg::vec3(0.333333, 0.333333, 0.521569);
            m_shininess = 9.84615;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_SILVER:
            m_ambientColor = vsg::vec3(0.19225, 0.19225, 0.19225);
            m_diffuseColor = vsg::vec3(0.50754, 0.50754, 0.50754);
            m_specularColor = vsg::vec3(0.508273, 0.508273, 0.508273);
            m_shininess = 0.4 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_POLISHED_SILVER:
            m_ambientColor = vsg::vec3(0.23125, 0.23125, 0.23125);
            m_diffuseColor = vsg::vec3(0.2775, 0.2775, 0.2775);
            m_specularColor = vsg::vec3(0.773911, 0.773911, 0.773911);
            m_shininess = 89.6;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_BLACK_PLASTIC:
            m_ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            m_diffuseColor = vsg::vec3(0.01, 0.01, 0.01);
            m_specularColor = vsg::vec3(0.50, 0.50, 0.50);
            m_shininess = 0.25 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_CYAN_PLASTIC:
            m_ambientColor = vsg::vec3(0.0, 0.1, 0.06);
            m_diffuseColor = vsg::vec3(0.0, 0.50980392, 0.50980392);
            m_specularColor = vsg::vec3(0.50196078, 0.50196078, 0.50196078);
            m_shininess = 0.25 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_GREEN_PLASTIC:
            m_ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            m_diffuseColor = vsg::vec3(0.1, 0.35, 0.1);
            m_specularColor = vsg::vec3(0.45, 0.55, 0.45);
            m_shininess = 0.25 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_RED_PLASTIC:
            m_ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            m_diffuseColor = vsg::vec3(0.5, 0.0, 0.0);
            m_specularColor = vsg::vec3(0.7, 0.6, 0.6);
            m_shininess = 0.25 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_WHITE_PLASTIC:
            m_ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            m_diffuseColor = vsg::vec3(0.55, 0.55, 0.55);
            m_specularColor = vsg::vec3(0.70, 0.70, 0.70);
            m_shininess = 0.25 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_YELLOW_PLASTIC:
            m_ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            m_diffuseColor = vsg::vec3(0.5, 0.5, 0.0);
            m_specularColor = vsg::vec3(0.60, 0.60, 0.50);
            m_shininess = 0.25 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_BLUE_PLASTIC:
            m_ambientColor = vsg::vec3(0.0, 0.0, 0.0);
            m_diffuseColor = vsg::vec3(0.0, 0.0, 0.5);
            m_specularColor = vsg::vec3(0.50, 0.50, 0.60);
            m_shininess = 0.25 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_BLACK_RUBBER:
            m_ambientColor = vsg::vec3(0.02, 0.02, 0.02);
            m_diffuseColor = vsg::vec3(0.01, 0.01, 0.01);
            m_specularColor = vsg::vec3(0.4, 0.4, 0.4);
            m_shininess = 0.078125 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_CYAN_RUBBER:
            m_ambientColor = vsg::vec3(0.0, 0.05, 0.05);
            m_diffuseColor = vsg::vec3(0.4, 0.5, 0.5);
            m_specularColor = vsg::vec3(0.04, 0.7, 0.7);
            m_shininess = 0.078125 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_GREEN_RUBBER:
            m_ambientColor = vsg::vec3(0.0, 0.05, 0.0);
            m_diffuseColor = vsg::vec3(0.4, 0.5, 0.4);
            m_specularColor = vsg::vec3(0.04, 0.7, 0.04);
            m_shininess = 0.078125 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_RED_RUBBER:
            m_ambientColor = vsg::vec3(0.05, 0.0, 0.0);
            m_diffuseColor = vsg::vec3(0.5, 0.4, 0.4);
            m_specularColor = vsg::vec3(0.7, 0.04, 0.04);
            m_shininess = 0.078125 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_WHITE_RUBBER:
            m_ambientColor = vsg::vec3(0.05, 0.05, 0.05);
            m_diffuseColor = vsg::vec3(0.5, 0.5, 0.5);
            m_specularColor = vsg::vec3(0.7, 0.7, 0.7);
            m_shininess = 0.078125 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_YELLOW_RUBBER:
            m_ambientColor = vsg::vec3(0.05, 0.05, 0.0);
            m_diffuseColor = vsg::vec3(0.5, 0.5, 0.4);
            m_specularColor = vsg::vec3(0.7, 0.7, 0.04);
            m_shininess = 0.078125 * 128.0;
            m_opacity = 1.0;
            break;
        case VSG_PHONG_MAT_BLUE_RUBBER:
            m_ambientColor = vsg::vec3(0.0, 0.0, 0.05);
            m_diffuseColor = vsg::vec3(0.4, 0.4, 0.5);
            m_specularColor = vsg::vec3(0.4, 0.04, 0.07);
            m_shininess = 0.078125 * 128.0;
            m_opacity = 1.0;
            break;
    }
}