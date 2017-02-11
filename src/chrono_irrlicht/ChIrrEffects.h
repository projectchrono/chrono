// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHIRREFFECTS_H
#define CHIRREFFECTS_H

#include <irrlicht.h>

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// CShaderPreprocessor
class CShaderPreprocessor {
  public:
    CShaderPreprocessor(irr::video::IVideoDriver* driverIn);
    irr::core::stringc ppShader(irr::core::stringc shaderProgram);
    irr::core::stringc ppShaderFF(irr::core::stringc shaderProgram);
    void addShaderDefine(const irr::core::stringc name, const irr::core::stringc value = "");
    void removeShaderDefine(const irr::core::stringc name);

  private:
    void initDefineMap();

    irr::video::IVideoDriver* driver;
    irr::core::map<irr::core::stringc, irr::core::stringc> DefineMap;
};

class EffectHandler;

/// DepthShaderCB
class DepthShaderCB : public irr::video::IShaderConstantSetCallBack {
  public:
    DepthShaderCB(EffectHandler* effectIn) : effect(effectIn){};

    virtual void OnSetConstants(irr::video::IMaterialRendererServices* services, irr::s32 userData) {
        irr::video::IVideoDriver* driver = services->getVideoDriver();

        worldViewProj = driver->getTransform(irr::video::ETS_PROJECTION);
        worldViewProj *= driver->getTransform(irr::video::ETS_VIEW);
        worldViewProj *= driver->getTransform(irr::video::ETS_WORLD);

        services->setVertexShaderConstant("mWorldViewProj", worldViewProj.pointer(), 16);

        services->setVertexShaderConstant("MaxD", &FarLink, 1);
    }

    EffectHandler* effect;
    irr::f32 FarLink;
    irr::core::matrix4 worldViewProj;
};

/// ShadowShaderCB
class ShadowShaderCB : public irr::video::IShaderConstantSetCallBack {
  public:
    ShadowShaderCB(EffectHandler* effectIn) : effect(effectIn){};

    virtual void OnSetMaterial(const irr::video::SMaterial& material) {}

    virtual void OnSetConstants(irr::video::IMaterialRendererServices* services, irr::s32 userData) {
        irr::video::IVideoDriver* driver = services->getVideoDriver();

        irr::core::matrix4 worldViewProj = driver->getTransform(irr::video::ETS_PROJECTION);
        worldViewProj *= driver->getTransform(irr::video::ETS_VIEW);
        worldViewProj *= driver->getTransform(irr::video::ETS_WORLD);
        services->setVertexShaderConstant("mWorldViewProj", worldViewProj.pointer(), 16);

        worldViewProj = ProjLink;
        worldViewProj *= ViewLink;
        worldViewProj *= driver->getTransform(irr::video::ETS_WORLD);
        services->setVertexShaderConstant("mWorldViewProj2", worldViewProj.pointer(), 16);

        driver->getTransform(irr::video::ETS_WORLD).getInverse(invWorld);
        irr::core::vector3df lightPosOS = LightLink;
        invWorld.transformVect(lightPosOS);
        services->setVertexShaderConstant("LightPos", reinterpret_cast<irr::f32*>(&lightPosOS.X), 4);

        services->setVertexShaderConstant("MaxD", reinterpret_cast<irr::f32*>(&FarLink), 1);
        services->setVertexShaderConstant("MAPRES", &MapRes, 1);

        services->setPixelShaderConstant("LightColour", reinterpret_cast<irr::f32*>(&LightColour.r), 4);
        float fclipborder = clipborder;
        services->setPixelShaderConstant("ClipBorder", reinterpret_cast<irr::f32*>(&fclipborder), 1);
    }

    EffectHandler* effect;
    irr::core::matrix4 invWorld;

    irr::video::SColorf LightColour;
    irr::core::matrix4 ProjLink;
    irr::core::matrix4 ViewLink;
    irr::core::vector3df LightLink;
    irr::f32 FarLink, MapRes;
    bool clipborder;  //***ALEX***
};

/// ScreenQuadCB
class ScreenQuadCB : public irr::video::IShaderConstantSetCallBack {
  public:
    ScreenQuadCB(EffectHandler* effectIn, bool defaultV = true) : effect(effectIn), defaultVertexShader(defaultV){};

    EffectHandler* effect;
    bool defaultVertexShader;

    virtual void OnSetConstants(irr::video::IMaterialRendererServices* services, irr::s32 userData);

    struct SUniformDescriptor {
        SUniformDescriptor() : fPointer(0), paramCount(0) {}

        SUniformDescriptor(const irr::f32* fPointerIn, irr::u32 paramCountIn)
            : fPointer(fPointerIn), paramCount(paramCountIn) {}

        const irr::f32* fPointer;
        irr::u32 paramCount;
    };

    irr::core::map<irr::core::stringc, SUniformDescriptor> uniformDescriptors;
};

struct SDefineExp {
    SDefineExp() : IfPos(-1), ElsePos(-1), EndPos(-1), IfExp(""), Inverse(false){};
    irr::s32 IfPos;
    irr::s32 ElsePos;
    irr::s32 EndPos;
    irr::core::stringc IfExp;
    bool Inverse;
};

inline irr::core::array<SDefineExp> grabDefineExpressions(irr::core::stringc& shaderProgram) {
    irr::s32 CurrentSearchPos = 1;
    irr::s32 FindHelper = 1;
    irr::s32 FindHelper2 = 1;

    irr::core::array<SDefineExp> DefineArray;

    // Dont bother stripping comments if theres no defines.
    if ((CurrentSearchPos = shaderProgram.find("##ifdef") == -1))
        return DefineArray;

    // Strip all comments, they get in the way.
    while ((CurrentSearchPos = shaderProgram.find("//")) > -1) {
        FindHelper = shaderProgram.findNext('\n', CurrentSearchPos);
        if (FindHelper != -1)
            for (irr::u32 i = CurrentSearchPos; i < (irr::u32)FindHelper; ++i)
                shaderProgram[i] = ' ';
        else
            for (irr::u32 i = CurrentSearchPos; i < shaderProgram.size(); ++i)
                shaderProgram[i] = ' ';
    }

    while ((CurrentSearchPos = shaderProgram.find("/*")) > -1) {
        FindHelper = shaderProgram.find("*/");
        if (FindHelper > CurrentSearchPos)
            for (irr::u32 i = CurrentSearchPos; i <= (irr::u32)(FindHelper + 1); ++i)
                shaderProgram[i] = ' ';
        else
            for (irr::u32 i = CurrentSearchPos; i < shaderProgram.size(); ++i)
                shaderProgram[i] = ' ';
    }

    while ((CurrentSearchPos = shaderProgram.find("##ifdef")) > -1) {
        SDefineExp DExp;

        DExp.IfPos = CurrentSearchPos;

        // Comment out the ##ifdef so that we do not find it again, and so that the compiler ignores it.
        shaderProgram[CurrentSearchPos] = '/';
        shaderProgram[CurrentSearchPos + 1] = '/';

        FindHelper = shaderProgram.findNext(' ', CurrentSearchPos);
        FindHelper2 = shaderProgram.findNext('\n', FindHelper);

        if (FindHelper == -1 || FindHelper2 == -1) {
            std::cerr << "Shader preprocessor encountered invalid if statement." << std::endl;
            return DefineArray;
        }

        // Find the appropriate expression and trim all white space.
        DExp.IfExp = shaderProgram.subString(FindHelper, FindHelper2 - FindHelper);
        DExp.IfExp.trim();

        // Record if its inverse and remove ! sign from expression.
        if (DExp.IfExp[0] == '!') {
            DExp.IfExp[0] = ' ';
            DExp.IfExp.trim();
            DExp.Inverse = true;
        }

        bool EndIfFound = false;

        FindHelper2 = CurrentSearchPos;
        irr::s32 IfEndScope = 0;

        while (!EndIfFound) {
            FindHelper = shaderProgram.findNext('#', FindHelper2);

            if (FindHelper == -1 || FindHelper >= (irr::s32)(shaderProgram.size() - 3)) {
                std::cerr << "Shader preprocessor encountered unmatched if statement." << std::endl;
                return DefineArray;
            }

            if (IfEndScope < 0) {
                std::cerr << "Shader preprocessor encountered unmatched endif statement." << std::endl;
                return DefineArray;
            }

            if (shaderProgram[FindHelper + 1] != '#') {
                FindHelper2 = FindHelper + 1;
                continue;
            } else if (shaderProgram[FindHelper + 2] == 'i') {
                IfEndScope++;
            } else if (shaderProgram[FindHelper + 2] == 'e' && shaderProgram[FindHelper + 3] == 'n') {
                if (IfEndScope == 0)
                    break;

                IfEndScope--;
            } else if (shaderProgram[FindHelper + 2] == 'e' && shaderProgram[FindHelper + 3] == 'l') {
                if (IfEndScope == 0) {
                    if (DExp.ElsePos != -1) {
                        std::cerr << "Shader preprocessor encountered duplicate else statements per if statement."
                                  << std::endl;
                        return DefineArray;
                    }

                    // Comment out the ##else so that we do not find it again, and so that the compiler ignores it.
                    shaderProgram[FindHelper] = '/';
                    shaderProgram[FindHelper + 1] = '/';

                    DExp.ElsePos = FindHelper;
                }
            }

            FindHelper2 = FindHelper + 2;
        }

        // Comment out the ##endif so that we do not find it again, and so that the compiler ignores it.
        shaderProgram[FindHelper] = '/';
        shaderProgram[FindHelper + 1] = '/';

        DExp.EndPos = FindHelper;

        // Add the define expression to the array.
        DefineArray.push_back(DExp);
    }

    return DefineArray;
}

inline CShaderPreprocessor::CShaderPreprocessor(irr::video::IVideoDriver* driverIn) : driver(driverIn) {
    initDefineMap();
};

inline void CShaderPreprocessor::initDefineMap() {
    if (driver->queryFeature(irr::video::EVDF_TEXTURE_NPOT))
        DefineMap["EVDF_TEXTURE_NPOT"] = "";
    if (driver->queryFeature(irr::video::EVDF_FRAMEBUFFER_OBJECT))
        DefineMap["EVDF_FRAMEBUFFER_OBJECT"] = "";
    if (driver->queryFeature(irr::video::EVDF_VERTEX_SHADER_1_1))
        DefineMap["EVDF_VERTEX_SHADER_1_1"] = "";
    if (driver->queryFeature(irr::video::EVDF_VERTEX_SHADER_2_0))
        DefineMap["EVDF_VERTEX_SHADER_2_0"] = "";
    if (driver->queryFeature(irr::video::EVDF_VERTEX_SHADER_3_0))
        DefineMap["EVDF_VERTEX_SHADER_3_0"] = "";
    if (driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_1_1))
        DefineMap["EVDF_PIXEL_SHADER_1_1"] = "";
    if (driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_1_2))
        DefineMap["EVDF_PIXEL_SHADER_1_2"] = "";
    if (driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_1_3))
        DefineMap["EVDF_PIXEL_SHADER_1_3"] = "";
    if (driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_1_4))
        DefineMap["EVDF_PIXEL_SHADER_1_4"] = "";
    if (driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_2_0))
        DefineMap["EVDF_PIXEL_SHADER_2_0"] = "";
    if (driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_3_0))
        DefineMap["EVDF_PIXEL_SHADER_3_0"] = "";

    // Commented for backwards compatibility.
    // DefineMap[driver->getVendorInfo()] = "";
}

inline void CShaderPreprocessor::addShaderDefine(const irr::core::stringc name, const irr::core::stringc value) {
    // No need for this as its already inited at startup.
    //// If DefineMap is empty then initialize it.
    // if(DefineMap.isEmpty())
    //	initDefineMap();

    DefineMap[name] = value;
}

inline void CShaderPreprocessor::removeShaderDefine(const irr::core::stringc name) {
    DefineMap.remove(name);
}

//! PreProcesses a shader using Irrlicht's built-in shader preprocessor.
inline irr::core::stringc CShaderPreprocessor::ppShader(irr::core::stringc shaderProgram) {
    irr::core::array<SDefineExp> DefineArray = grabDefineExpressions(shaderProgram);

    // No need for this as its already inited at startup.
    //// If DefineMap is empty then initialize it.
    // if(DefineMap.isEmpty())
    //	initDefineMap();

    for (irr::u32 i = 0; i < DefineArray.size(); ++i) {
        if (DefineArray[i].IfPos == -1)
            break;

        // Either it is true and not inversed, or it is false, but inversed.
        // (Wish C++ had a built-in (logical) XOR operator sometimes. :P)
        if ((DefineMap.find(DefineArray[i].IfExp) && !DefineArray[i].Inverse) ||
            (!DefineMap.find(DefineArray[i].IfExp) && DefineArray[i].Inverse)) {
            if (DefineArray[i].ElsePos > -1) {
                // If there is an else statement then clear the else section.
                if (DefineArray[i].EndPos != -1) {
                    for (int z = DefineArray[i].ElsePos; z <= DefineArray[i].EndPos + 6; ++z)
                        shaderProgram[z] = ' ';
                }
            }
        } else if (DefineArray[i].ElsePos != -1) {
            // If there is an else statement then clear the if section.
            for (int z = DefineArray[i].IfPos; z <= DefineArray[i].ElsePos + 5; ++z)
                shaderProgram[z] = ' ';
        } else {
            // Else just clear the whole block.
            if (DefineArray[i].EndPos != -1) {
                for (int z = DefineArray[i].IfPos; z <= DefineArray[i].EndPos + 6; ++z)
                    shaderProgram[z] = ' ';
            }
        }
    }

    irr::core::map<irr::core::stringc, irr::core::stringc>::ParentFirstIterator DefIter;
    irr::s32 DefFinder = 1;

    // Replace all shader defines.
    for (DefIter = DefineMap.getParentFirstIterator(); !DefIter.atEnd(); DefIter++) {
        if (DefIter->getValue().size() == 0)
            continue;

        // Replace all occurances.
        while ((DefFinder = shaderProgram.find(DefIter->getKey().c_str())) > -1) {
            // Clear the define from the code.
            for (irr::u32 z = DefFinder; z < DefFinder + DefIter->getKey().size(); ++z)
                shaderProgram[z] = ' ';

            // Stitch value and shader program together. (Is there a faster way?)
            shaderProgram = shaderProgram.subString(0, DefFinder) + DefIter->getValue() +
                            shaderProgram.subString(DefFinder, shaderProgram.size() - 1);
        }
    }

    return shaderProgram;
}

inline std::string getFileContent(const std::string pFile) {
    std::ifstream File(pFile.c_str(), std::ios::in);
    std::string Content;

    if (File.is_open()) {
        for (std::string Line; std::getline(File, Line);)
            Content += Line + "\n";

        File.close();
    }

    return Content;
}

//! PreProcesses a shader using Irrlicht's built-in shader preprocessor.
inline irr::core::stringc CShaderPreprocessor::ppShaderFF(irr::core::stringc shaderProgram) {
    return ppShader(getFileContent(shaderProgram.c_str()).c_str());
}

//***ALEX*** fixes for Irrlicht 1.8

/// CScreenQuad
class CScreenQuad {
  public:
    CScreenQuad() {
        Material.Wireframe = false;
        Material.Lighting = false;
        Material.ZWriteEnable = false;

        Vertices[0] = irr::video::S3DVertex(-1.0f, -1.0f, 0.0f, 0, 0, 1, irr::video::SColor(0x0), 0.0f, 1.0f);
        Vertices[1] = irr::video::S3DVertex(-1.0f, 1.0f, 0.0f, 0, 0, 1, irr::video::SColor(0x0), 0.0f, 0.0f);
        Vertices[2] = irr::video::S3DVertex(1.0f, 1.0f, 0.0f, 0, 0, 1, irr::video::SColor(0x0), 1.0f, 0.0f);
        Vertices[3] = irr::video::S3DVertex(1.0f, -1.0f, 0.0f, 0, 0, 1, irr::video::SColor(0x0), 1.0f, 1.0f);
    }

    virtual ~CScreenQuad() {}

    virtual void render(irr::video::IVideoDriver* driver) {
        const irr::u16 indices[6] = {0, 1, 2, 0, 2, 3};

        driver->setMaterial(Material);
        driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
        driver->drawIndexedTriangleList(&Vertices[0], 4, &indices[0], 2);
    }

    virtual irr::video::SMaterial& getMaterial() { return Material; }

    irr::video::ITexture* rt[2];

  private:
    irr::video::S3DVertex Vertices[4];
    irr::video::SMaterial Material;
};

/*
class CScreenQuad
{
public:
    CScreenQuad()
    {
        Material.Wireframe = false;
        Material.Lighting = false;
        Material.ZWriteEnable = false;

        Vertices[0] = irr::video::S3DVertex(-1.0f,-1.0f,0.0f,0,0,1,irr::video::SColor(0x0),0.0f,1.0f);
        Vertices[1] = irr::video::S3DVertex(-1.0f, 1.0f,0.0f,0,0,1,irr::video::SColor(0x0),0.0f,0.0f);
        Vertices[2] = irr::video::S3DVertex( 1.0f, 1.0f,0.0f,0,0,1,irr::video::SColor(0x0),1.0f,0.0f);
        Vertices[3] = irr::video::S3DVertex( 1.0f,-1.0f,0.0f,0,0,1,irr::video::SColor(0x0),1.0f,1.0f);
        Vertices[4] = irr::video::S3DVertex(-1.0f,-1.0f,0.0f,0,0,1,irr::video::SColor(0x0),0.0f,1.0f);
        Vertices[5] = irr::video::S3DVertex( 1.0f, 1.0f,0.0f,0,0,1,irr::video::SColor(0x0),1.0f,0.0f);
    }

    virtual void render(irr::video::IVideoDriver* driver)
    {
        u16 indices[6] = {0, 1, 2, 3, 4, 5};

        driver->setMaterial(Material);
        driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
        driver->drawIndexedTriangleList(&Vertices[0], 6, &indices[0], 2);
    }

    virtual irr::video::SMaterial& getMaterial()
    {
        return Material;
    }

    irr::video::ITexture* rt[2];

private:
    irr::video::S3DVertex Vertices[6];
    irr::video::SMaterial Material;
};
*/

//////////////////////////////////////EffectShaders.h

enum E_SHADER_EXTENSION {
    ESE_GLSL,
    ESE_HLSL,

    ESE_COUNT
};

//***ALEX***
// from ... const char* LIGHT_MODULATE_P ... to ... const char* const LIGHT_MODULATE_P .. to avoid multiple defined
// symbols
const char* const LIGHT_MODULATE_P[ESE_COUNT] = {
    "uniform sampler2D ColorMapSampler;\n"
    "uniform sampler2D ScreenMapSampler;\n"
    ""
    "void main() "
    "{		"
    "	vec4 finalCol = texture2D(ColorMapSampler, gl_TexCoord[0].xy);\n"
    "	vec4 lightCol = texture2D(ScreenMapSampler, gl_TexCoord[0].xy);\n"
    ""
    "	gl_FragColor = finalCol * lightCol;\n"
    "}",
    "sampler2D ColorMapSampler : register(s0);\n"
    "sampler2D ScreenMapSampler : register(s1);\n"
    ""
    "float4 pixelMain(float2 TexCoords : TEXCOORD0) : COLOR0"
    "{		"
    "	float4 finalCol = tex2D(ColorMapSampler, TexCoords);\n"
    "	float4 lightCol = tex2D(ScreenMapSampler, TexCoords);\n"
    ""
    "	return finalCol * lightCol;\n"
    "}"};

const char* const SHADOW_PASS_1P[ESE_COUNT] = {
    "void main() "
    "{"
    "	vec4 vInfo = gl_TexCoord[0];\n"
    "	float depth = vInfo.z / vInfo.x;\n"
    "   gl_FragColor = vec4(depth, depth * depth, 0.0, 0.0);\n"
    "}",
    "float4 pixelMain(float4 ClipPos: TEXCOORD0) : COLOR0"
    "{"
    "	float depth = ClipPos.z / ClipPos.x;\n"
    "	return float4(depth, depth * depth, 0.0, 0.0);\n"
    "}"};

const char* const SHADOW_PASS_1PT[ESE_COUNT] = {
    "uniform sampler2D ColorMapSampler;\n"
    ""
    "void main() "
    "{"
    "	vec4 vInfo = gl_TexCoord[0];\n"
    ""
    "	float depth = vInfo.z / vInfo.x;\n"
    ""
    "	float alpha = texture2D(ColorMapSampler, gl_TexCoord[1].xy).a;\n"
    ""
    "    gl_FragColor = vec4(depth, depth * depth, 0.0, alpha);\n"
    "}",
    "sampler2D ColorMapSampler : register(s0);\n"
    ""
    "float4 pixelMain(float4 Color: TEXCOORD0, float2 Texcoords: TEXCOORD1) : COLOR0"
    "{"
    "	float depth = Color.z / Color.w;\n"
    "	"
    "	float alpha = tex2D(ColorMapSampler, Texcoords).a;\n"
    "	"
    "	return float4(depth, depth * depth, 0.0, alpha);\n"
    "}"};

const char* const SHADOW_PASS_1V[ESE_COUNT] = {
    "uniform mat4 mWorldViewProj;\n"
    "uniform float MaxD;\n"
    ""
    "void main()"
    "{"
    "	vec4 tPos = mWorldViewProj * gl_Vertex;\n"
    "	gl_Position = tPos;\n"
    "	gl_TexCoord[0] = vec4(MaxD, tPos.y, tPos.z, tPos.w);\n"
    ""
    "	gl_TexCoord[1].xy = gl_MultiTexCoord0.xy;\n"
    "}",
    "float4x4 mWorldViewProj;\n"
    "float MaxD;\n"
    ""
    "struct VS_OUTPUT "
    "{"
    "	float4 Position: POSITION0;\n"
    "	float4 ClipPos: TEXCOORD0;\n"
    "	float2 Texcoords: TEXCOORD1;\n"
    "	float4 VColor: TEXCOORD2;\n"
    "};\n"
    ""
    "VS_OUTPUT vertexMain(float3 Position : POSITION0, float2 Texcoords : TEXCOORD0, float4 vColor : COLOR0)"
    "{"
    "	VS_OUTPUT  OUT;\n"
    "	float4 hpos = mul(float4(Position.x, Position.y, Position.z, 1.0), mWorldViewProj);\n"
    "   OUT.ClipPos = hpos;\n"
    "	OUT.ClipPos.x = MaxD;\n"
    "   OUT.Position = hpos;\n"
    "	OUT.Texcoords = Texcoords;\n"
    "	OUT.VColor = vColor;\n"
    "	return(OUT);\n"
    "}"};

const char* const SHADOW_PASS_2P[ESE_COUNT] = {
    "uniform sampler2D ShadowMapSampler;\n"
    "uniform vec4 LightColour;\n"
    "varying float lightVal;\n"
    ""
    "\n##ifdef VSM\n"
    "float testShadow(vec2 texCoords, vec2 offset, float RealDist)\n"
    "{\n"
    "	vec4 shadTexCol = texture2D(ShadowMapSampler, texCoords + offset);\n"
    ""
    "	float lit_factor = (RealDist <= shadTexCol.x) ? 1.0 : 0.0;\n"
    ""
    "	float E_x2 = shadTexCol.y;\n"
    "	float Ex_2 = shadTexCol.x * shadTexCol.x;\n"
    "	float variance = min(max(E_x2 - Ex_2, 0.00001) + 0.000001, 1.0);\n"
    "	float m_d = (shadTexCol.x - RealDist);\n"
    "	float p = variance / (variance + m_d * m_d);\n"
    ""
    "	return (1.0 - max(lit_factor, p)) / float(SAMPLE_AMOUNT);\n"
    "}\n"
    "##else\n"
    "float testShadow(vec2 smTexCoord, vec2 offset, float realDistance)"
    "{"
    "	vec4 texDepth = texture2D(ShadowMapSampler, vec2( smTexCoord + offset));\n"
    "	float extractedDistance = texDepth.r;\n"
    "	"
    "	return (extractedDistance <= realDistance) ? (1.0  / float(SAMPLE_AMOUNT)) : 0.0;\n"
    "}\n"
    "##endif\n"
    "\n"
    "vec2 offsetArray[16];\n"
    "\n"
    "void main() \n"
    "{"
    "	vec4 SMPos = gl_TexCoord[0];\n"
    "	vec4 MVar = gl_TexCoord[1];\n"
    ""
    "	offsetArray[0] = vec2(0.0, 0.0);\n"
    "	offsetArray[1] = vec2(0.0, 1.0);\n"
    "	offsetArray[2] = vec2(1.0, 1.0);\n"
    "	offsetArray[3] = vec2(-1.0, -1.0);\n"
    "	offsetArray[4] = vec2(-2.0, 0.0);\n"
    "	offsetArray[5] = vec2(0.0, -2.0);\n"
    "	offsetArray[6] = vec2(2.0, -2.0);\n"
    "	offsetArray[7] = vec2(-2.0, 2.0);\n"
    "	offsetArray[8] = vec2(3.0, 0.0);\n"
    "	offsetArray[9] = vec2(0.0, 3.0);\n"
    "	offsetArray[10] = vec2(3.0, 3.0);\n"
    "	offsetArray[11] = vec2(-3.0, -3.0);\n"
    "	offsetArray[12] = vec2(-4.0, 0.0);\n"
    "	offsetArray[13] = vec2(0.0, -4.0);\n"
    "	offsetArray[14] = vec2(4.0, -4.0);\n"
    "	offsetArray[15] = vec2(-4.0, 4.0);\n"
    ""
    "    SMPos.xy  = SMPos.xy / SMPos.w / 2.0 + vec2(0.5, 0.5);\n"
    ""
    "	vec4 finalCol = vec4(0.0, 0.0, 0.0, 0.0);\n"
    ""
    "	// If this point is within the light's frustum.\n"
    "##ifdef ROUND_SPOTLIGHTS\n"
    "	float lengthToCenter = length(SMPos.xy - vec2(0.5, 0.5));\n"
    "	if(SMPos.z - 0.01 > 0.0 && SMPos.z + 0.01 < MVar.z)\n"
    "##else\n"
    "	vec2 clampedSMPos = clamp(SMPos.xy, vec2(0.0, 0.0), vec2(1.0, 1.0));\n"
    "	if(clampedSMPos.x == SMPos.x && clampedSMPos.y == SMPos.y && SMPos.z > 0.0 && SMPos.z < MVar.z)\n"
    "##endif\n"
    "	{"
    "		float lightFactor = 1.0;\n"
    "		float realDist = MVar.x / MVar.z - 0.002;\n"
    "	"
    "		for(int i = 0;i < SAMPLE_AMOUNT; i++)"
    "			lightFactor -= testShadow(SMPos.xy, offsetArray[i] * MVar.w, realDist);\n"
    ""
    "		// Multiply with diffuse.\n"
    "##ifdef ROUND_SPOTLIGHTS\n"
    "		finalCol = LightColour * lightFactor * MVar.y * clamp(5.0 - 10.0 * lengthToCenter, 0.0, 1.0);\n"
    "##else\n"
    "		finalCol = LightColour * lightFactor * MVar.y;\n"
    "##endif\n"
    "	}"
    ""
    "	gl_FragColor = finalCol;\n"
    "}",
    "sampler2D ShadowMapSampler : register(s0);\n"
    "float4 LightColour;\n"
    "float ClipBorder;\n"
    "\n"
    "##ifdef VSM\n"
    "float calcShadow(float2 texCoords, float2 offset, float RealDist)"
    "{"
    "	float4 shadTexCol = tex2D(ShadowMapSampler, texCoords + offset);\n"
    ""
    "	float lit_factor = (RealDist <= shadTexCol.x);\n"
    ""
    "	float E_x2 = shadTexCol.y;\n"
    "	float Ex_2 = shadTexCol.x * shadTexCol.x;\n"
    "	float variance = min(max(E_x2 - Ex_2, 0.00001) + 0.000001, 1.0);\n"
    "	float m_d = (shadTexCol.x - RealDist);\n"
    "	float p = variance / (variance + m_d * m_d);\n"
    "	  "
    "	return (1.0 - max(lit_factor, p)) / SAMPLE_AMOUNT;\n"
    "}\n"
    "##else\n"
    "float calcShadow(float2 texCoords, float2 offset, float RealDist)"
    "{"
    "   float4 shadTexCol = tex2D(ShadowMapSampler,texCoords + offset);\n"
    "   float extractedDistance = shadTexCol.r;\n"
    "      "
    "   return (extractedDistance <= RealDist ? (1.0  / SAMPLE_AMOUNT) : 0.0);\n"
    "}\n"
    "##endif\n"
    ""
    "float4 pixelMain"
    "("
    "   float4 SMPos       : TEXCOORD0,"
    "   float4 MVar        : TEXCOORD1,"
    "   float2 TexCoords    : TEXCOORD2"
    ") : COLOR0"
    "{"
    "	const float2 offsetArray[16] = "
    "	{"
    "		float2(0.0, 0.0),"
    "		float2(0.0, 1.0),"
    "		float2(1.0, -1.0),"
    "		float2(-1.0, 1.0),"
    "		float2(-2.0, 0.0),"
    "		float2(0.0, -2.0),"
    "		float2(-2.0, -2.0),"
    "		float2(2.0, 2.0),"
    "		float2(3.0, 0.0),"
    "		float2(0.0, 3.0),"
    "		float2(3.0, -3.0),"
    "		float2(-3.0, 3.0),"
    "		float2(-4.0, 0.0),"
    "		float2(0.0, -4.0),"
    "		float2(-4.0, -4.0),"
    "		float2(4.0, 4.0)"
    "	};\n"
    ""
    "	SMPos.xy = SMPos.xy / SMPos.w + float2(0.5, 0.5);\n"
    ""
    "	float4 finalCol = float4(0.0, 0.0, 0.0, 0.0);\n"
    ""
    "	// If this point is within the light's frustum.\n"
    "##ifdef ROUND_SPOTLIGHTS\n"
    "	float lengthToCenter = length(SMPos.xy - float2(0.5, 0.5));\n"
    "	if(lengthToCenter < 0.5 && SMPos.z > 0.0 && SMPos.z < MVar[3])\n"
    "##else\n"
    "	float2 clampedSMPos = saturate(SMPos.xy);\n"
    "	if(clampedSMPos.x == SMPos.x && clampedSMPos.y == SMPos.y && SMPos.z > 0.0 && SMPos.z < MVar[3])\n"
    "##endif\n"
    "	{"
    "		float lightFactor = 1.0;\n"
    "		float realDistance = MVar[0] / MVar[3] - 0.005;\n"
    "	"
    "		for(int i = 0;i < SAMPLE_AMOUNT; ++i)"
    "			lightFactor -= calcShadow(SMPos.xy, offsetArray[i] * MVar[2], realDistance);\n"
    ""
    "		// Multiply with diffuse.\n"
    "##ifdef ROUND_SPOTLIGHTS\n"
    "		finalCol = LightColour * lightFactor * MVar[1] * clamp(5.0 - 10.0 * lengthToCenter, 0.0, 1.0);\n"
    "##else\n"
    "		finalCol = LightColour * lightFactor * MVar[1];\n"
    "##endif\n"
    "	}"
    "   else \n"
    "   { \n"
    "     if (!ClipBorder) finalCol = LightColour; \n"
    "   } \n"
    "	"
    "	return finalCol;\n"
    "}"};

const char* const SHADOW_PASS_2V[ESE_COUNT] = {
    "struct VS_OUTPUT "
    "{"
    "	vec4 Position;\n"
    "	vec4 ShadowMapSamplingPos;\n"
    "	vec4 MVar;\n"
    "};\n"
    ""
    "uniform float MaxD, MAPRES;\n"
    "uniform vec3 LightPos;\n"
    "uniform mat4 mWorldViewProj;\n"
    "uniform mat4 mWorldViewProj2;\n"
    ""
    "VS_OUTPUT vertexMain( in vec3 Position) "
    "{"
    "	VS_OUTPUT OUT;\n"
    ""
    "	OUT.Position = (mWorldViewProj * vec4(Position.x, Position.y, Position.z, 1.0));\n"
    "	OUT.ShadowMapSamplingPos = (mWorldViewProj2 * vec4(Position.x, Position.y, Position.z, 1.0));\n"
    ""
    "	vec3 lightDir = normalize(LightPos - Position);\n"
    "	"
    "	OUT.MVar.x = OUT.ShadowMapSamplingPos.z;\n"
    "	OUT.MVar.y = dot(normalize(gl_Normal.xyz), lightDir);\n"
    "	OUT.MVar.z = MaxD;\n"
    "	OUT.MVar.w = 1.0 / MAPRES;\n"
    ""
    "	return OUT;\n"
    "}"
    ""
    "void main() "
    "{"
    "	VS_OUTPUT vOut = vertexMain(gl_Vertex.xyz);\n"
    ""
    "	gl_Position = vOut.Position;\n"
    "	gl_TexCoord[0] = vOut.ShadowMapSamplingPos;\n"
    "	gl_TexCoord[1] = vOut.MVar;\n"
    "}",
    "float4x4 mWorldViewProj;\n"
    "float4x4 mWorldViewProj2;\n"
    "float3 LightPos;\n"
    "float ShadDark;\n"
    "float MaxD;\n"
    "float EnableLighting;\n"
    "float MAPRES;\n"
    ""
    "struct VS_OUTPUT "
    "{"
    "	float4 Position				: POSITION0;\n"
    "	float4 ShadowMapSamplingPos : TEXCOORD0; "
    "	float4 MVar        			: TEXCOORD1;\n"
    "	float2 TexCoords            : TEXCOORD2;\n"
    "};\n"
    ""
    "VS_OUTPUT vertexMain( "
    "   	float3 Position	: POSITION0,"
    "	float2 TexCoords : TEXCOORD0,"
    "	float2 TexCoords2 : TEXCOORD1,"
    "	float3 Normal : NORMAL"
    "  )"
    "{"
    "	VS_OUTPUT  OUT;\n"
    "    OUT.Position = mul(float4(Position.x,Position.y,Position.z,1.0), mWorldViewProj);\n"
    "	float4 SMPos = mul(float4(Position.x,Position.y,Position.z,1.0), mWorldViewProj2);\n"
    "	SMPos.xy = float2(SMPos.x, -SMPos.y) / 2.0;\n"
    "	"
    "	OUT.ShadowMapSamplingPos = SMPos;\n"
    "		"
    "	float3 LightDir = normalize(LightPos - Position.xyz);\n"
    "	"
    "	OUT.MVar = float4(SMPos.z, dot(Normal, LightDir), 1.0 / MAPRES, MaxD);\n"
    "	OUT.TexCoords = TexCoords;\n"
    "	"
    "	return(OUT);\n"
    "}"};

const char* const SIMPLE_P[ESE_COUNT] = {
    "uniform sampler2D ColorMapSampler;\n"
    ""
    "void main() "
    "{		"
    "	vec4 finalCol = texture2D(ColorMapSampler, gl_TexCoord[0].xy);\n"
    "	gl_FragColor = finalCol;\n"
    "}",
    "sampler2D ColorMapSampler : register(s0);\n"
    ""
    "float4 pixelMain(float2 TexCoords : TEXCOORD0) : COLOR0"
    "{		"
    "	float4 finalCol = tex2D(ColorMapSampler, TexCoords);\n"
    "	return finalCol;\n"
    "}"};

const char* const WHITE_WASH_P[ESE_COUNT] = {
    "uniform sampler2D ColorMapSampler;\n"
    ""
    "void main() "
    "{"
    "	float alpha = texture2D(ColorMapSampler, gl_TexCoord[1].xy).a;\n"
    ""
    "    gl_FragColor = vec4(1.0, 1.0, 1.0, alpha);\n"
    "}",
    "sampler2D ColorMapSampler : register(s0);\n"
    ""
    "float4 pixelMain(float4 Color: TEXCOORD0, float2 Texcoords: TEXCOORD1) : COLOR0"
    "{"
    "	float alpha = tex2D(ColorMapSampler, Texcoords).a;\n"
    ""
    "	return float4(1.0, 1.0, 1.0, alpha);\n"
    "}"};

const char* const WHITE_WASH_P_ADD[ESE_COUNT] = {
    "uniform sampler2D ColorMapSampler;\n"
    "float luminance(vec3 color)"
    "{"
    "	return clamp(color.r * 0.3 + color.g * 0.59 + color.b * 0.11, 0.0, 1.0);\n"
    "}"
    "void main() "
    "{"
    "	vec4 diffuseTex = texture2D(ColorMapSampler, gl_TexCoord[1].xy);\n"
    "	//diffuseTex *= gl_TexCoord[2];\n"
    ""
    "    gl_FragColor = vec4(1.0, 1.0, 1.0, luminance(diffuseTex.rgb));\n"
    "}",
    "sampler2D ColorMapSampler : register(s0);\n"
    ""
    "float luminance(float3 color)"
    "{"
    "	return clamp(color.r * 0.3 + color.g * 0.59 + color.b * 0.11, 0.0, 1.0);\n"
    "}"
    ""
    "float4 pixelMain(float4 Color : TEXCOORD0, float2 Texcoords : TEXCOORD1, float4 VColor : TEXCOORD2) : COLOR0"
    "{"
    "	float4 diffuseTex = tex2D(ColorMapSampler, Texcoords);\n"
    "	diffuseTex *= VColor;\n"
    ""
    "	return float4(1.0, 1.0, 1.0, luminance(diffuseTex.rgb));\n"
    "}"};

const char* const SCREEN_QUAD_V[ESE_COUNT] = {
    "uniform float screenX, screenY; \n"
    "uniform vec3 LineStarts0, LineStarts1, LineStarts2, LineStarts3; \n"
    "uniform vec3 LineEnds0, LineEnds1, LineEnds2, LineEnds3; \n"
    "void main() \n"
    "{\n"
    "	gl_Position = vec4(gl_Vertex.x, gl_Vertex.y, 0.0, 1.0); \n"
    "	vec2 tCoords; \n"
    "	tCoords.x = 0.5 * (1.0 + gl_Vertex.x); \n"
    "	tCoords.y = 0.5 * (1.0 + gl_Vertex.y); \n"
    "	gl_TexCoord[0].xy = tCoords.xy; \n"
    "	tCoords.y = 1.0 - tCoords.y; \n"
    "	vec3 tLStart = mix(LineStarts0, LineStarts1, tCoords.x); \n"
    "	vec3 bLStart = mix(LineStarts2, LineStarts3, tCoords.x); \n"
    "	gl_TexCoord[1].xyz = mix(tLStart, bLStart, tCoords.y); \n"
    "	vec3 tLEnd = mix(LineEnds0, LineEnds1, tCoords.x); \n"
    "	vec3 bLEnd = mix(LineEnds2, LineEnds3, tCoords.x); \n"
    "	gl_TexCoord[2].xyz = mix(tLEnd, bLEnd, tCoords.y); \n"
    "	gl_TexCoord[3].xy = vec2(screenX, screenY); \n"
    "}",
    "float screenX, screenY; \n"
    "float3 LineStarts0, LineStarts1, LineStarts2, LineStarts3; \n"
    "float3 LineEnds0, LineEnds1, LineEnds2, LineEnds3; \n"
    "struct VS_OUTPUT \n"
    "{"
    "	float4 Position		: POSITION0;"
    "	float2 TexCoords	: TEXCOORD0;"
    "	float3 LStart		: TEXCOORD1;"
    "	float3 LEnd			: TEXCOORD2;"
    "	float2 ScreenSize	: TEXCOORD3;"
    "}; \n"
    "VS_OUTPUT vertexMain(float3 Position : POSITION0) \n"
    "{ \n"
    "	VS_OUTPUT OUT; \n"
    "   OUT.Position = float4(Position.x,Position.y, 0.0, 1.0); \n"
    "	OUT.TexCoords.x = 0.5 * (1.0 + Position.x + (1.0 / screenX)); \n"
    "	OUT.TexCoords.y = 1.0 - 0.5 * (1.0 + Position.y - (1.0 / screenY)); \n"
    "	float3 tLStart = lerp(LineStarts0, LineStarts1, OUT.TexCoords.x); \n"
    "	float3 bLStart = lerp(LineStarts2, LineStarts3, OUT.TexCoords.x); \n"
    "	OUT.LStart = lerp(tLStart, bLStart, OUT.TexCoords.y); \n"
    "	float3 tLEnd = lerp(LineEnds0, LineEnds1, OUT.TexCoords.x); \n"
    "	float3 bLEnd = lerp(LineEnds2, LineEnds3, OUT.TexCoords.x); \n"
    "	OUT.LEnd = lerp(tLEnd, bLEnd, OUT.TexCoords.y); \n"
    "	OUT.ScreenSize = float2(screenX, screenY); \n"
    "	return(OUT); \n"
    "} \n"};

const char* const VSM_BLUR_P[ESE_COUNT] = {
    "uniform sampler2D ColorMapSampler;\n"
    "\n"
    "vec2 offsetArray[5];\n"
    "\n"
    "void main() \n"
    "{\n"
    "\n"
    "##ifdef VERTICAL_VSM_BLUR\n"
    "	offsetArray[0] = vec2(0.0, 0.0);\n"
    "	offsetArray[1] = vec2(0.0, -1.5 / gl_TexCoord[3].y);\n"
    "	offsetArray[2] = vec2(0.0, 1.5 / gl_TexCoord[3].y);\n"
    "	offsetArray[3] = vec2(0.0, -2.5 / gl_TexCoord[3].y);\n"
    "	offsetArray[4] = vec2(0.0, 2.5 / gl_TexCoord[3].y);\n"
    "##else\n"
    "	offsetArray[0] = vec2(0.0, 0.0);\n"
    "	offsetArray[1] = vec2(-1.5 / gl_TexCoord[3].x, 0.0);\n"
    "	offsetArray[2] = vec2(1.5 / gl_TexCoord[3].x, 0.0);\n"
    "	offsetArray[3] = vec2(-2.5 / gl_TexCoord[3].x, 0.0);\n"
    "	offsetArray[4] = vec2(2.5 / gl_TexCoord[3].x, 0.0);\n"
    "##endif\n"
    "\n"
    "	vec4 BlurCol = vec4(0.0, 0.0, 0.0, 0.0);\n"
    "\n"
    "	for(int i = 0;i < 5;++i)\n"
    "		BlurCol += texture2D(ColorMapSampler, clamp(gl_TexCoord[0].xy + offsetArray[i], vec2(0.001, 0.001), "
    "vec2(0.999, 0.999)));\n"
    "\n"
    "	gl_FragColor = BlurCol / 5.0;\n"
    "}\n",
    "sampler2D ColorMapSampler : register(s0);\n"
    "\n"
    "float4 pixelMain ( float4 Texcoords : TEXCOORD0, float2 ScreenSize : TEXCOORD3 ) : COLOR0\n"
    "{\n"
    "	float2 offsetArray[5];\n"
    "##ifdef VERTICAL_VSM_BLUR\n"
    "	offsetArray[0] = float2(0, 0);\n"
    "	offsetArray[1] = float2(0, 1.5 / ScreenSize.y);\n"
    "	offsetArray[2] = float2(0, -1.5 / ScreenSize.y);\n"
    "	offsetArray[3] = float2(0, 2.5 / ScreenSize.y);\n"
    "	offsetArray[4] = float2(0, -2.5 / ScreenSize.y);\n"
    "##else\n"
    "	offsetArray[0] = float2(0, 0);\n"
    "	offsetArray[1] = float2(1.5 / ScreenSize.x, 0);\n"
    "	offsetArray[2] = float2(-1.5 / ScreenSize.x, 0);\n"
    "	offsetArray[3] = float2(2.5 / ScreenSize.x, 0);\n"
    "	offsetArray[4] = float2(-2.5 / ScreenSize.x, 0);\n"
    "##endif\n"
    "\n"
    "	float4 finalVal = float4(0.0, 0.0, 0.0, 0.0);\n"
    "\n"
    "	for(int i = 0;i < 5;++i)\n"
    "		finalVal += tex2D(ColorMapSampler, clamp(Texcoords.xy + offsetArray[i], float2(0.001, 0.001), "
    "float2(0.999, 0.999)));\n"
    "\n"
    "	return finalVal / 5.0;\n"
    "}\n"};

///////////////////////////// EffectHandler.h

/// Shadow mode enums, sets whether a node recieves shadows, casts shadows, or both.
/// If the mode is ESM_CAST, it will not be affected by shadows or lighting.
enum E_SHADOW_MODE { ESM_RECEIVE, ESM_CAST, ESM_BOTH, ESM_EXCLUDE, ESM_COUNT };

/// Various filter types, up to 16 samples PCF.
enum E_FILTER_TYPE { EFT_NONE, EFT_4PCF, EFT_8PCF, EFT_12PCF, EFT_16PCF, EFT_COUNT };

struct SShadowLight {
    /// Shadow light constructor. The first parameter is the square shadow map resolution.
    /// This should be a power of 2 number, and within reasonable size to achieve optimal
    /// performance and quality. Recommended sizes are 512 to 4096 subject to your target
    /// hardware and quality requirements. The next two parameters are position and target,
    /// the next one is the light color. The next two are very important parameters,
    /// the far value and the near value. The higher the near value, and the lower the
    /// far value, the better the depth precision of the shadows will be, however it will
    /// cover a smaller volume. The next is the FOV, if the light was to be considered
    /// a camera, this would be similar to setting the camera's field of view. The last
    /// parameter is whether the light is directional or not, if it is, an orthogonal
    /// projection matrix will be created instead of a perspective one.
    SShadowLight(const irr::u32 shadowMapResolution,
                 const irr::core::vector3df& position,
                 const irr::core::vector3df& target,
                 irr::video::SColorf lightColour = irr::video::SColor(0xffffffff),
                 irr::f32 nearValue = 10.0,
                 irr::f32 farValue = 100.0,
                 irr::f32 fov = 90.0 * irr::core::DEGTORAD64,
                 bool directional = false)
        : pos(position),
          tar(target),
          farPlane(directional ? 1.0f : farValue),
          diffuseColour(lightColour),
          mapRes(shadowMapResolution) {
        nearValue = nearValue <= 0.0f ? 0.1f : nearValue;

        updateViewMatrix();

        if (directional)
            projMat.buildProjectionMatrixOrthoLH(fov, fov, nearValue, farValue);
        else
            projMat.buildProjectionMatrixPerspectiveFovLH(fov, 1.0f, nearValue, farValue);

        clipborder = true;  //***ALEX***
    }

    /// Sets the light's position.
    void setPosition(const irr::core::vector3df& position) {
        pos = position;
        updateViewMatrix();
    }

    /// Sets the light's target.
    void setTarget(const irr::core::vector3df& target) {
        tar = target;
        updateViewMatrix();
    }

    /// Gets the light's position.
    const irr::core::vector3df& getPosition() const { return pos; }

    /// Gets the light's target.
    const irr::core::vector3df& getTarget() const { return tar; }

    /// Sets the light's view matrix.
    void setViewMatrix(const irr::core::matrix4& matrix) {
        viewMat = matrix;
        irr::core::matrix4 vInverse;
        viewMat.getInverse(vInverse);
        pos = vInverse.getTranslation();
    }

    /// Sets the light's projection matrix.
    void setProjectionMatrix(const irr::core::matrix4& matrix) { projMat = matrix; }

    /// Gets the light's view matrix.
    irr::core::matrix4& getViewMatrix() { return viewMat; }

    /// Gets the light's projection matrix.
    irr::core::matrix4& getProjectionMatrix() { return projMat; }

    /// Gets the light's far value.
    irr::f32 getFarValue() const { return farPlane; }

    /// Gets the light's color.
    const irr::video::SColorf& getLightColor() const { return diffuseColour; }

    /// Sets the light's color.
    void setLightColor(const irr::video::SColorf& lightColour) { diffuseColour = lightColour; }

    /// Sets the shadow map resolution for this light.
    void setShadowMapResolution(const irr::u32 shadowMapResolution) { mapRes = shadowMapResolution; }

    /// Gets the shadow map resolution for this light.
    const irr::u32 getShadowMapResolution() const { return mapRes; }

    ///***ALEX***
    void setClipBorder(bool mb) { clipborder = mb; }
    ///***ALEX***
    bool getClipBorder() const { return clipborder; }

  private:
    void updateViewMatrix() {
        viewMat.buildCameraLookAtMatrixLH(pos, tar,
                                          (pos - tar).dotProduct(irr::core::vector3df(1.0f, 0.0f, 1.0f)) == 0.0f
                                              ? irr::core::vector3df(0.0f, 0.0f, 1.0f)
                                              : irr::core::vector3df(0.0f, 1.0f, 0.0f));
    }

    irr::video::SColorf diffuseColour;
    irr::core::vector3df pos, tar;
    irr::f32 farPlane;
    irr::core::matrix4 viewMat, projMat;
    irr::u32 mapRes;
    bool clipborder;  //***ALEX***
};

// This is a general interface that can be overidden if you want to perform operations before or after
// a specific post-processing effect. You will be passed an instance of the EffectHandler.
// The function names themselves should be self-explanatory ;)
class EffectHandler;
class IPostProcessingRenderCallback {
  public:
    virtual void OnPreRender(EffectHandler* effect) = 0;
    virtual void OnPostRender(EffectHandler* effect) = 0;

    virtual ~IPostProcessingRenderCallback();
};

// Shader callback prototypes.
class DepthShaderCB;
class ShadowShaderCB;
class ScreenQuadCB;

/// Main effect handling class, use this to apply shadows and effects.
class EffectHandler {
  public:
    /*	EffectHandler constructor. Initializes the EffectHandler.

        Parameters:
        irrlichtDevice: Current Irrlicht device.
        screenRTTSize: Size of screen render target for post processing. Default is screen size.
        useVSMShadows: Shadows will use VSM filtering. It is recommended to only use EFT_NONE when this is enabled.
        useRoundSpotlights: Shadow lights will have a soft round spot light mask. Default is false.
        use32BitDepthBuffers: XEffects will use 32-bit depth buffers if this is true, otherwise 16-bit. Default is
       false.
    */
    EffectHandler(irr::IrrlichtDevice* irrlichtDevice,
                  const irr::core::dimension2du& screenRTTSize = irr::core::dimension2du(0, 0),
                  const bool useVSMShadows = false,
                  const bool useRoundSpotLights = false,
                  const bool use32BitDepthBuffers = false);

    /// Destructor.
    ~EffectHandler();

    /// Adds a shadow light. Check out the shadow light constructor for more information.
    void addShadowLight(const SShadowLight& shadowLight) { LightList.push_back(shadowLight); }

    /// Retrieves a reference to a shadow light. You may get the max amount from getShadowLightCount.
    SShadowLight& getShadowLight(irr::u32 index) { return LightList[index]; }

    /// Retrieves the current number of shadow lights.
    const irr::u32 getShadowLightCount() const { return LightList.size(); }

    /// Retrieves the shadow map texture for the specified square shadow map resolution.
    /// Only one shadow map is kept for each resolution, so if multiple lights are using
    /// the same resolution, you will only see the last light drawn's output.
    /// The secondary param specifies whether to retrieve the secondary shadow map used in blurring.
    irr::video::ITexture* getShadowMapTexture(const irr::u32 resolution, const bool secondary = false);

    /// Retrieves the screen depth map texture if the depth pass is enabled. This is unrelated to the shadow map, and is
    /// meant to be used for post processing effects that require screen depth info, eg. DOF or SSAO.
    irr::video::ITexture* getDepthMapTexture() { return DepthRTT; }

    /// This function is now unrelated to shadow mapping. It simply adds a node to the screen space depth map render,
    /// for use
    /// with post processing effects that require screen depth info. If you want the functionality of the old method (A
    /// node that
    /// only casts but does not recieve shadows, use addShadowToNode with the ESM_CAST shadow mode.
    void addNodeToDepthPass(irr::scene::ISceneNode* node);

    /// This function is now unrelated to shadow mapping. It simply removes a node to the screen space depth map render,
    /// for use
    /// with post processing effects that require screen depth info.
    void removeNodeFromDepthPass(irr::scene::ISceneNode* node);

    /// Enables/disables an additional pass before applying post processing effects (If there are any) which records
    /// screen depth info
    /// to the depth buffer for use with post processing effects that require screen depth info, such as SSAO or DOF.
    /// For nodes to be
    /// rendered in this pass, they must first be added using addNodeToDepthPass(SceneNode).
    void enableDepthPass(bool enableDepthPass);

    /// Removes shadows from a scene node.
    void removeShadowFromNode(irr::scene::ISceneNode* node) {
        SShadowNode tmpShadowNode = {node, ESM_RECEIVE, EFT_NONE};
        irr::s32 i = ShadowNodeArray.binary_search(tmpShadowNode);

        if (i != -1)
            ShadowNodeArray.erase(i);
    }

    // Excludes a scene node from lighting calculations, avoiding any side effects that may
    // occur from XEffect's light modulation on this particular scene node.
    void excludeNodeFromLightingCalculations(irr::scene::ISceneNode* node) {
        SShadowNode tmpShadowNode = {node, ESM_EXCLUDE, EFT_NONE};
        ShadowNodeArray.push_back(tmpShadowNode);
    }

    /// Updates the effects handler. This must be done between IVideoDriver::beginScene and IVideoDriver::endScene.
    /// This function now replaces smgr->drawAll(). So place it where smgr->drawAll() would normally go. Please note
    /// that the clear colour from IVideoDriver::beginScene is not preserved, so you must instead specify the clear
    /// colour using EffectHandler::setClearColour(Colour).
    /// A render target may be passed as the output target, else rendering will commence on the backbuffer.
    void update(irr::video::ITexture* outputTarget = 0);

    /// Adds a shadow to the scene node. The filter type specifies how many shadow map samples
    /// to take, a higher value can produce a smoother or softer result. The shadow mode can
    /// be either ESM_BOTH, ESM_CAST, or ESM_RECEIVE. ESM_BOTH casts and receives shadows,
    /// ESM_CAST only casts shadows, and is unaffected by shadows or lighting, and ESM_RECEIVE
    /// only receives but does not cast shadows.
    void addShadowToNode(irr::scene::ISceneNode* node,
                         E_FILTER_TYPE filterType = EFT_NONE,
                         E_SHADOW_MODE shadowMode = ESM_BOTH);

    /// Returns the device time divided by 100, for use with the shader callbacks.
    irr::f32 getTime() { return device->getTimer()->getTime() / 100.0f; }

    /// Sets the scene clear colour, for when the scene is cleared before smgr->drawAll().
    void setClearColour(irr::video::SColor ClearCol) { ClearColour = ClearCol; }

    /**
    A very easy to use post processing function. Simply add a material type to apply to the screen as a post processing
    effect and it will be applied. You can add as many material types as you desire, and they will be double buffered
    and
    executed in sequance.

    For the material types, I recommend using "ScreenQuadCB" as the callback and refering to the texture names that are
    passed
    (When using OpenGL, in DirectX uniforms are not required to bind textures).
    Please note that this will only work in OpenGL on vanilla Irrlicht, DX requires the large RTT patch to be able to
    create
    sufficiently sized rendertargets for post processing. (Or you can just remove the engine check for Pow2).

    The structure of the textures is as follows:

    Texture1 - "ColorMapSampler"
    This is passed on from the previous post processing effect as they are executed in sequance. For example, if you do
    a
    horizontal blur on the first post processing material, then a vertical blur in the second material, you will use
    this
    sampler to access the post processed data of the horizontal blur when it is time to do the vertical blur. If
    accessed
    from the first post processing material, it will just contain the untainted screen map data.

    Texture2 - "ScreenMapSampler"
    The second texture will always contain the untainted screen map data, from when the scene is first rendered. It will
    remain unchanged no matter how many post processing materials are applied. This kind of data is necessary, for
    example
    in bloom or DOF, you would require a copy of the blurred scene data and a copy of the normal untainted, unblurred
    screen
    data, and mix between them based on certain factors such as depth or luminance.

    Texture3 - "DepthMapSampler"
    If a depth pass has been enabled using enableDepthPass, then this sampler will contain the screen space depth
    information.
    For better quality this is encoded to 16bits, and can be decoded like so:
        Texture.red + (Texture.green / 256.0f);
    That is by adding the red channel to the green channel which is first divided by 256.
    The data can still be used without decoding, in 8 bit precision, by just accessing the red component of the texture.
    Though
    this is not recommended as 8 bit precision is usually not sufficient for modern post processing effects.

    Texture4 - "UserMapSampler"
    A custom texture that can be set by the user using setPostProcessingUserTexture(irr::video::ITexture* userTexture).

    The last parameter is the render callback, you may pass 0 if you do not need one.
    Please see IPostProcessingRenderCallback for more info about this callback.
    */
    void addPostProcessingEffect(irr::s32 MaterialType, IPostProcessingRenderCallback* callback = 0);

    /// Sets the IPostProcessingRenderCallback for the specified post processing effect.
    /// The old callback if previously set will be automatically deleted.
    void setPostProcessingRenderCallback(irr::s32 MaterialType, IPostProcessingRenderCallback* callback = 0) {
        SPostProcessingPair tempPair(MaterialType, 0);
        irr::s32 i = PostProcessingRoutines.binary_search(tempPair);

        if (i != -1) {
            if (PostProcessingRoutines[i].renderCallback)
                delete PostProcessingRoutines[i].renderCallback;

            PostProcessingRoutines[i].renderCallback = callback;
        }
    }

    /// Removes the first encountered post processing effect with the specified material type.
    void removePostProcessingEffect(irr::s32 MaterialType) {
        SPostProcessingPair tempPair(MaterialType, 0);
        irr::s32 i = PostProcessingRoutines.binary_search(tempPair);

        if (i != -1) {
            if (PostProcessingRoutines[i].renderCallback)
                delete PostProcessingRoutines[i].renderCallback;

            PostProcessingRoutines.erase(i);
        }
    }

    /// Adds a post processing effect by reading a pixel shader from a file. The vertex shader is taken care of.
    /// The vertex shader will pass the correct screen quad texture coordinates via the TEXCOORD0 semantic in
    /// Direct3D or the gl_TexCoord[0] varying in OpenGL.
    /// See addPostProcessingEffect for more info.
    /// Returns the Irrlicht material type of the post processing effect.
    irr::s32 addPostProcessingEffectFromFile(const irr::core::stringc& filename,
                                             IPostProcessingRenderCallback* callback = 0);

    /// Sets a shader parameter for a post-processing effect. The first parameter is the material type, the second
    /// is the uniform paratmeter name, the third is a float pointer that points to the data and the last is the
    /// component count of the data. Please note that the float pointer must remain valid during render time.
    /// To disable the setting of a parameter you may simply pass a null float pointer.
    void setPostProcessingEffectConstant(const irr::s32 materialType,
                                         const irr::core::stringc& name,
                                         const irr::f32* data,
                                         const irr::u32 count);

    /// Returns the screen quad scene node. This is not required in any way, but some advanced users may want to adjust
    /// its material settings accordingly.
    const CScreenQuad& getScreenQuad() { return ScreenQuad; }

    /// Sets the active scene manager.
    void setActiveSceneManager(irr::scene::ISceneManager* smgrIn) { smgr = smgrIn; }

    /// Gets the active scene manager.
    irr::scene::ISceneManager* getActiveSceneManager() { return smgr; }

    /// This allows the user to specify a custom, fourth texture to be used in the post-processing effects.
    /// See addPostProcessingEffect for more info.
    void setPostProcessingUserTexture(irr::video::ITexture* userTexture) {
        ScreenQuad.getMaterial().setTexture(3, userTexture);
    }

    /// Sets the global ambient color for shadowed scene nodes.
    void setAmbientColor(irr::video::SColor ambientColour) { AmbientColour = ambientColour; }

    /// Gets the global ambient color.
    irr::video::SColor getAmbientColor() const { return AmbientColour; }

    /// Generates a randomized texture composed of uniformly distributed 3 dimensional vectors.
    irr::video::ITexture* generateRandomVectorTexture(const irr::core::dimension2du& dimensions,
                                                      const irr::core::stringc& name = "randVec");

    /// Sets a new screen render target resolution.
    void setScreenRenderTargetResolution(const irr::core::dimension2du& resolution);

    /// Returns the device that this EffectHandler was initialized with.
    irr::IrrlichtDevice* getIrrlichtDevice() { return device; }

  private:
    struct SShadowNode {
        bool operator<(const SShadowNode& other) const { return node < other.node; }

        irr::scene::ISceneNode* node;

        E_SHADOW_MODE shadowMode;
        E_FILTER_TYPE filterType;
    };

    struct SPostProcessingPair {
        SPostProcessingPair(const irr::s32 materialTypeIn,
                            ScreenQuadCB* callbackIn,
                            IPostProcessingRenderCallback* renderCallbackIn = 0)
            : materialType(materialTypeIn), callback(callbackIn), renderCallback(renderCallbackIn) {}

        bool operator<(const SPostProcessingPair& other) const { return materialType < other.materialType; }

        ScreenQuadCB* callback;
        IPostProcessingRenderCallback* renderCallback;
        irr::s32 materialType;
    };

    SPostProcessingPair obtainScreenQuadMaterialFromFile(
        const irr::core::stringc& filename,
        irr::video::E_MATERIAL_TYPE baseMaterial = irr::video::EMT_SOLID);

    irr::IrrlichtDevice* device;
    irr::video::IVideoDriver* driver;
    irr::scene::ISceneManager* smgr;
    irr::core::dimension2du mapRes;

    irr::s32 Depth;
    irr::s32 DepthT;
    irr::s32 DepthWiggle;
    irr::s32 Shadow[EFT_COUNT];
    irr::s32 LightModulate;
    irr::s32 Simple;
    irr::s32 WhiteWash;
    irr::s32 WhiteWashTRef;
    irr::s32 WhiteWashTAdd;
    irr::s32 WhiteWashTAlpha;
    irr::s32 VSMBlurH;
    irr::s32 VSMBlurV;

    DepthShaderCB* depthMC;
    ShadowShaderCB* shadowMC;

    irr::video::ITexture* ScreenRTT;
    irr::video::ITexture* DepthRTT;

    irr::core::array<SPostProcessingPair> PostProcessingRoutines;
    irr::core::array<SShadowLight> LightList;
    irr::core::array<SShadowNode> ShadowNodeArray;
    irr::core::array<irr::scene::ISceneNode*> DepthPassArray;

    irr::core::dimension2du ScreenRTTSize;
    irr::video::SColor ClearColour;
    irr::video::SColor AmbientColour;
    CScreenQuad ScreenQuad;

    bool shadowsUnsupported;
    bool use32BitDepth;
    bool useVSM;
    bool DepthPass;
};

////////////////////////////////// EffectHandler.cpp

inline EffectHandler::EffectHandler(irr::IrrlichtDevice* dev,
                                    const irr::core::dimension2du& screenRTTSize,
                                    const bool useVSMShadows,
                                    const bool useRoundSpotLights,
                                    const bool use32BitDepthBuffers)
    : device(dev),
      smgr(dev->getSceneManager()),
      driver(dev->getVideoDriver()),
      ScreenRTTSize(screenRTTSize.getArea() == 0 ? dev->getVideoDriver()->getScreenSize() : screenRTTSize),
      ClearColour(0x0),
      shadowsUnsupported(false),
      DepthRTT(0),
      DepthPass(false),
      depthMC(0),
      shadowMC(0),
      AmbientColour(0x0),
      use32BitDepth(use32BitDepthBuffers),
      useVSM(useVSMShadows) {
    bool tempTexFlagMipMaps = driver->getTextureCreationFlag(irr::video::ETCF_CREATE_MIP_MAPS);
    bool tempTexFlag32 = driver->getTextureCreationFlag(irr::video::ETCF_ALWAYS_32_BIT);

    ScreenRTT = driver->addRenderTargetTexture(ScreenRTTSize);
    ScreenQuad.rt[0] = driver->addRenderTargetTexture(ScreenRTTSize);
    ScreenQuad.rt[1] = driver->addRenderTargetTexture(ScreenRTTSize);

    driver->setTextureCreationFlag(irr::video::ETCF_CREATE_MIP_MAPS, tempTexFlagMipMaps);
    driver->setTextureCreationFlag(irr::video::ETCF_ALWAYS_32_BIT, tempTexFlag32);

    CShaderPreprocessor sPP(driver);

    E_SHADER_EXTENSION shaderExt = (driver->getDriverType() == irr::video::EDT_DIRECT3D9) ? ESE_HLSL : ESE_GLSL;

    irr::video::IGPUProgrammingServices* gpu = driver->getGPUProgrammingServices();

    if (gpu &&
        ((driver->getDriverType() == irr::video::EDT_OPENGL && driver->queryFeature(irr::video::EVDF_ARB_GLSL)) ||
         (driver->getDriverType() == irr::video::EDT_DIRECT3D9 &&
          driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_2_0)))) {
        depthMC = new DepthShaderCB(this);
        shadowMC = new ShadowShaderCB(this);

        Depth =
            gpu->addHighLevelShaderMaterial(sPP.ppShader(SHADOW_PASS_1V[shaderExt]).c_str(), "vertexMain",
                                            irr::video::EVST_VS_2_0, sPP.ppShader(SHADOW_PASS_1P[shaderExt]).c_str(),
                                            "pixelMain", irr::video::EPST_PS_2_0, depthMC, irr::video::EMT_SOLID);

        DepthT = gpu->addHighLevelShaderMaterial(
            sPP.ppShader(SHADOW_PASS_1V[shaderExt]).c_str(), "vertexMain", irr::video::EVST_VS_2_0,
            sPP.ppShader(SHADOW_PASS_1PT[shaderExt]).c_str(), "pixelMain", irr::video::EPST_PS_2_0, depthMC,
            irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF);

        WhiteWash =
            gpu->addHighLevelShaderMaterial(sPP.ppShader(SHADOW_PASS_1V[shaderExt]).c_str(), "vertexMain",
                                            irr::video::EVST_VS_2_0, sPP.ppShader(WHITE_WASH_P[shaderExt]).c_str(),
                                            "pixelMain", irr::video::EPST_PS_2_0, depthMC, irr::video::EMT_SOLID);

        WhiteWashTRef = gpu->addHighLevelShaderMaterial(
            sPP.ppShader(SHADOW_PASS_1V[shaderExt]).c_str(), "vertexMain", irr::video::EVST_VS_2_0,
            sPP.ppShader(WHITE_WASH_P[shaderExt]).c_str(), "pixelMain", irr::video::EPST_PS_2_0, depthMC,
            irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF);

        WhiteWashTAdd = gpu->addHighLevelShaderMaterial(
            sPP.ppShader(SHADOW_PASS_1V[shaderExt]).c_str(), "vertexMain", irr::video::EVST_VS_2_0,
            sPP.ppShader(WHITE_WASH_P_ADD[shaderExt]).c_str(), "pixelMain", irr::video::EPST_PS_2_0, depthMC,
            irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL);

        WhiteWashTAlpha = gpu->addHighLevelShaderMaterial(
            sPP.ppShader(SHADOW_PASS_1V[shaderExt]).c_str(), "vertexMain", irr::video::EVST_VS_2_0,
            sPP.ppShader(WHITE_WASH_P[shaderExt]).c_str(), "pixelMain", irr::video::EPST_PS_2_0, depthMC,
            irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL);

        if (useRoundSpotLights)
            sPP.addShaderDefine("ROUND_SPOTLIGHTS");

        if (useVSMShadows)
            sPP.addShaderDefine("VSM");

        const irr::u32 sampleCounts[EFT_COUNT] = {1, 4, 8, 12, 16};

        const irr::video::E_VERTEX_SHADER_TYPE vertexProfile = driver->queryFeature(irr::video::EVDF_VERTEX_SHADER_3_0)
                                                                   ? irr::video::EVST_VS_3_0
                                                                   : irr::video::EVST_VS_2_0;

        const irr::video::E_PIXEL_SHADER_TYPE pixelProfile =
            driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_3_0) ? irr::video::EPST_PS_3_0 : irr::video::EPST_PS_2_0;

        for (irr::u32 i = 0; i < EFT_COUNT; i++) {
            sPP.addShaderDefine("SAMPLE_AMOUNT", irr::core::stringc(sampleCounts[i]));
            Shadow[i] = gpu->addHighLevelShaderMaterial(sPP.ppShader(SHADOW_PASS_2V[shaderExt]).c_str(), "vertexMain",
                                                        vertexProfile, sPP.ppShader(SHADOW_PASS_2P[shaderExt]).c_str(),
                                                        "pixelMain", pixelProfile, shadowMC, irr::video::EMT_SOLID);
        }

        // Set resolution preprocessor defines.
        sPP.addShaderDefine("SCREENX", irr::core::stringc(ScreenRTTSize.Width));
        sPP.addShaderDefine("SCREENY", irr::core::stringc(ScreenRTTSize.Height));

        // Create screen quad shader callback.
        ScreenQuadCB* SQCB = new ScreenQuadCB(this, true);

        // Light modulate.
        LightModulate = gpu->addHighLevelShaderMaterial(
            sPP.ppShader(SCREEN_QUAD_V[shaderExt]).c_str(), "vertexMain", vertexProfile,
            sPP.ppShader(LIGHT_MODULATE_P[shaderExt]).c_str(), "pixelMain", pixelProfile, SQCB);

        // Simple present.
        Simple = gpu->addHighLevelShaderMaterial(sPP.ppShader(SCREEN_QUAD_V[shaderExt]).c_str(), "vertexMain",
                                                 vertexProfile, sPP.ppShader(SIMPLE_P[shaderExt]).c_str(), "pixelMain",
                                                 pixelProfile, SQCB, irr::video::EMT_TRANSPARENT_ADD_COLOR);

        // VSM blur.
        VSMBlurH = gpu->addHighLevelShaderMaterial(sPP.ppShader(SCREEN_QUAD_V[shaderExt]).c_str(), "vertexMain",
                                                   vertexProfile, sPP.ppShader(VSM_BLUR_P[shaderExt]).c_str(),
                                                   "pixelMain", pixelProfile, SQCB);

        sPP.addShaderDefine("VERTICAL_VSM_BLUR");

        VSMBlurV = gpu->addHighLevelShaderMaterial(sPP.ppShader(SCREEN_QUAD_V[shaderExt]).c_str(), "vertexMain",
                                                   vertexProfile, sPP.ppShader(VSM_BLUR_P[shaderExt]).c_str(),
                                                   "pixelMain", pixelProfile, SQCB);

        // Drop the screen quad callback.
        SQCB->drop();
    } else {
        Depth = irr::video::EMT_SOLID;
        DepthT = irr::video::EMT_SOLID;
        WhiteWash = irr::video::EMT_SOLID;
        WhiteWashTRef = irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF;
        WhiteWashTAdd = irr::video::EMT_TRANSPARENT_ADD_COLOR;
        WhiteWashTAlpha = irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL;
        Simple = irr::video::EMT_SOLID;

        for (irr::u32 i = 0; i < EFT_COUNT; ++i)
            Shadow[i] = irr::video::EMT_SOLID;

        device->getLogger()->log("XEffects: Shader effects not supported on this system.");
        shadowsUnsupported = true;
    }
}

inline EffectHandler::~EffectHandler() {
    if (ScreenRTT)
        driver->removeTexture(ScreenRTT);

    if (ScreenQuad.rt[0])
        driver->removeTexture(ScreenQuad.rt[0]);

    if (ScreenQuad.rt[1])
        driver->removeTexture(ScreenQuad.rt[1]);

    if (DepthRTT)
        driver->removeTexture(DepthRTT);
}

inline void EffectHandler::setScreenRenderTargetResolution(const irr::core::dimension2du& resolution) {
    bool tempTexFlagMipMaps = driver->getTextureCreationFlag(irr::video::ETCF_CREATE_MIP_MAPS);
    bool tempTexFlag32 = driver->getTextureCreationFlag(irr::video::ETCF_ALWAYS_32_BIT);

    if (ScreenRTT)
        driver->removeTexture(ScreenRTT);

    ScreenRTT = driver->addRenderTargetTexture(resolution);

    if (ScreenQuad.rt[0])
        driver->removeTexture(ScreenQuad.rt[0]);

    ScreenQuad.rt[0] = driver->addRenderTargetTexture(resolution);

    if (ScreenQuad.rt[1])
        driver->removeTexture(ScreenQuad.rt[1]);

    ScreenQuad.rt[1] = driver->addRenderTargetTexture(resolution);

    if (DepthRTT != 0) {
        driver->removeTexture(DepthRTT);
        DepthRTT = driver->addRenderTargetTexture(resolution);
    }

    driver->setTextureCreationFlag(irr::video::ETCF_CREATE_MIP_MAPS, tempTexFlagMipMaps);
    driver->setTextureCreationFlag(irr::video::ETCF_ALWAYS_32_BIT, tempTexFlag32);

    ScreenRTTSize = resolution;
}

inline void EffectHandler::enableDepthPass(bool enableDepthPass) {
    DepthPass = enableDepthPass;

    if (DepthPass && DepthRTT == 0)
        DepthRTT = driver->addRenderTargetTexture(ScreenRTTSize, "depthRTT",
                                                  use32BitDepth ? irr::video::ECF_G32R32F : irr::video::ECF_G16R16F);
}

inline void EffectHandler::addPostProcessingEffect(irr::s32 MaterialType, IPostProcessingRenderCallback* callback) {
    SPostProcessingPair pPair(MaterialType, 0);
    pPair.renderCallback = callback;
    PostProcessingRoutines.push_back(pPair);
}

inline void EffectHandler::addShadowToNode(irr::scene::ISceneNode* node,
                                           E_FILTER_TYPE filterType,
                                           E_SHADOW_MODE shadowMode) {
    SShadowNode snode = {node, shadowMode, filterType};
    ShadowNodeArray.push_back(snode);
}

inline void EffectHandler::addNodeToDepthPass(irr::scene::ISceneNode* node) {
    if (DepthPassArray.binary_search(node) == -1)
        DepthPassArray.push_back(node);
}

inline void EffectHandler::removeNodeFromDepthPass(irr::scene::ISceneNode* node) {
    irr::s32 i = DepthPassArray.binary_search(node);

    if (i != -1)
        DepthPassArray.erase(i);
}

inline void EffectHandler::update(irr::video::ITexture* outputTarget) {
    if (shadowsUnsupported || smgr->getActiveCamera() == 0)
        return;

    this->smgr->getRootSceneNode()->OnAnimate(device->getTimer()->getTime());

    if (!ShadowNodeArray.empty() && !LightList.empty()) {
        driver->setRenderTarget(ScreenQuad.rt[0], true, true, AmbientColour);

        irr::scene::ICameraSceneNode* activeCam = smgr->getActiveCamera();
        activeCam->OnAnimate(device->getTimer()->getTime());
        activeCam->OnRegisterSceneNode();
        activeCam->render();

        const irr::u32 ShadowNodeArraySize = ShadowNodeArray.size();
        const irr::u32 LightListSize = LightList.size();
        for (irr::u32 l = 0; l < LightListSize; ++l) {
            // Set max distance constant for depth shader.
            depthMC->FarLink = LightList[l].getFarValue();

            driver->setTransform(irr::video::ETS_VIEW, LightList[l].getViewMatrix());
            driver->setTransform(irr::video::ETS_PROJECTION, LightList[l].getProjectionMatrix());

            irr::video::ITexture* currentShadowMapTexture = getShadowMapTexture(LightList[l].getShadowMapResolution());
            driver->setRenderTarget(currentShadowMapTexture, true, true, irr::video::SColor(0xffffffff));

            for (irr::u32 i = 0; i < ShadowNodeArraySize; ++i) {
                if (ShadowNodeArray[i].shadowMode == ESM_RECEIVE || ShadowNodeArray[i].shadowMode == ESM_EXCLUDE)
                    continue;

                const irr::u32 CurrentMaterialCount = ShadowNodeArray[i].node->getMaterialCount();
                irr::core::array<irr::s32> BufferMaterialList(CurrentMaterialCount);
                BufferMaterialList.set_used(0);

                for (irr::u32 m = 0; m < CurrentMaterialCount; ++m) {
                    BufferMaterialList.push_back(ShadowNodeArray[i].node->getMaterial(m).MaterialType);
                    ShadowNodeArray[i].node->getMaterial(m).MaterialType = (irr::video::E_MATERIAL_TYPE)(
                        BufferMaterialList[m] == irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF ? DepthT : Depth);
                }

                ShadowNodeArray[i].node->OnAnimate(device->getTimer()->getTime());
                ShadowNodeArray[i].node->render();

                const irr::u32 BufferMaterialListSize = BufferMaterialList.size();
                for (irr::u32 m = 0; m < BufferMaterialListSize; ++m)
                    ShadowNodeArray[i].node->getMaterial(m).MaterialType =
                        (irr::video::E_MATERIAL_TYPE)BufferMaterialList[m];
            }

            // Blur the shadow map texture if we're using VSM filtering.
            if (useVSM) {
                irr::video::ITexture* currentSecondaryShadowMap =
                    getShadowMapTexture(LightList[l].getShadowMapResolution(), true);

                driver->setRenderTarget(currentSecondaryShadowMap, true, true, irr::video::SColor(0xffffffff));
                ScreenQuad.getMaterial().setTexture(0, currentShadowMapTexture);
                ScreenQuad.getMaterial().MaterialType = (irr::video::E_MATERIAL_TYPE)VSMBlurH;

                ScreenQuad.render(driver);

                driver->setRenderTarget(currentShadowMapTexture, true, true, irr::video::SColor(0xffffffff));
                ScreenQuad.getMaterial().setTexture(0, currentSecondaryShadowMap);
                ScreenQuad.getMaterial().MaterialType = (irr::video::E_MATERIAL_TYPE)VSMBlurV;

                ScreenQuad.render(driver);
            }

            driver->setRenderTarget(ScreenQuad.rt[1], true, true, irr::video::SColor(0xffffffff));

            driver->setTransform(irr::video::ETS_VIEW, activeCam->getViewMatrix());
            driver->setTransform(irr::video::ETS_PROJECTION, activeCam->getProjectionMatrix());

            shadowMC->LightColour = LightList[l].getLightColor();
            shadowMC->LightLink = LightList[l].getPosition();
            shadowMC->FarLink = LightList[l].getFarValue();
            shadowMC->ViewLink = LightList[l].getViewMatrix();
            shadowMC->ProjLink = LightList[l].getProjectionMatrix();
            shadowMC->MapRes = (irr::f32)LightList[l].getShadowMapResolution();
            shadowMC->clipborder = LightList[l].getClipBorder();  //***ALEX***

            for (irr::u32 i = 0; i < ShadowNodeArraySize; ++i) {
                if (ShadowNodeArray[i].shadowMode == ESM_CAST || ShadowNodeArray[i].shadowMode == ESM_EXCLUDE)
                    continue;

                const irr::u32 CurrentMaterialCount = ShadowNodeArray[i].node->getMaterialCount();
                irr::core::array<irr::s32> BufferMaterialList(CurrentMaterialCount);
                irr::core::array<irr::video::ITexture*> BufferTextureList(CurrentMaterialCount);

                for (irr::u32 m = 0; m < CurrentMaterialCount; ++m) {
                    BufferMaterialList.push_back(ShadowNodeArray[i].node->getMaterial(m).MaterialType);
                    BufferTextureList.push_back(ShadowNodeArray[i].node->getMaterial(m).getTexture(0));

                    ShadowNodeArray[i].node->getMaterial(m).MaterialType =
                        (irr::video::E_MATERIAL_TYPE)Shadow[ShadowNodeArray[i].filterType];
                    ShadowNodeArray[i].node->getMaterial(m).setTexture(0, currentShadowMapTexture);
                }

                ShadowNodeArray[i].node->OnAnimate(device->getTimer()->getTime());
                ShadowNodeArray[i].node->render();

                for (irr::u32 m = 0; m < CurrentMaterialCount; ++m) {
                    ShadowNodeArray[i].node->getMaterial(m).MaterialType =
                        (irr::video::E_MATERIAL_TYPE)BufferMaterialList[m];
                    ShadowNodeArray[i].node->getMaterial(m).setTexture(0, BufferTextureList[m]);
                }
            }

            driver->setRenderTarget(ScreenQuad.rt[0], false, false, irr::video::SColor(0x0));
            ScreenQuad.getMaterial().setTexture(0, ScreenQuad.rt[1]);
            ScreenQuad.getMaterial().MaterialType = (irr::video::E_MATERIAL_TYPE)Simple;

            ScreenQuad.render(driver);
        }

        // Render all the excluded and casting-only nodes.
        for (irr::u32 i = 0; i < ShadowNodeArraySize; ++i) {
            if (ShadowNodeArray[i].shadowMode != ESM_CAST && ShadowNodeArray[i].shadowMode != ESM_EXCLUDE)
                continue;

            const irr::u32 CurrentMaterialCount = ShadowNodeArray[i].node->getMaterialCount();
            irr::core::array<irr::s32> BufferMaterialList(CurrentMaterialCount);
            BufferMaterialList.set_used(0);

            for (irr::u32 m = 0; m < CurrentMaterialCount; ++m) {
                BufferMaterialList.push_back(ShadowNodeArray[i].node->getMaterial(m).MaterialType);

                switch (BufferMaterialList[m]) {
                    case irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF:
                        ShadowNodeArray[i].node->getMaterial(m).MaterialType =
                            (irr::video::E_MATERIAL_TYPE)WhiteWashTRef;
                        break;
                    case irr::video::EMT_TRANSPARENT_ADD_COLOR:
                        ShadowNodeArray[i].node->getMaterial(m).MaterialType =
                            (irr::video::E_MATERIAL_TYPE)WhiteWashTAdd;
                        break;
                    case irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL:
                        ShadowNodeArray[i].node->getMaterial(m).MaterialType =
                            (irr::video::E_MATERIAL_TYPE)WhiteWashTAlpha;
                        break;
                    default:
                        ShadowNodeArray[i].node->getMaterial(m).MaterialType = (irr::video::E_MATERIAL_TYPE)WhiteWash;
                        break;
                }
            }

            ShadowNodeArray[i].node->OnAnimate(device->getTimer()->getTime());
            ShadowNodeArray[i].node->render();

            for (irr::u32 m = 0; m < CurrentMaterialCount; ++m)
                ShadowNodeArray[i].node->getMaterial(m).MaterialType =
                    (irr::video::E_MATERIAL_TYPE)BufferMaterialList[m];
        }
    } else {
        driver->setRenderTarget(ScreenQuad.rt[0], true, true, irr::video::SColor(0xffffffff));
    }

    driver->setRenderTarget(ScreenQuad.rt[1], true, true, ClearColour);
    smgr->drawAll();

    const irr::u32 PostProcessingRoutinesSize = PostProcessingRoutines.size();

    driver->setRenderTarget(PostProcessingRoutinesSize ? ScreenRTT : outputTarget, true, true, irr::video::SColor(0x0));

    ScreenQuad.getMaterial().setTexture(0, ScreenQuad.rt[1]);
    ScreenQuad.getMaterial().setTexture(1, ScreenQuad.rt[0]);

    ScreenQuad.getMaterial().MaterialType = (irr::video::E_MATERIAL_TYPE)LightModulate;
    ScreenQuad.render(driver);

    // Perform depth pass after rendering, to ensure animations stay up to date.
    if (DepthPass) {
        driver->setRenderTarget(DepthRTT, true, true, irr::video::SColor(0xffffffff));

        // Set max distance constant for depth shader.
        depthMC->FarLink = smgr->getActiveCamera()->getFarValue();

        for (irr::u32 i = 0; i < DepthPassArray.size(); ++i) {
            irr::core::array<irr::s32> BufferMaterialList(DepthPassArray[i]->getMaterialCount());
            BufferMaterialList.set_used(0);

            for (irr::u32 g = 0; g < DepthPassArray[i]->getMaterialCount(); ++g)
                BufferMaterialList.push_back(DepthPassArray[i]->getMaterial(g).MaterialType);

            DepthPassArray[i]->setMaterialType((irr::video::E_MATERIAL_TYPE)Depth);
            DepthPassArray[i]->OnAnimate(device->getTimer()->getTime());
            DepthPassArray[i]->render();

            for (irr::u32 g = 0; g < DepthPassArray[i]->getMaterialCount(); ++g)
                DepthPassArray[i]->getMaterial(g).MaterialType = (irr::video::E_MATERIAL_TYPE)BufferMaterialList[g];
        }

        driver->setRenderTarget(0, false, false);
    }

    if (PostProcessingRoutinesSize) {
        bool Alter = false;
        ScreenQuad.getMaterial().setTexture(1, ScreenRTT);
        ScreenQuad.getMaterial().setTexture(2, DepthRTT);
        for (irr::u32 i = 0; i < PostProcessingRoutinesSize; ++i) {
            ScreenQuad.getMaterial().MaterialType = (irr::video::E_MATERIAL_TYPE)PostProcessingRoutines[i].materialType;

            Alter = !Alter;
            ScreenQuad.getMaterial().setTexture(0, i == 0 ? ScreenRTT : ScreenQuad.rt[int(!Alter)]);
            driver->setRenderTarget(i >= PostProcessingRoutinesSize - 1 ? outputTarget : ScreenQuad.rt[int(Alter)],
                                    true, true, ClearColour);

            if (PostProcessingRoutines[i].renderCallback)
                PostProcessingRoutines[i].renderCallback->OnPreRender(this);
            ScreenQuad.render(driver);
            if (PostProcessingRoutines[i].renderCallback)
                PostProcessingRoutines[i].renderCallback->OnPostRender(this);
        }
    }
}

inline irr::video::ITexture* EffectHandler::getShadowMapTexture(const irr::u32 resolution, const bool secondary) {
    // Using Irrlicht cache now.
    irr::core::stringc shadowMapName = irr::core::stringc("XEFFECTS_SM_") + irr::core::stringc(resolution);

    if (secondary)
        shadowMapName += "_2";

    irr::video::ITexture* shadowMapTexture = driver->getTexture(shadowMapName);

    if (shadowMapTexture == 0) {
        device->getLogger()->log("XEffects: Please ignore previous warning, it is harmless.");

        shadowMapTexture =
            driver->addRenderTargetTexture(irr::core::dimension2du(resolution, resolution), shadowMapName,
                                           use32BitDepth ? irr::video::ECF_G32R32F : irr::video::ECF_G16R16F);
    }

    return shadowMapTexture;
}

inline irr::video::ITexture* EffectHandler::generateRandomVectorTexture(const irr::core::dimension2du& dimensions,
                                                                        const irr::core::stringc& name) {
    irr::video::IImage* tmpImage = driver->createImage(irr::video::ECF_A8R8G8B8, dimensions);

    srand(device->getTimer()->getRealTime());

    for (irr::u32 x = 0; x < dimensions.Width; ++x) {
        for (irr::u32 y = 0; y < dimensions.Height; ++y) {
            irr::core::vector3df randVec;

            // Reject vectors outside the unit sphere to get a uniform distribution.
            do {
                randVec =
                    irr::core::vector3df((irr::f32)rand() / (irr::f32)RAND_MAX, (irr::f32)rand() / (irr::f32)RAND_MAX,
                                         (irr::f32)rand() / (irr::f32)RAND_MAX);
            } while (randVec.getLengthSQ() > 1.0f);

            const irr::video::SColorf randCol(randVec.X, randVec.Y, randVec.Z);
            tmpImage->setPixel(x, y, randCol.toSColor());
        }
    }

    irr::video::ITexture* randTexture = driver->addTexture(name, tmpImage);

    tmpImage->drop();

    return randTexture;
}

inline EffectHandler::SPostProcessingPair EffectHandler::obtainScreenQuadMaterialFromFile(
    const irr::core::stringc& filename,
    irr::video::E_MATERIAL_TYPE baseMaterial) {
    CShaderPreprocessor sPP(driver);

    sPP.addShaderDefine("SCREENX", irr::core::stringc(ScreenRTTSize.Width));
    sPP.addShaderDefine("SCREENY", irr::core::stringc(ScreenRTTSize.Height));

    irr::video::E_VERTEX_SHADER_TYPE VertexLevel =
        driver->queryFeature(irr::video::EVDF_VERTEX_SHADER_3_0) ? irr::video::EVST_VS_3_0 : irr::video::EVST_VS_2_0;
    irr::video::E_PIXEL_SHADER_TYPE PixelLevel =
        driver->queryFeature(irr::video::EVDF_PIXEL_SHADER_3_0) ? irr::video::EPST_PS_2_0 : irr::video::EPST_PS_2_0;

    E_SHADER_EXTENSION shaderExt = (driver->getDriverType() == irr::video::EDT_DIRECT3D9) ? ESE_HLSL : ESE_GLSL;

    irr::video::IGPUProgrammingServices* gpu = driver->getGPUProgrammingServices();

    const irr::core::stringc shaderString = sPP.ppShaderFF(filename.c_str());

    ScreenQuadCB* SQCB = new ScreenQuadCB(this, true);

    irr::s32 PostMat =
        gpu->addHighLevelShaderMaterial(sPP.ppShader(SCREEN_QUAD_V[shaderExt]).c_str(), "vertexMain", VertexLevel,
                                        shaderString.c_str(), "pixelMain", PixelLevel, SQCB, baseMaterial);

    SPostProcessingPair pPair(PostMat, SQCB);

    SQCB->drop();

    return pPair;
}

inline void EffectHandler::setPostProcessingEffectConstant(const irr::s32 materialType,
                                                           const irr::core::stringc& name,
                                                           const irr::f32* data,
                                                           const irr::u32 count) {
    SPostProcessingPair tempPair(materialType, 0);
    irr::s32 matIndex = PostProcessingRoutines.binary_search(tempPair);

    if (matIndex != -1)
        PostProcessingRoutines[matIndex].callback->uniformDescriptors[name] =
            ScreenQuadCB::SUniformDescriptor(data, count);
}

inline irr::s32 EffectHandler::addPostProcessingEffectFromFile(const irr::core::stringc& filename,
                                                               IPostProcessingRenderCallback* callback) {
    SPostProcessingPair pPair = obtainScreenQuadMaterialFromFile(filename);
    pPair.renderCallback = callback;
    PostProcessingRoutines.push_back(pPair);

    return pPair.materialType;
}

///////////////ScreenQuadCB impl.

inline void ScreenQuadCB::OnSetConstants(irr::video::IMaterialRendererServices* services, irr::s32 userData) {
    if (services->getVideoDriver()->getDriverType() == irr::video::EDT_OPENGL) {
        //***ALEX*** modified for Irrlicht 1.8
        /*
        irr::s32 TexVar = 0;
        services->setPixelShaderConstant("ColorMapSampler", &TexVar, 1);

        TexVar = 1;
        services->setPixelShaderConstant("ScreenMapSampler", &TexVar, 1);

        TexVar = 2;
        services->setPixelShaderConstant("DepthMapSampler", &TexVar, 1);

        TexVar = 3;
        services->setPixelShaderConstant("UserMapSampler", &TexVar, 1);
        */
        /// Version for Irrlicht 1.7.3
        irr::u32 TexVar = 0;
        services->setPixelShaderConstant("ColorMapSampler", (irr::f32*)(&TexVar), 1);

        TexVar = 1;
        services->setPixelShaderConstant("ScreenMapSampler", (irr::f32*)(&TexVar), 1);

        TexVar = 2;
        services->setPixelShaderConstant("DepthMapSampler", (irr::f32*)(&TexVar), 1);

        TexVar = 3;
        services->setPixelShaderConstant("UserMapSampler", (irr::f32*)(&TexVar), 1);
    }

    if (defaultVertexShader) {
        const irr::core::dimension2du currentRTTSize = services->getVideoDriver()->getCurrentRenderTargetSize();
        const irr::f32 screenX = (irr::f32)currentRTTSize.Width, screenY = (irr::f32)currentRTTSize.Height;

        services->setVertexShaderConstant("screenX", &screenX, 1);
        services->setVertexShaderConstant("screenY", &screenY, 1);

        irr::scene::ISceneManager* smgr = effect->getActiveSceneManager();
        irr::scene::ICameraSceneNode* cam = smgr->getActiveCamera();

        const irr::core::position2di tLeft = services->getVideoDriver()->getViewPort().UpperLeftCorner;
        const irr::core::position2di bRight = services->getVideoDriver()->getViewPort().LowerRightCorner;

        const irr::core::line3df sLines[4] = {smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(
                                                  irr::core::position2di(tLeft.X, tLeft.Y), cam),
                                              smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(
                                                  irr::core::position2di(bRight.X, tLeft.Y), cam),
                                              smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(
                                                  irr::core::position2di(tLeft.X, bRight.Y), cam),
                                              smgr->getSceneCollisionManager()->getRayFromScreenCoordinates(
                                                  irr::core::position2di(bRight.X, bRight.Y), cam)};

        services->setVertexShaderConstant("LineStarts0", &sLines[0].start.X, 3);
        services->setVertexShaderConstant("LineStarts1", &sLines[1].start.X, 3);
        services->setVertexShaderConstant("LineStarts2", &sLines[2].start.X, 3);
        services->setVertexShaderConstant("LineStarts3", &sLines[3].start.X, 3);

        services->setVertexShaderConstant("LineEnds0", &sLines[0].end.X, 3);
        services->setVertexShaderConstant("LineEnds1", &sLines[1].end.X, 3);
        services->setVertexShaderConstant("LineEnds2", &sLines[2].end.X, 3);
        services->setVertexShaderConstant("LineEnds3", &sLines[3].end.X, 3);
    }

    if (uniformDescriptors.size()) {
        irr::core::map<irr::core::stringc, SUniformDescriptor>::Iterator mapIter = uniformDescriptors.getIterator();

        for (; !mapIter.atEnd(); mapIter++) {
            if (mapIter.getNode()->getValue().fPointer == 0)
                continue;

            services->setPixelShaderConstant(mapIter.getNode()->getKey().c_str(),
                                             mapIter.getNode()->getValue().fPointer,
                                             mapIter.getNode()->getValue().paramCount);
        }
    }
}

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
