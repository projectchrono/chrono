#include <vsg/io/VSG.h>
#include <vsg/io/mem_stream.h>
static auto chronoPbrShader_frag = []() {
static const char str[] = 
R"(#vsga 1.1.0
Root id=1 vsg::ShaderStage
{
  userObjects 0
  mask 18446744073709551615
  stage 16
  entryPointName "main"
  module id=2 vsg::ShaderModule
  {
    userObjects 0
    hints id=0
    source "#version 450
#extension GL_ARB_separate_shader_objects : enable
#pragma import_defines (VSG_DIFFUSE_MAP, VSG_GREYSCALE_DIFFUSE_MAP, VSG_EMISSIVE_MAP, VSG_LIGHTMAP_MAP, VSG_NORMAL_MAP, VSG_METALLROUGHNESS_MAP, VSG_SPECULAR_MAP, VSG_TWO_SIDED_LIGHTING, VSG_WORKFLOW_SPECGLOSS, SHADOWMAP_DEBUG)

#define VIEW_DESCRIPTOR_SET 0
#define MATERIAL_DESCRIPTOR_SET 1

const float PI = 3.14159265359;
const float RECIPROCAL_PI = 0.31830988618;
const float RECIPROCAL_PI2 = 0.15915494;
const float EPSILON = 1e-6;
const float c_MinRoughness = 0.04;

#ifdef VSG_DIFFUSE_MAP
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 0) uniform sampler2D diffuseMap;
#endif

#ifdef VSG_METALLROUGHNESS_MAP
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 1) uniform sampler2D mrMap;
#endif

#ifdef VSG_NORMAL_MAP
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 2) uniform sampler2D normalMap;
#endif

#ifdef VSG_LIGHTMAP_MAP
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 3) uniform sampler2D aoMap;
#endif

#ifdef VSG_EMISSIVE_MAP
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 4) uniform sampler2D emissiveMap;
#endif

#ifdef VSG_SPECULAR_MAP
layout(set = MATERIAL_DESCRIPTOR_SET, binding = 5) uniform sampler2D specularMap;
#endif

layout(set = MATERIAL_DESCRIPTOR_SET, binding = 10) uniform PbrData
{
    vec4 baseColorFactor;
    vec4 emissiveFactor;
    vec4 diffuseFactor;
    vec4 specularFactor;
    float metallicFactor;
    float roughnessFactor;
    float alphaMask;
    float alphaMaskCutoff;
} pbr;

// ViewDependentState
layout(set = VIEW_DESCRIPTOR_SET, binding = 0) uniform LightData
{
    vec4 values[2048];
} lightData;

layout(set = VIEW_DESCRIPTOR_SET, binding = 2) uniform sampler2DArrayShadow shadowMaps;

layout(location = 0) in vec3 eyePos;
layout(location = 1) in vec3 normalDir;
layout(location = 2) in vec4 vertexColor;
layout(location = 3) in vec2 texCoord0;
layout(location = 5) in vec3 viewDir;

layout(location = 0) out vec4 outColor;


// Encapsulate the various inputs used by the various functions in the shading equation
// We store values in this struct to simplify the integration of alternative implementations
// of the shading terms, outlined in the Readme.MD Appendix.
struct PBRInfo
{
    float NdotL;                  // cos angle between normal and light direction
    float NdotV;                  // cos angle between normal and view direction
    float NdotH;                  // cos angle between normal and half vector
    float LdotH;                  // cos angle between light direction and half vector
    float VdotH;                  // cos angle between view direction and half vector
    float VdotL;                  // cos angle between view direction and light direction
    float perceptualRoughness;    // roughness value, as authored by the model creator (input to shader)
    float metalness;              // metallic value at the surface
    vec3 reflectance0;            // full reflectance color (normal incidence angle)
    vec3 reflectance90;           // reflectance color at grazing angle
    float alphaRoughness;         // roughness mapped to a more linear change in the roughness (proposed by [2])
    vec3 diffuseColor;            // color contribution from diffuse lighting
    vec3 specularColor;           // color contribution from specular lighting
};


vec4 SRGBtoLINEAR(vec4 srgbIn)
{
    vec3 linOut = pow(srgbIn.xyz, vec3(2.2));
    return vec4(linOut,srgbIn.w);
}

vec4 LINEARtoSRGB(vec4 srgbIn)
{
    vec3 linOut = pow(srgbIn.xyz, vec3(1.0 / 2.2));
    return vec4(linOut, srgbIn.w);
}

float rcp(const in float value)
{
    return 1.0 / value;
}

float pow5(const in float value)
{
    return value * value * value * value * value;
}

// Find the normal for this fragment, pulling either from a predefined normal map
// or from the interpolated mesh normal and tangent attributes.
vec3 getNormal()
{
    vec3 result;
#ifdef VSG_NORMAL_MAP
    // Perturb normal, see http://www.thetenthplanet.de/archives/1180
    vec3 tangentNormal = texture(normalMap, texCoord0).xyz * 2.0 - 1.0;

    //tangentNormal *= vec3(2,2,1);

    vec3 q1 = dFdx(eyePos);
    vec3 q2 = dFdy(eyePos);
    vec2 st1 = dFdx(texCoord0);
    vec2 st2 = dFdy(texCoord0);

    vec3 N = normalize(normalDir);
    vec3 T = normalize(q1 * st2.t - q2 * st1.t);
    vec3 B = -normalize(cross(N, T));
    mat3 TBN = mat3(T, B, N);

    result = normalize(TBN * tangentNormal);
#else
    result = normalize(normalDir);
#endif
#ifdef VSG_TWO_SIDED_LIGHTING
    if (!gl_FrontFacing)
        result = -result;
#endif
    return result;
}

// Basic Lambertian diffuse
// Implementation from Lambert's Photometria https://archive.org/details/lambertsphotome00lambgoog
// See also [1], Equation 1
vec3 BRDF_Diffuse_Lambert(PBRInfo pbrInputs)
{
    return pbrInputs.diffuseColor * RECIPROCAL_PI;
}

vec3 BRDF_Diffuse_Custom_Lambert(PBRInfo pbrInputs)
{
    return pbrInputs.diffuseColor * RECIPROCAL_PI * pow(pbrInputs.NdotV, 0.5 + 0.3 * pbrInputs.perceptualRoughness);
}

// [Gotanda 2012, \"Beyond a Simple Physically Based Blinn-Phong Model in Real-Time\"]
vec3 BRDF_Diffuse_OrenNayar(PBRInfo pbrInputs)
{
    float a = pbrInputs.alphaRoughness;
    float s = a;// / ( 1.29 + 0.5 * a );
    float s2 = s * s;
    float VoL = 2 * pbrInputs.VdotH * pbrInputs.VdotH - 1;		// double angle identity
    float Cosri = pbrInputs.VdotL - pbrInputs.NdotV * pbrInputs.NdotL;
    float C1 = 1 - 0.5 * s2 / (s2 + 0.33);
    float C2 = 0.45 * s2 / (s2 + 0.09) * Cosri * ( Cosri >= 0 ? 1.0 / max(pbrInputs.NdotL, pbrInputs.NdotV) : 1 );
    return pbrInputs.diffuseColor / PI * ( C1 + C2 ) * ( 1 + pbrInputs.perceptualRoughness * 0.5 );
}

// [Gotanda 2014, \"Designing Reflectance Models for New Consoles\"]
vec3 BRDF_Diffuse_Gotanda(PBRInfo pbrInputs)
{
    float a = pbrInputs.alphaRoughness;
    float a2 = a * a;
    float F0 = 0.04;
    float VoL = 2 * pbrInputs.VdotH * pbrInputs.VdotH - 1;		// double angle identity
    float Cosri = VoL - pbrInputs.NdotV * pbrInputs.NdotL;
    float a2_13 = a2 + 1.36053;
    float Fr = ( 1 - ( 0.542026*a2 + 0.303573*a ) / a2_13 ) * ( 1 - pow( 1 - pbrInputs.NdotV, 5 - 4*a2 ) / a2_13 ) * ( ( -0.733996*a2*a + 1.50912*a2 - 1.16402*a ) * pow( 1 - pbrInputs.NdotV, 1 + rcp(39*a2*a2+1) ) + 1 );
    //float Fr = ( 1 - 0.36 * a ) * ( 1 - pow( 1 - NoV, 5 - 4*a2 ) / a2_13 ) * ( -2.5 * Roughness * ( 1 - NoV ) + 1 );
    float Lm = ( max( 1 - 2*a, 0 ) * ( 1 - pow5( 1 - pbrInputs.NdotL ) ) + min( 2*a, 1 ) ) * ( 1 - 0.5*a * (pbrInputs.NdotL - 1) ) * pbrInputs.NdotL;
    float Vd = ( a2 / ( (a2 + 0.09) * (1.31072 + 0.995584 * pbrInputs.NdotV) ) ) * ( 1 - pow( 1 - pbrInputs.NdotL, ( 1 - 0.3726732 * pbrInputs.NdotV * pbrInputs.NdotV ) / ( 0.188566 + 0.38841 * pbrInputs.NdotV ) ) );
    float Bp = Cosri < 0 ? 1.4 * pbrInputs.NdotV * pbrInputs.NdotL * Cosri : Cosri;
    float Lr = (21.0 / 20.0) * (1 - F0) * ( Fr * Lm + Vd + Bp );
    return pbrInputs.diffuseColor * RECIPROCAL_PI * Lr;
}

vec3 BRDF_Diffuse_Burley(PBRInfo pbrInputs)
{
    float energyBias = mix(pbrInputs.perceptualRoughness, 0.0, 0.5);
    float energyFactor = mix(pbrInputs.perceptualRoughness, 1.0, 1.0 / 1.51);
    float fd90 = energyBias + 2.0 * pbrInputs.VdotH * pbrInputs.VdotH * pbrInputs.perceptualRoughness;
    float f0 = 1.0;
    float lightScatter = f0 + (fd90 - f0) * pow(1.0 - pbrInputs.NdotL, 5.0);
    float viewScatter = f0 + (fd90 - f0) * pow(1.0 - pbrInputs.NdotV, 5.0);

    return pbrInputs.diffuseColor * lightScatter * viewScatter * energyFactor;
}

vec3 BRDF_Diffuse_Disney(PBRInfo pbrInputs)
{
	float Fd90 = 0.5 + 2.0 * pbrInputs.perceptualRoughness * pbrInputs.VdotH * pbrInputs.VdotH;
    vec3 f0 = vec3(0.1);
	vec3 invF0 = vec3(1.0, 1.0, 1.0) - f0;
	float dim = min(invF0.r, min(invF0.g, invF0.b));
	float result = ((1.0 + (Fd90 - 1.0) * pow(1.0 - pbrInputs.NdotL, 5.0 )) * (1.0 + (Fd90 - 1.0) * pow(1.0 - pbrInputs.NdotV, 5.0 ))) * dim;
	return pbrInputs.diffuseColor * result;
}

// The following equation models the Fresnel reflectance term of the spec equation (aka F())
// Implementation of fresnel from [4], Equation 15
vec3 specularReflection(PBRInfo pbrInputs)
{
    //return pbrInputs.reflectance0 + (pbrInputs.reflectance90 - pbrInputs.reflectance0) * pow(clamp(1.0 - pbrInputs.VdotH, 0.0, 1.0), 5.0);
    return pbrInputs.reflectance0 + (pbrInputs.reflectance90 - pbrInputs.reflectance90*pbrInputs.reflectance0) * exp2((-5.55473 * pbrInputs.VdotH - 6.98316) * pbrInputs.VdotH);
}

// This calculates the specular geometric attenuation (aka G()),
// where rougher material will reflect less light back to the viewer.
// This implementation is based on [1] Equation 4, and we adopt their modifications to
// alphaRoughness as input as originally proposed in [2].
float geometricOcclusion(PBRInfo pbrInputs)
{
    float NdotL = pbrInputs.NdotL;
    float NdotV = pbrInputs.NdotV;
    float r = pbrInputs.alphaRoughness * pbrInputs.alphaRoughness;

    float attenuationL = 2.0 * NdotL / (NdotL + sqrt(r + (1.0 - r) * (NdotL * NdotL)));
    float attenuationV = 2.0 * NdotV / (NdotV + sqrt(r + (1.0 - r) * (NdotV * NdotV)));
    return attenuationL * attenuationV;
}

// The following equation(s) model the distribution of microfacet normals across the area being drawn (aka D())
// Implementation from \"Average Irregularity Representation of a Roughened Surface for Ray Reflection\" by T. S. Trowbridge, and K. P. Reitz
// Follows the distribution function recommended in the SIGGRAPH 2013 course notes from EPIC Games [1], Equation 3.
float microfacetDistribution(PBRInfo pbrInputs)
{
    float roughnessSq = pbrInputs.alphaRoughness * pbrInputs.alphaRoughness;
    float f = (pbrInputs.NdotH * roughnessSq - pbrInputs.NdotH) * pbrInputs.NdotH + 1.0;
    return roughnessSq / (PI * f * f);
}

vec3 BRDF(vec3 u_LightColor, vec3 v, vec3 n, vec3 l, vec3 h, float perceptualRoughness, float metallic, vec3 specularEnvironmentR0, vec3 specularEnvironmentR90, float alphaRoughness, vec3 diffuseColor, vec3 specularColor, float ao)
{
    float unclmapped_NdotL = dot(n, l);

    vec3 reflection = -normalize(reflect(v, n));
    reflection.y *= -1.0f;

    float NdotL = clamp(unclmapped_NdotL, 0.001, 1.0);
    float NdotV = clamp(abs(dot(n, v)), 0.001, 1.0);
    float NdotH = clamp(dot(n, h), 0.0, 1.0);
    float LdotH = clamp(dot(l, h), 0.0, 1.0);
    float VdotH = clamp(dot(v, h), 0.0, 1.0);
    float VdotL = clamp(dot(v, l), 0.0, 1.0);

    PBRInfo pbrInputs = PBRInfo(NdotL,
                                NdotV,
                                NdotH,
                                LdotH,
                                VdotH,
                                VdotL,
                                perceptualRoughness,
                                metallic,
                                specularEnvironmentR0,
                                specularEnvironmentR90,
                                alphaRoughness,
                                diffuseColor,
                                specularColor);

    // Calculate the shading terms for the microfacet specular shading model
    vec3 F = specularReflection(pbrInputs);
    float G = geometricOcclusion(pbrInputs);
    float D = microfacetDistribution(pbrInputs);

    // Calculation of analytical lighting contribution
    vec3 diffuseContrib = (1.0 - F) * BRDF_Diffuse_Disney(pbrInputs);
    vec3 specContrib = F * G * D / (4.0 * NdotL * NdotV);
    // Obtain final intensity as reflectance (BRDF) scaled by the energy of the light (cosine law)
    vec3 color = NdotL * u_LightColor * (diffuseContrib + specContrib);

    color *= ao;

#ifdef VSG_EMISSIVE_MAP
    vec3 emissive = SRGBtoLINEAR(texture(emissiveMap, texCoord0)).rgb * pbr.emissiveFactor.rgb;
#else
    vec3 emissive = pbr.emissiveFactor.rgb;
#endif
    color += emissive;

    return color;
}

float convertMetallic(vec3 diffuse, vec3 specular, float maxSpecular)
{
    float perceivedDiffuse = sqrt(0.299 * diffuse.r * diffuse.r + 0.587 * diffuse.g * diffuse.g + 0.114 * diffuse.b * diffuse.b);
    float perceivedSpecular = sqrt(0.299 * specular.r * specular.r + 0.587 * specular.g * specular.g + 0.114 * specular.b * specular.b);

    if (perceivedSpecular < c_MinRoughness)
    {
        return 0.0;
    }

    float a = c_MinRoughness;
    float b = perceivedDiffuse * (1.0 - maxSpecular) / (1.0 - c_MinRoughness) + perceivedSpecular - 2.0 * c_MinRoughness;
    float c = c_MinRoughness - perceivedSpecular;
    float D = max(b * b - 4.0 * a * c, 0.0);
    return clamp((-b + sqrt(D)) / (2.0 * a), 0.0, 1.0);
}

void main()
{
    float brightnessCutoff = 0.001;

    float perceptualRoughness = 0.0;
    float metallic;
    vec3 diffuseColor;
    vec4 baseColor;

    float ambientOcclusion = 1.0;

    vec3 f0 = vec3(0.04);

#ifdef VSG_DIFFUSE_MAP
    #ifdef VSG_GREYSCALE_DIFFUSE_MAP
        float v = texture(diffuseMap, texCoord0.st).s * pbr.baseColorFactor;
        baseColor = vertexColor * vec4(v, v, v, 1.0);
    #else
        baseColor = vertexColor * SRGBtoLINEAR(texture(diffuseMap, texCoord0)) * pbr.baseColorFactor;
    #endif
#else
    baseColor = vertexColor * pbr.baseColorFactor;
#endif

    if (pbr.alphaMask == 1.0f)
    {
        if (baseColor.a < pbr.alphaMaskCutoff)
            discard;
    }

#ifdef VSG_WORKFLOW_SPECGLOSS
    #ifdef VSG_DIFFUSE_MAP
        vec4 diffuse = SRGBtoLINEAR(texture(diffuseMap, texCoord0));
    #else
        vec4 diffuse = vec4(1.0);
    #endif

    #ifdef VSG_SPECULAR_MAP
        vec4 specular_texel = texture(specularMap, texCoord0);
        vec3 specular = SRGBtoLINEAR(specular_texel).rgb;
        perceptualRoughness = 1.0 - specular_texel.a;
    #else
        vec3 specular = vec3(0.0);
        perceptualRoughness = 0.0;
    #endif

        float maxSpecular = max(max(specular.r, specular.g), specular.b);

        // Convert metallic value from specular glossiness inputs
        metallic = convertMetallic(diffuse.rgb, specular, maxSpecular);

        const float epsilon = 1e-6;
        vec3 baseColorDiffusePart = diffuse.rgb * ((1.0 - maxSpecular) / (1 - c_MinRoughness) / max(1 - metallic, epsilon)) * pbr.diffuseFactor.rgb;
        vec3 baseColorSpecularPart = specular - (vec3(c_MinRoughness) * (1 - metallic) * (1 / max(metallic, epsilon))) * pbr.specularFactor.rgb;
        baseColor = vec4(mix(baseColorDiffusePart, baseColorSpecularPart, metallic * metallic), diffuse.a);
#else
        perceptualRoughness = pbr.roughnessFactor;
        metallic = pbr.metallicFactor;

    #ifdef VSG_METALLROUGHNESS_MAP
        vec4 mrSample = texture(mrMap, texCoord0);
        perceptualRoughness = mrSample.g * perceptualRoughness;
        metallic = mrSample.b * metallic;
    #endif
#endif

#ifdef VSG_LIGHTMAP_MAP
    ambientOcclusion = texture(aoMap, texCoord0).r;
#endif

    diffuseColor = baseColor.rgb * (vec3(1.0) - f0);
    diffuseColor *= 1.0 - metallic;

    float alphaRoughness = perceptualRoughness * perceptualRoughness;

    vec3 specularColor = mix(f0, baseColor.rgb, metallic);

    // Compute reflectance.
    float reflectance = max(max(specularColor.r, specularColor.g), specularColor.b);

    // For typical incident reflectance range (between 4% to 100%) set the grazing reflectance to 100% for typical fresnel effect.
    // For very low reflectance range on highly diffuse objects (below 4%), incrementally reduce grazing reflecance to 0%.
    float reflectance90 = clamp(reflectance * 25.0, 0.0, 1.0);
    vec3 specularEnvironmentR0 = specularColor.rgb;
    vec3 specularEnvironmentR90 = vec3(1.0, 1.0, 1.0) * reflectance90;

    vec3 n = getNormal();
    vec3 v = normalize(viewDir);    // Vector from surface point to camera

    float shininess = 100.0f;

    vec3 color = vec3(0.0, 0.0, 0.0);

    vec4 lightNums = lightData.values[0];
    int numAmbientLights = int(lightNums[0]);
    int numDirectionalLights = int(lightNums[1]);
    int numPointLights = int(lightNums[2]);
    int numSpotLights = int(lightNums[3]);
    int index = 1;

    if (numAmbientLights>0)
    {
        // ambient lights
        for(int i = 0; i<numAmbientLights; ++i)
        {
            vec4 ambient_color = lightData.values[index++];
)"
R"(            color += (baseColor.rgb * ambient_color.rgb) * (ambient_color.a * ambientOcclusion);
        }
    }

    // index used to step through the shadowMaps array
    int shadowMapIndex = 0;

    if (numDirectionalLights>0)
    {
        // directional lights
        for(int i = 0; i<numDirectionalLights; ++i)
        {
            vec4 lightColor = lightData.values[index++];
            vec3 direction = -lightData.values[index++].xyz;
            vec4 shadowMapSettings = lightData.values[index++];

            float brightness = lightColor.a;

            // check shadow maps if required
            bool matched = false;
            while ((shadowMapSettings.r > 0.0 && brightness > brightnessCutoff) && !matched)
            {
                mat4 sm_matrix = mat4(lightData.values[index++],
                                      lightData.values[index++],
                                      lightData.values[index++],
                                      lightData.values[index++]);

                vec4 sm_tc = (sm_matrix) * vec4(eyePos, 1.0);

                if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 /* && sm_tc.z <= 1.0*/)
                {
                    matched = true;

                    float coverage = texture(shadowMaps, vec4(sm_tc.st, shadowMapIndex, sm_tc.z)).r;
                    brightness *= (1.0-coverage);

#ifdef SHADOWMAP_DEBUG
                    if (shadowMapIndex==0) color = vec3(1.0, 0.0, 0.0);
                    else if (shadowMapIndex==1) color = vec3(0.0, 1.0, 0.0);
                    else if (shadowMapIndex==2) color = vec3(0.0, 0.0, 1.0);
                    else if (shadowMapIndex==3) color = vec3(1.0, 1.0, 0.0);
                    else if (shadowMapIndex==4) color = vec3(0.0, 1.0, 1.0);
                    else color = vec3(1.0, 1.0, 1.0);
#endif
                }

                ++shadowMapIndex;
                shadowMapSettings.r -= 1.0;
            }

            if (shadowMapSettings.r > 0.0)
            {
                // skip lightData and shadowMap entries for shadow maps that we haven't visited for this light
                // so subsequent light pointions are correct.
                index += 4 * int(shadowMapSettings.r);
                shadowMapIndex += int(shadowMapSettings.r);
            }

            // if light is too dim/shadowed to effect the rendering skip it
            if (brightness <= brightnessCutoff ) continue;

            vec3 l = direction;         // Vector from surface point to light
            vec3 h = normalize(l+v);    // Half vector between both l and v
            float scale = brightness;

            color.rgb += BRDF(lightColor.rgb * scale, v, n, l, h, perceptualRoughness, metallic, specularEnvironmentR0, specularEnvironmentR90, alphaRoughness, diffuseColor, specularColor, ambientOcclusion);
        }
    }

    if (numPointLights>0)
    {
        // point light
        for(int i = 0; i<numPointLights; ++i)
        {
            vec4 lightColor = lightData.values[index++];
            vec3 position = lightData.values[index++].xyz;

            vec3 delta = position - eyePos;
            float distance2 = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
            vec3 direction = delta / sqrt(distance2);

            vec3 l = direction;         // Vector from surface point to light
            vec3 h = normalize(l+v);    // Half vector between both l and v
            float scale = lightColor.a / distance2;

            color.rgb += BRDF(lightColor.rgb * scale, v, n, l, h, perceptualRoughness, metallic, specularEnvironmentR0, specularEnvironmentR90, alphaRoughness, diffuseColor, specularColor, ambientOcclusion);
        }
    }

    if (numSpotLights>0)
    {
        // spot light
        for(int i = 0; i<numSpotLights; ++i)
        {
            vec4 lightColor = lightData.values[index++];
            vec4 position_cosInnerAngle = lightData.values[index++];
            vec4 lightDirection_cosOuterAngle = lightData.values[index++];

            vec3 delta = position_cosInnerAngle.xyz - eyePos;
            float distance2 = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
            vec3 direction = delta / sqrt(distance2);
            float dot_lightdirection = -dot(lightDirection_cosOuterAngle.xyz, direction);

            vec3 l = direction;        // Vector from surface point to light
            vec3 h = normalize(l+v);    // Half vector between both l and v
            float scale = (lightColor.a * smoothstep(lightDirection_cosOuterAngle.w, position_cosInnerAngle.w, dot_lightdirection)) / distance2;

            color.rgb += BRDF(lightColor.rgb * scale, v, n, l, h, perceptualRoughness, metallic, specularEnvironmentR0, specularEnvironmentR90, alphaRoughness, diffuseColor, specularColor, ambientOcclusion);
        }
    }

    outColor = LINEARtoSRGB(vec4(color, baseColor.a));
}
"
    code 6057
     119734787 65536 524299 960 0 131089 1 393227 1 1280527431 1685353262 808793134
     0 196622 0 1 720911 4 4 1852399981 0 70 371 441
     600 945 959 196624 4 7 196611 2 450 589828 1096764487 1935622738
     1918988389 1600484449 1684105331 1868526181 1667590754 29556 262149 4 1852399981 0 458757 11
     1162758476 1869894209 1111970387 879130152 59 262149 10 1650946675 28233 327685 15 1316250983
     1634562671 10348 262149 17 1230127696 7300718 327686 17 0 1953457230 76 327686
     17 1 1953457230 86 327686 17 2 1953457230 72 327686 17 3
     1953457228 72 327686 17 4 1953457238 72 327686 17 5 1953457238 76
     524294 17 6 1668441456 1970565221 1867672673 1852335989 7566181 393222 17 7 1635018093
     1936027244 115 458758 17 8 1818649970 1635017573 811950958 0 458758 17 9
     1818649970 1635017573 962945902 48 458758 17 10 1752198241 1970229857 1701734503 29555 458758
     17 11 1717987684 1130722165 1919904879 0 458758 17 12 1667592307 1918987381 1869377347
     114 1441797 21 1178882626 1718174815 1702065510 1936278623 679044462 1970435187 1345156195 1850298946 1714253670
     828779825 758212141 1714237798 828779825 758212141 1982673254 1982673766 1714238310 1719020849 1719020851 3879219 327685
     20 1232233072 1953853550 115 1441797 24 1667592307 1918987381 1818649938 1769235301 1932029551 1668641396
     1112550772 1718503762 828779887 758212141 1714237798 828779825 758212141 1714237798 1719020849 1719020851 828779827 862352941
     862352941 15153 327685 23 1232233072 1953853550 115 1441797 28 1836016999 1769108581 1667452771
     1769174380 1932029551 1668641396 1112550772 1718503762 828779887 758212141 1714237798 828779825 758212141 1714237798 1719020849
     1719020851 828779827 862352941 862352941 15153 327685 27 1232233072 1953853550 115 1507333 31
     1919117677 1667327599 1766093925 1769108595 1769239906 1932029551 1668641396 1112550772 1718503762 828779887 758212141 1714237798
     828779825 758212141 1714237798 1719020849 1719020851 828779827 862352941 862352941 15153 327685 30 1232233072
     1953853550 115 1048581 49 1178882626 862352936 862352955 862352955 862352955 862352955 993093179 1983590758
     1983591270 1715155814 1719024433 1719024435 828783411 59 393221 36 1766612853 1131702375 1919904879 0
     196613 37 118 196613 38 110 196613 39 108 196613 40 104
     458757 41 1668441456 1970565221 1867672673 1852335989 7566181 327685 42 1635018093 1667853420 0
     524293 43 1667592307 1918987381 1769369157 1835954034 1383362149 48 524293 44 1667592307 1918987381
     1769369157 1835954034 1383362149 12345 393221 45 1752198241 1970229857 1701734503 29555 393221 46
     1717987684 1130722165 1919904879 0 393221 47 1667592307 1918987381 1869377347 114 196613 48
     28513 262149 51 1332636012 29813 262149 68 1970496882 29804 327685 70 1836216174
     1766091873 114 262149 76 809067590 0 196613 92 12390 262149 95 1182166633
     48 196613 100 7170404 262149 112 1970496882 29804 262149 168 1953457230 76
     262149 171 1953457230 86 196613 174 114 393221 181 1702130785 1952544110 1282305897
     0 393221 196 1702130785 1952544110 1450078057 0 327685 216 1735749490 1936027240 7426931
     196613 222 102 458757 244 1818455669 1886413165 1314874469 1282699108 0 327685 248
     1818649970 1769235301 28271 262149 259 1953457230 76 262149 263 1953457230 86 262149
     269 1953457230 72 262149 275 1953457228 72 262149 280 1953457238 72 262149
     285 1953457238 76 327685 290 1232233072 1953853550 115 196613 305 70 262149
     306 1634886000 109 196613 309 71 262149 310 1634886000 109 196613 313
     68 262149 314 1634886000 109 393221 317 1717987684 1130722165 1920233071 25193 262149
     321 1634886000 109 327685 325 1667592307 1953394499 6449522 262149 338 1869377379 114
     327685 349 1936289125 1702259059 0 262149 350 1148346960 6386785 458758 350 0
     1702060386 1869377347 1667319410 7499636 458758 350 1 1936289125 1702259059 1952670022 29295 458758
     350 2 1717987684 1181053813 1869898593 114 458758 350 3 1667592307 1918987381 1952670022
     29295 458758 350 4 1635018093 1667853420 1952670022 29295 458758 350 5 1735749490
     1936027240 1667319411 7499636 393222 350 6 1752198241 1935756641 107 458758 350 7
     1752198241 1935756641 1953842027 6710895 196613 352 7496304 458757 363 1734963810 1701737576 1967354739
     1717989236 0 458757 364 1668441456 1970565221 1867672673 1852335989 7566181 458757 365 1768058209
     1333030501 1970037603 1852795251 0 196613 366 12390 327685 369 1702060386 1869377347 114
     327685 371 1953654134 1866692709 7499628 327685 395 1635018093 1667853420 0 393221 398
     1717987684 1130722165 1919904879 0 393221 408 1752198241 1970229857 1701734503 29555 393221 412
     1667592307 1918987381 1869377347 114 327685 419 1818649970 1635017573 6644590 393221 428 1818649970
     1635017573 962945902 48 524293 433 1667592307 1918987381 1769369157 1835954034 1383362149 48 524293
     435 1667592307 1918987381 1769369157 1835954034 1383362149 12345 196613 438 110 196613 440
     118 262149 441 2003134838 7498052 327685 444 1852401779 1936027241 115 262149 446
     1869377379 114 327685 448 1751607660 1836404340 115 327685 451 1751607628 1952531572 97
     327686 451 0 1970037110 29541 327685 453 1751607660 1952531572 97 458757 457
     1097692526 1701405293 1766618222 1937008743 0 524293 461 1148024174 1667592809 1852795252 1766616161 1937008743
     0 393221 465 1349350766 1953393007 1751607628 29556 393221 469 1399682414 1282699120 1952999273
     115 262149 473 1701080681 120 196613 478 105 393221 487 1768058209 1601465957
     1869377379 114 393221 506 1684105331 1632466799 1684949360 30821 196613 511 105 327685
     520 1751607660 1819231092 29295 327685 525 1701996900 1869182051 110 458757 532 1684105331
     1632466799 1952797552 1735289204 115 327685 537 1734963810 1701737576 29555 262149 541 1668571501
     6579560 327685 560 1834970483 1769108577 120 262149 598 1952410995 99 262149 600
     1348827493 29551 327685 637 1702260579 1701273970 0 327685 641 1684105331 1632466799 29552
     196613 687 108 196613 689 104 262149 694 1818321779 101 262149 700
     1634886000 109 262149 701 1634886000 109 262149 703 1634886000 109 262149 705
     1634886000 109 262149 707 1634886000 109 262149 709 1634886000 109 262149 711
     1634886000 109 262149 713 1634886000 109 262149 715 1634886000 109 262149 717
     1634886000 109 262149 719 1634886000 109 262149 721 1634886000 109 262149 723
     1634886000 109 196613 734 105 327685 743 1751607660 1819231092 29295 327685 748
     1769172848 1852795252 0 262149 754 1953260900 97 327685 758 1953720676 1701015137 50
     327685 776 1701996900 1869182051 110 196613 782 108 196613 784 104 262149
     789 1818321779 101 262149 798 1634886000 109 262149 799 1634886000 109 262149
     801 1634886000 109 262149 803 1634886000 109 262149 805 1634886000 109 262149
     807 1634886000 109 262149 809 1634886000 109 262149 811 1634886000 109 262149
     813 1634886000 109 262149 815 1634886000 109 262149 817 1634886000 109 262149
     819 1634886000 109 262149 821 1634886000 109 196613 832 105 327685 841
     1751607660 1819231092 29295 524293 846 1769172848 1852795252 1936679775 1701736009 1735278962 25964 655365
     851 1751607660 1919501428 1769235301 1667198575 1968141167 1098016116 1701603182 0 262149 856 1953260900
     97 327685 861 1953720676 1701015137 50 327685 879 1701996900 1869182051 110 458757
     885 1601466212 1751607660 1919509620 1769235301 28271 196613 891 108 196613 893 104
     262149 898 1818321779 101 262149 914 1634886000 109 262149 915 1634886000 109
     262149 917 1634886000 109 262149 919 1634886000 109 262149 921 1634886000 109
     262149 923 1634886000 109 262149 925 1634886000 109 262149 927 1634886000 109
     262149 929 1634886000 109 262149 931 1634886000 109 262149 933 1634886000 109
     262149 935 1634886000 109 262149 937 1634886000 109 327685 945 1131705711 1919904879
     0 262149 953 1634886000 109 327685 959 1131963764 1685221231 48 262215 70
     30 1 327752 350 0 35 0 327752 350 1 35 16
     327752 350 2 35 32 327752 350 3 35 48 327752 350
     4 35 64 327752 350 5 35 68 327752 350 6 35
     72 327752 350 7 35 76 196679 350 2 262215 352 34
     1 262215 352 33 10 262215 371 30 2 262215 441 30
     5 262215 450 6 16 327752 451 0 35 0 196679 451
     2 262215 453 34 0 262215 453 33 0 262215 600 30
     0 262215 641 34 0 262215 641 33 2 262215 945 30
     0 262215 959 30 3 131091 2 196641 3 2 196630 6
     32 262167 7 6 4 262176 8 7 7 262177 9 7
     8 262167 13 6 3 196641 14 13 983070 17 6 6
     6 6 6 6 6 6 13 13 6 13 13 262176
     18 7 17 262177 19 13 18 262177 26 6 18 262176
     33 7 13 262176 34 7 6 1048609 35 13 33 33
     33 33 33 34 34 33 33 34 33 33 34 262187
     6 54 1055439407 393260 13 55 54 54 54 262165 58 32
     0 262187 58 59 3 262176 69 1 13 262203 69 70
     1 262187 6 77 1056964608 262187 6 78 1073741824 262165 79 32
     1 262187 79 80 6 262187 79 84 4 262187 6 93
     1036831949 393260 13 94 93 93 93 262187 6 96 1065353216 393260
     13 97 96 96 96 262187 58 101 0 262187 58 104
     1 262187 58 107 2 262187 79 115 0 262187 6 119
     1084227584 262187 79 125 1 262187 79 135 11 262187 79 142
     8 262187 79 145 9 262187 6 154 3232874585 262187 6 158
     1088386572 262187 79 175 10 262187 79 223 2 262187 6 236
     1078530011 262187 6 254 3212836864 262187 6 261 981668463 262187 6 273
     0 262187 6 331 1082130432 655390 350 7 7 7 7 6
     6 6 6 262176 351 2 350 262203 351 352 2 262176
     353 2 7 262187 6 367 1025758986 393260 13 368 367 367
     367 262176 370 1 7 262203 370 371 1 262176 376 2
     6 131092 379 262187 79 385 7 262187 79 392 5 262187
     6 430 1103626240 262203 69 441 1 262187 6 445 1120403456 393260
     13 447 273 273 273 262187 58 449 2048 262172 450 7
     449 196638 451 450 262176 452 2 451 262203 452 453 2
     262176 456 7 79 262176 540 7 379 196650 379 542 262168
     558 7 4 262176 559 7 558 262203 69 600 1 196649
     379 636 589849 638 6 1 1 1 0 1 0 196635
     639 638 262176 640 0 639 262203 640 641 0 262167 643
     6 2 262176 944 3 7 262203 944 945 3 262187 6
     955 1050868099 262187 6 956 1042479491 262187 6 957 897988541 262176 958
     1 643 262203 958 959 1 327734 2 4 0 3 131320
     5 262203 34 363 7 262203 34 364 7 262203 34 365
     7 262203 33 366 7 262203 8 369 7 262203 34 395
     7 262203 33 398 7 262203 34 408 7 262203 33 412
     7 262203 34 419 7 262203 34 428 7 262203 33 433
     7 262203 33 435 7 262203 33 438 7 262203 33 440
     7 262203 34 444 7 262203 33 446 7 262203 8 448
     7 262203 456 457 7 262203 456 461 7 262203 456 465
     7 262203 456 469 7 262203 456 473 7 262203 456 478
     7 262203 8 487 7 262203 456 506 7 262203 456 511
     7 262203 8 520 7 262203 33 525 7 262203 8 532
     7 262203 34 537 7 262203 540 541 7 262203 559 560
     7 262203 8 598 7 262203 34 637 7 262203 33 687
     7 262203 33 689 7 262203 34 694 7 262203 33 700
     7 262203 33 701 7 262203 33 703 7 262203 33 705
     7 262203 33 707 7 262203 34 709 7 262203 34 711
     7 262203 33 713 7 262203 33 715 7 262203 34 717
     7 262203 33 719 7 262203 33 721 7 262203 34 723
     7 262203 456 734 7 262203 8 743 7 262203 33 748
     7 262203 33 754 7 262203 34 758 7 262203 33 776
     7 262203 33 782 7 262203 33 784 7 262203 34 789
     7 262203 33 798 7 262203 33 799 7 262203 33 801
)"
R"(     7 262203 33 803 7 262203 33 805 7 262203 34 807
     7 262203 34 809 7 262203 33 811 7 262203 33 813
     7 262203 34 815 7 262203 33 817 7 262203 33 819
     7 262203 34 821 7 262203 456 832 7 262203 8 841
     7 262203 8 846 7 262203 8 851 7 262203 33 856
     7 262203 34 861 7 262203 33 879 7 262203 34 885
     7 262203 33 891 7 262203 33 893 7 262203 34 898
     7 262203 33 914 7 262203 33 915 7 262203 33 917
     7 262203 33 919 7 262203 33 921 7 262203 34 923
     7 262203 34 925 7 262203 33 927 7 262203 33 929
     7 262203 34 931 7 262203 33 933 7 262203 33 935
     7 262203 34 937 7 262203 8 953 7 196670 363 261
     196670 364 273 196670 365 96 196670 366 368 262205 7 372
     371 327745 353 373 352 115 262205 7 374 373 327813 7
     375 372 374 196670 369 375 327745 376 377 352 80 262205
     6 378 377 327860 379 380 378 96 196855 382 0 262394
     380 381 382 131320 381 327745 34 383 369 59 262205 6
     384 383 327745 376 386 352 385 262205 6 387 386 327864
     379 388 384 387 196855 390 0 262394 388 389 390 131320
     389 65788 131320 390 131321 382 131320 382 327745 376 393 352
     392 262205 6 394 393 196670 364 394 327745 376 396 352
     84 262205 6 397 396 196670 395 397 262205 7 399 369
     524367 13 400 399 399 0 1 2 262205 13 401 366
     327811 13 402 97 401 327813 13 403 400 402 196670 398
     403 262205 6 404 395 327811 6 405 96 404 262205 13
     406 398 327822 13 407 406 405 196670 398 407 262205 6
     409 364 262205 6 410 364 327813 6 411 409 410 196670
     408 411 262205 13 413 366 262205 7 414 369 524367 13
     415 414 414 0 1 2 262205 6 416 395 393296 13
     417 416 416 416 524300 13 418 1 46 413 415 417
     196670 412 418 327745 34 420 412 101 262205 6 421 420
     327745 34 422 412 104 262205 6 423 422 458764 6 424
     1 40 421 423 327745 34 425 412 107 262205 6 426
     425 458764 6 427 1 40 424 426 196670 419 427 262205
     6 429 419 327813 6 431 429 430 524300 6 432 1
     43 431 273 96 196670 428 432 262205 13 434 412 196670
     433 434 262205 6 436 428 327822 13 437 97 436 196670
     435 437 262201 13 439 15 196670 438 439 262205 13 442
     441 393228 13 443 1 69 442 196670 440 443 196670 444
     445 196670 446 447 393281 353 454 453 115 115 262205 7
     455 454 196670 448 455 327745 34 458 448 101 262205 6
     459 458 262254 79 460 459 196670 457 460 327745 34 462
     448 104 262205 6 463 462 262254 79 464 463 196670 461
     464 327745 34 466 448 107 262205 6 467 466 262254 79
     468 467 196670 465 468 327745 34 470 448 59 262205 6
     471 470 262254 79 472 471 196670 469 472 196670 473 125
     262205 79 474 457 327853 379 475 474 115 196855 477 0
     262394 475 476 477 131320 476 196670 478 115 131321 479 131320
     479 262390 481 482 0 131321 483 131320 483 262205 79 484
     478 262205 79 485 457 327857 379 486 484 485 262394 486
     480 481 131320 480 262205 79 488 473 327808 79 489 488
     125 196670 473 489 393281 353 490 453 115 488 262205 7
     491 490 196670 487 491 262205 7 492 369 524367 13 493
     492 492 0 1 2 262205 7 494 487 524367 13 495
     494 494 0 1 2 327813 13 496 493 495 327745 34
     497 487 59 262205 6 498 497 262205 6 499 365 327813
     6 500 498 499 327822 13 501 496 500 262205 13 502
     446 327809 13 503 502 501 196670 446 503 131321 482 131320
     482 262205 79 504 478 327808 79 505 504 125 196670 478
     505 131321 479 131320 481 131321 477 131320 477 196670 506 115
     262205 79 507 461 327853 379 508 507 115 196855 510 0
     262394 508 509 510 131320 509 196670 511 115 131321 512 131320
     512 262390 514 515 0 131321 516 131320 516 262205 79 517
     511 262205 79 518 461 327857 379 519 517 518 262394 519
     513 514 131320 513 262205 79 521 473 327808 79 522 521
     125 196670 473 522 393281 353 523 453 115 521 262205 7
     524 523 196670 520 524 262205 79 526 473 327808 79 527
     526 125 196670 473 527 393281 353 528 453 115 526 262205
     7 529 528 524367 13 530 529 529 0 1 2 262271
     13 531 530 196670 525 531 262205 79 533 473 327808 79
     534 533 125 196670 473 534 393281 353 535 453 115 533
     262205 7 536 535 196670 532 536 327745 34 538 520 59
     262205 6 539 538 196670 537 539 196670 541 542 131321 543
     131320 543 262390 545 546 0 131321 547 131320 547 327745 34
     548 532 101 262205 6 549 548 327866 379 550 549 273
     262205 6 551 537 262205 6 552 363 327866 379 553 551
     552 327847 379 554 550 553 262205 379 555 541 262312 379
     556 555 327847 379 557 554 556 262394 557 544 545 131320
     544 262205 79 561 473 327808 79 562 561 125 196670 473
     562 393281 353 563 453 115 561 262205 7 564 563 262205
     79 565 473 327808 79 566 565 125 196670 473 566 393281
     353 567 453 115 565 262205 7 568 567 262205 79 569
     473 327808 79 570 569 125 196670 473 570 393281 353 571
     453 115 569 262205 7 572 571 262205 79 573 473 327808
     79 574 573 125 196670 473 574 393281 353 575 453 115
     573 262205 7 576 575 327761 6 577 564 0 327761 6
     578 564 1 327761 6 579 564 2 327761 6 580 564
     3 327761 6 581 568 0 327761 6 582 568 1 327761
     6 583 568 2 327761 6 584 568 3 327761 6 585
     572 0 327761 6 586 572 1 327761 6 587 572 2
     327761 6 588 572 3 327761 6 589 576 0 327761 6
     590 576 1 327761 6 591 576 2 327761 6 592 576
     3 458832 7 593 577 578 579 580 458832 7 594 581
     582 583 584 458832 7 595 585 586 587 588 458832 7
     596 589 590 591 592 458832 558 597 593 594 595 596
     196670 560 597 262205 558 599 560 262205 13 601 600 327761
     6 602 601 0 327761 6 603 601 1 327761 6 604
     601 2 458832 7 605 602 603 604 96 327825 7 606
     599 605 196670 598 606 327745 34 607 598 101 262205 6
     608 607 327870 379 609 608 273 196855 611 0 262394 609
     610 611 131320 610 327745 34 612 598 101 262205 6 613
     612 327868 379 614 613 96 131321 611 131320 611 458997 379
     615 609 544 614 610 196855 617 0 262394 615 616 617
     131320 616 327745 34 618 598 104 262205 6 619 618 327870
     379 620 619 273 131321 617 131320 617 458997 379 621 615
     611 620 616 196855 623 0 262394 621 622 623 131320 622
     327745 34 624 598 104 262205 6 625 624 327868 379 626
     625 96 131321 623 131320 623 458997 379 627 621 617 626
     622 196855 629 0 262394 627 628 629 131320 628 327745 34
     630 598 107 262205 6 631 630 327870 379 632 631 273
     131321 629 131320 629 458997 379 633 627 623 632 628 196855
     635 0 262394 633 634 635 131320 634 196670 541 636 262205
     639 642 641 262205 7 644 598 458831 643 645 644 644
     0 1 262205 79 646 506 262255 6 647 646 327745 34
     648 598 107 262205 6 649 648 327761 6 650 645 0
     327761 6 651 645 1 458832 7 652 650 651 647 649
     327761 6 653 652 3 393305 6 654 642 652 653 196670
     637 654 262205 6 655 637 327811 6 656 96 655 262205
     6 657 537 327813 6 658 657 656 196670 537 658 131321
     635 131320 635 262205 79 659 506 327808 79 660 659 125
     196670 506 660 327745 34 661 532 101 262205 6 662 661
     327811 6 663 662 96 327745 34 664 532 101 196670 664
     663 131321 546 131320 546 131321 543 131320 545 327745 34 665
     532 101 262205 6 666 665 327866 379 667 666 273 196855
     669 0 262394 667 668 669 131320 668 327745 34 670 532
     101 262205 6 671 670 262254 79 672 671 327812 79 673
     84 672 262205 79 674 473 327808 79 675 674 673 196670
     473 675 327745 34 676 532 101 262205 6 677 676 262254
     79 678 677 262205 79 679 506 327808 79 680 679 678
     196670 506 680 131321 669 131320 669 262205 6 681 537 262205
     6 682 363 327868 379 683 681 682 196855 685 0 262394
     683 684 685 131320 684 131321 515 131320 685 262205 13 688
     525 196670 687 688 262205 13 690 687 262205 13 691 440
     327809 13 692 690 691 393228 13 693 1 69 692 196670
     689 693 262205 6 695 537 196670 694 695 262205 7 696
     520 524367 13 697 696 696 0 1 2 262205 6 698
     694 327822 13 699 697 698 196670 700 699 262205 13 702
     440 196670 701 702 262205 13 704 438 196670 703 704 262205
     13 706 687 196670 705 706 262205 13 708 689 196670 707
     708 262205 6 710 364 196670 709 710 262205 6 712 395
     196670 711 712 262205 13 714 433 196670 713 714 262205 13
     716 435 196670 715 716 262205 6 718 408 196670 717 718
     262205 13 720 398 196670 719 720 262205 13 722 412 196670
     721 722 262205 6 724 365 196670 723 724 1114169 13 725
     49 700 701 703 705 707 709 711 713 715 717 719
     721 723 262205 13 726 446 327809 13 727 726 725 196670
     446 727 131321 515 131320 515 262205 79 728 511 327808 79
     729 728 125 196670 511 729 131321 512 131320 514 131321 510
     131320 510 262205 79 730 465 327853 379 731 730 115 196855
     733 0 262394 731 732 733 131320 732 196670 734 115 131321
     735 131320 735 262390 737 738 0 131321 739 131320 739 262205
     79 740 734 262205 79 741 465 327857 379 742 740 741
     262394 742 736 737 131320 736 262205 79 744 473 327808 79
     745 744 125 196670 473 745 393281 353 746 453 115 744
     262205 7 747 746 196670 743 747 262205 79 749 473 327808
     79 750 749 125 196670 473 750 393281 353 751 453 115
     749 262205 7 752 751 524367 13 753 752 752 0 1
     2 196670 748 753 262205 13 755 748 262205 13 756 600
     327811 13 757 755 756 196670 754 757 327745 34 759 754
     101 262205 6 760 759 327745 34 761 754 101 262205 6
     762 761 327813 6 763 760 762 327745 34 764 754 104
     262205 6 765 764 327745 34 766 754 104 262205 6 767
     766 327813 6 768 765 767 327809 6 769 763 768 327745
     34 770 754 107 262205 6 771 770 327745 34 772 754
     107 262205 6 773 772 327813 6 774 771 773 327809 6
     775 769 774 196670 758 775 262205 13 777 754 262205 6
     778 758 393228 6 779 1 31 778 393296 13 780 779
     779 779 327816 13 781 777 780 196670 776 781 262205 13
     783 776 196670 782 783 262205 13 785 782 262205 13 786
     440 327809 13 787 785 786 393228 13 788 1 69 787
     196670 784 788 327745 34 790 743 59 262205 6 791 790
     262205 6 792 758 327816 6 793 791 792 196670 789 793
     262205 7 794 743 524367 13 795 794 794 0 1 2
     262205 6 796 789 327822 13 797 795 796 196670 798 797
     262205 13 800 440 196670 799 800 262205 13 802 438 196670
     801 802 262205 13 804 782 196670 803 804 262205 13 806
     784 196670 805 806 262205 6 808 364 196670 807 808 262205
     6 810 395 196670 809 810 262205 13 812 433 196670 811
     812 262205 13 814 435 196670 813 814 262205 6 816 408
     196670 815 816 262205 13 818 398 196670 817 818 262205 13
     820 412 196670 819 820 262205 6 822 365 196670 821 822
     1114169 13 823 49 798 799 801 803 805 807 809 811
     813 815 817 819 821 262205 13 824 446 327809 13 825
     824 823 196670 446 825 131321 738 131320 738 262205 79 826
     734 327808 79 827 826 125 196670 734 827 131321 735 131320
     737 131321 733 131320 733 262205 79 828 469 327853 379 829
     828 115 196855 831 0 262394 829 830 831 131320 830 196670
     832 115 131321 833 131320 833 262390 835 836 0 131321 837
     131320 837 262205 79 838 832 262205 79 839 469 327857 379
     840 838 839 262394 840 834 835 131320 834 262205 79 842
     473 327808 79 843 842 125 196670 473 843 393281 353 844
     453 115 842 262205 7 845 844 196670 841 845 262205 79
     847 473 327808 79 848 847 125 196670 473 848 393281 353
     849 453 115 847 262205 7 850 849 196670 846 850 262205
     79 852 473 327808 79 853 852 125 196670 473 853 393281
     353 854 453 115 852 262205 7 855 854 196670 851 855
     262205 7 857 846 524367 13 858 857 857 0 1 2
     262205 13 859 600 327811 13 860 858 859 196670 856 860
     327745 34 862 856 101 262205 6 863 862 327745 34 864
     856 101 262205 6 865 864 327813 6 866 863 865 327745
     34 867 856 104 262205 6 868 867 327745 34 869 856
     104 262205 6 870 869 327813 6 871 868 870 327809 6
     872 866 871 327745 34 873 856 107 262205 6 874 873
     327745 34 875 856 107 262205 6 876 875 327813 6 877
     874 876 327809 6 878 872 877 196670 861 878 262205 13
     880 856 262205 6 881 861 393228 6 882 1 31 881
     393296 13 883 882 882 882 327816 13 884 880 883 196670
     879 884 262205 7 886 851 524367 13 887 886 886 0
     1 2 262205 13 888 879 327828 6 889 887 888 262271
     6 890 889 196670 885 890 262205 13 892 879 196670 891
     892 262205 13 894 891 262205 13 895 440 327809 13 896
     894 895 393228 13 897 1 69 896 196670 893 897 327745
     34 899 841 59 262205 6 900 899 327745 34 901 851
     59 262205 6 902 901 327745 34 903 846 59 262205 6
     904 903 262205 6 905 885 524300 6 906 1 49 902
     904 905 327813 6 907 900 906 262205 6 908 861 327816
     6 909 907 908 196670 898 909 262205 7 910 841 524367
     13 911 910 910 0 1 2 262205 6 912 898 327822
     13 913 911 912 196670 914 913 262205 13 916 440 196670
     915 916 262205 13 918 438 196670 917 918 262205 13 920
     891 196670 919 920 262205 13 922 893 196670 921 922 262205
     6 924 364 196670 923 924 262205 6 926 395 196670 925
     926 262205 13 928 433 196670 927 928 262205 13 930 435
     196670 929 930 262205 6 932 408 196670 931 932 262205 13
     934 398 196670 933 934 262205 13 936 412 196670 935 936
     262205 6 938 365 196670 937 938 1114169 13 939 49 914
     915 917 919 921 923 925 927 929 931 933 935 937
     262205 13 940 446 327809 13 941 940 939 196670 446 941
     131321 836 131320 836 262205 79 942 832 327808 79 943 942
     125 196670 832 943 131321 833 131320 835 131321 831 131320 831
     262205 13 946 446 327745 34 947 369 59 262205 6 948
     947 327761 6 949 946 0 327761 6 950 946 1 327761
     6 951 946 2 458832 7 952 949 950 951 948 196670
     953 952 327737 7 954 11 953 196670 945 954 65789 65592
     327734 7 11 0 9 196663 8 10 131320 12 262203 33
     51 7 262205 7 52 10 524367 13 53 52 52 0
     1 2 458764 13 56 1 26 53 55 196670 51 56
     262205 13 57 51 327745 34 60 10 59 262205 6 61
     60 327761 6 62 57 0 327761 6 63 57 1 327761
     6 64 57 2 458832 7 65 62 63 64 61 131326
     65 65592 327734 13 15 0 14 131320 16 262203 33 68
     7 262205 13 71 70 393228 13 72 1 69 71 196670
     68 72 262205 13 73 68 131326 73 65592 327734 13 21
     0 19 196663 18 20 131320 22 262203 34 76 7 262203
     33 92 7 262203 33 95 7 262203 34 100 7 262203
     34 112 7 327745 34 81 20 80 262205 6 82 81
     327813 6 83 78 82 327745 34 85 20 84 262205 6
     86 85 327813 6 87 83 86 327745 34 88 20 84
     262205 6 89 88 327813 6 90 87 89 327809 6 91
     77 90 196670 76 91 196670 92 94 262205 13 98 92
     327811 13 99 97 98 196670 95 99 327745 34 102 95
     101 262205 6 103 102 327745 34 105 95 104 262205 6
     106 105 327745 34 108 95 107 262205 6 109 108 458764
     6 110 1 37 106 109 458764 6 111 1 37 103
     110 196670 100 111 262205 6 113 76 327811 6 114 113
     96 327745 34 116 20 115 262205 6 117 116 327811 6
     118 96 117 458764 6 120 1 26 118 119 327813 6
     121 114 120 327809 6 122 96 121 262205 6 123 76
     327811 6 124 123 96 327745 34 126 20 125 262205 6
     127 126 327811 6 128 96 127 458764 6 129 1 26
     128 119 327813 6 130 124 129 327809 6 131 96 130
     327813 6 132 122 131 262205 6 133 100 327813 6 134
     132 133 196670 112 134 327745 33 136 20 135 262205 13
     137 136 262205 6 138 112 327822 13 139 137 138 131326
     139 65592 327734 13 24 0 19 196663 18 23 131320 25
     327745 33 143 23 142 262205 13 144 143 327745 33 146
     23 145 262205 13 147 146 327745 33 148 23 145 262205
     13 149 148 327745 33 150 23 142 262205 13 151 150
     327813 13 152 149 151 327811 13 153 147 152 327745 34
     155 23 84 262205 6 156 155 327813 6 157 154 156
     327811 6 159 157 158 327745 34 160 23 84 262205 6
     161 160 327813 6 162 159 161 393228 6 163 1 29
     162 327822 13 164 153 163 327809 13 165 144 164 131326
     165 65592 327734 6 28 0 26 196663 18 27 131320 29
     262203 34 168 7 262203 34 171 7 262203 34 174 7
     262203 34 181 7 262203 34 196 7 327745 34 169 27
     115 262205 6 170 169 196670 168 170 327745 34 172 27
     125 262205 6 173 172 196670 171 173 327745 34 176 27
     175 262205 6 177 176 327745 34 178 27 175 262205 6
     179 178 327813 6 180 177 179 196670 174 180 262205 6
     182 168 327813 6 183 78 182 262205 6 184 168 262205
     6 185 174 262205 6 186 174 327811 6 187 96 186
     262205 6 188 168 262205 6 189 168 327813 6 190 188
     189 327813 6 191 187 190 327809 6 192 185 191 393228
)"
R"(     6 193 1 31 192 327809 6 194 184 193 327816 6
     195 183 194 196670 181 195 262205 6 197 171 327813 6
     198 78 197 262205 6 199 171 262205 6 200 174 262205
     6 201 174 327811 6 202 96 201 262205 6 203 171
     262205 6 204 171 327813 6 205 203 204 327813 6 206
     202 205 327809 6 207 200 206 393228 6 208 1 31
     207 327809 6 209 199 208 327816 6 210 198 209 196670
     196 210 262205 6 211 181 262205 6 212 196 327813 6
     213 211 212 131326 213 65592 327734 6 31 0 26 196663
     18 30 131320 32 262203 34 216 7 262203 34 222 7
     327745 34 217 30 175 262205 6 218 217 327745 34 219
     30 175 262205 6 220 219 327813 6 221 218 220 196670
     216 221 327745 34 224 30 223 262205 6 225 224 262205
     6 226 216 327813 6 227 225 226 327745 34 228 30
     223 262205 6 229 228 327811 6 230 227 229 327745 34
     231 30 223 262205 6 232 231 327813 6 233 230 232
     327809 6 234 233 96 196670 222 234 262205 6 235 216
     262205 6 237 222 327813 6 238 236 237 262205 6 239
     222 327813 6 240 238 239 327816 6 241 235 240 131326
     241 65592 327734 13 49 0 35 196663 33 36 196663 33
     37 196663 33 38 196663 33 39 196663 33 40 196663 34
     41 196663 34 42 196663 33 43 196663 33 44 196663 34
     45 196663 33 46 196663 33 47 196663 34 48 131320 50
     262203 34 244 7 262203 33 248 7 262203 34 259 7
     262203 34 263 7 262203 34 269 7 262203 34 275 7
     262203 34 280 7 262203 34 285 7 262203 18 290 7
     262203 33 305 7 262203 18 306 7 262203 34 309 7
     262203 18 310 7 262203 34 313 7 262203 18 314 7
     262203 33 317 7 262203 18 321 7 262203 33 325 7
     262203 33 338 7 262203 33 349 7 262205 13 245 38
     262205 13 246 39 327828 6 247 245 246 196670 244 247
     262205 13 249 37 262205 13 250 38 458764 13 251 1
     71 249 250 393228 13 252 1 69 251 262271 13 253
     252 196670 248 253 327745 34 255 248 104 262205 6 256
     255 327813 6 257 256 254 327745 34 258 248 104 196670
     258 257 262205 6 260 244 524300 6 262 1 43 260
     261 96 196670 259 262 262205 13 264 38 262205 13 265
     37 327828 6 266 264 265 393228 6 267 1 4 266
     524300 6 268 1 43 267 261 96 196670 263 268 262205
     13 270 38 262205 13 271 40 327828 6 272 270 271
     524300 6 274 1 43 272 273 96 196670 269 274 262205
     13 276 39 262205 13 277 40 327828 6 278 276 277
     524300 6 279 1 43 278 273 96 196670 275 279 262205
     13 281 37 262205 13 282 40 327828 6 283 281 282
     524300 6 284 1 43 283 273 96 196670 280 284 262205
     13 286 37 262205 13 287 39 327828 6 288 286 287
     524300 6 289 1 43 288 273 96 196670 285 289 262205
     6 291 259 262205 6 292 263 262205 6 293 269 262205
     6 294 275 262205 6 295 280 262205 6 296 285 262205
     6 297 41 262205 6 298 42 262205 13 299 43 262205
     13 300 44 262205 6 301 45 262205 13 302 46 262205
     13 303 47 1048656 17 304 291 292 293 294 295 296
     297 298 299 300 301 302 303 196670 290 304 262205 17
     307 290 196670 306 307 327737 13 308 24 306 196670 305
     308 262205 17 311 290 196670 310 311 327737 6 312 28
     310 196670 309 312 262205 17 315 290 196670 314 315 327737
     6 316 31 314 196670 313 316 262205 13 318 305 393296
     13 319 96 96 96 327811 13 320 319 318 262205 17
     322 290 196670 321 322 327737 13 323 21 321 327813 13
     324 320 323 196670 317 324 262205 13 326 305 262205 6
     327 309 327822 13 328 326 327 262205 6 329 313 327822
     13 330 328 329 262205 6 332 259 327813 6 333 331
     332 262205 6 334 263 327813 6 335 333 334 393296 13
     336 335 335 335 327816 13 337 330 336 196670 325 337
     262205 6 339 259 262205 13 340 36 327822 13 341 340
     339 262205 13 342 317 262205 13 343 325 327809 13 344
     342 343 327813 13 345 341 344 196670 338 345 262205 6
     346 48 262205 13 347 338 327822 13 348 347 346 196670
     338 348 327745 353 354 352 125 262205 7 355 354 524367
     13 356 355 355 0 1 2 196670 349 356 262205 13
     357 349 262205 13 358 338 327809 13 359 358 357 196670
     338 359 262205 13 360 338 131326 360 65592
  }
  NumSpecializationConstants 0
}
)";
vsg::VSG io;
return io.read_cast<vsg::ShaderStage>(reinterpret_cast<const uint8_t*>(str), sizeof(str));
};
