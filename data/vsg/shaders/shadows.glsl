layout(set = VIEW_DESCRIPTOR_SET, binding = 2) uniform texture2DArray shadowMaps;
#ifdef VSG_SHADOWS_PCSS
layout(set = VIEW_DESCRIPTOR_SET, binding = 3) uniform sampler shadowMapDirectSampler;
#endif
layout(set = VIEW_DESCRIPTOR_SET, binding = 4) uniform sampler shadowMapShadowSampler;

#if defined(VSG_SHADOWS_PCSS) || defined(VSG_SHADOWS_SOFT)
layout(constant_id = 0) const int shadowSamples = 16;

const int POISSON_DISK_SAMPLE_COUNT = 64;
const vec2 POISSON_DISK[POISSON_DISK_SAMPLE_COUNT] = {
    vec2(0.171472949063334, -0.8759142124127289),
    vec2(-0.1597672367362282, -0.9870644436411294),
    vec2(0.459697404042723, -0.8797412480505925),
    vec2(-0.3406895149639895, -0.82344072126091),
    vec2(-0.18011878164860493, -0.752673990614504),
    vec2(-0.01884770433660303, -0.7254192612116284),
    vec2(0.6144282076078238, -0.7121903439292749),
    vec2(0.1819601661258839, -0.6295125014796584),
    vec2(0.5696397140833599, -0.44800960234710685),
    vec2(-0.5311960973007043, -0.624279464282425),
    vec2(0.020650613816033614, -0.5581860968799359),
    vec2(0.38703123536143025, -0.5641755522846804),
    vec2(0.9052817101356537, -0.39936079009693787),
    vec2(0.7580135740654145, -0.4786856314890952),
    vec2(-0.30411976742779834, -0.48241538424801134),
    vec2(-0.505431959819757, -0.4276317626664276),
    vec2(-0.6600816830594087, -0.6331140935867211),
    vec2(-0.7975056676071671, -0.40957824210315424),
    vec2(0.2924778762580064, -0.3475145711718533),
    vec2(0.44252241482953325, -0.2659707461940645),
    vec2(-0.6241270408248075, -0.23094051769162596),
    vec2(-0.9138660377712776, -0.24042345091987666),
    vec2(-0.35570851106582574, -0.24762365106169779),
    vec2(0.9537344682210409, -0.13057946999488368),
    vec2(-0.006900019862990347, -0.25144628066897423),
    vec2(-0.698289978266092, -0.044637697647857646),
    vec2(0.5998885897645027, -0.23256852527042843),
    vec2(0.5582293140488868, 0.05634098985743327),
    vec2(-0.22782389126784444, -0.041355782331441264),
    vec2(-0.7026586581344971, 0.07399959917682604),
    vec2(0.3811626945604113, -0.05806040976666236),
    vec2(-0.1730576116608928, 0.11740348906220209),
    vec2(-0.5029332998560511, 0.1810152662195826),
    vec2(0.9806786102912116, 0.12146111532730393),
    vec2(0.023426166165222726, 0.013188008019334365),
    vec2(0.6705878741872812, 0.16439250606226838),
    vec2(0.7851830096686888, -0.01013595379709466),
    vec2(-0.07188359766368346, 0.20583196941854698),
    vec2(-0.8952731933727088, 0.2335154567298469),
    vec2(0.9060311492430866, 0.294333514916297),
    vec2(0.420534723772714, 0.6756751045566158),
    vec2(0.5386689661544318, 0.3389959292899256),
    vec2(-0.6730733987387932, 0.2931916967070102),
    vec2(-0.24810171005618564, 0.3605606959031977),
    vec2(-0.9048852880468572, 0.011094737605371743),
    vec2(0.6159910237674525, 0.47463589141275503),
    vec2(-0.07721288648155807, 0.39524667291002724),
    vec2(-0.8240173658848018, 0.5137812748207496),
    vec2(0.4364030788088093, 0.18255601185026235),
    vec2(0.19648517082664976, 0.5733772368850192),
    vec2(-0.364784578862518, 0.4992754241918426),
    vec2(-0.0603147280376663, 0.576476143664273),
    vec2(0.2267580025181592, 0.25817128520342786),
    vec2(-0.3982961540074632, 0.6813355551493548),
    vec2(0.7876024137377209, 0.5134471337683711),
    vec2(-0.22000277759700929, 0.7390659411823756),
    vec2(-0.26204354493233917, 0.88198927824126),
    vec2(0.1275517630407843, 0.7737872016994914),
    vec2(0.6428791055357954, 0.6889132748855609),
    vec2(0.463017393827676, 0.8217837822840931),
    vec2(-0.6630404728880878, 0.48627036913572896),
    vec2(-0.07705431457109976, 0.8326383985261969),
    vec2(0.11566549974066362, 0.9702036838357961),
    vec2(-0.42746287957561835, 0.8371864490608351),
};

// Interleaved Gradient Noise
// https://www.iryoku.com/next-generation-post-processing-in-call-of-duty-advanced-warfare
float quick_hash(vec2 pos) {
    const vec3 magic = vec3(0.06711056f, 0.00583715f, 52.9829189f);
    return fract(magic.z * fract(dot(pos, magic.xy)));
}
#endif


#ifdef VSG_SHADOWS_PCSS
#include "shadows_pcss.glsl"
#endif

#ifdef VSG_SHADOWS_SOFT
#include "shadows_soft.glsl"
#endif

#ifdef VSG_SHADOWS_HARD
#include "shadows_hard.glsl"
#endif

float calculateShadowCoverageForDirectionalLight(int lightDataIndex, int shadowMapIndex, vec3 T, vec3 B, inout vec3 color)
{
    vec4 shadowMapSettings = lightData.values[lightDataIndex];
    int shadowMapCount = int(shadowMapSettings.r);
    if (shadowMapCount > 0)
    {
        if (shadowMapSettings.g < 0.0)
        {
#ifdef VSG_SHADOWS_HARD
            return calculateShadowCoverageForDirectionalLightHard(lightDataIndex, shadowMapIndex, color);
#else
            return 0;
#endif
        }
        else if (shadowMapSettings.b < 0.0)
        {
#ifdef VSG_SHADOWS_SOFT
            return calculateShadowCoverageForDirectionalLightSoft(lightDataIndex, shadowMapIndex, T, B, color);
#elif defined(VSG_SHADOWS_HARD)
            return calculateShadowCoverageForDirectionalLightHard(lightDataIndex, shadowMapIndex, color);
#else
            return 0;
#endif
        }
        else
        {
#ifdef VSG_SHADOWS_PCSS
            return calculateShadowCoverageForDirectionalLightPCSS(lightDataIndex, shadowMapIndex, T, B, color);
#elif defined(VSG_SHADOWS_HARD)
            return calculateShadowCoverageForDirectionalLightHard(lightDataIndex, shadowMapIndex, color);
#else
            return 0;
#endif
        }
    }
    return 0;
}

float calculateShadowCoverageForSpotLight(int lightDataIndex, int shadowMapIndex, vec3 T, vec3 B, float lightDist, inout vec3 color)
{
    vec4 shadowMapSettings = lightData.values[lightDataIndex];
    int shadowMapCount = int(shadowMapSettings.r);
    if (shadowMapCount > 0)
    {
        if (shadowMapSettings.g < 0.0)
        {
#ifdef VSG_SHADOWS_HARD
            return calculateShadowCoverageForSpotLightHard(lightDataIndex, shadowMapIndex, color);
#else
            return 0;
#endif
        }
        else if (shadowMapSettings.b < 0.0)
        {
#ifdef VSG_SHADOWS_SOFT
            return calculateShadowCoverageForSpotLightSoft(lightDataIndex, shadowMapIndex, T, B, color);
#elif defined(VSG_SHADOWS_HARD)
            return calculateShadowCoverageForSpotLightHard(lightDataIndex, shadowMapIndex, color);
#else
            return 0;
#endif
        }
        else
        {
#ifdef VSG_SHADOWS_PCSS
            return calculateShadowCoverageForSpotLightPCSS(lightDataIndex, shadowMapIndex, T, B, lightDist, color);
#elif defined(VSG_SHADOWS_HARD)
            return calculateShadowCoverageForSpotLightHard(lightDataIndex, shadowMapIndex, color);
#else
            return 0;
#endif
        }
    }
    return 0;
}
