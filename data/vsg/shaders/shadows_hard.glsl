float calculateShadowCoverageForDirectionalLightHard(int lightDataIndex, int shadowMapIndex, inout vec3 color)
{
    vec4 shadowMapSettings = lightData.values[lightDataIndex++];
    int shadowMapCount = int(shadowMapSettings.r);

    while (shadowMapCount > 0)
    {
        float overallCoverage = 0;
        mat4 sm_matrix = mat4(lightData.values[lightDataIndex],
                              lightData.values[lightDataIndex+1],
                              lightData.values[lightDataIndex+2],
                              lightData.values[lightDataIndex+3]);

        vec4 sm_tc = sm_matrix * vec4(eyePos, 1.0);
        sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);

        if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 && sm_tc.z <= 1.0)
        {
            overallCoverage = texture(sampler2DArrayShadow(shadowMaps, shadowMapShadowSampler), vec4(sm_tc.st, shadowMapIndex, sm_tc.z)).r;

#ifdef SHADOWMAP_DEBUG
            if (shadowMapIndex==0) color = vec3(1.0, 0.0, 0.0);
            else if (shadowMapIndex==1) color = vec3(0.0, 1.0, 0.0);
            else if (shadowMapIndex==2) color = vec3(0.0, 0.0, 1.0);
            else if (shadowMapIndex==3) color = vec3(1.0, 1.0, 0.0);
            else if (shadowMapIndex==4) color = vec3(0.0, 1.0, 1.0);
            else color = vec3(1.0, 1.0, 1.0);
#endif

            return overallCoverage;
        }

        lightDataIndex += 8;
        ++shadowMapIndex;
        --shadowMapCount;
    }

    return 0.0;
}

float calculateShadowCoverageForSpotLightHard(int lightDataIndex, int shadowMapIndex, inout vec3 color)
{
    // identical to directional lights
    vec4 shadowMapSettings = lightData.values[lightDataIndex++];
    int shadowMapCount = int(shadowMapSettings.r);

    while (shadowMapCount > 0)
    {
        float overallCoverage = 0;
        mat4 sm_matrix = mat4(lightData.values[lightDataIndex],
                              lightData.values[lightDataIndex+1],
                              lightData.values[lightDataIndex+2],
                              lightData.values[lightDataIndex+3]);

        vec4 sm_tc = sm_matrix * vec4(eyePos, 1.0);
        sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);

        if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 && sm_tc.z <= 1.0)
        {
            overallCoverage = texture(sampler2DArrayShadow(shadowMaps, shadowMapShadowSampler), vec4(sm_tc.st, shadowMapIndex, sm_tc.z)).r;

#ifdef SHADOWMAP_DEBUG
            if (shadowMapIndex==0) color = vec3(1.0, 0.0, 0.0);
            else if (shadowMapIndex==1) color = vec3(0.0, 1.0, 0.0);
            else if (shadowMapIndex==2) color = vec3(0.0, 0.0, 1.0);
            else if (shadowMapIndex==3) color = vec3(1.0, 1.0, 0.0);
            else if (shadowMapIndex==4) color = vec3(0.0, 1.0, 1.0);
            else color = vec3(1.0, 1.0, 1.0);
#endif

            return overallCoverage;
        }

        lightDataIndex += 8;
        ++shadowMapIndex;
        --shadowMapCount;
    }

    return 0.0;
}
