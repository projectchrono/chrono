float calculateShadowCoverageForDirectionalLightSoft(int lightDataIndex, int shadowMapIndex, vec3 T, vec3 B, inout vec3 color)
{
    vec4 shadowMapSettings = lightData.values[lightDataIndex++];
    int shadowMapCount = int(shadowMapSettings.r);

    const float viableSampleRatio = 1;

    // Godot's implementation
    float diskRotation = quick_hash(gl_FragCoord.xy) * 2 * PI;
    mat2 diskRotationMatrix = mat2(cos(diskRotation), sin(diskRotation), -sin(diskRotation), cos(diskRotation));

    float overallCoverage = 0;
    float overallSampleCount = 0;
    while (shadowMapCount > 0)
    {
        mat4 sm_matrix = mat4(lightData.values[lightDataIndex],
                              lightData.values[lightDataIndex+1],
                              lightData.values[lightDataIndex+2],
                              lightData.values[lightDataIndex+3]);

        float coverage = 0;
        int viableSamples = 0;
        for (int i = 0; i < POISSON_DISK_SAMPLE_COUNT; i += POISSON_DISK_SAMPLE_COUNT / min(shadowSamples, POISSON_DISK_SAMPLE_COUNT))
        {
            float penumbraRadius = shadowMapSettings.g;
            vec2 rotatedDisk = penumbraRadius * diskRotationMatrix * POISSON_DISK[i];
            vec4 sm_tc = sm_matrix * vec4(eyePos + rotatedDisk.x * T + rotatedDisk.y * B, 1.0);
            sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);
            if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 && sm_tc.z <= 1.0)
            {
                coverage += texture(sampler2DArrayShadow(shadowMaps, shadowMapShadowSampler), vec4(sm_tc.st, shadowMapIndex, sm_tc.z)).r;
                ++viableSamples;
            }
        }

        coverage /= max(viableSamples, 1);
        overallSampleCount += viableSamples;
        overallCoverage = mix(overallCoverage, coverage, viableSamples / max(overallSampleCount, 1));

        if (overallSampleCount >= viableSampleRatio * min(shadowSamples, POISSON_DISK_SAMPLE_COUNT))
        {
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

float calculateShadowCoverageForSpotLightSoft(int lightDataIndex, int shadowMapIndex, vec3 T, vec3 B, inout vec3 color)
{
    // identical to directional lights
    vec4 shadowMapSettings = lightData.values[lightDataIndex++];
    int shadowMapCount = int(shadowMapSettings.r);

    const float viableSampleRatio = 1;

    // Godot's implementation
    float diskRotation = quick_hash(gl_FragCoord.xy) * 2 * PI;
    mat2 diskRotationMatrix = mat2(cos(diskRotation), sin(diskRotation), -sin(diskRotation), cos(diskRotation));

    float overallCoverage = 0;
    float overallSampleCount = 0;
    while (shadowMapCount > 0)
    {
        mat4 sm_matrix = mat4(lightData.values[lightDataIndex],
                              lightData.values[lightDataIndex+1],
                              lightData.values[lightDataIndex+2],
                              lightData.values[lightDataIndex+3]);

        float coverage = 0;
        int viableSamples = 0;
        for (int i = 0; i < POISSON_DISK_SAMPLE_COUNT; i += POISSON_DISK_SAMPLE_COUNT / min(shadowSamples, POISSON_DISK_SAMPLE_COUNT))
        {
            float penumbraRadius = shadowMapSettings.g;
            vec2 rotatedDisk = penumbraRadius * diskRotationMatrix * POISSON_DISK[i];
            vec4 sm_tc = sm_matrix * vec4(eyePos + rotatedDisk.x * T + rotatedDisk.y * B, 1.0);
            sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);
            if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 && sm_tc.z <= 1.0)
            {
                coverage += texture(sampler2DArrayShadow(shadowMaps, shadowMapShadowSampler), vec4(sm_tc.st, shadowMapIndex, sm_tc.z)).r;
                ++viableSamples;
            }
        }

        coverage /= max(viableSamples, 1);
        overallSampleCount += viableSamples;
        overallCoverage = mix(overallCoverage, coverage, viableSamples / max(overallSampleCount, 1));

        if (overallSampleCount >= viableSampleRatio * min(shadowSamples, POISSON_DISK_SAMPLE_COUNT))
        {
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
