float calculateShadowCoverageForDirectionalLightPCSS(int lightDataIndex, int shadowMapIndex, vec3 T, vec3 B, inout vec3 color)
{
    vec4 shadowMapSettings = lightData.values[lightDataIndex++];
    int shadowMapCount = int(shadowMapSettings.r);
    int originalShadowMapIndex = shadowMapIndex;
    int originalIndex = lightDataIndex;

    const float blockerSearchRadius = shadowMapSettings.g;
    const float viableSampleRatio = 1;

    // Godot's implementation
    float diskRotation = quick_hash(gl_FragCoord.xy) * 2 * PI;
    mat2 diskRotationMatrix = mat2(cos(diskRotation), sin(diskRotation), -sin(diskRotation), cos(diskRotation));

    // blocker search
    bool matched = false;
    float overallBlockerDistances = 0;
    float overallBlockerCount = 0;
    int overallViableSamples = 0;
    while (shadowMapCount > 0 && !matched)
    {
        mat4 sm_matrix = mat4(lightData.values[lightDataIndex],
                              lightData.values[lightDataIndex+1],
                              lightData.values[lightDataIndex+2],
                              lightData.values[lightDataIndex+3]);
        float blockerDistances = 0.0;
        int blockerCount = 0;
        int viableSamples = 0;
        // always sample the original coordinates as otherwise blockers smaller than the search radius may be missed with small sample counts
        vec4 sm_tc = sm_matrix * vec4(eyePos, 1.0);
        sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);
        if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 && sm_tc.z <= 1.0)
        {
            ++viableSamples;
            float blockerDistance = texture(sampler2DArray(shadowMaps, shadowMapDirectSampler), vec3(sm_tc.st, shadowMapIndex)).r;
            if (blockerDistance > sm_tc.z) {
                blockerDistances += blockerDistance;
                ++blockerCount;
            }
        }
        for (int i = 0; i < POISSON_DISK_SAMPLE_COUNT; i += POISSON_DISK_SAMPLE_COUNT / min(shadowSamples / 2, POISSON_DISK_SAMPLE_COUNT))
        {
            vec2 rotatedDisk = blockerSearchRadius * diskRotationMatrix * POISSON_DISK[i];
            sm_tc = sm_matrix * vec4(eyePos + rotatedDisk.x * T + rotatedDisk.y * B, 1.0);
            sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);
            if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 && sm_tc.z <= 1.0)
            {
                ++viableSamples;
                float blockerDistance = texture(sampler2DArray(shadowMaps, shadowMapDirectSampler), vec3(sm_tc.st, shadowMapIndex)).r;
                if (blockerDistance > sm_tc.z) {
                    blockerDistances += blockerDistance;
                    ++blockerCount;
                }
            }
        }

        overallViableSamples += viableSamples;
        if (overallViableSamples >= viableSampleRatio * min(shadowSamples / 2, POISSON_DISK_SAMPLE_COUNT))
            matched = true;

        if (blockerCount > 0)
        {
            // if averaging like this is legal, then calculating the penumbra radius in light space should be legal, too
            blockerDistances /= blockerCount;

            mat4 sm_matrix_inv = mat4(lightData.values[lightDataIndex+4],
                                      lightData.values[lightDataIndex+5],
                                      lightData.values[lightDataIndex+6],
                                      lightData.values[lightDataIndex+7]);
            vec4 sm_tc = sm_matrix * vec4(eyePos, 1.0);
            sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);
            vec4 averageBlockerEuclidean = sm_matrix_inv * vec4(sm_tc.xy, blockerDistances, sm_tc.w);
            averageBlockerEuclidean.xyz /= averageBlockerEuclidean.w;
            float dist = distance(averageBlockerEuclidean.xyz, eyePos);

            overallBlockerCount += blockerCount;
            overallBlockerDistances = mix(overallBlockerDistances, dist, blockerCount / overallBlockerCount);
        }

        lightDataIndex += 8;
        ++shadowMapIndex;
        --shadowMapCount;
    }

    // there's something there, compute shadow
    // note - can't skip if all viable samples so far were blockers - if they're distant, the penumbra could be wider than the blocker search
    if (overallBlockerCount > 0)
    {
        float overallCoverage = 0;

        shadowMapCount = int(shadowMapSettings.r);
        shadowMapIndex = originalShadowMapIndex;
        lightDataIndex = originalIndex;

        float penumbraRadius = overallBlockerDistances * shadowMapSettings.b;

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
    }

    return 0.0;
}

float calculateShadowCoverageForSpotLightPCSS(int lightDataIndex, int shadowMapIndex, vec3 T, vec3 B, float lightDist, inout vec3 color)
{
    vec4 shadowMapSettings = lightData.values[lightDataIndex++];
    int shadowMapCount = int(shadowMapSettings.r);
    int originalShadowMapIndex = shadowMapIndex;
    int originalIndex = lightDataIndex;

    const float blockerSearchRadius = shadowMapSettings.g;
    const float viableSampleRatio = 1;

    // Godot's implementation
    float diskRotation = quick_hash(gl_FragCoord.xy) * 2 * PI;
    mat2 diskRotationMatrix = mat2(cos(diskRotation), sin(diskRotation), -sin(diskRotation), cos(diskRotation));

    // blocker search
    bool matched = false;
    float overallBlockerDistances = 0;
    float overallBlockerCount = 0;
    int overallViableSamples = 0;
    while (shadowMapCount > 0 && !matched)
    {
        mat4 sm_matrix = mat4(lightData.values[lightDataIndex],
                              lightData.values[lightDataIndex+1],
                              lightData.values[lightDataIndex+2],
                              lightData.values[lightDataIndex+3]);
        float blockerDistances = 0.0;
        int blockerCount = 0;
        int viableSamples = 0;
        // always sample the original coordinates as otherwise blockers smaller than the search radius may be missed with small sample counts
        vec4 sm_tc = sm_matrix * vec4(eyePos, 1.0);
        sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);
        if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 && sm_tc.z <= 1.0)
        {
            ++viableSamples;
            float blockerDistance = texture(sampler2DArray(shadowMaps, shadowMapDirectSampler), vec3(sm_tc.st, shadowMapIndex)).r;
            if (blockerDistance > sm_tc.z) {
                blockerDistances += blockerDistance;
                ++blockerCount;
            }
        }
        for (int i = 0; i < POISSON_DISK_SAMPLE_COUNT; i += POISSON_DISK_SAMPLE_COUNT / min(shadowSamples / 2, POISSON_DISK_SAMPLE_COUNT))
        {
            vec2 rotatedDisk = blockerSearchRadius * diskRotationMatrix * POISSON_DISK[i];
            sm_tc = sm_matrix * vec4(eyePos + rotatedDisk.x * T + rotatedDisk.y * B, 1.0);
            sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);
            if (sm_tc.x >= 0.0 && sm_tc.x <= 1.0 && sm_tc.y >= 0.0 && sm_tc.y <= 1.0 && sm_tc.z >= 0.0 && sm_tc.z <= 1.0)
            {
                ++viableSamples;
                float blockerDistance = texture(sampler2DArray(shadowMaps, shadowMapDirectSampler), vec3(sm_tc.st, shadowMapIndex)).r;
                if (blockerDistance > sm_tc.z) {
                    blockerDistances += blockerDistance;
                    ++blockerCount;
                }
            }
        }

        overallViableSamples += viableSamples;
        if (overallViableSamples >= viableSampleRatio * min(shadowSamples / 2, POISSON_DISK_SAMPLE_COUNT))
            matched = true;

        if (blockerCount > 0)
        {
            // if averaging like this is legal, then calculating the penumbra radius in light space should be legal, too
            blockerDistances /= blockerCount;

            mat4 sm_matrix_inv = mat4(lightData.values[lightDataIndex+4],
                                      lightData.values[lightDataIndex+5],
                                      lightData.values[lightDataIndex+6],
                                      lightData.values[lightDataIndex+7]);
            vec4 sm_tc = sm_matrix * vec4(eyePos, 1.0);
            sm_tc = vec4(sm_tc.xyz / sm_tc.w, 1.0);
            vec4 averageBlockerEuclidean = sm_matrix_inv * vec4(sm_tc.xy, blockerDistances, sm_tc.w);
            averageBlockerEuclidean.xyz /= averageBlockerEuclidean.w;
            float dist = distance(averageBlockerEuclidean.xyz, eyePos);

            overallBlockerCount += blockerCount;
            overallBlockerDistances = mix(overallBlockerDistances, dist, blockerCount / overallBlockerCount);
        }

        lightDataIndex += 8;
        ++shadowMapIndex;
        --shadowMapCount;
    }

    // there's something there, compute shadow
    // note - can't skip if all viable samples so far were blockers - if they're distant, the penumbra could be wider than the blocker search
    if (overallBlockerCount > 0)
    {
        float overallCoverage = 0;

        shadowMapCount = int(shadowMapSettings.r);
        shadowMapIndex = originalShadowMapIndex;
        lightDataIndex = originalIndex;

        float penumbraRadius = overallBlockerDistances * shadowMapSettings.b / (lightDist - overallBlockerDistances);

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
    }

    return 0.0;
}
