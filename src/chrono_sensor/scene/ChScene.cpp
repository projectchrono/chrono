// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
//
// =============================================================================

#include "chrono_sensor/scene/ChScene.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChScene::ChScene() {
    m_keyframes = std::deque<Keyframe>();

    m_background.color = {0.5f, 0.6f, 0.7f};
    m_background.has_texture = false;
    // m_background.env_te
}

CH_SENSOR_API ChScene::~ChScene() {
    // while (m_keyframes.size() > 0) {
    //     delete[] m_keyframes.back().bodies;
    //     m_keyframes.pop_back();
    // }
}

void ChScene::AddPointLight(ChVector<float> pos, ChVector<float> color, float max_range) {
    PointLight p;
    p.pos = optix::make_float3(pos.x(), pos.y(), pos.z());
    p.color = optix::make_float3(color.x(), color.y(), color.z());
    p.max_range = max_range;
    p.casts_shadow = 1;

    m_pointlights.push_back(p);
}

std::deque<Keyframe> CH_SENSOR_API ChScene::GetKeyframesCopy() {
    std::lock_guard<std::mutex> lck(keyframe_mutex);
    return m_keyframes;
}

void CH_SENSOR_API ChScene::PackFrame(ChSystem* pSystem) {
    // create a new keyframe
    // std::cout << "Num Keyframes: " << m_keyframes.size() << "\n";
    // Keyframe k;
    // k.bodies = new SceneBody[pSystem->Get_bodylist().size()];
    //
    // for (int i = 0; i < pSystem->Get_bodylist().size(); i++) {
    //     // k.bodies[i].pos[0] = pSystem->Get_bodylist()[i]->GetPos().x();
    //     // k.bodies[i].pos[1] = pSystem->Get_bodylist()[i]->GetPos().y();
    //     // k.bodies[i].pos[2] = pSystem->Get_bodylist()[i]->GetPos().z();
    //     // k.bodies[i].pos[3] = pSystem->Get_bodylist()[i]->GetRot().e0();
    //     // k.bodies[i].pos[4] = pSystem->Get_bodylist()[i]->GetRot().e1();
    //     // k.bodies[i].pos[5] = pSystem->Get_bodylist()[i]->GetRot().e2();
    //     // k.bodies[i].pos[6] = pSystem->Get_bodylist()[i]->GetRot().e3();
    //     // k.bodies[i].chronobody = *(pSystem->Get_bodylist()[i].get());
    // }
    //
    // k.pointlights = m_pointlights;
    //
    // // push that keyframe into the deque
    // m_current_frame = k;
    //
    // std::lock_guard<std::mutex> lck(keyframe_mutex);
    // m_keyframes.push_front(m_current_frame);
    // while (m_keyframes.size() > max_keyframes) {
    //     delete[] m_keyframes.back().bodies;
    //     m_keyframes.pop_back();
    // }
}

bool CH_SENSOR_API ChScene::SetMaxKeyframes(int num_frames) {
    // if (num_frames < 1) {
    //     return false;
    // } else {
    //     max_keyframes = num_frames;
    //     while (m_keyframes.size() > max_keyframes) {
    //         delete[] m_keyframes.back().bodies;
    //         m_keyframes.pop_back();
    //     }
    // }

    return false;
}

}  // namespace sensor
}  // namespace chrono
