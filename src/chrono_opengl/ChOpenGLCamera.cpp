// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
// Implementation of quaternion based GL camera class
// =============================================================================

#include "chrono_opengl/ChOpenGLCamera.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/quaternion.hpp>

namespace chrono {
namespace opengl {

ChOpenGLCamera::ChOpenGLCamera() {
    camera_mode = FREE;

    field_of_view = 45;
    rotation_quaternion = glm::quat(1, 0, 0, 0);
    camera_position = glm::vec3(0, 0, 0);
    camera_position_delta = glm::vec3(0, 0, 0);
    camera_look_at = glm::vec3(0, 0, 1);
    camera_direction = glm::vec3(0, 0, 1);
    camera_up = glm::vec3(0, 1, 0);
    camera_scale = .5f;
    camera_pitch = 0;
    camera_heading = 0;
    camera_mouse_scale = .02f;
    max_pitch_rate = 5;
    max_heading_rate = 5;
    move_camera = false;
    viewport_x = 0;
    viewport_y = 0;
}
ChOpenGLCamera::~ChOpenGLCamera() {}

void ChOpenGLCamera::Reset() {
    camera_up = glm::vec3(0, 1, 0);
}

void ChOpenGLCamera::Update() {
    camera_direction = glm::normalize(camera_look_at - camera_position);
    // need to set the matrix state. this is only important because lighting
    // doesn't work if this isn't done
    // glViewport(viewport_x, viewport_y, window_width, window_height);

    if (camera_mode == ORTHO) {
        // our projection matrix will be an orthogonal one in this case
        // if the values are not floating point, this command does not work properly
        // need to multiply by aspect!!! (otherise will not scale properly)
        projection = glm::ortho(-1.5f * float(aspect), 1.5f * float(aspect), -1.5f, 1.5f, -10.0f, 10.f);
    } else if (camera_mode == FREE) {
        projection = glm::perspective(field_of_view, aspect, near_clip, far_clip);
        // detmine axis for pitch rotation
        glm::vec3 axis = glm::cross(camera_direction, camera_up);
        // compute quaternion for pitch based on the camera pitch angle
        glm::quat pitch_quat = glm::angleAxis(camera_pitch, axis);
        // determine heading quaternion from the camera up vector and the heading
        // angle
        glm::quat heading_quat = glm::angleAxis(camera_heading, camera_up);
        // add the two quaternions
        glm::quat temp = glm::cross(pitch_quat, heading_quat);
        temp = glm::normalize(temp);
        // update the direction from the quaternion
        camera_direction = glm::rotate(temp, camera_direction);
        // add the camera delta
        camera_position += camera_position_delta;
        // set the look at to be infront of the camera
        camera_look_at = camera_position + camera_direction * 1.0f;
        // damping for smooth camera
        camera_heading *= .5;
        camera_pitch *= .5;
        camera_position_delta = camera_position_delta * .5f;
    }
    // compute the MVP
    view = glm::lookAt(camera_position, camera_look_at, camera_up);
    model = glm::mat4(1.0f);
    MVP = projection * view * model;
}

// Setting Functions
void ChOpenGLCamera::SetMode(CameraType cam_mode) {
    camera_mode = cam_mode;
    camera_up = glm::vec3(0, 1, 0);
    rotation_quaternion = glm::quat(1, 0, 0, 0);
}

void ChOpenGLCamera::SetPosition(glm::vec3 pos) {
    camera_position = pos;
}

void ChOpenGLCamera::SetLookAt(glm::vec3 pos) {
    camera_look_at = pos;
}
void ChOpenGLCamera::SetFOV(double fov) {
    field_of_view = fov;
}
void ChOpenGLCamera::SetViewport(int loc_x, int loc_y, int width, int height) {
    viewport_x = loc_x;
    viewport_y = loc_y;
    window_width = width;
    window_height = height;
    // need to use doubles division here, it will not work otherwise and it is
    // possible to get a zero aspect ratio with integer rounding
    aspect = double(width) / double(height);
    ;
}
void ChOpenGLCamera::SetClipping(double near_clip_distance, double far_clip_distance) {
    near_clip = near_clip_distance;
    far_clip = far_clip_distance;
}

void ChOpenGLCamera::Move(CameraDirection dir) {
    if (camera_mode == FREE) {
        switch (dir) {
            case UP:
                camera_position_delta += camera_up * camera_scale;
                break;
            case DOWN:
                camera_position_delta -= camera_up * camera_scale;
                break;
            case LEFT:
                camera_position_delta -= glm::cross(camera_direction, camera_up) * camera_scale;
                break;
            case RIGHT:
                camera_position_delta += glm::cross(camera_direction, camera_up) * camera_scale;
                break;
            case FORWARD:
                camera_position_delta += camera_direction * camera_scale;
                break;
            case BACK:
                camera_position_delta -= camera_direction * camera_scale;
                break;
        }
    }
}
void ChOpenGLCamera::ChangePitch(float degrees) {
    // Check bounds with the max pitch rate so that we aren't moving too fast
    if (degrees < -max_pitch_rate) {
        degrees = -max_pitch_rate;
    } else if (degrees > max_pitch_rate) {
        degrees = max_pitch_rate;
    }
    camera_pitch += degrees * camera_mouse_scale;

    // Check bounds for the camera pitch
    if (camera_pitch > 360.0f) {
        camera_pitch -= 360.0f;
    } else if (camera_pitch < -360.0f) {
        camera_pitch += 360.0f;
    }
}
void ChOpenGLCamera::ChangeHeading(float degrees) {
    // Check bounds with the max heading rate so that we aren't moving too fast
    if (degrees < -max_heading_rate) {
        degrees = -max_heading_rate;
    } else if (degrees > max_heading_rate) {
        degrees = max_heading_rate;
    }
    // This controls how the heading is changed if the camera is pointed straight
    // up or down
    // The heading delta direction changes
    if ((camera_pitch > 90 && camera_pitch < 270) || (camera_pitch < -90 && camera_pitch > -270)) {
        camera_heading -= degrees * camera_mouse_scale;
    } else {
        camera_heading += degrees * camera_mouse_scale;
    }
    // Check bounds for the camera heading
    if (camera_heading > 360.0f) {
        camera_heading -= 360.0f;
    } else if (camera_heading < -360.0f) {
        camera_heading += 360.0f;
    }
}
void ChOpenGLCamera::Move2D(int x, int y) {
    // compute the mouse delta from the previous mouse position
    glm::vec3 mouse_delta = mouse_position - glm::vec3(x, y, 0);
    // if the camera is moving, meaning that the mouse was clicked and dragged,
    // change the pitch and heading
    if (move_camera) {
        ChangeHeading(.08f * mouse_delta.x);
        ChangePitch(.08f * mouse_delta.y);
    }
    mouse_position = glm::vec3(x, y, 0);
}

void ChOpenGLCamera::SetPos(int button, int state, int x, int y) {
    if (button == 3 && state == GLFW_PRESS) {
        camera_position_delta += camera_up * .05f;
    } else if (button == 4 && state == GLFW_PRESS) {
        camera_position_delta -= camera_up * .05f;
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_PRESS) {
        move_camera = true;
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_RELEASE) {
        move_camera = false;
    }
    mouse_position = glm::vec3(x, y, 0);
}

CameraType ChOpenGLCamera::GetMode() {
    return camera_mode;
}

void ChOpenGLCamera::GetViewport(int& loc_x, int& loc_y, int& width, int& height) {
    loc_x = viewport_x;
    loc_y = viewport_y;
    width = window_width;
    height = window_height;
}

void ChOpenGLCamera::GetMatricies(glm::mat4& P, glm::mat4& V, glm::mat4& M) {
    P = projection;
    V = view;
    M = model;
}
}
}
