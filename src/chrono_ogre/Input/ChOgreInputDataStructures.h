/*
Author: Charles Ricchio

Data structures for passing around user input information
*/

#pragma once

#include "chrono_ogre/ChOgreApi.h"
#include <cinttypes>

namespace ChOgre {

typedef int32_t keycode_t;
typedef int32_t scancode_t;

///////////////////////////////////////////////////
struct CHOGRE_DLL_TAG ChOgreKeyState {
    bool down;
    double timestamp;
};
///////////////////////////////////////////////////

///////////////////////////////////////////////////
struct CHOGRE_DLL_TAG ChOgreMouseState {
    ChOgreKeyState left;
    ChOgreKeyState right;
    ChOgreKeyState middle;
    ChOgreKeyState x1;
    ChOgreKeyState x2;

    typedef struct __posState_t {
        double x, y;
        double xrel, yrel;
        double timestamp;
    } __posState;

    __posState position;

    typedef struct __wheelState_t {
        double x, y;
        double timestamp;
    } __wheelState;

    __wheelState wheel;
};
///////////////////////////////////////////////////

///////////////////////////////////////////////////
struct CHOGRE_DLL_TAG ChOgreControllerState {
    typedef struct __axisState_t {
        double value;
        double timestamp;
    } __axisState;

    __axisState lstickx, lsticky;
    __axisState rstickx, rsticky;
    __axisState ltrigger, rtrigger;

    ChOgreKeyState a, b, x, y;
    ChOgreKeyState back, start;
    ChOgreKeyState lstick, rstick;
    ChOgreKeyState d_left, d_right, d_up, d_down;
    ChOgreKeyState lbumper, rbumper;

    bool active;
};
///////////////////////////////////////////////////

///////////////////////////////////////////////////
struct CHOGRE_DLL_TAG ChOgreWheelState {
    typedef struct __hatState_t {
        bool up, up_right, right, down_right, down, down_left, left, up_left, centered;
        double timestamp;
        void reset() {
            up = false;
            up_right = false;
            right = false;
            down_right = false;
            down = false;
            down_left = false;
            left = false;
            up_left = false;
            centered = false;
        }
    } __hatState;

    ChOgreControllerState::__axisState wheel;
    ChOgreControllerState::__axisState accelerator;
    ChOgreControllerState::__axisState brake;
    ChOgreControllerState::__axisState clutch;

    ChOgreKeyState lpaddle, rpaddle;
    ChOgreKeyState lwheelb1, rwheelb1, lwheelb2, rwheelb2, lwheelb3, rwheelb3;
    ChOgreKeyState gear1, gear2, gear3, gear4, gear5, gear6, reverse;
    ChOgreKeyState red1, red2, red3, red4;
    ChOgreKeyState black_up, black_down, black_left, black_right;

    __hatState d_pad;

    bool active;
};
///////////////////////////////////////////////////

typedef SDL_HapticEffect CHOGRE_DLL_TAG ChOgreHapticEffect;
}