/*
Author: Charles Ricchio

An input manager based on SDL, as opposed to OIS. Will handle keyboard, mouse, and joystick input.
*/

#if defined(_WIN32) || defined(WIN32)

#include <windows.h>

#endif

#include <string>
#include <climits>
#include <algorithm>
#include "../Input/ChOgre_SDLInputHandler.h"

namespace ChOgre {

std::string const ChOgre_SDLInputHandler::WheelGUID = "6d049bc2000000000000504944564944";

ChOgre_SDLInputHandler::ChOgre_SDLInputHandler(Ogre::RenderWindow* renderWindow) {
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_VIDEO | SDL_INIT_HAPTIC) != 0) {
        Ogre::LogManager::getSingleton().logMessage(
            Ogre::LogMessageLevel::LML_CRITICAL,
            "\n\nCould not initialize SDL: " + std::string(SDL_GetError()) + "\n\n");
    }

#if defined(_WIN32) || defined(WIN32)

    HWND windowHnd = 0;

    // Get window handle
    renderWindow->getCustomAttribute("WINDOW", &windowHnd);

    m_pSDLWindow = SDL_CreateWindowFrom(windowHnd);

#endif

    if (m_pSDLWindow == nullptr) {
        Ogre::LogManager::getSingleton().logMessage(
            Ogre::LogMessageLevel::LML_CRITICAL, "\n\nCould make SDL window: " + std::string(SDL_GetError()) + "\n\n");
    }

    m_pController = nullptr;

    int _nJoysticks = SDL_NumJoysticks();

    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        m_pController = SDL_JoystickOpen(i);

        SDL_JoystickGUID k = SDL_JoystickGetGUID(m_pController);
        char* l = new char[33];
        SDL_JoystickGetGUIDString(k, l, 33);
        if (l == WheelGUID) {
            m_WheelState.active = true;
            m_ControllerState.active = false;
        } else {
            m_ControllerState.active = true;
            m_WheelState.active = false;
        }
        delete l;

        if (m_pController) {
            m_pHaptic = SDL_HapticOpen(0);
            SDL_HapticRumbleInit(m_pHaptic);
            break;
        } else {
            Ogre::LogManager::getSingleton().logMessage(
                Ogre::LML_CRITICAL,
                "Could not open gamecontroller " + std::to_string(i) + ": " + std::string(SDL_GetError()) + "\n");
        }
    }

    AxisThreshold = INPUT_DEADZONE;
    WindowClose = false;
}

ChOgre_SDLInputHandler::~ChOgre_SDLInputHandler() {
    SDL_Quit();
}

void ChOgre_SDLInputHandler::update() {
    SDL_Event _event;
    while (SDL_PollEvent(&_event)) {
        if (_event.type == SDL_KEYDOWN) {
            bool was_set = false;

            if (m_KeyStates_keycode[_event.key.keysym.sym].down) {
                was_set = true;
            }

            m_KeyStates_scancode[_event.key.keysym.scancode].down = true;
            m_KeyStates_scancode[_event.key.keysym.scancode].timestamp = (double)(_event.key.timestamp) / 1000.0;
            m_KeyStates_keycode[_event.key.keysym.sym].down = true;
            m_KeyStates_keycode[_event.key.keysym.sym].timestamp = (double)(_event.key.timestamp) / 1000.0;

            if (!was_set) {
                m_CallKeyboardCallbacks(_event.key.keysym.scancode, _event.key.keysym.sym,
                                        m_KeyStates_keycode[_event.key.keysym.sym]);
            }
#ifdef _DEBUG
            Ogre::LogManager::getSingleton().logMessage(
                Ogre::LML_NORMAL, "Key press ; Scanecode: " + std::to_string(_event.key.keysym.scancode) +
                                      " Keycode: " + std::to_string(_event.key.keysym.sym) + "\n");
#endif
        } else if (_event.type == SDL_KEYUP) {
            bool was_set = false;

            if (m_KeyStates_keycode[_event.key.keysym.sym].down) {
                was_set = true;
            }

            m_KeyStates_scancode[_event.key.keysym.scancode].down = false;
            m_KeyStates_scancode[_event.key.keysym.scancode].timestamp = (double)(_event.key.timestamp) / 1000.0;
            m_KeyStates_keycode[_event.key.keysym.sym].down = false;
            m_KeyStates_keycode[_event.key.keysym.sym].timestamp = (double)(_event.key.timestamp) / 1000.0;

            if (was_set) {
                m_CallKeyboardCallbacks(_event.key.keysym.scancode, _event.key.keysym.sym,
                                        m_KeyStates_keycode[_event.key.keysym.sym]);
            }
#ifdef _DEBUG
            Ogre::LogManager::getSingleton().logMessage(
                Ogre::LML_NORMAL, "Key release ; Scanecode: " + std::to_string(_event.key.keysym.scancode) +
                                      " Keycode: " + std::to_string(_event.key.keysym.sym) + "\n");
#endif
        } else if (_event.type == SDL_MOUSEBUTTONDOWN) {
            switch (_event.button.button) {
                case SDL_BUTTON_LEFT:
                    m_MouseState.left.down = true;
                    m_MouseState.left.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
                case SDL_BUTTON_RIGHT:
                    m_MouseState.right.down = true;
                    m_MouseState.right.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
                case SDL_BUTTON_MIDDLE:
                    m_MouseState.middle.down = true;
                    m_MouseState.middle.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
                case SDL_BUTTON_X1:
                    m_MouseState.x1.down = true;
                    m_MouseState.x1.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
                case SDL_BUTTON_X2:
                    m_MouseState.x2.down = true;
                    m_MouseState.x2.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
            }

            m_CallMouseCallbacks();
        } else if (_event.type == SDL_MOUSEBUTTONUP) {
            switch (_event.button.button) {
                case SDL_BUTTON_LEFT:
                    m_MouseState.left.down = false;
                    m_MouseState.left.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
                case SDL_BUTTON_RIGHT:
                    m_MouseState.right.down = false;
                    m_MouseState.right.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
                case SDL_BUTTON_MIDDLE:
                    m_MouseState.middle.down = false;
                    m_MouseState.middle.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
                case SDL_BUTTON_X1:
                    m_MouseState.x1.down = false;
                    m_MouseState.x1.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
                case SDL_BUTTON_X2:
                    m_MouseState.x2.down = false;
                    m_MouseState.x2.timestamp = (double)(_event.button.timestamp) / 1000.0;
                    break;
            }

            m_CallMouseCallbacks();
        } else if (_event.type == SDL_MOUSEMOTION) {
            m_MouseState.position.timestamp = (double)(_event.motion.timestamp) / 1000.0;

            int _w, _h;

            SDL_GetWindowSize(m_pSDLWindow, &_w, &_h);

            m_MouseState.position.x = (double)(_event.motion.x) / (double)_w;
            m_MouseState.position.y = (double)(_event.motion.y) / (double)_h;
            m_MouseState.position.xrel = (double)(_event.motion.xrel) / (double)INT_MAX;
            m_MouseState.position.yrel = (double)(_event.motion.yrel) / (double)INT_MAX;

            if (_event.motion.state & SDL_BUTTON_LMASK) {
                m_MouseState.left.down = true;
            } else {
                m_MouseState.left.down = false;
            }

            if (_event.motion.state & SDL_BUTTON_RMASK) {
                m_MouseState.right.down = true;
            } else {
                m_MouseState.right.down = false;
            }

            if (_event.motion.state & SDL_BUTTON_MMASK) {
                m_MouseState.middle.down = true;
            } else {
                m_MouseState.middle.down = false;
            }

            if (_event.motion.state & SDL_BUTTON_X1MASK) {
                m_MouseState.x1.down = true;
            } else {
                m_MouseState.x1.down = false;
            }

            if (_event.motion.state & SDL_BUTTON_X2MASK) {
                m_MouseState.x2.down = true;
            } else {
                m_MouseState.x2.down = false;
            }

            m_CallMouseCallbacks();

        } else if (_event.type == SDL_MOUSEWHEEL) {
            m_MouseState.wheel.timestamp = (double)(_event.wheel.timestamp) / 1000.0;
            m_MouseState.wheel.x = (double)(_event.wheel.x) / (double)INT_MAX;
            m_MouseState.wheel.y = (double)(_event.wheel.y) / (double)INT_MAX;

            m_CallMouseCallbacks();
        } else if (_event.type == SDL_JOYAXISMOTION) {
            if (std::abs(((double)_event.jaxis.value / (double)SHRT_MAX)) > AxisThreshold) {
                switch (_event.jaxis.axis) {
                    case 0:
                        m_ControllerState.lstickx.value = ((double)_event.jaxis.value / (double)SHRT_MAX);
                        m_ControllerState.lstickx.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        m_WheelState.wheel.value = ((double)_event.jaxis.value / (double)SHRT_MAX);
                        m_WheelState.wheel.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 1:
                        m_ControllerState.lsticky.value = ((double)_event.jaxis.value / (double)SHRT_MAX);
                        m_ControllerState.lsticky.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        m_WheelState.accelerator.value =
                            std::abs((((double)_event.jaxis.value / (double)SHRT_MAX) - 1) / 2);
                        m_WheelState.accelerator.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 2:
                        m_ControllerState.rstickx.value = ((double)_event.jaxis.value / (double)SHRT_MAX);
                        m_ControllerState.rstickx.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        m_WheelState.brake.value = std::abs((((double)_event.jaxis.value / (double)SHRT_MAX) - 1) / 2);
                        m_WheelState.brake.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 3:
                        m_ControllerState.rsticky.value = ((double)_event.jaxis.value / (double)SHRT_MAX);
                        m_ControllerState.rsticky.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 4:
                        m_ControllerState.ltrigger.value = ((double)_event.jaxis.value / (double)SHRT_MAX);
                        m_ControllerState.ltrigger.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        m_WheelState.clutch.value = std::abs((((double)_event.jaxis.value / (double)SHRT_MAX) - 1) / 2);
                        m_WheelState.clutch.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 5:
                        m_ControllerState.rtrigger.value = ((double)_event.jaxis.value / (double)SHRT_MAX);
                        m_ControllerState.rtrigger.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;
                }

            } else {
                switch (_event.jaxis.axis) {
                    case 0:
                        m_ControllerState.lstickx.value = 0;
                        m_ControllerState.lstickx.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        m_WheelState.wheel.value = 0;
                        m_WheelState.wheel.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 1:
                        m_ControllerState.lsticky.value = 0;
                        m_ControllerState.lsticky.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        m_WheelState.accelerator.value = 0;
                        m_WheelState.accelerator.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 2:
                        m_ControllerState.rstickx.value = 0;
                        m_ControllerState.rstickx.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        m_WheelState.brake.value = 0;
                        m_WheelState.brake.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 3:
                        m_ControllerState.rsticky.value = 0;
                        m_ControllerState.rsticky.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 4:
                        m_ControllerState.ltrigger.value = 0;
                        m_ControllerState.ltrigger.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        m_WheelState.clutch.value = 0;
                        m_WheelState.clutch.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;

                    case 5:
                        m_ControllerState.rtrigger.value = 0;
                        m_ControllerState.rtrigger.timestamp = (double)(_event.jaxis.timestamp) / 1000.0;
                        break;
                }
            }

#ifdef _DEBUG
            if (((((double)_event.jaxis.value) / ((double)SHRT_MAX)) > INPUT_DEADZONE) ||
                ((((double)_event.jaxis.value) / ((double)SHRT_MAX)) < -(INPUT_DEADZONE))) {
                Ogre::LogManager::getSingleton().logMessage(
                    Ogre::LML_NORMAL, "Controller axis motion - Axis:" + std::to_string(_event.jaxis.axis) + "\n");
            }
#endif
        } else if (_event.type == SDL_JOYBUTTONDOWN) {
            switch (_event.jbutton.button) {
                case 0:
                    m_ControllerState.d_up.down = true;
                    m_ControllerState.d_up.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.red1.down = true;
                    m_WheelState.red1.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 1:
                    m_ControllerState.d_down.down = true;
                    m_ControllerState.d_down.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.red2.down = true;
                    m_WheelState.red2.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 2:
                    m_ControllerState.d_left.down = true;
                    m_ControllerState.d_left.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.red3.down = true;
                    m_WheelState.red3.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 3:
                    m_ControllerState.d_right.down = true;
                    m_ControllerState.d_right.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.red4.down = true;
                    m_WheelState.red4.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 4:
                    m_ControllerState.start.down = true;
                    m_ControllerState.start.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.rpaddle.down = true;
                    m_WheelState.rpaddle.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 5:
                    m_ControllerState.back.down = true;
                    m_ControllerState.back.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.lpaddle.down = true;
                    m_WheelState.lpaddle.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 6:
                    m_ControllerState.lstick.down = true;
                    m_ControllerState.lstick.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.rwheelb1.down = true;
                    m_WheelState.rwheelb1.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 7:
                    m_ControllerState.rstick.down = true;
                    m_ControllerState.rstick.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.lwheelb1.down = true;
                    m_WheelState.lwheelb1.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 8:
                    m_ControllerState.lbumper.down = true;
                    m_ControllerState.lbumper.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear1.down = true;
                    m_WheelState.gear1.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 9:
                    m_ControllerState.rbumper.down = true;
                    m_ControllerState.rbumper.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear2.down = true;
                    m_WheelState.gear2.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 10:
                    m_ControllerState.a.down = true;
                    m_ControllerState.a.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear3.down = true;
                    m_WheelState.gear3.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 11:
                    m_ControllerState.b.down = true;
                    m_ControllerState.b.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear4.down = true;
                    m_WheelState.gear4.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 12:
                    m_ControllerState.x.down = true;
                    m_ControllerState.x.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear5.down = true;
                    m_WheelState.gear5.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 13:
                    m_ControllerState.y.down = true;
                    m_ControllerState.y.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear6.down = true;
                    m_WheelState.gear6.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 14:
                    m_WheelState.reverse.down = true;
                    m_WheelState.reverse.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 15:
                    m_WheelState.black_up.down = true;
                    m_WheelState.black_up.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 16:
                    m_WheelState.black_left.down = true;
                    m_WheelState.black_left.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 17:
                    m_WheelState.black_down.down = true;
                    m_WheelState.black_down.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 18:
                    m_WheelState.black_right.down = true;
                    m_WheelState.black_right.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 19:
                    m_WheelState.rwheelb2.down = true;
                    m_WheelState.rwheelb2.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 20:
                    m_WheelState.lwheelb2.down = true;
                    m_WheelState.lwheelb2.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 21:
                    m_WheelState.rwheelb3.down = true;
                    m_WheelState.rwheelb3.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 22:
                    m_WheelState.lwheelb3.down = true;
                    m_WheelState.lwheelb3.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;
            }

#ifdef _DEBUG
            Ogre::LogManager::getSingleton().logMessage(
                Ogre::LML_NORMAL, "Controller button press - Button:" + std::to_string(_event.jbutton.button) + "\n");
#endif
        } else if (_event.type == SDL_JOYBUTTONUP) {
            switch (_event.jbutton.button) {
                case 0:
                    m_ControllerState.d_up.down = false;
                    m_ControllerState.d_up.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.red1.down = false;
                    m_WheelState.red1.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 1:
                    m_ControllerState.d_down.down = false;
                    m_ControllerState.d_down.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.red2.down = false;
                    m_WheelState.red2.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 2:
                    m_ControllerState.d_left.down = false;
                    m_ControllerState.d_left.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.red3.down = false;
                    m_WheelState.red3.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 3:
                    m_ControllerState.d_right.down = false;
                    m_ControllerState.d_right.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.red4.down = false;
                    m_WheelState.red4.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 4:
                    m_ControllerState.start.down = false;
                    m_ControllerState.start.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.rpaddle.down = false;
                    m_WheelState.rpaddle.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 5:
                    m_ControllerState.back.down = false;
                    m_ControllerState.back.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.lpaddle.down = false;
                    m_WheelState.lpaddle.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 6:
                    m_ControllerState.lstick.down = false;
                    m_ControllerState.lstick.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.rwheelb1.down = false;
                    m_WheelState.rwheelb1.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 7:
                    m_ControllerState.rstick.down = false;
                    m_ControllerState.rstick.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.lwheelb1.down = false;
                    m_WheelState.lwheelb1.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 8:
                    m_ControllerState.lbumper.down = false;
                    m_ControllerState.lbumper.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear1.down = false;
                    m_WheelState.gear1.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 9:
                    m_ControllerState.rbumper.down = false;
                    m_ControllerState.rbumper.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear2.down = false;
                    m_WheelState.gear2.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 10:
                    m_ControllerState.a.down = false;
                    m_ControllerState.a.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear3.down = false;
                    m_WheelState.gear3.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 11:
                    m_ControllerState.b.down = false;
                    m_ControllerState.b.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear4.down = false;
                    m_WheelState.gear4.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 12:
                    m_ControllerState.x.down = false;
                    m_ControllerState.x.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear5.down = false;
                    m_WheelState.gear5.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 13:
                    m_ControllerState.y.down = false;
                    m_ControllerState.y.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    m_WheelState.gear6.down = false;
                    m_WheelState.gear6.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 14:
                    m_WheelState.reverse.down = false;
                    m_WheelState.reverse.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 15:
                    m_WheelState.black_up.down = false;
                    m_WheelState.black_up.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 16:
                    m_WheelState.black_left.down = false;
                    m_WheelState.black_left.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 17:
                    m_WheelState.black_down.down = false;
                    m_WheelState.black_down.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 18:
                    m_WheelState.black_right.down = false;
                    m_WheelState.black_right.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 19:
                    m_WheelState.rwheelb2.down = false;
                    m_WheelState.rwheelb2.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 20:
                    m_WheelState.lwheelb2.down = false;
                    m_WheelState.lwheelb2.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 21:
                    m_WheelState.rwheelb3.down = false;
                    m_WheelState.rwheelb3.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;

                case 22:
                    m_WheelState.lwheelb3.down = false;
                    m_WheelState.lwheelb3.timestamp = (double)(_event.jbutton.timestamp) / 1000.0;
                    break;
            }

#ifdef _DEBUG
            Ogre::LogManager::getSingleton().logMessage(
                Ogre::LML_NORMAL, "Controller button release - Button:" + std::to_string(_event.jbutton.button) + "\n");
#endif
        } else if (_event.type == SDL_JOYHATMOTION) {
            m_WheelState.d_pad.reset();
            switch (_event.jhat.value) {
                case SDL_HAT_UP:
                    m_WheelState.d_pad.up = true;
                    break;
                case SDL_HAT_RIGHTUP:
                    m_WheelState.d_pad.up_right = true;
                    break;
                case SDL_HAT_RIGHT:
                    m_WheelState.d_pad.right = true;
                    break;
                case SDL_HAT_RIGHTDOWN:
                    m_WheelState.d_pad.down_right = true;
                    break;
                case SDL_HAT_DOWN:
                    m_WheelState.d_pad.down = true;
                    break;
                case SDL_HAT_LEFTDOWN:
                    m_WheelState.d_pad.down_left = true;
                    break;
                case SDL_HAT_LEFT:
                    m_WheelState.d_pad.left = true;
                    break;
                case SDL_HAT_LEFTUP:
                    m_WheelState.d_pad.up_left = true;
                    break;
                case SDL_HAT_CENTERED:
                    m_WheelState.d_pad.centered = true;
                    break;
            }
        } else if (_event.type == SDL_JOYDEVICEADDED) {
            if (_event.jdevice.which == 0) {
                if (m_pController) {
                    SDL_JoystickClose(m_pController);
                    m_pController = nullptr;
                }
                m_pController = SDL_JoystickOpen(0);

                SDL_JoystickGUID k = SDL_JoystickGetGUID(m_pController);
                char* l = new char[33];
                SDL_JoystickGetGUIDString(k, l, 33);
                if (l == WheelGUID) {
                    m_WheelState.active = true;
                    m_ControllerState.active = false;
                } else {
                    m_ControllerState.active = true;
                    m_WheelState.active = false;
                }
                delete l;

                m_pHaptic = SDL_HapticOpen(0);
                SDL_HapticRumbleInit(m_pHaptic);
            }
        } else if (_event.type == SDL_JOYDEVICEREMOVED) {
            if (_event.jdevice.which == 0) {
                if (m_pController) {
                    SDL_JoystickClose(m_pController);
                    m_pController = nullptr;
                    m_ControllerState.active = false;
                    m_WheelState.active = false;
                    SDL_HapticClose(m_pHaptic);
                }
            }
        } else if (_event.type == SDL_WINDOWEVENT) {
            if (_event.window.event == SDL_WINDOWEVENT_CLOSE) {
                WindowClose = true;
            }
        }
    }
}

void ChOgre_SDLInputHandler::grabMouse(bool grab) {
    SDL_bool _grab = grab ? SDL_TRUE : SDL_FALSE;
    SDL_SetWindowGrab(m_pSDLWindow, _grab);
}

void ChOgre_SDLInputHandler::runHapticEffect(ChOgreHapticEffect& effect, int iterations) {
    int _effect_id = SDL_HapticNewEffect(m_pHaptic, &effect);

    // SDL_HapticRunEffect(m_pHaptic, _effect_id, iterations);
    SDL_HapticRumbleInit(m_pHaptic);
    SDL_HapticRumblePlay(m_pHaptic, 0.5, 2000);

    SDL_HapticDestroyEffect(m_pHaptic, _effect_id);
}

void ChOgre_SDLInputHandler::runHapticRumble(float strength, double length) {
    SDL_HapticRumblePlay(m_pHaptic, strength, ((unsigned int)(length * 1000.0)));
}

void ChOgre_SDLInputHandler::stopHapticRumble() {
    SDL_HapticRumbleStop(m_pHaptic);
}

ChOgreKeyState& ChOgre_SDLInputHandler::getKeyState(SDL_Scancode scancode) {
    return m_KeyStates_scancode[scancode];
}

ChOgreKeyState& ChOgre_SDLInputHandler::getKeyState(SDL_Keycode keycode) {
    return m_KeyStates_keycode[keycode];
}

ChOgreMouseState& ChOgre_SDLInputHandler::getMouseState() {
    return m_MouseState;
}

ChOgreControllerState& ChOgre_SDLInputHandler::getControllerState() {
    return m_ControllerState;
}

ChOgreWheelState& ChOgre_SDLInputHandler::getWheelState() {
    return m_WheelState;
}

///////////////////////////////////////////////////////////////////////////////////////////
void ChOgre_SDLInputHandler::addCallback(ChOgreKeyboardCallback& callback) {
    m_KeyboardCallbackPtrs.push_back(&callback);
}

void ChOgre_SDLInputHandler::addCallback(ChOgreMouseCallback& callback) {
    m_MouseCallbackPtrs.push_back(&callback);
}

void ChOgre_SDLInputHandler::addCallback(ChOgreControllerCallback& callback) {
    m_ControllerCallbackPtrs.push_back(&callback);
}

void ChOgre_SDLInputHandler::addCallback(ChOgreWindowCallback& callback) {
    m_WindowCallbackPtrs.push_back(&callback);
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
void ChOgre_SDLInputHandler::removeCallback(ChOgreKeyboardCallback& callback) {
    auto a = *std::find(m_KeyboardCallbackPtrs.begin(), m_KeyboardCallbackPtrs.end(), &callback);
    auto b = m_KeyboardCallbackPtrs.back();
    std::swap(a, b);
    m_KeyboardCallbackPtrs.pop_back();
}

void ChOgre_SDLInputHandler::removeCallback(ChOgreMouseCallback& callback) {
    auto a = *std::find(m_MouseCallbackPtrs.begin(), m_MouseCallbackPtrs.end(), &callback);
    auto b = m_MouseCallbackPtrs.back();
    std::swap(a, b);
    m_MouseCallbackPtrs.pop_back();
}

void ChOgre_SDLInputHandler::removeCallback(ChOgreControllerCallback& callback) {
    auto a = *std::find(m_ControllerCallbackPtrs.begin(), m_ControllerCallbackPtrs.end(), &callback);
    auto b = m_ControllerCallbackPtrs.back();
    std::swap(a, b);
    m_ControllerCallbackPtrs.pop_back();
}

void ChOgre_SDLInputHandler::removeCallback(ChOgreWindowCallback& callback) {
    auto a = *std::find(m_WindowCallbackPtrs.begin(), m_WindowCallbackPtrs.end(), &callback);
    auto b = m_WindowCallbackPtrs.back();
    std::swap(a, b);
    m_WindowCallbackPtrs.pop_back();
}
///////////////////////////////////////////////////////////////////////////////////////////

void ChOgre_SDLInputHandler::m_CallKeyboardCallbacks(scancode_t ScanCode,
                                                     keycode_t KeyCode,
                                                     const ChOgreKeyState& KeyState) {
    std::for_each(
        m_KeyboardCallbackPtrs.begin(), m_KeyboardCallbackPtrs.end(),

        [ScanCode, KeyCode, &KeyState](ChOgreKeyboardCallback* ptr) { ptr->call(ScanCode, KeyCode, KeyState); });
}

void ChOgre_SDLInputHandler::m_CallMouseCallbacks() {
    std::for_each(m_MouseCallbackPtrs.begin(), m_MouseCallbackPtrs.end(),

                  [this](ChOgreMouseCallback* ptr) { ptr->call(this->getMouseState()); });
}

void ChOgre_SDLInputHandler::m_CallControllerCallbacks() {
    std::for_each(m_ControllerCallbackPtrs.begin(), m_ControllerCallbackPtrs.end(),

                  [this](ChOgreControllerCallback* ptr) { ptr->call(this->getControllerState()); });
}

void ChOgre_SDLInputHandler::m_CallWindowCallbacks() {
    std::for_each(m_WindowCallbackPtrs.begin(), m_WindowCallbackPtrs.end(),

                  [this](ChOgreWindowCallback* ptr) { ptr->call(); });
}
}
