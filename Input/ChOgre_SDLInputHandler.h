/*
Author: Charles Ricchio

An input manager based on SDL, as opposed to OIS. Will handle keyboard, mouse, and joystick input.
*/

#pragma once

#include <OGRE\Ogre.h>
#include <SDL.h>

#include "../ChOgre.h"

#define INPUT_DEADZONE  0.25

namespace ChOgre {

	typedef struct ChOgreKeyState_t {

		bool down;
		double timestamp;

	} CHOGRE_DLL_TAG ChOgreKeyState;

	typedef struct ChOgreMouseState_t {

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

	} CHOGRE_DLL_TAG ChOgreMouseState;

	typedef struct ChOgreControllerState_t {

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

	} CHOGRE_DLL_TAG ChOgreControllerState;

	typedef struct ChOgreWheelState_t {

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
		ChOgreKeyState lwheelb1, rwheelb1,
				   lwheelb2, rwheelb2,
				   lwheelb3, rwheelb3;
		ChOgreKeyState gear1, gear2, gear3, gear4, gear5, gear6, reverse;
		ChOgreKeyState red1, red2, red3, red4;
		ChOgreKeyState black_up, black_down, black_left, black_right;

		__hatState d_pad;

		bool active;

	} CHOGRE_DLL_TAG ChOgreWheelState;

	typedef SDL_HapticEffect CHOGRE_DLL_TAG ChOgreHapticEffect;

	class CHOGRE_DLL_TAG ChOgre_SDLInputHandler {

	public:

		ChOgre_SDLInputHandler(Ogre::RenderWindow* renderWindow);
		~ChOgre_SDLInputHandler();

		virtual void update();

		virtual void grabMouse(bool grab);

		virtual void runHapticEffect(ChOgreHapticEffect& Effect, int Iterations);
		virtual void runHapticRumble(float Strength, double Length);
		virtual void stopHapticRumble();

		virtual ChOgreKeyState& getKeyState(SDL_Scancode scancode);
		virtual ChOgreKeyState& getKeyState(SDL_Keycode keycode);

		virtual ChOgreMouseState& getMouseState();

		virtual ChOgreControllerState& getControllerState();

		virtual ChOgreWheelState& getWheelState();

		double AxisThreshold;

		bool WindowClose;

	protected:

		SDL_Window* m_pSDLWindow;

		std::map<SDL_Scancode, ChOgreKeyState> m_KeyStates_scancode;
		std::map<SDL_Keycode, ChOgreKeyState> m_KeyStates_keycode;

		ChOgreMouseState m_MouseState;
		ChOgreControllerState m_ControllerState;
		ChOgreWheelState m_WheelState;
		
		SDL_Joystick* m_pController;
		SDL_Haptic* m_pHaptic;

		static std::string const WheelGUID;

	private:



	};

}