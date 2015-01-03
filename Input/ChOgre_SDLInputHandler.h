/*
Author: Charles Ricchio

An input manager based on SDL, as opposed to OIS. Will handle keyboard, mouse, and joystick input.
*/

#pragma once

#include <OGRE\Ogre.h>
#include <SDL.h>

#define INPUT_DEADZONE  0.25

namespace EnvironmentCore {

	typedef struct ECKeyState_t {

		bool down;
		double timestamp;

	} ECKeyState;

	typedef struct ECMouseState_t {

		ECKeyState left;
		ECKeyState right;
		ECKeyState middle;
		ECKeyState x1;
		ECKeyState x2;

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

	} ECMouseState;

	typedef struct ECControllerState_t {

		typedef struct __axisState_t {
			double value;
			double timestamp;
		} __axisState;

		__axisState lstickx, lsticky;
		__axisState rstickx, rsticky;
		__axisState ltrigger, rtrigger;

		ECKeyState a, b, x, y;
		ECKeyState back, start;
		ECKeyState lstick, rstick;
		ECKeyState d_left, d_right, d_up, d_down;
		ECKeyState lbumper, rbumper;

		bool active;

	} ECControllerState;

	typedef struct ECWheelState_t {

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

		ECControllerState::__axisState wheel;
		ECControllerState::__axisState accelerator;
		ECControllerState::__axisState brake;
		ECControllerState::__axisState clutch;

		ECKeyState lpaddle, rpaddle;
		ECKeyState lwheelb1, rwheelb1,
				   lwheelb2, rwheelb2,
				   lwheelb3, rwheelb3;
		ECKeyState gear1, gear2, gear3, gear4, gear5, gear6, reverse;
		ECKeyState red1, red2, red3, red4;
		ECKeyState black_up, black_down, black_left, black_right;

		__hatState d_pad;

		bool active;

	} ECWheelState;

	typedef SDL_HapticEffect ECHapticEffect;

	class EC_SDL_InputManager {

	public:

		EC_SDL_InputManager(Ogre::RenderWindow* renderWindow);
		~EC_SDL_InputManager();

		virtual void update();

		virtual void grabMouse(bool grab);

		virtual void runHapticEffect(ECHapticEffect& Effect, int Iterations);
		virtual void runHapticRumble(float Strength, double Length);
		virtual void stopHapticRumble();

		virtual ECKeyState& getKeyState(SDL_Scancode scancode);
		virtual ECKeyState& getKeyState(SDL_Keycode keycode);

		virtual ECMouseState& getMouseState();

		virtual ECControllerState& getControllerState();

		virtual ECWheelState& getWheelState();

		double AxisThreshold;

		bool WindowClose;

	protected:

		SDL_Window* m_pSDLWindow;

		std::map<SDL_Scancode, ECKeyState> m_KeyStates_scancode;
		std::map<SDL_Keycode, ECKeyState> m_KeyStates_keycode;

		ECMouseState m_MouseState;
		ECControllerState m_ControllerState;
		ECWheelState m_WheelState;
		
		SDL_Joystick* m_pController;
		SDL_Haptic* m_pHaptic;

		static std::string const WheelGUID;

	private:



	};

}