/*
Author: Charles Ricchio

An input manager based on SDL, as opposed to OIS. Will handle keyboard, mouse, and joystick input.
*/

#pragma once

#include <OGRE\Ogre.h>
#include <SDL.h>
#include <vector>
#include <map>

#include "../ChOgre.h"
#include "ChOgreInputDataStructures.h"

#include "ChOgreInputCallback.h"
#include "ChOgreKeyboardCallback.h"
#include "ChOgreMouseCallback.h"
#include "ChOgreControllerCallback.h"

#define INPUT_DEADZONE  0.25

namespace ChOgre {

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

		void addCallback(ChOgreKeyboardCallback& callback);
		void addCallback(ChOgreMouseCallback& callback);
		void addCallback(ChOgreControllerCallback& callback);

		void removeCallback(ChOgreKeyboardCallback& callback);
		void removeCallback(ChOgreMouseCallback& callback);
		void removeCallback(ChOgreControllerCallback& callback);

		double AxisThreshold;

		bool WindowClose;

	protected:

		SDL_Window* m_pSDLWindow;

		void m_CallKeyboardCallbacks(keycode_t KeyCode, const ChOgreKeyState& KeyState);
		void m_CallMouseCallbacks();
		void m_CallControllerCallbacks();

		std::vector<ChOgreKeyboardCallback*> m_KeyboardCallbackPtrs;
		std::vector<ChOgreMouseCallback*> m_MouseCallbackPtrs;
		std::vector<ChOgreControllerCallback*> m_ControllerCallbackPtrs;

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