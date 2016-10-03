#pragma once

#include <vector>
#include <map>
#include <glfw3.h>

#include "chrono_ogre/ChOgreApi.h"
#include "ChOgreInputDataStructures.h"

#include "ChOgreInputCallback.h"
#include "ChOgreKeyboardCallback.h"
#include "ChOgreMouseCallback.h"
#include "ChOgreControllerCallback.h"
#include "ChOgreWindowCallback.h"

#define INPUT_DEADZONE 0.07

namespace chrono{
namespace ChOgre {

class CHOGRE_DLL_TAG ChOgre_GLFWInputHandler {

public:

	ChOgre_GLFWInputHandler();
	~ChOgre_GLFWInputHandler();

	virtual void update();

	virtual void grabMouse(bool grab);

	virtual void runHapticEffect(ChOgreHapticEffect& Effect, int Iterations);
	virtual void runHapticRumble(float Strength, double Length);
	virtual void stopHapticRumble();

	virtual ChOgreKeyState& getKeyState_scancode(int scancode);
	virtual ChOgreKeyState& getKeyState_keycode(int keycode);

	virtual ChOgreMouseState& getMouseState();

	virtual ChOgreControllerState& getControllerState();

	virtual ChOgreWheelState& getWheelState();

	void addCallback(ChOgreKeyboardCallback& callback);
	void addCallback(ChOgreMouseCallback& callback);
	void addCallback(ChOgreControllerCallback& callback);
	void addCallback(ChOgreWindowCallback& callback);

	void removeCallback(ChOgreKeyboardCallback& callback);
	void removeCallback(ChOgreMouseCallback& callback);
	void removeCallback(ChOgreControllerCallback& callback);
	void removeCallback(ChOgreWindowCallback& callback);

	double AxisThreshold;

	bool isWindowToClose();

	void callKeyboardCallbacks(scancode_t ScanCode, keycode_t KeyCode, const ChOgreKeyState& KeyState);
	void callMouseCallbacks();
	void callControllerCallbacks();
	void callWindowCallbacks();

protected:

	GLFWwindow* m_pGLFWwindow;

	bool m_windowClose;

	bool m_grabMouse;
	bool m_disabled;

	std::vector<ChOgreKeyboardCallback*> m_KeyboardCallbackPtrs;
	std::vector<ChOgreMouseCallback*> m_MouseCallbackPtrs;
	std::vector<ChOgreControllerCallback*> m_ControllerCallbackPtrs;
	std::vector<ChOgreWindowCallback*> m_WindowCallbackPtrs;

	std::map<int, ChOgreKeyState> m_KeyStates_scancode;
	std::map<int, ChOgreKeyState> m_KeyStates_keycode;

	ChOgreMouseState m_MouseState;
	ChOgreControllerState m_ControllerState;
	ChOgreWheelState m_WheelState;

	static std::string const WheelGUID;

};

}
}
