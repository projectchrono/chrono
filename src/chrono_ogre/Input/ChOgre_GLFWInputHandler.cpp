#include "ChOgre_GLFWInputHandler.h"

#include <chrono>
#include <algorithm>
#include <iostream>

std::map<GLFWwindow*, chrono::ChOgre::ChOgre_GLFWInputHandler*> HandlersByWindow;

static void glfwKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {

	chrono::ChOgre::ChOgreKeyState state;

	state.down = (action == GLFW_PRESS || action == GLFW_REPEAT) ? true : false;

	auto time = std::chrono::system_clock::now();
	std::chrono::duration<double, std::milli> time_ms = time.time_since_epoch();

	state.timestamp = (double)time_ms.count();

	HandlersByWindow[window]->getKeyState_keycode(key) = state;
	HandlersByWindow[window]->getKeyState_scancode(scancode) = state;

	HandlersByWindow[window]->callKeyboardCallbacks(scancode, key, state);
}

static void glfwMouseCallback(GLFWwindow* window, int button, int action, int mods) {
	auto time = std::chrono::system_clock::now();
	std::chrono::duration<double, std::milli> time_ms = time.time_since_epoch();

	auto timestamp = (double)time_ms.count();

	switch (button) {
		case GLFW_MOUSE_BUTTON_1:
			HandlersByWindow[window]->getMouseState().left.down = (action == GLFW_PRESS) ? true : false;
			HandlersByWindow[window]->getMouseState().left.timestamp = timestamp;
			break;
		case GLFW_MOUSE_BUTTON_2:
			HandlersByWindow[window]->getMouseState().right.down = (action == GLFW_PRESS) ? true : false;
			HandlersByWindow[window]->getMouseState().right.timestamp = timestamp;
			break;
		case GLFW_MOUSE_BUTTON_3:
			HandlersByWindow[window]->getMouseState().middle.down = (action == GLFW_PRESS) ? true : false;
			HandlersByWindow[window]->getMouseState().middle.timestamp = timestamp;
			break;
		case GLFW_MOUSE_BUTTON_4:
			HandlersByWindow[window]->getMouseState().x1.down = (action == GLFW_PRESS) ? true : false;
			HandlersByWindow[window]->getMouseState().x1.timestamp = timestamp;
			break;
		case GLFW_MOUSE_BUTTON_5:
			HandlersByWindow[window]->getMouseState().x2.down = (action == GLFW_PRESS) ? true : false;
			HandlersByWindow[window]->getMouseState().x2.timestamp = timestamp;
			break;
	}

	HandlersByWindow[window]->callMouseCallbacks();
}

static void glfwMouseMoveCallback(GLFWwindow* window, double x, double y) {
	auto time = std::chrono::system_clock::now();
	std::chrono::duration<double, std::milli> time_ms = time.time_since_epoch();

	auto timestamp = (double)time_ms.count();

	double x_current = HandlersByWindow[window]->getMouseState().position.x;
	double y_current = HandlersByWindow[window]->getMouseState().position.y;

	HandlersByWindow[window]->getMouseState().position.x = x;
	HandlersByWindow[window]->getMouseState().position.y = y;
	HandlersByWindow[window]->getMouseState().position.xrel = x - x_current;
	HandlersByWindow[window]->getMouseState().position.yrel = y - y_current;
	HandlersByWindow[window]->getMouseState().position.timestamp = timestamp;

	HandlersByWindow[window]->callMouseCallbacks();
}

static void glfwMouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
	auto time = std::chrono::system_clock::now();
	std::chrono::duration<double, std::milli> time_ms = time.time_since_epoch();

	auto timestamp = (double)time_ms.count();

	HandlersByWindow[window]->getMouseState().wheel.x = xoffset;
	HandlersByWindow[window]->getMouseState().wheel.y = yoffset;
	HandlersByWindow[window]->getMouseState().wheel.timestamp = timestamp;

	HandlersByWindow[window]->callMouseCallbacks();
}

namespace chrono{
namespace ChOgre {

	std::string const ChOgre_GLFWInputHandler::WheelGUID = "6d049bc2000000000000504944564944";

	ChOgre_GLFWInputHandler::ChOgre_GLFWInputHandler() {

		if (!glfwInit()) {
    	std::cout << "\n\nGLFW Failed to init\n\n\n";
			return;
		}

		m_pGLFWwindow = glfwGetCurrentContext();

		if (m_pGLFWwindow == NULL) {
			std::cout << "\n\nERROR GLFW failed to anchor to window\n\n\n";
		}

		HandlersByWindow[m_pGLFWwindow] = this;

		glfwSetKeyCallback(m_pGLFWwindow, glfwKeyCallback);
		glfwSetMouseButtonCallback(m_pGLFWwindow, glfwMouseCallback);
		glfwSetCursorPosCallback(m_pGLFWwindow, glfwMouseMoveCallback);
		glfwSetScrollCallback(m_pGLFWwindow, glfwMouseScrollCallback);


		m_grabMouse = false;
		m_disabled = false;
	}

	ChOgre_GLFWInputHandler::~ChOgre_GLFWInputHandler() {
		auto handlerLoc = HandlersByWindow.find(m_pGLFWwindow);

		HandlersByWindow.erase(handlerLoc);
	}

	void ChOgre_GLFWInputHandler::update() {
		if (glfwWindowShouldClose(m_pGLFWwindow)) {
			m_windowClose = true;
		}

		if (m_grabMouse) {
			int w, h = 0;
			glfwGetWindowSize(m_pGLFWwindow, &w, &h);
			glfwSetCursorPos(m_pGLFWwindow, static_cast<double>(w) / 2.0, static_cast<double>(h) / 2.0);
		}
	}

	void ChOgre_GLFWInputHandler::grabMouse(bool grab) {
		m_grabMouse = grab;
	}

	void ChOgre_GLFWInputHandler::runHapticEffect(ChOgreHapticEffect& Effect, int Iterations) {

	}

	void ChOgre_GLFWInputHandler::runHapticRumble(float Strength, double Length) {

	}

	void ChOgre_GLFWInputHandler::stopHapticRumble() {

	}

	ChOgreKeyState& ChOgre_GLFWInputHandler::getKeyState_scancode(int scancode) {
		return m_KeyStates_scancode[scancode];
	}

	ChOgreKeyState& ChOgre_GLFWInputHandler::getKeyState_keycode(int keycode) {
		return m_KeyStates_keycode[keycode];
	}

	ChOgreMouseState& ChOgre_GLFWInputHandler::getMouseState() {
		return m_MouseState;
	}

	ChOgreControllerState& ChOgre_GLFWInputHandler::getControllerState() {
		return m_ControllerState;
	}

	ChOgreWheelState& ChOgre_GLFWInputHandler::getWheelState() {
		return m_WheelState;
	}

	void ChOgre_GLFWInputHandler::addCallback(ChOgreKeyboardCallback& callback) {
		m_KeyboardCallbackPtrs.push_back(&callback);
	}

	void ChOgre_GLFWInputHandler::addCallback(ChOgreMouseCallback& callback) {
		m_MouseCallbackPtrs.push_back(&callback);
	}

	void ChOgre_GLFWInputHandler::addCallback(ChOgreControllerCallback& callback) {
		m_ControllerCallbackPtrs.push_back(&callback);
	}

	void ChOgre_GLFWInputHandler::addCallback(ChOgreWindowCallback& callback) {
		m_WindowCallbackPtrs.push_back(&callback);
	}


	void ChOgre_GLFWInputHandler::removeCallback(ChOgreKeyboardCallback& callback) {
		auto a = *std::find(m_KeyboardCallbackPtrs.begin(), m_KeyboardCallbackPtrs.end(), &callback);
    auto b = m_KeyboardCallbackPtrs.back();
    std::swap(a, b);
    m_KeyboardCallbackPtrs.pop_back();
	}

	void ChOgre_GLFWInputHandler::removeCallback(ChOgreMouseCallback& callback) {
		auto a = *std::find(m_MouseCallbackPtrs.begin(), m_MouseCallbackPtrs.end(), &callback);
    auto b = m_MouseCallbackPtrs.back();
    std::swap(a, b);
    m_MouseCallbackPtrs.pop_back();
	}

	void ChOgre_GLFWInputHandler::removeCallback(ChOgreControllerCallback& callback) {
		auto a = *std::find(m_ControllerCallbackPtrs.begin(), m_ControllerCallbackPtrs.end(), &callback);
    auto b = m_ControllerCallbackPtrs.back();
    std::swap(a, b);
    m_ControllerCallbackPtrs.pop_back();
	}

	void ChOgre_GLFWInputHandler::removeCallback(ChOgreWindowCallback& callback) {
		auto a = *std::find(m_WindowCallbackPtrs.begin(), m_WindowCallbackPtrs.end(), &callback);
		auto b = m_WindowCallbackPtrs.back();
		std::swap(a, b);
		m_WindowCallbackPtrs.pop_back();
	}

	bool ChOgre_GLFWInputHandler::isWindowToClose() {
		return m_windowClose;
	}

	void ChOgre_GLFWInputHandler::callKeyboardCallbacks(scancode_t ScanCode, keycode_t KeyCode, const ChOgreKeyState& KeyState) {
		std::for_each(
        m_KeyboardCallbackPtrs.begin(), m_KeyboardCallbackPtrs.end(),

        [ScanCode, KeyCode, &KeyState](ChOgreKeyboardCallback* ptr) { ptr->call(ScanCode, KeyCode, KeyState); });
	}

	void ChOgre_GLFWInputHandler::callMouseCallbacks() {
		std::for_each(m_MouseCallbackPtrs.begin(), m_MouseCallbackPtrs.end(),

                  [this](ChOgreMouseCallback* ptr) { ptr->call(this->getMouseState()); });
	}

	void ChOgre_GLFWInputHandler::callControllerCallbacks() {
		std::for_each(m_ControllerCallbackPtrs.begin(), m_ControllerCallbackPtrs.end(),

                  [this](ChOgreControllerCallback* ptr) { ptr->call(this->getControllerState()); });
	}

	void ChOgre_GLFWInputHandler::callWindowCallbacks() {
		std::for_each(m_WindowCallbackPtrs.begin(), m_WindowCallbackPtrs.end(),

                  [this](ChOgreWindowCallback* ptr) { ptr->call(); });
	}

}
}
