#include "ChOgreGUIManager.h"

namespace ChOgre {

	ChOgreGUIManager::_KeyCallback::_KeyCallback(MyGUI::Gui** GUI) {
		m_pGUI = GUI;
	}

	void ChOgreGUIManager::_KeyCallback::call(scancode_t ScanCode, keycode_t KeyCode, const ChOgreKeyState& KeyState) {
		if (KeyState.down) {
			MyGUI::InputManager::getInstance().injectKeyPress(MyGUI::KeyCode((MyGUI::KeyCode::Enum)ScanCode), SDL_GetKeyName(KeyCode)[0]);
		}
		if (!KeyState.down) {
			MyGUI::InputManager::getInstance().injectKeyRelease(MyGUI::KeyCode((MyGUI::KeyCode::Enum)ScanCode));
		}
	}

	ChOgreGUIManager::_MouseCallback::_MouseCallback(MyGUI::Gui** GUI) {
		m_pGUI = GUI;
	}

	void ChOgreGUIManager::_MouseCallback::call(const ChOgreMouseState& MouseState) {
		MyGUI::InputManager::getInstance().injectMouseMove(MouseState.position.x, MouseState.position.y, MouseState.wheel.y);

		if (MouseState.left.down) {
			MyGUI::InputManager::getInstance().injectMousePress(MouseState.position.x, MouseState.position.y, MyGUI::MouseButton::Left);
		}
		else if (!MouseState.left.down) {
			MyGUI::InputManager::getInstance().injectMouseRelease(MouseState.position.x, MouseState.position.y, MyGUI::MouseButton::Left);
		}
		if (MouseState.right.down) {
			MyGUI::InputManager::getInstance().injectMousePress(MouseState.position.x, MouseState.position.y, MyGUI::MouseButton::Right);
		}
		else if (!MouseState.right.down) {
			MyGUI::InputManager::getInstance().injectMouseRelease(MouseState.position.x, MouseState.position.y, MyGUI::MouseButton::Right);
		}
		if (MouseState.middle.down) {
			MyGUI::InputManager::getInstance().injectMousePress(MouseState.position.x, MouseState.position.y, MyGUI::MouseButton::Middle);
		}
		else if (!MouseState.middle.down) {
			MyGUI::InputManager::getInstance().injectMouseRelease(MouseState.position.x, MouseState.position.y, MyGUI::MouseButton::Middle);
		}
	}



	ChOgreGUIManager::ChOgreGUIManager(Ogre::RenderWindow* RenderWindow, Ogre::SceneManager* SceneManager, ChOgre_SDLInputHandler* InputManager) : m_keyboard(&m_pGUI), m_mouse(&m_pGUI) {
		m_pInputManager = InputManager;
		m_pSceneManager = SceneManager;
		m_pRenderWindow = RenderWindow;

		m_pPlatform = new MyGUI::OgrePlatform();
		m_pPlatform->initialise(m_pRenderWindow, m_pSceneManager);
		m_pGUI = new MyGUI::Gui();
		m_pGUI->initialise();

		m_pInputManager->addCallback(m_keyboard);
		m_pInputManager->addCallback(m_mouse);
	}

	ChOgreGUIManager::~ChOgreGUIManager() {
		m_pGUI->shutdown();
		delete m_pGUI;
		m_pPlatform->shutdown();
		delete m_pPlatform;
	}

	void ChOgreGUIManager::setVisible(bool Visible) {
		m_isVisible = Visible;
	}

}