#include "ChOgreGUIManager.h"

namespace ChOgre {

ChOgreGUIManager::ChOgreGUIManager(Ogre::RenderWindow* RenderWindow,
                                   Ogre::SceneManager* SceneManager,
                                   ChOgre_SDLInputHandler* InputManager) {
    m_pInputManager = InputManager;
    m_pSceneManager = SceneManager;
    m_pRenderWindow = RenderWindow;

    m_pPlatform = new MyGUI::OgrePlatform();
    m_pPlatform->initialise(m_pRenderWindow, m_pSceneManager);
    m_pGUI = new MyGUI::Gui();
    m_pGUI->initialise();

    m_pGUI->setVisiblePointer(false);

    MyGUI::LayerManager::getInstance().resizeView(MyGUI::IntSize(RenderWindow->getWidth(), RenderWindow->getHeight()));

    m_pInputManager->addCallback(m_keyboard);
    m_pInputManager->addCallback(m_mouse);

    m_keyboard.call = [](scancode_t s, keycode_t k, const ChOgreKeyState& ks) {
        if (ks.down) {
            MyGUI::InputManager::getInstance().injectKeyPress(MyGUI::KeyCode((MyGUI::KeyCode::Enum)s),
                                                              SDL_GetKeyName(k)[0]);
            //(*m_ppGUI)->injectKeyPress(MyGUI::KeyCode((MyGUI::KeyCode::Enum)ScanCode), SDL_GetKeyName(KeyCode)[0]);
        }
        if (!ks.down) {
            MyGUI::InputManager::getInstance().injectKeyRelease(MyGUI::KeyCode((MyGUI::KeyCode::Enum)s));
            //(*m_ppGUI)->injectKeyRelease(MyGUI::KeyCode((MyGUI::KeyCode::Enum)ScanCode));
        }
    };

    m_mouse.call = [](const ChOgreMouseState& m) {
        float _x = m.position.x * MyGUI::RenderManager::getInstance().getViewSize().width;
        float _y = m.position.y * MyGUI::RenderManager::getInstance().getViewSize().height;

        if (m.position.xrel != 0 || m.position.yrel != 0) {
            MyGUI::InputManager::getInstance().injectMouseMove(_x, _y, m.wheel.y);
        }

        if (m.left.down) {
            MyGUI::InputManager::getInstance().injectMousePress(_x, _y, MyGUI::MouseButton::Left);
        } else if (m.left.down == false) {
            MyGUI::InputManager::getInstance().injectMouseRelease(_x, _y, MyGUI::MouseButton::Left);
        }
        if (m.right.down) {
            MyGUI::InputManager::getInstance().injectMousePress(_x, _y, MyGUI::MouseButton::Right);
        } else if (m.right.down == false) {
            MyGUI::InputManager::getInstance().injectMouseRelease(_x, _y, MyGUI::MouseButton::Right);
        }
        if (m.middle.down) {
            MyGUI::InputManager::getInstance().injectMousePress(_x, _y, MyGUI::MouseButton::Middle);
        } else if (m.middle.down == false) {
            MyGUI::InputManager::getInstance().injectMouseRelease(_x, _y, MyGUI::MouseButton::Middle);
        }
    };

    m_window.call = []() {
        // MyGUI::LayerManager::getInstance().resizeView(MyGUI::IntSize(m_pRenderWindow->getWidth(),
        // m_pRenderWindow->getHeight()));
    };
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