#pragma once

#include "ChOgreGUIButton.h"
#include "chrono_ogre/Input/ChOgre_SDLInputHandler.h"
#include <MYGUI/MyGUI.h>
#include <MYGUI/MyGUI_OgrePlatform.h>
#include <memory>
#include <core/ChVector.h>

namespace ChOgre {

class CHOGRE_DLL_TAG ChOgreGUIManager {
  public:
    ChOgreGUIManager(Ogre::RenderWindow* RenderWindow,
                     Ogre::SceneManager* SceneManager,
                     ChOgre_SDLInputHandler* InputManager);
    ~ChOgreGUIManager();

    void setVisible(bool Visible);
    bool isVisible() { return m_isVisible; }

    template <typename T>
    inline std::unique_ptr<T> createWidget(const ChFloat3& Position, const ChFloat3& Size) {
        static_assert(std::is_base_of<ChOgreGUIElement, T>::value, "T must inherit from ChOgreGUIElement");
        return std::unique_ptr<T>(new T(Position, Size, m_pGUI));
    }

  protected:
    Ogre::SceneManager* m_pSceneManager;
    Ogre::RenderWindow* m_pRenderWindow;
    ChOgre_SDLInputHandler* m_pInputManager;

    MyGUI::Gui* m_pGUI;
    MyGUI::OgrePlatform* m_pPlatform;

    bool m_isVisible;

    ChOgreKeyboardCallback m_keyboard;
    ChOgreMouseCallback m_mouse;
    ChOgreWindowCallback m_window;

  private:
};
}