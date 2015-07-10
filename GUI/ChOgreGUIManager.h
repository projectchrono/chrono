#pragma once

#include "ChOgreGUIButton.h"
#include "../Input/ChOgre_SDLInputHandler.h"
#include <MYGUI/MyGUI.h>
#include <MYGUI/MyGUI_OgrePlatform.h>
#include <memory>
#include <core/ChVector.h>

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreGUIManager {

	public:

		ChOgreGUIManager(Ogre::RenderWindow* RenderWindow, Ogre::SceneManager* SceneManager, ChOgre_SDLInputHandler* InputManager);
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


		class _KeyCallback : public ChOgreKeyboardCallback {

		public:

			_KeyCallback(MyGUI::Gui** GUI);

			void call(scancode_t ScanCode, keycode_t KeyCode, const ChOgreKeyState& KeyState) override;

		protected:

			MyGUI::Gui** m_ppGUI;

		};

		class _MouseCallback : public ChOgreMouseCallback {

		public:

			_MouseCallback(MyGUI::Gui** GUI);

			void call(const ChOgreMouseState& MouseState) override;

		protected:

			MyGUI::Gui** m_ppGUI;

		};

		class _WindowCallback : public ChOgreWindowCallback {
			
		public:
			
			_WindowCallback(Ogre::RenderWindow* RenderWindow);

			void call() override;

		protected:

			Ogre::RenderWindow* m_pRenderWindow;

		};

		_KeyCallback m_keyboard;
		_MouseCallback m_mouse;
		_WindowCallback m_window;

	private:

	};

}