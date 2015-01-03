#pragma once

#include "ECGUIElement.h"
#include "ECGUIPanel.h"
#include "EC_SDL_InputManager.h"
#include <OGRE\Overlay\OgreFontManager.h>
#include <OGRE\Overlay\OgreFont.h>
#include <OGRE\Overlay\OgreTextAreaOverlayElement.h>

namespace EnvironmentCore {

	class ECGUIButton : public ECGUIPanel {

	public:

		ECGUIButton(Ogre::Overlay* Overlay, EC_SDL_InputManager* InputManager);
		~ECGUIButton();

		virtual void setTextColor(double r, double g, double b);
		virtual void setText(std::string Text);
		virtual void setFont(double CharacterHeight);
		virtual void update();

		virtual bool isPressed();
		bool down;

	protected:

		bool m_db;
		bool m_pressed;

		EC_SDL_InputManager* m_pInputManager;
		Ogre::TextAreaOverlayElement* m_pText;
		Ogre::FontPtr m_Font;

		virtual void _resizePanel();

	private:



	};

}