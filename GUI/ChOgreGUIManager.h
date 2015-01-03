#pragma once

#include "EC_SDL_InputManager.h"
#include "ECGUIElement.h"
#include "ECGUIPanel.h"
#include "ECGUIText.h"
#include "ECGUIButton.h"
#include <OGRE\Ogre.h>
#include <OGRE\OgrePrerequisites.h>
#include <OGRE\Overlay\OgreOverlaySystem.h>
#include <OGRE\Overlay\OgreOverlay.h>
#include <OGRE\Overlay\OgreOverlayManager.h>

namespace EnvironmentCore {

	class ECGUIManager {

	public:

		ECGUIManager(Ogre::SceneManager* SceneManager, EC_SDL_InputManager* InputManager);
		~ECGUIManager();

		virtual void setActive(bool Active);

		virtual ECGUIPanel* createPanel(std::string Name = "");
		virtual ECGUIText* createText(std::string Name = "");
		virtual ECGUIButton* createButton(std::string Name = "");

		virtual void removeElement(std::string Name);
		virtual void removeElement(ECGUIElement* Element);

		virtual void update();

		template <typename t>
		t* getElement(std::string Name);

	protected:

		Ogre::Overlay* m_pOverlay;
		Ogre::SceneManager* m_pSceneManager;
		Ogre::OverlaySystem* m_pOverlaySystem;
		EC_SDL_InputManager* m_pInputManager;

		std::vector<ECGUIElement*> m_ElementList;

	private:



	};

}