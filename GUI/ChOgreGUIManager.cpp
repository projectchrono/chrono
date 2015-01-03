#include "ECGUIManager.h"

namespace EnvironmentCore {

	ECGUIManager::ECGUIManager(Ogre::SceneManager* SceneManager, EC_SDL_InputManager* InputManager) {
		m_pInputManager = InputManager;
		m_pSceneManager = SceneManager;
		m_pOverlaySystem = new Ogre::OverlaySystem();
		SceneManager->addRenderQueueListener(m_pOverlaySystem);
		m_pOverlay = Ogre::OverlayManager::getSingleton().create("EnvironmentCoreGUI");
		setActive(true);
	}

	ECGUIManager::~ECGUIManager() {
		Ogre::OverlayManager::getSingleton().destroy(m_pOverlay);
		m_pSceneManager->removeRenderQueueListener(m_pOverlaySystem);
	}

	void ECGUIManager::setActive(bool Active) {
		if (Active) {
			m_pOverlay->show();
		}
		else {
			m_pOverlay->hide();
		}
	}

	ECGUIPanel* ECGUIManager::createPanel(std::string Name) {

		ECGUIPanel* _ret = new ECGUIPanel(m_pOverlay);
		_ret->setName(Name);

		m_ElementList.push_back(_ret);

		setActive(true);

		return _ret;
	}

	ECGUIText* ECGUIManager::createText(std::string Name) {
		ECGUIText* _ret = new ECGUIText(m_pOverlay);
		_ret->setName(Name);

		m_ElementList.push_back(_ret);

		setActive(true);

		return _ret;
	}

	ECGUIButton* ECGUIManager::createButton(std::string Name) {
		ECGUIButton* _ret = new ECGUIButton(m_pOverlay, m_pInputManager);
		_ret->setName(Name);

		m_ElementList.push_back(_ret);

		setActive(true);

		return _ret;
	}

	void ECGUIManager::removeElement(std::string Name) {
		for (unsigned int i = 0; i < m_ElementList.size(); i++) {
			if (m_ElementList[i]->getName() == Name) {
				delete m_ElementList[i];
				m_ElementList[i] = m_ElementList.back();
				m_ElementList.pop_back();
			}
		}
	}

	void ECGUIManager::removeElement(ECGUIElement* Element) {
		for (unsigned int i = 0; i < m_ElementList.size(); i++) {
			if (m_ElementList[i] == Element) {
				delete m_ElementList[i];
				m_ElementList[i] = m_ElementList.back();
				m_ElementList.pop_back();
			}
		}
	}

	void ECGUIManager::update() {
		for (unsigned int i = 0; i < m_ElementList.size(); i++) {
			m_ElementList[i]->update();
		}
	}

	template <typename t>
	t* ECGUIManager::getElement(std::string Name) {

		t _l;
		ECGUIElement* _e = dynamic_cast<ECGUIElement*>(&_l);

		if (_e) {
			for (unsigned int i = 0; i < m_ElementList.size(); i++) {
				if (m_ElementList[i]->getName() == Name) {
					return (t*)m_ElementList[i];
				}
			}
		}

		return nullptr;
	}

}