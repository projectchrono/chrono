#include "ECGUIElement.h"

namespace EnvironmentCore {

	ECGUIElement::ECGUIElement(Ogre::Overlay* Overlay) {
		m_pOverlay = Overlay;
	}

	ECGUIElement::~ECGUIElement() {

	}

	void ECGUIElement::setName(std::string Name) {
		m_Name = Name;
	}

	std::string ECGUIElement::getName() {
		return m_Name;
	}

}