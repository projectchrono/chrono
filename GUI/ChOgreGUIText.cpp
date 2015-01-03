/*
Author: Charles Ricchio
*/

#include "ECGUIText.h"

namespace EnvironmentCore {

	unsigned int ECGUIText::g_count = 0;

	ECGUIText::ECGUIText(Ogre::Overlay* Overlay) : ECGUIElement(Overlay) {
		m_pPanel = static_cast<Ogre::OverlayContainer*>(Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "TextPanel" + std::to_string(g_count)));
		m_pText = static_cast<Ogre::TextAreaOverlayElement*>(Ogre::OverlayManager::getSingleton().createOverlayElement("TextArea", "Text" + std::to_string(g_count)));

		m_pOverlay->add2D(m_pPanel);
		m_pPanel->addChild(m_pText);

		
		m_Font = Ogre::FontManager::getSingleton().create("default", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		

		m_Font->setType(Ogre::FT_TRUETYPE);
		m_Font->setSource("Minecraftia.ttf");
		m_Font->setTrueTypeSize(8);
		m_Font->setTrueTypeResolution(70);
		m_Font->setAntialiasColour(false);
		m_Font->load();

		g_count++;
	}

	ECGUIText::~ECGUIText() {
		Ogre::FontManager::getSingleton().remove((Ogre::ResourcePtr)m_Font);
		m_pOverlay->remove2D(m_pPanel);
		Ogre::OverlayManager::getSingleton().destroyOverlayElement(m_pPanel);
		Ogre::OverlayManager::getSingleton().destroyOverlayElement(m_pText);
		g_count--;
	}

	void ECGUIText::setName(std::string Name) {
		m_Name = Name;
	}

	void ECGUIText::setPosition(double x, double y) {
		m_pPanel->setPosition((Ogre::Real)x, (Ogre::Real)y);
	}

	void ECGUIText::setSize(double x, double y) {
		m_pPanel->setDimensions((Ogre::Real)x, (Ogre::Real)y);
		m_pText->setDimensions((Ogre::Real)x, (Ogre::Real)y);
	}

	void ECGUIText::setColor(double r, double g, double b) {

		Ogre::Real _r = (Ogre::Real)r;
		Ogre::Real _g = (Ogre::Real)g;
		Ogre::Real _b = (Ogre::Real)b;

		m_pText->setColour(Ogre::ColourValue(_r, _g, _b));
	}

	void ECGUIText::setText(std::string Text) {
		m_pText->setCaption(Text);
	}

	void ECGUIText::setFont(double Size) {
		m_pText->setFontName("default");
		m_pText->setCharHeight((Ogre::Real)Size);
	}

	chrono::ChVector<> ECGUIText::getPosition() {
		chrono::ChVector<> _ret((double)m_pPanel->getLeft(), (double)m_pPanel->getTop(), 0);
		return _ret;
	}

	chrono::ChVector<> ECGUIText::getSize() {
		chrono::ChVector<> _ret((double)m_pPanel->getWidth(), (double)m_pPanel->getHeight(), 0);
		return _ret;
	}

}