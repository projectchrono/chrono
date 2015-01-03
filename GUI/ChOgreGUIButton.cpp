#include "ECGUIButton.h"

namespace EnvironmentCore {

	ECGUIButton::ECGUIButton(Ogre::Overlay* Overlay, EC_SDL_InputManager* InputManager) : ECGUIPanel(Overlay) {
		m_pInputManager = InputManager; 

		m_pText = static_cast<Ogre::TextAreaOverlayElement*>(Ogre::OverlayManager::getSingleton().createOverlayElement("TextArea", "ButtonText" + std::to_string(g_count)));
		
		m_pPanel->addChild(m_pText);
		
		m_Font = Ogre::FontManager::getSingleton().create("default", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
		

		m_Font->setType(Ogre::FT_TRUETYPE);
		m_Font->setSource("Minecraftia.ttf");
		m_Font->setTrueTypeSize(8);
		m_Font->setTrueTypeResolution(70);
		m_Font->setAntialiasColour(false);
		m_Font->load();

		m_db = true;
		m_pressed = false;
	}

	ECGUIButton::~ECGUIButton() {
		Ogre::FontManager::getSingleton().remove((Ogre::ResourcePtr)m_Font);
		Ogre::OverlayManager::getSingleton().destroyOverlayElement(m_pText);
	}

	void ECGUIButton::setTextColor(double r, double g, double b) {
		Ogre::Real _r = (Ogre::Real)r;
		Ogre::Real _g = (Ogre::Real)g;
		Ogre::Real _b = (Ogre::Real)b;

		m_pText->setColour(Ogre::ColourValue(_r, _g, _b));
	}

	void ECGUIButton::setText(std::string Text) {
		m_pText->setCaption(Text);
		_resizePanel();
	}

	void ECGUIButton::setFont(double Size) {
		m_pText->setFontName("default");
		m_pText->setCharHeight((Ogre::Real)Size);
		_resizePanel();
	}

	void ECGUIButton::update() {
		double _mX = m_pInputManager->getMouseState().position.x;
		double _mY = m_pInputManager->getMouseState().position.y;

		double _pX = (double)m_pPanel->getLeft();
		double _pY = (double)m_pPanel->getTop();
		double _pW = (double)m_pPanel->getWidth();
		double _pH = (double)m_pPanel->getHeight();

		if (_mX > _pX && _mX < (_pX + _pW) && _mY > _pY && _mY < (_pY + _pH) && m_pInputManager->getMouseState().left.down) {
			down = true;
			if (m_db) {
				m_pressed = true;
			}
			m_db = false;
		}

		if (!m_pInputManager->getMouseState().left.down) {
			m_db = true;
			down = false;
		}
	}

	bool ECGUIButton::isPressed() {
		bool _ret = m_pressed;
		m_pressed = false;
		return _ret;
	}

	void ECGUIButton::_resizePanel() {
		double _font_a_r = m_Font->getGlyphAspectRatio(0x0042);
		m_pPanel->setWidth(m_pText->getCaption().asUTF8().length() * ( ( _font_a_r * m_pText->getCharHeight() ) / 1.9 ) );
		m_pPanel->setHeight(m_pText->getCharHeight());
	}

}