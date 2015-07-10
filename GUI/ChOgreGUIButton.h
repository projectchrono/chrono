#pragma once

#include "../Input/ChOgre_SDLInputHandler.h"
#include "ChOgreGUICallback.h"
#include "ChOgreGUIElement.h"

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreGUIButton : public ChOgreGUIElement {

	public:

		ChOgreGUIButton();
		ChOgreGUIButton(MyGUI::Gui* GUI);
		ChOgreGUIButton(const ChInt3& Position, const ChInt3& Size, MyGUI::Gui* GUI);
		~ChOgreGUIButton();

		virtual inline void setColor(float r, float g, float b);
		virtual inline void setTextColor(float r, float g, float b);
		virtual inline void setText(const std::string& Text);
		virtual inline void setFont(const std::string& Name);
		virtual void setPosition(const chrono::ChVector<int>& Position);
		virtual void setSize(const chrono::ChVector<int>& Size);
		virtual void update();

		virtual void setClickCallback(ChOgreGUICallback& Callback);

	protected:

		bool m_db;
		bool m_pressed;

		MyGUI::ButtonPtr m_pButton;

	private:



	};

	typedef std::unique_ptr<ChOgreGUIButton> ChOgreGUIButtonPtr;

}