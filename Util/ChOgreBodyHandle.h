#pragma once

#include "../ChOgre.h"
#include "../Graphics/ChOgreBody.h"

namespace ChOgre {

	class CHOGRE_DLL_TAG ChOgreBodyHandle {

	public:

		ChOgreBodyHandle();
		ChOgreBodyHandle(const ChOgreBodyHandle& other);
		ChOgreBodyHandle(ChOgreBodyHandle&& other);
		ChOgreBodyHandle(ChOgreBody& Body);
		ChOgreBodyHandle(ChOgreBodySharedPtr& BodyPtr);
		~ChOgreBodyHandle();

		ChOgreBodyHandle& operator=(const ChOgreBodyHandle& other);
		ChOgreBodyHandle& operator=(ChOgreBodyHandle&& other);

		chrono::ChSharedBodyPtr operator-> ();
		ChOgreBody& body();
		void setBodyPtr(ChOgreBodySharedPtr& BodyPtr);

	private:

		ChOgreBodySharedPtr m_pBody;

	};

}