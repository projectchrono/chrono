%{

/* Includes the header in the wrapper code */
#include "physics/ChObject.h"

using namespace chrono;

%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
//%include "../physics/ChObject.h"    

namespace chrono
{

class ChObj : public ChShared {
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChObj, ChShared);

public:

	ChObj();							
	virtual ~ChObj();
	void Copy(ChObj* source);

	int  GetIdentifier () { return identifier; }	
	void SetIdentifier (int id) { identifier = id; }

	double GetChTime () { return ChTime; }	
	void   SetChTime (double m_time) { ChTime = m_time; }

	char* GetName ();
	void SetName (char myname[]);

	std::string GetNameString ();
	void SetNameString (std::string& myname);

	virtual void StreamOUT(ChStreamOutBinary& mstream);
	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutAscii& mstream);
};

}; // end namespace