%{

/* Includes the header in the wrapper code */
#include "core/ChCoordsys.h"

%}
 
/* Parse the header file to generate wrappers */
/* %include "../core/ChCoordsys.h"  */


namespace chrono
{

template <class Real = double>
class ChCoordsys 
{
public:
	chrono::ChVector<Real>		pos;
	chrono::ChQuaternion<Real>  rot;
	
	ChCoordsys();
	ChCoordsys(const chrono::ChVector<Real> mv, const chrono::ChQuaternion<Real> mq = QUNIT);
	ChCoordsys(const chrono::ChCoordsys<Real>& other);
	// ChCoordsys<Real>& operator=(const chrono::ChCoordsys<Real>& other);
	bool operator<=(const ChCoordsys<Real>&other) const;
	bool operator>=(const ChCoordsys<Real>&other) const;
	bool operator==(const ChCoordsys<Real>& other) const;
	bool operator!=(const ChCoordsys<Real>& other) const;
	void Force2D ();
	bool	Equals ( const ChCoordsys<Real>& other) const;
	bool	Equals ( const ChCoordsys<Real>& other, Real tol) const;
	void    SetIdentity();
	chrono::ChVector<Real> TrasformParentToLocal (
								const chrono::ChVector<Real>& parent		///< point to transform, given in parent coordinates
								) const;
	chrono::ChVector<Real> TrasformLocalToParent (
								const chrono::ChVector<Real>& local			///< point to transform, given in local coordinates
								) const;
	void StreamOUT(chrono::ChStreamOutAscii& mstream);
	void StreamOUT(chrono::ChStreamOutBinary& mstream);
	void StreamIN(chrono::ChStreamInBinary& mstream);
};
}; // end namespace


%template(ChCoordsysD) chrono::ChCoordsys<double>; 
%template(ChCoordsysF) chrono::ChCoordsys<float>; 


%constant chrono::ChCoordsys<double> CSYSNULL = chrono::ChCoordsys<double>(chrono::VNULL,chrono::QNULL);
%constant chrono::ChCoordsys<double> CSYSNORM = chrono::ChCoordsys<double>(chrono::VNULL,chrono::QUNIT);
