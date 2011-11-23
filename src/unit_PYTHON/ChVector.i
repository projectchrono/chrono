/* %module ChronoEngine_PYTHON_mod  */

%{

/* Includes the header in the wrapper code */
#include "core/ChVector.h"

%}
 
/* Parse the header file to generate wrappers */
/* %include "../core/ChVector.h"  */

namespace chrono
{

template <class Real = double>
class ChVector
{
public:
	Real x;
	Real y;
	Real z;
	ChVector(): x(0), y(0), z(0);
	ChVector(const Real nx, const Real ny, const Real nz);
	ChVector(const ChVector<Real>& other)	:x(other.x), y(other.y), z(other.z);
	template <class RealB>
	ChVector(const ChVector<RealB>& other)	:x((Real)other.x), y((Real)other.y), z((Real)other.z);
	ChVector<Real>& operator=(const ChVector<Real>& other)	{if (&other == this) return *this;  x = other.x; y = other.y; z = other.z; return *this; }
	template <class RealB>
	ChVector<Real>& operator=(const ChVector<RealB>& other)	{ x = (Real)other.x; y = (Real)other.y; z = (Real)other.z; return *this; }
	ChVector<Real> operator-() const { return ChVector<Real>(-x, -y, -z);   }
	inline Real& operator()(const int el);
	inline const Real& operator()(const int el) const;
	ChVector<Real> operator+(const ChVector<Real>& other) const;
	ChVector<Real>& operator+=(const ChVector<Real>& other);
	ChVector<Real> operator-(const ChVector<Real>& other) const;
	ChVector<Real>& operator-=(const ChVector<Real>& other);
	ChVector<Real> operator*(const ChVector<Real>& other) const;
	ChVector<Real>& operator*=(const ChVector<Real>& other);
	ChVector<Real> operator*(const Real v) const;
	ChVector<Real>& operator*=(const Real v);
	ChVector<Real> operator/(const ChVector<Real>& other) const;
	ChVector<Real>& operator/=(const ChVector<Real>& other);
	ChVector<Real> operator/(const Real v) const;
	ChVector<Real>& operator/=(const Real v);
	bool operator<=(const ChVector<Real>&other) const;
	bool operator>=(const ChVector<Real>&other) const;
	bool operator<(const ChVector<Real>&other) const;
	bool operator>(const ChVector<Real>&other) const;
	bool operator==(const ChVector<Real>& other) const;
	bool operator!=(const ChVector<Real>& other) const;
	void Set(const Real nx, const Real ny, const Real nz);
	void Set(const ChVector<Real>& p);
	void Set(const Real p);
	void SetNull();
	bool	Equals ( const ChVector<Real>& other) const;
	bool	Equals ( const ChVector<Real>& other, Real tol) const;
	void 	Add    ( const ChVector<Real> A, const ChVector<Real> B);
	void 	Sub	   ( const ChVector<Real> A, const ChVector<Real> B);
	void	Cross  ( const ChVector<Real> A, const ChVector<Real> B);
	double  Dot   ( const ChVector<Real> A, const ChVector<Real> B);
	double  Dot   ( const ChVector<Real> B);
	void 	Mul	   ( const ChVector<Real> A, const Real v);
	void    Scale (const Real v);
	double	Length	 ();
	double	Length2	 ();
	double	LengthInf ();
	bool    Normalize ();
	void    SetLength (Real v);
	void StreamOUT(chrono::ChStreamOutAscii& mstream);
	void StreamOUT(chrono::ChStreamOutBinary& mstream);
	void StreamIN(chrono::ChStreamInBinary& mstream);
};

};

%template(ChVectorD) chrono::ChVector<double>; 
%template(ChVectorF) chrono::ChVector<float>; 
