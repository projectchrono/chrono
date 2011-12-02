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
	ChVector(const ChVector<Real>& other);
	template <class RealB>
	ChVector(const ChVector<RealB>& other);
	// ChVector<Real>& operator=(const ChVector<Real>& other);
	template <class RealB>
	// ChVector<Real>& operator=(const ChVector<RealB>& other);
	ChVector<Real> operator-();
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
	ChVector<Real> operator%(const ChVector<Real>& other);
	ChVector<Real>& operator%=(const ChVector<Real>& other);
	//double operator^(const ChVector<Real>& other);
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
	ChVector<Real> GetNormalized();
	void    SetLength (Real v);
	void StreamOUT(chrono::ChStreamOutAscii& mstream);
	void StreamOUT(chrono::ChStreamOutBinary& mstream);
	void StreamIN(chrono::ChStreamInBinary& mstream);
};

};

%template(ChVectorD) chrono::ChVector<double>; 
%template(ChVectorF) chrono::ChVector<float>; 

%constant chrono::ChVector<double> VNULL = chrono::ChVector<double>(0,0,0);
%constant chrono::ChVector<double> VECT_X= chrono::ChVector<double>(1,0,0);
%constant chrono::ChVector<double> VECT_Y= chrono::ChVector<double>(0,1,0);
%constant chrono::ChVector<double> VECT_Z= chrono::ChVector<double>(0,0,1);


%extend chrono::ChVector<double>{
		public:
					// Add function to support python 'print(...)'
			char *__str__() 
					{
						static char temp[256];
						sprintf(temp,"[ %g, %g, %g ]", $self->x,$self->y,$self->z);
						return &temp[0];
					}
					// operator  ^  as ^ in c++ 
			double __xor__(const ChVector<double>& other) const 
					{ 
						return $self->operator^(other);
					}
		};
