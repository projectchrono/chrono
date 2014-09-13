%{

/* Includes the header in the wrapper code */
#include "core/ChQuaternion.h"

%}
 
/* Parse the header file to generate wrappers */
/* %include "../core/ChQuaternion.h"  */


namespace chrono
{

template <class Real = double>
class ChQuaternion 
{
public:

	Real e0;
	Real e1;
	Real e2;
	Real e3;
	
	ChQuaternion();
	ChQuaternion(const Real ne0, const Real ne1, const Real ne2,const Real ne3);
	ChQuaternion(const Real ns, const chrono::ChVector<Real> nv);
	ChQuaternion(const ChQuaternion<Real>& other);
	// ChQuaternion<Real>& operator=(const ChQuaternion<Real>& other);
	ChQuaternion<Real>  operator-() const;
	// ChQuaternion<Real>  operator!() const;
	ChQuaternion<Real>  operator+(const ChQuaternion<Real>& other);
	ChQuaternion<Real>& operator+=(const ChQuaternion<Real>& other);
	ChQuaternion<Real>  operator-(const ChQuaternion<Real>& other) const;
	ChQuaternion<Real>& operator-=(const ChQuaternion<Real>& other);
	ChQuaternion<Real>  operator*(const ChQuaternion<Real>& other) const;
	ChQuaternion<Real>& operator*=(const ChQuaternion<Real>& other);
	ChQuaternion<Real>  operator>>(const ChQuaternion<Real>& other) const;
	ChQuaternion<Real>& operator>>=(const ChQuaternion<Real>& other);
	ChQuaternion<Real>  operator*(const Real v) const;
	ChQuaternion<Real>& operator*=(const Real v);
	ChQuaternion<Real>  operator/(const ChQuaternion<Real>& other) const;
	ChQuaternion<Real>& operator/=(const ChQuaternion<Real>& other);
	ChQuaternion<Real>  operator/(const Real v) const;
	ChQuaternion<Real>& operator/=(const Real v);
	ChQuaternion<Real> operator%(const ChQuaternion<Real>& other) const;
	ChQuaternion<Real>& operator%=(const ChQuaternion<Real>& other);
	// double operator^(const ChQuaternion<Real>& other) const;
	bool operator<=(const ChQuaternion<Real>&other) const;
	bool operator>=(const ChQuaternion<Real>&other) const;
	bool operator==(const ChQuaternion<Real>& other) const;
	bool operator!=(const ChQuaternion<Real>& other) const;
	void Set(const Real ne0, const Real ne1, const Real ne2, const Real ne3);
	void Set(const ChQuaternion<Real>& n);
	void Set(const Real p);
	void SetNull();
	void SetUnit();
	void SetScalar(const Real s);
	void SetVector(const chrono::ChVector<Real>& mv);
	chrono::ChVector<Real> GetVector();
	bool	Equals ( const ChQuaternion<Real>& other) const;
	bool	Equals ( const ChQuaternion<Real>& other, Real tol) const;
	void 	Add    ( const ChQuaternion<Real> A, const ChQuaternion<Real> B);
	void 	Sub	   ( const ChQuaternion<Real> A, const ChQuaternion<Real> B);
	void	Cross  ( const ChQuaternion<Real> qa, const ChQuaternion<Real> qb);
	double  Dot   ( const ChQuaternion<Real> A, const ChQuaternion<Real> B);
	void 	Mul	   ( const ChQuaternion<Real> A, const Real v);
	void    Scale  ( const Real v);
	double	Length	 ();
	double	Length2	 ();
	double	LengthInf ();
	bool    Normalize ();
	ChQuaternion<Real> GetNormalized();
	void	Conjugate (const ChQuaternion<Real>& A);
	void	Conjugate ();
	ChQuaternion<Real> 	GetConjugate () const;
	chrono::ChVector<Real> Rotate(const chrono::ChVector<Real> A) const;
	chrono::ChVector<Real> RotateBack(const chrono::ChVector<Real> A) const;
	void  Q_from_AngAxis(const double angle, const chrono::ChVector<Real> axis);
	void  Q_from_AngX (const double angleX);
	void  Q_from_AngY (const double angleY);
	void  Q_from_AngZ (const double angleZ);
	void  Q_to_AngAxis (double& a_angle, chrono::ChVector<Real>& a_axis);
	void  Q_from_NasaAngles(const chrono::ChVector<Real> mang);
	chrono::ChVector<Real>  Q_to_NasaAngles();
	void Qdt_from_Wabs (const chrono::ChVector<Real> w, 
						const ChQuaternion<Real> q);
	void Qdt_from_Wrel (const chrono::ChVector<Real> w, 
						const ChQuaternion<Real> q);
	void Qdt_to_Wabs (  chrono::ChVector<Real> w, 
						const ChQuaternion<Real> q);
	void Qdt_to_Wrel (  chrono::ChVector<Real> w, 
						const ChQuaternion<Real> q);
	void Qdtdt_from_Aabs (	const chrono::ChVector<Real>     a, 
							const ChQuaternion<Real> q, 
							const ChQuaternion<Real> q_dt);
	void Qdtdt_from_Arel      ( const chrono::ChVector<Real>	 a, 
								const ChQuaternion<Real> q, 
								const ChQuaternion<Real> q_dt);
	void Qdt_from_AngAxis (		const ChQuaternion<Real>  q, 
								const double              angle_dt, 
								const chrono::ChVector<Real>      axis);
	void Qdtdt_from_AngAxis (	const ChQuaternion<Real>  q, 
								const ChQuaternion<Real>  q_dt,
								const double              angle_dtdt, 
								const chrono::ChVector<Real>      axis);
	void ImmQ_complete (const chrono::ChVector<Real>& qimm);
	void ImmQ_dt_complete (const ChQuaternion<Real>& mq, const chrono::ChVector<Real>& qimm_dt);
	void ImmQ_dtdt_complete (const ChQuaternion<Real>& mq, const ChQuaternion<Real>& mqdt, const chrono::ChVector<Real>& qimm_dtdt);
	chrono::ChVector<Real> GetXaxis() const;
	chrono::ChVector<Real> GetYaxis() const;
	chrono::ChVector<Real> GetZaxis() const;
	void StreamOUT(chrono::ChStreamOutAscii& mstream);
	void StreamOUT(chrono::ChStreamOutBinary& mstream);
	void StreamIN(chrono::ChStreamInBinary& mstream);
};

typedef ChQuaternion<double> Quaternion;
typedef ChQuaternion<float>  QuaternionF;

double		Qlength (const Quaternion& q);
Quaternion	Qadd (const Quaternion& qa, const Quaternion& qb);
Quaternion	Qsub (const Quaternion& qa, const Quaternion& qb);
Quaternion	Qscale (const Quaternion& q, double fact);
Quaternion	Qnorm (const Quaternion& q);
Quaternion	Q_from_AngAxis (double angle, const chrono::Vector& axis);
Quaternion	Q_from_NasaAngles(const chrono::Vector& RxRyRz);
chrono::Vector Q_to_NasaAngles(const Quaternion& mq);
Quaternion	Q_from_AngZ (double angleZ);
Quaternion	Q_from_AngX (double angleX);
Quaternion	Q_from_AngY (double angleY);
void			Q_to_AngAxis (Quaternion* quat, double* a_angle, chrono::Vector* a_axis);
Quaternion	Qdt_from_Wrel (const chrono::Vector& w, const Quaternion& q);
Quaternion	Qdt_from_Wabs (const chrono::Vector& w, const Quaternion& q);
Quaternion	Qdt_from_AngAxis (const Quaternion& quat, double angle_dt, const chrono::Vector& axis);
Quaternion	Qdtdt_from_Aabs (chrono::Vector a, Quaternion q, Quaternion q_dt);
Quaternion	Qdtdt_from_Arel (chrono::Vector a, Quaternion q, Quaternion q_dt);
Quaternion	Qdtdt_from_AngAxis (double angle_dtdt, const chrono::Vector& axis, const Quaternion& q, const Quaternion& q_dt);
Quaternion	Qconjugate (const Quaternion& q);
Quaternion	Qcross (const Quaternion& qa, const Quaternion& qb);
bool		Qequal (const Quaternion& qa, const Quaternion& qb);
bool		Qnotnull (const Quaternion& qa);
Quaternion	ImmQ_complete (chrono::Vector* qimm);
Quaternion	ImmQ_dt_complete (Quaternion* mq, chrono::Vector* qimm_dt);
Quaternion	ImmQ_dtdt_complete (Quaternion* mq, Quaternion* mqdt, chrono::Vector* qimm_dtdt);
chrono::Vector	 VaxisXfromQuat (const Quaternion& quat);


#define ANGLESET_ANGLE_AXIS		0
#define ANGLESET_EULERO			1
#define ANGLESET_CARDANO		2
#define ANGLESET_HPB			3
#define ANGLESET_RXYZ			4
#define ANGLESET_RODRIGUEZ		5
#define ANGLESET_QUATERNION		6

chrono::Vector     Quat_to_Angle (int angset, const chrono::Quaternion& mquat);
chrono::Vector	 Angle_to_Angle(int setfrom, int setto, const chrono::Vector& mangles);
Quaternion Angle_to_Quat (int angset, const chrono::Vector& mangles);
Quaternion AngleDT_to_QuatDT (int angset, const chrono::Vector& mangles, const chrono::Quaternion& q);
Quaternion AngleDTDT_to_QuatDTDT (int angset, const chrono::Vector& mangles, const chrono::Quaternion& q);

};


%template(ChQuaternionD) chrono::ChQuaternion<double>; 
//%template(ChQuaternionF) chrono::ChQuaternion<float>; 

// Constants seem not to work...
//%constant chrono::ChQuaternion<double> QNULL = chrono::ChQuaternion<double>(0.,0.,0.,0.);
//%constant chrono::ChQuaternion<double> QUNIT = chrono::ChQuaternion<double>(1.,0.,0.,0.);


%extend chrono::ChQuaternion<double>{
		public:
					// Add function to support python 'print(...)'
			char *__str__() 
					{
						static char temp[256];
						sprintf(temp,"[ %g, %g, %g, %g ]", $self->e0,$self->e1,$self->e2,$self->e3);
						return &temp[0];
					}
					// operator  ~  as ! in c++ 
			ChQuaternion<double> __invert__() const  
					{
						return $self->operator!();
					}
					// operator  ^  as ^ in c++ 
			double __xor__(const ChQuaternion<double>& other) const 
					{ 
						return $self->operator^(other);
					}
		};


// This because constants do not work well, so implement them in script-side

%pythoncode %{

	QNULL  = ChQuaternionD(0,0,0,0)
	QUNIT  = ChQuaternionD(1,0,0,0)

%}