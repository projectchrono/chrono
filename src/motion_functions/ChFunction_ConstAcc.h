#ifndef CHFUNCT_CONSTACC_H
#define CHFUNCT_CONSTACC_H

//////////////////////////////////////////////////
//  
//   ChFunction_ConstAcc.h
//
//   Function objects, 
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Base.h"


namespace chrono 
{

#define FUNCT_CONSTACC	8



/// CONSTANT ACCELERATION FUNCTION:  
/// h = height, amount of displacement
/// end = duration of motion,
/// av  = fraction of 1st acceleration end  (0..1)  
/// aw  = fraction of 2nd acceleration start (0..1) , with aw>av;

class ChApi ChFunction_ConstAcc : public ChFunction
{
	CH_RTTI(ChFunction_ConstAcc, ChFunction);
private:
	double h;
	double av;
	double aw;
	double end;
public:
	ChFunction_ConstAcc () {h =1; av=0.5; aw=0.5;  end=1;}
	ChFunction_ConstAcc (double m_h, double m_av, double m_aw, double m_end)  {h = m_h; Set_end(m_end); Set_avw(m_av,m_aw);};
	~ChFunction_ConstAcc () {};
	void Copy (ChFunction_ConstAcc* source);
	ChFunction* new_Duplicate ();

	void Set_end  (double m_end)  {if (m_end<0) m_end = 0; end = m_end;}
	void Set_av   (double m_av)   {if (m_av<0) m_av = 0; if (m_av>1) m_av = 1; av = m_av; if (av>aw) av = aw;}
	void Set_aw   (double m_aw)   {if (m_aw<0) m_aw = 0; if (m_aw>1) m_aw = 1; aw = m_aw; if (aw<av) aw = av;}
	void Set_h    (double m_h)    {h = m_h;}
	void Set_avw  (double m_av, double m_aw)	{av=0; aw=1; Set_av(m_av); Set_aw(m_aw);}

	double Get_end () {return end;}
	double Get_av ()  {return av;}
	double Get_aw ()  {return aw;}
	double Get_h ()   {return h;}

	double Get_y      (double x) ;
	double Get_y_dx   (double x) ;
	double Get_y_dxdx (double x) ;

	double Get_Ca_pos ();
	double Get_Ca_neg ();
	double Get_Cv ();

	void Extimate_x_range (double& xmin, double& xmax) {xmin = 0.0; xmax = end;};

	int Get_Type () {return (FUNCT_CONSTACC);}

	OPT_VARIABLES_START
		"h",
		"end",
		"aw",
		"av",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



} // END_OF_NAMESPACE____


#endif
