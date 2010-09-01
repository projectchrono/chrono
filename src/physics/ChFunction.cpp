///////////////////////////////////////////////////
//
//   ChFunction.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>


#include "physics/ChFunction.h"
#include "physics/ChGlobal.h"
#include "core/ChLinearAlgebra.h"


namespace chrono
{


//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction> a_registration;

void ChFunction::Copy (ChFunction* source)
{
	Set_yconst (source->y);
}

ChFunction* ChFunction::new_Duplicate ()
{
	ChFunction* m_func;
	m_func = new ChFunction;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction::Get_y_dN (double x, int derivate)
{
	switch (derivate)
	{
	case 0:
		return Get_y(x);
	case 1:
		return Get_y_dx(x);
	case 2:
		return Get_y_dxdx(x);
	default:
		return Get_y(x);
	}
}

void ChFunction::Extimate_y_range (double xmin, double xmax, double& ymin, double& ymax, int derivate)
{
	ymin= 10000;
	ymax=-10000;
	for (double mx = xmin; mx < xmax; mx+= (xmax-xmin)/100.0)
	{
		if (Get_y_dN(mx, derivate) < ymin) ymin = Get_y_dN(mx, derivate);
		if (Get_y_dN(mx, derivate) > ymax) ymax = Get_y_dN(mx, derivate);
	}
	if (fabs(ymax - ymin) <10e-12) {ymin= -0.5; ymax= +1.0;}
	ymax += 0.12 * (ymax-ymin);
	ymin -= 0.12 * (ymax-ymin);
}

// some analysis functions
double ChFunction::Compute_max(double xmin, double xmax, double sampling_step, int derivate)
{
	double mret = -1E30;
	for (double mx = xmin; mx<= xmax; mx +=sampling_step)
	{
		if (this->Get_y_dN(mx, derivate) > mret)
			mret = this->Get_y_dN(mx, derivate);
	}
	return mret;
}

double ChFunction::Compute_min(double xmin, double xmax, double sampling_step, int derivate)
{
	double mret = +1E30;
	for (double mx = xmin; mx<= xmax; mx += sampling_step)
	{
		if (this->Get_y_dN(mx, derivate) < mret)
			mret = this->Get_y_dN(mx, derivate);
	}
	return mret;
}

double ChFunction::Compute_mean(double xmin, double xmax, double sampling_step, int derivate)
{
	double mret = 0;
	int numpts = 0;
	for (double mx = xmin; mx<= xmax; mx = mx+sampling_step)
	{
		numpts++;
		mret += this->Get_y_dN(mx, derivate);
	}
	return mret/((double)numpts);

}

double ChFunction::Compute_sqrmean(double xmin, double xmax, double sampling_step, int derivate)
{
	double mret = 0;
	int numpts = 0;
	for (double mx = xmin; mx<= xmax; mx = mx+sampling_step)
	{
		numpts++;
		mret += pow (this->Get_y_dN(mx, derivate) , 2.);
	}
	return sqrt (mret/((double)numpts));
}

double ChFunction::Compute_int(double xmin, double xmax, double sampling_step, int derivate)
{
	double mret = 0;
	double ya = this->Get_y_dN(xmin, derivate);
	double yb = 0;
	for (double mx = xmin+sampling_step; mx<= xmax; mx += sampling_step)
	{
		yb = this->Get_y_dN(mx, derivate);
		mret += sampling_step*(ya+yb)*0.5; // trapezoidal quadrature
		ya = yb;
	}
	return mret;
}


////////////


int ChFunction::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	char** mvars;
	int i=0;
	mvars= this->GetOptVariables();
	while (*(mvars+i)!=0)
	{
		chjs_propdata* mdata = new chjs_propdata;
		strcpy(mdata->propname, *(mvars+i));
		strcpy(mdata->label,    *(mvars+i));
		mdata->haschildren = FALSE;
		mtree->AddTail(mdata);
		i++;
	}

	// now dirty trick, because of 'C' variable is inherited by all functions,
	// but used by plain base class only..
	if (this->Get_Type()==FUNCT_CONST)
	{
		chjs_propdata* mdata = new chjs_propdata;
		strcpy(mdata->propname, "C");
		strcpy(mdata->label,    mdata->propname);
		mdata->haschildren = FALSE;
		mtree->AddTail(mdata);
		i++;
	}
	return i;
}


static int _recurse_VariableTreeToFullNameVar(ChList<chjs_propdata>* mtree, ChList<chjs_fullnamevar>* mlist, char* maccumulator)
{
	int i= 0;

	int mentrypos = strlen (maccumulator);

	ChNode<chjs_propdata>* mnode = mtree->GetHead();
	while(mnode)
	{
		if (strlen(maccumulator)+strlen(mnode->data->propname) < 120-1)
		{
			strcat(maccumulator, mnode->data->label);

			if (mnode->data->children.Count())
			{
				strcat(maccumulator, ".");
				_recurse_VariableTreeToFullNameVar(&mnode->data->children, mlist, maccumulator);
			}
			else
			{
				chjs_fullnamevar* mfullname = new chjs_fullnamevar;
				strcpy(mfullname->propname, maccumulator);
				strcpy(mfullname->label, maccumulator);
				mfullname->active= TRUE;
				mfullname->script=NULL;
				mlist->AddTail(mfullname);
				i++;
			}

			maccumulator[mentrypos]=0;
		}

		mnode = mnode->next;
	}
	return i;
}

int ChFunction::VariableTreeToFullNameVar(ChList<chjs_propdata>* mtree, ChList<chjs_fullnamevar>* mlist)
{
	char accumulator[120];
	strcpy(accumulator, "context().");

	int i= _recurse_VariableTreeToFullNameVar(mtree, mlist, accumulator);

	return i;
}


int ChFunction::OptVariableCount()
{
	ChList<chjs_propdata> mtree;
	ChList<chjs_fullnamevar> mlist;
	MakeOptVariableTree(&mtree);
	VariableTreeToFullNameVar(&mtree, &mlist);
	return mlist.Count();
};




////////////

void ChFunction::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// stream out all member data
	mstream << this->y;
}

void ChFunction::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in all member data
	mstream >> this->y;
}

void ChFunction::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_CONST  \n";

	//***TO DO***
}





int ChFunction::FilePostscriptPlot(ChFile_ps* m_file, int plotY, int plotDY, int plotDDY)
{
	int mresol = 800;
	ChMatrixDynamic<> yf(mresol,1);
	ChMatrixDynamic<> xf(mresol,1);
	double mx, xmin, xmax;
	ChPageVect mp;
		// xmin
	mp = m_file->Get_G_p();
	mp = m_file->To_graph_from_page(mp);
	xmin = mp.x;
		// xmax
	mp = m_file->Get_G_p();
	mp.x = mp.x + m_file->Get_Gs_p().x;
	mp = m_file->To_graph_from_page(mp);
	xmax = mp.x;

	if (plotY)
	{
		mx = xmin;
		for (int j = 0; j<mresol; j++)
		{
			mp.x = mx;
			mp.y = this->Get_y(mx);
			xf.SetElement(j,0,	mp.x);
			yf.SetElement(j,0,	mp.y);
			mx += ((xmax-xmin)/((double)mresol-1.0));
		}
		m_file->DrawGraphXY(&yf, &xf);
	}
	if (plotDY)
	{
		mx = xmin;
		for (int j = 0; j<mresol; j++)
		{
			mp.x = mx;
			mp.y = this->Get_y_dx(mx);
			xf.SetElement(j,0,	mp.x);
			yf.SetElement(j,0,	mp.y);
			mx += ((xmax-xmin)/((double)mresol-1.0));
		}
		m_file->DrawGraphXY(&yf, &xf);
	}
	if (plotDDY)
	{
		mx = xmin;
		for (int j = 0; j<mresol; j++)
		{
			mp.x = mx;
			mp.y = this->Get_y_dxdx(mx);
			xf.SetElement(j,0,	mp.x);
			yf.SetElement(j,0,	mp.y);
			mx += ((xmax-xmin)/((double)mresol-1.0));
		}
		m_file->DrawGraphXY(&yf, &xf);
	}
	return 1;
}


int ChFunction::FileAsciiPairsSave(ChStreamOutAscii& m_file, double mxmin, double mxmax, int msamples)
{
	if (msamples<=1) throw (ChException("Warning! too short range or too long sampling period: no points can be saved"));
	if (msamples>=100000) throw (ChException("Warning! Too many points should be saved"));
	if (mxmax<=mxmin) throw (ChException("Warning! Cannot save ChFunction if Xmax < Xmin"));

	m_file.SetNumFormat("%0.8f");
	
	double period = (mxmax-mxmin)/((double)msamples-1);
	
	double mX = mxmin;
	for (int cnt = 1; cnt <= msamples ; cnt++)
	{
		m_file << mX;
		m_file << "    ";
		m_file << this->Get_y(mX);
		m_file.CR();
		mX += period;
	}
	return 1;
}



//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Ramp


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Ramp> a_registration_ramp;

void ChFunction_Ramp::Copy (ChFunction_Ramp* source)
{
	Set_y0  (source->y0);
	Set_ang (source->ang);
}

ChFunction* ChFunction_Ramp::new_Duplicate ()
{
	ChFunction_Ramp* m_func;
	m_func = new ChFunction_Ramp;
	m_func->Copy(this);
	return (m_func);
}


void ChFunction_Ramp::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << y0;
	mstream << ang;
}

void ChFunction_Ramp::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> y0;
	mstream >> ang;
}

void ChFunction_Ramp::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_RAMP  \n";

	//***TO DO***
}





//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Sine

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Sine> a_registration_sine;


void ChFunction_Sine::Copy (ChFunction_Sine* source)
{
	Set_phase (source->phase);
	Set_freq(source->freq);
	Set_amp (source->amp);
}

ChFunction* ChFunction_Sine::new_Duplicate ()
{
	ChFunction_Sine* m_func;
	m_func = new ChFunction_Sine;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Sine::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << phase;
	mstream << freq;
	mstream << amp;
}

void ChFunction_Sine::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	double dfoo;
	mstream >> dfoo; Set_phase(dfoo);
	mstream >> dfoo; Set_freq(dfoo);
	mstream >> dfoo; Set_amp(dfoo);
}

void ChFunction_Sine::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_SINE  \n";

	//***TO DO***
}




//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Sigma

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Sigma> a_registration_sigma;

void ChFunction_Sigma::Copy (ChFunction_Sigma* source)
{
	Set_start (source->start);
	Set_end (source->end);
	Set_amp (source->amp);
}

ChFunction* ChFunction_Sigma::new_Duplicate ()
{
	ChFunction_Sigma* m_func;
	m_func = new ChFunction_Sigma;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Sigma::Extimate_x_range (double& xmin, double& xmax)
{
	double mdx = end-start;
	xmin = start +mdx*0.1;
	xmax = end -mdx*0.1;
}

double ChFunction_Sigma::Get_y      (double x)
{
	double ret;
	double A = (end - start);
	if (x < start) return 0;
	if (x > end) return amp;
	else
	{
		ret = amp *( (3*(pow(((x-start)/A),2))) - 2*(pow(((x-start)/A),3)) );
	}
	return ret;
}

double ChFunction_Sigma::Get_y_dx   (double x)
{
	double ret;
	double A = (end - start);
	if ((x < start) || (x > end)) ret = 0;
	else
	{
		ret = amp * ( 6*((x-start) / pow(A,2)) - 6*(pow((x-start),2)/pow(A,3))  );
	}
	return ret;
}

double ChFunction_Sigma::Get_y_dxdx (double x)
{
	double ret;
	double A = (end - start);
	if ((x < start) || (x > end)) ret = 0;
	else
	{
		ret = amp * ( 6*(1/pow(A,2)) - 12*((x-start)/pow(A,3))  );
	}
	return ret;
}

void ChFunction_Sigma::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << start;
	mstream << end;
	mstream << amp;
}

void ChFunction_Sigma::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> start;
	mstream >> end;
	mstream >> amp;
}

void ChFunction_Sigma::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_CONST  \n";

	//***TO DO***
}



//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_ConstAcc

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_ConstAcc> a_registration_constacc;


void ChFunction_ConstAcc::Copy (ChFunction_ConstAcc* source)
{
	h = source->h;
	av = source->av;
	aw = source->aw;
	end = source->end;
}

ChFunction* ChFunction_ConstAcc::new_Duplicate ()
{
	ChFunction_ConstAcc* m_func;
	m_func = new ChFunction_ConstAcc;
	m_func->Copy(this);
	return (m_func);
}


double ChFunction_ConstAcc::Get_y      (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return h;
	double ev = av*end;
	double ew = aw*end;
	double A = 2*h/((ev)*(end-ev+ew));
	double B = 2*h/((end-ew)*(end-ev+ew));
	if ((x>0)&&(x<ev))
	{
		ret = 0.5*A*x*x;
	}
	if ((x>=ev)&&(x<=ew))
	{
		ret = A*ev*(x-ev*0.5);
	}
	if ((x>ew)&&(x<end))
	{
		ret = A*ev*(x-ev*0.5) - B*0.5*pow((x-ew),2);
	}
	return ret;
}

double ChFunction_ConstAcc::Get_y_dx   (double x)
{
	double ret = 0;
	double ev = av*end;
	double ew = aw*end;
	double A = 2*h/((ev)*(end-ev+ew));
	double B = 2*h/((end-ew)*(end-ev+ew));
	if ((x>0)&&(x<ev))
	{
		ret = A*x;
	}
	if ((x>=ev)&&(x<=ew))
	{
		ret = A*ev;
	}
	if ((x>ew)&&(x<end))
	{
		ret = A*ev - B*(x-ew);
	}
	return ret;
}

double ChFunction_ConstAcc::Get_y_dxdx (double x)
{
	double ret = 0;
	double ev = av*end;
	double ew = aw*end;
	double A = 2*h/((ev)*(end-ev+ew));
	double B = 2*h/((end-ew)*(end-ev+ew));
	if ((x>0)&&(x<ev))
	{
		ret = A;
	}
	if ((x>=ev)&&(x<=ew))
	{
		ret = 0;
	}
	if ((x>ew)&&(x<end))
	{
		ret = -B;
	}
	return ret;
}

double ChFunction_ConstAcc::Get_Ca_pos ()
{
	return 2*(end*end)/(av*end*(end - av*end + aw*end));
}
double ChFunction_ConstAcc::Get_Ca_neg ()
{
	return 2*(end*end)/((end - aw*end)*(end - av*end + aw*end));
}
double ChFunction_ConstAcc::Get_Cv ()
{
	return 2*(end)/(end - av*end + aw*end);
}

void ChFunction_ConstAcc::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << h;
	mstream << aw;
	mstream << av;
	mstream << end;
}

void ChFunction_ConstAcc::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> h;
	mstream >> aw;
	mstream >> av;
	mstream >> end;
}

void ChFunction_ConstAcc::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_CONST  \n";

	//***TO DO***
}




//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Poly345

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Poly345> a_registration_poly345;


void ChFunction_Poly345::Copy (ChFunction_Poly345* source)
{
	h = source->h;
	end = source->end;
}

ChFunction* ChFunction_Poly345::new_Duplicate ()
{
	ChFunction_Poly345* m_func;
	m_func = new ChFunction_Poly345;
	m_func->Copy(this);
	return (m_func);
}


double ChFunction_Poly345::Get_y      (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return h;
	double a = x/end;
	ret = h* ( 10*pow(a,3) - 15*pow(a,4) + 6*pow(a,5) );
	return ret;
}

double ChFunction_Poly345::Get_y_dx   (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return 0;
	double a = x/end;
	ret = h*(1/end) * ( 30*pow(a,2) - 60*pow(a,3) + 30*pow(a,4) );
	return ret;
}

double ChFunction_Poly345::Get_y_dxdx (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return 0;
	double a = x/end;
	ret = h*(1/(end*end)) * ( 60*a - 180*pow(a,2) + 120*pow(a,3) );
	return ret;
}

void ChFunction_Poly345::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << h;
	mstream << end;
}

void ChFunction_Poly345::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> h;
	mstream >> end;
}

void ChFunction_Poly345::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_CONST  \n";

	//***TO DO***
}





//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Fillet3

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Fillet3> a_registration_fillet3;


void ChFunction_Fillet3::Copy (ChFunction_Fillet3* source)
{
	end = source->end;
	y1  = source->y1;
	y2 = source->y2;
	dy1 = source->dy1;
	dy2 = source->dy2;

	SetupCoefficients();
}

ChFunction* ChFunction_Fillet3::new_Duplicate ()
{
	ChFunction_Fillet3* m_func;
	m_func = new ChFunction_Fillet3;
	m_func->Copy(this);
	return (m_func);
}


double ChFunction_Fillet3::Get_y      (double x)
{
	double ret = 0;
	if (x<=0) return y1;
	if (x>=end) return y2;
	double a = x/end;
	ret = c1*pow(x,3) + c2*pow(x,2) + c3*x + c4;
 	return ret;
}

double ChFunction_Fillet3::Get_y_dx      (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return 0;
	double a = x/end;
	ret = 3*c1*pow(x,2) + 2*c2*x + c3;
 	return ret;
}

double ChFunction_Fillet3::Get_y_dxdx      (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return 0;
	double a = x/end;
	ret = 6*c1*x + 2*c2;
 	return ret;
}

int ChFunction_Fillet3::SetupCoefficients()
{
	ChMatrixDynamic<> ma(4,4);
	ChMatrixDynamic<> mb(4,1);
	ChMatrixDynamic<> mx(4,1);

	mb(0,0) = y1;
	mb(1,0) = y2;
	mb(2,0) = dy1;
	mb(3,0) = dy2;

	ma(0,3) = 1.0;

	ma(1,0) = pow(end,3);
	ma(1,1) = pow(end,2);
	ma(1,2) = end;
	ma(1,3) = 1.0;

	ma(2,2) = 1.0;

	ma(3,0) = 3*pow(end,2);
	ma(3,1) = 2*end;
	ma(3,2) = 1.0;

	ChLinearAlgebra::Solve_LinSys(ma, &mb, &mx);

	c1= mx(0,0);	c2= mx(1,0);	c3= mx(2,0);	c4= mx(3,0);

 	return TRUE;
}

void ChFunction_Fillet3::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << end;
	mstream << y1;
	mstream << y2;
	mstream << dy1;
	mstream << dy2;
}

void ChFunction_Fillet3::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> end;
	mstream >> y1;
	mstream >> y2;
	mstream >> dy1;
	mstream >> dy2;
	SetupCoefficients();
}

void ChFunction_Fillet3::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_FILLET3  \n";

	//***TO DO***
}






//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Mocap

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Mocap> a_registration_mocap;


ChFunction_Mocap::ChFunction_Mocap ()
{
	array_y = NULL;
	array_y_dt = NULL;
	array_y_dtdt = NULL;

	Set_samples (2);	// this creates arrays
	Set_samp_freq(50);
}

ChFunction_Mocap::ChFunction_Mocap (int m_samples, double freq)
{
	array_y = NULL;
	array_y_dt = NULL;
	array_y_dtdt = NULL;

	Set_samples (m_samples); 	// this creates arrays
	Set_samp_freq(freq);
}

ChFunction_Mocap::~ChFunction_Mocap ()
{
	if (array_y != NULL) delete array_y;
	if (array_y_dt != NULL) delete array_y_dt;
	if (array_y_dtdt != NULL) delete array_y_dtdt;
}

void ChFunction_Mocap::Copy (ChFunction_Mocap* source)
{
	Set_samples (source->samples);	// this creates arrays
	Set_samp_freq(source->samp_freq);

	if (source->Get_array_y()) array_y->CopyFromMatrix(*source->Get_array_y());
	if (source->Get_array_y_dt()) array_y_dt->CopyFromMatrix(*source->Get_array_y_dt());
	if (source->Get_array_y_dtdt()) array_y_dtdt->CopyFromMatrix(*source->Get_array_y_dtdt());
}

ChFunction* ChFunction_Mocap::new_Duplicate ()
{
	ChFunction_Mocap* m_func;
	m_func = new ChFunction_Mocap;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Mocap::Extimate_x_range (double& xmin, double& xmax)
{
	xmin = 0.0;
	xmax = Get_timetot();
}


void ChFunction_Mocap::Set_samp_freq (double m_fr)
{
	samp_freq = m_fr;
	timetot = ((double)samples / samp_freq);
}

void ChFunction_Mocap::Set_samples (int m_samples)
{
	samples = m_samples;

	if (samples<2)
		samples =2;

	timetot = ((double)samples / samp_freq);

	if (array_y != NULL) delete array_y;
	if (array_y_dt != NULL) delete array_y_dt;
	if (array_y_dtdt != NULL) delete array_y_dtdt;

	array_y = new ChMatrixDynamic<> (1, samples);
	array_y_dt = new ChMatrixDynamic<> (1, samples);
	array_y_dtdt = new ChMatrixDynamic<> (1, samples);
}



// compute all the y_dt basing on y, using the
// trapezioidal rule for numerical differentiation

void ChFunction_Mocap::Compute_array_dt  (ChMatrix<>* array_A, ChMatrix<>* array_A_dt)
{
	int i, ia, ib;
	double y_dt;

	for (i=0; i < samples; i++)
	{
		ia = i - 1;		// boundaries cases
		if (ia <= 0) { ia = 0;};
		ib = i + 1;
		if (ib >= samples) {ib = i;};
						// trapezioidal differentiation
		y_dt= ( (array_A->GetElement(0,ib))-
			    (array_A->GetElement(0,ia))  ) / Get_timeslice();

		array_A_dt->SetElement(0,i, y_dt);
	}
}


// Interpolation of the in-between values, given the discrete
// sample array (uniformly spaced points)

double ChFunction_Mocap::LinInterp (ChMatrix<>* m_array, double x, double x_max)
{
	double position;
	double weightA, weightB;
	int ia, ib;

	position =((double)samples * (x/x_max));

	ia = (int) floor(position);
	if (ia < 0) {ia = 0;};
	if (ia >=samples) {ia= samples-1;}

	ib = ia + 1;
	if (ib < 0) {ib = 0;};
	if (ib >= samples) {ib = samples-1;};

	weightB = position - (int) position;
	weightA = 1- weightB;

	return ( weightA*(m_array->GetElement(0,ia)) +
			 weightB*(m_array->GetElement(0,ib))   );
}


// Setup of arrays, provided as external vectors of
// samples. These functions automatically compute the
// derivatives (the arrays .._y_dt and y_dtdt)

void ChFunction_Mocap::Set_array_y	  (ChMatrix<>* m_array_y)
{
	array_y->CopyFromMatrix(*m_array_y);

	Compute_array_dt (array_y,    array_y_dt);
	Compute_array_dt (array_y_dt, array_y_dtdt);
}

void ChFunction_Mocap::Set_array_y_dt	  (ChMatrix<>* m_array_y_dt)
{
	// *** TO DO  ***
}

void ChFunction_Mocap::Set_array_y_dtdt   (ChMatrix<>* m_array_y_dtdt)
{
	// *** TO DO  ***
}


// Parsing of external files, to create mocap streams
// from the output of mocap devices

int  ChFunction_Mocap::Parse_array_AOA ()
{
	// *** TO DO **** //
	return TRUE;
}

int	 ChFunction_Mocap::Parse_array_Elite ()
{
	// *** TO DO **** //
	return TRUE;
}




// Return the value of the evaluated function, using
// linear interpolation to guess the in-between points,
// having the array samples as references.

double ChFunction_Mocap::Get_y      (double x)
{
	return LinInterp (array_y, x, timetot);
}

double ChFunction_Mocap::Get_y_dx   (double x)
{
	return LinInterp (array_y_dt, x, timetot);
}

double ChFunction_Mocap::Get_y_dxdx (double x)
{
	return LinInterp (array_y_dtdt, x, timetot);
}



// File parsing and dumping

void ChFunction_Mocap::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << this->Get_samples();
	mstream << this->Get_samp_freq();
	mstream << *this->array_y;
}

void ChFunction_Mocap::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	double dfoo;
	int ifoo;
	mstream >> ifoo;	Set_samples(ifoo);
	mstream >> dfoo;	Set_samp_freq(dfoo);
	mstream >> *this->array_y;
}

void ChFunction_Mocap::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_MOCAP  \n";

	//***TO DO***
}





//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Poly

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Poly> a_registration_poly;


ChFunction_Poly::ChFunction_Poly ()
{
	order = 0;
	for (int i=0; i< POLY_COEFF_ARRAY; i++)
		{coeff[i]=0 ;};
}

void ChFunction_Poly::Copy (ChFunction_Poly* source)
{
	order = source->order;
	for (int i = 0; i< POLY_COEFF_ARRAY; i++)
	{
		Set_coeff (source->Get_coeff(i), i);
	}
}

ChFunction* ChFunction_Poly::new_Duplicate ()
{
	ChFunction_Poly* m_func;
	m_func = new ChFunction_Poly;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Poly::Get_y      (double x)
{
	double total = 0;
	int i;
	for (i=0; i<=order; i++)
	{
		total += (coeff[i] * pow(x, (double)i));
	}
	return total;
}

double ChFunction_Poly::Get_y_dx   (double x)
{
	double total = 0;
	int i;

	if (order < 1) return 0; // constant function

	for (i=1; i<=order; i++)
	{
		total += ( (double)i * coeff[i] * pow(x, ((double)(i-1))) );
	}
	return total;
}

double ChFunction_Poly::Get_y_dxdx (double x)
{
	double total = 0;
	int i;

	if (order < 2) return 0; // constant function

	for (i=2; i<=order; i++)
	{
		total += ( (double)(i*(i-1)) * coeff[i] * pow(x, ((double)(i-2))) );
	}
	return total;
}

int ChFunction_Poly::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// add variables to own tree
	char msubvar[50];
	for (int mord=0; mord<this->order; mord++)
	{
		sprintf(msubvar,"C%d", mord);

		chjs_propdata* mdataA = new chjs_propdata;
		strcpy(mdataA->propname, msubvar);
		strcpy(mdataA->label,    mdataA->propname);
		mdataA->haschildren = FALSE;
		mtree->AddTail(mdataA);
		i++;
	}

	return i;
}

void ChFunction_Poly::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << this->order;
	for (int i= 0; i<= order; i++)
	{
		mstream << Get_coeff(i);
	}

}

void ChFunction_Poly::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	int ifoo;
	double dfoo;
	mstream >> ifoo;		this->Set_order(ifoo);
	for (int i= 0; i<= order; i++)
	{
		mstream >> dfoo;
		Set_coeff(dfoo,i);
	}
}

void ChFunction_Poly::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_POLY  \n";

	//***TO DO***
}






//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Operation

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Operation> a_registration_operation;


void ChFunction_Operation::Copy (ChFunction_Operation* source)
{
	op_type = source->op_type;
	fa = source->fa->new_Duplicate();
	fb = source->fb->new_Duplicate();
}

ChFunction* ChFunction_Operation::new_Duplicate ()
{
	ChFunction_Operation* m_func;
	m_func = new ChFunction_Operation;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Operation::Get_y      (double x)
{
	double res;

	switch (op_type)
	{
	case ChOP_ADD:
		res = fa->Get_y(x) + fb->Get_y(x); break;
	case ChOP_SUB:
		res = fa->Get_y(x) - fb->Get_y(x); break;
	case ChOP_MUL:
		res = fa->Get_y(x) * fb->Get_y(x); break;
	case ChOP_DIV:
		res = fa->Get_y(x) / fb->Get_y(x); break;
	case ChOP_POW:
		res = pow (fa->Get_y(x), fb->Get_y(x)); break;
	case ChOP_MAX:
		res = ChMax (fa->Get_y(x), fb->Get_y(x)); break;
	case ChOP_MIN:
		res = ChMin (fa->Get_y(x), fb->Get_y(x)); break;
	case ChOP_MODULO:
		res = fmod (fa->Get_y(x), fb->Get_y(x)); break;
	case ChOP_FABS :
		res = fabs (fa->Get_y(x)); break;
	case ChOP_FUNCT :
		res = fa->Get_y(fb->Get_y(x)); break;
	default:
		res = 0; break;
	}
	return res;
}
/*
double ChFunction_Operation::Get_y_dx   (double x)
{
	double res = 0;
	res = ChFunction::Get_y_dx(x); // default: numerical differentiation
	return res;
}

double ChFunction_Operation::Get_y_dxdx (double x)
{
	double res = 0;
	res = ChFunction::Get_y_dxdx(x); // default: numerical differentiation
	return res;
}
*/
void ChFunction_Operation::Extimate_x_range (double& xmin, double& xmax)
{
	double amin, amax, bmin, bmax;
	fa->Extimate_x_range(amin,amax);
	fb->Extimate_x_range(bmin,bmax);
	xmin = ChMin(amin, bmin);
	xmax = ChMax(amax, bmax);
}

int ChFunction_Operation::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for the two children..

	chjs_propdata* mdataA = new chjs_propdata;
	strcpy(mdataA->propname, "fa");
	strcpy(mdataA->label,    mdataA->propname);
	mdataA->haschildren = TRUE;
	mtree->AddTail(mdataA);

	i += this->fa->MakeOptVariableTree(&mdataA->children);

	chjs_propdata* mdataB = new chjs_propdata;
	strcpy(mdataB->propname, "fb");
	strcpy(mdataB->label,    mdataB->propname);
	mdataB->haschildren = TRUE;
	mtree->AddTail(mdataB);

	i += this->fb->MakeOptVariableTree(&mdataB->children);

	return i;
}

void ChFunction_Operation::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << op_type;
	mstream.AbstractWrite(fa);
	mstream.AbstractWrite(fb);
}

void ChFunction_Operation::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> op_type;

	if (fa) delete fa; fa=NULL;
	mstream.AbstractReadCreate(&fa);
	if (fb) delete fb; fb=NULL;
	mstream.AbstractReadCreate(&fb);
}

void ChFunction_Operation::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_OPERATION  \n";

	//***TO DO***
}





//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Derive

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Derive> a_registration_derive;


void ChFunction_Derive::Copy (ChFunction_Derive* source)
{
	order = source->order;
	fa = source->fa->new_Duplicate();
}

ChFunction* ChFunction_Derive::new_Duplicate ()
{
	ChFunction_Derive* m_func;
	m_func = new ChFunction_Derive;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Derive::Get_y      (double x)
{
	return fa->Get_y_dx(x);
}

void ChFunction_Derive::Extimate_x_range (double& xmin, double& xmax)
{
	fa->Extimate_x_range(xmin,xmax);
}


int ChFunction_Derive::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for children..

	chjs_propdata* mdataA = new chjs_propdata;
	strcpy(mdataA->propname, "fa");
	strcpy(mdataA->label,    mdataA->propname);
	mdataA->haschildren = TRUE;
	mtree->AddTail(mdataA);

	i += this->fa->MakeOptVariableTree(&mdataA->children);

	return i;
}

void ChFunction_Derive::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << order;
	mstream.AbstractWrite(fa);
}

void ChFunction_Derive::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> order;

	if (fa) delete fa; fa=NULL;
	mstream.AbstractReadCreate(&fa);
}

void ChFunction_Derive::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_DERIVE  \n";

	//***TO DO***
}




//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Integrate

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Integrate> a_registration_integrate;


ChFunction_Integrate::ChFunction_Integrate()
{
	order = 1;
	fa = new ChFunction;
	C_start= x_start=0;
	x_end=1;
	num_samples=2000;
	array_x = new ChMatrixDynamic<>(num_samples,1);
}

void ChFunction_Integrate::Copy (ChFunction_Integrate* source)
{
	order = source->order;
	fa = source->fa->new_Duplicate();
	C_start = source->C_start;
	x_start = source->x_start;
	x_end = source->x_end;
	num_samples= source->num_samples;
	array_x->CopyFromMatrix(*source->array_x);
}

ChFunction* ChFunction_Integrate::new_Duplicate ()
{
	ChFunction_Integrate* m_func;
	m_func = new ChFunction_Integrate;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Integrate::ComputeIntegral()
{
	double mstep= (x_end - x_start)/((double)(num_samples-1));
	double x_a , x_b, y_a, y_b, F_b;

	double F_sum = this->Get_C_start();

	this->array_x->SetElement(0,0, this->Get_C_start());

	for (int i=1; i<this->num_samples; i++)
	{
		x_b = x_start + ((double)i)*(mstep);
		x_a = x_b - mstep;
		y_a = this->fa->Get_y(x_a);
		y_b = this->fa->Get_y(x_b);
		 // trapezoidal rule..
		F_b = F_sum + mstep * (y_a + y_b )* 0.5;
		this->array_x->SetElement(i,0, F_b );
		F_sum = F_b;
	}
}

double ChFunction_Integrate::Get_y      (double x)
{
	if ((x<x_start)||(x>x_end))
		return 0.0;
	int i_a, i_b;
	double position = (double)(num_samples-1)*((x-x_start)/(x_end-x_start));
	i_a = (int)(floor(position));
	i_b = i_a+1;

	if (i_a==num_samples-1)
		return array_x->GetElement(num_samples-1,0);

	if ((i_a<0)||(i_b>=num_samples))
		return 0.0;

	double weightB = position - (double)i_a;
	double weightA = 1- weightB;

	return ( weightA*(array_x->GetElement(i_a,0)) +
			 weightB*(array_x->GetElement(i_b,0))   );
}

void ChFunction_Integrate::Extimate_x_range (double& xmin, double& xmax)
{
	xmin = x_start;
	xmax = x_end;
}

int ChFunction_Integrate::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for children..

	chjs_propdata* mdataA = new chjs_propdata;
	strcpy(mdataA->propname, "fa");
	strcpy(mdataA->label,    mdataA->propname);
	mdataA->haschildren = TRUE;
	mtree->AddTail(mdataA);

	i += this->fa->MakeOptVariableTree(&mdataA->children);

	return i;
}

void ChFunction_Integrate::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << Get_order();
	mstream << Get_C_start();
	mstream << Get_x_start();
	mstream << Get_x_end();
	mstream << Get_num_samples();
	mstream.AbstractWrite(fa);
}

void ChFunction_Integrate::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	int ifoo;
	double dfoo;
	mstream >> ifoo;			Set_order(ifoo);
	mstream >> dfoo;			Set_C_start(dfoo);
	mstream >> dfoo;			Set_x_start(dfoo);
	mstream >> dfoo;			Set_x_end(dfoo);
	mstream >> ifoo;			Set_num_samples(ifoo);
	if (fa) delete fa; fa=NULL;
	mstream.AbstractReadCreate(&fa);
}

void ChFunction_Integrate::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_INTEGRATE  \n";

	//***TO DO***
}





//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Mirror

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Mirror> a_registration_mirror;


void ChFunction_Mirror::Copy (ChFunction_Mirror* source)
{
	mirror_axis = source->mirror_axis;
	fa = source->fa->new_Duplicate();
}

ChFunction* ChFunction_Mirror::new_Duplicate ()
{
	ChFunction_Mirror* m_func;
	m_func = new ChFunction_Mirror;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Mirror::Get_y      (double x)
{
	if (x<= this->mirror_axis)
		return fa->Get_y(x);
	return fa->Get_y(2*this->mirror_axis - x);
}

void ChFunction_Mirror::Extimate_x_range (double& xmin, double& xmax)
{
	fa->Extimate_x_range(xmin,xmax);
}

int ChFunction_Mirror::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for children..

	chjs_propdata* mdataA = new chjs_propdata;
	strcpy(mdataA->propname, "fa");
	strcpy(mdataA->label,    mdataA->propname);
	mdataA->haschildren = TRUE;
	mtree->AddTail(mdataA);

	i += this->fa->MakeOptVariableTree(&mdataA->children);

	return i;
}


void ChFunction_Mirror::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << mirror_axis;
	mstream.AbstractWrite(fa);
}

void ChFunction_Mirror::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> mirror_axis;
	if (fa) delete fa; fa=NULL;
	mstream.AbstractReadCreate(&fa);
}

void ChFunction_Mirror::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_MIRROR  \n";

	//***TO DO***
}




//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Repeat

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Repeat> a_registration_repeat;


void ChFunction_Repeat::Copy (ChFunction_Repeat* source)
{
	window_start = source->window_start;
	window_length = source->window_length;
	fa = source->fa->new_Duplicate();
}

ChFunction* ChFunction_Repeat::new_Duplicate ()
{
	ChFunction_Repeat* m_func;
	m_func = new ChFunction_Repeat;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Repeat::Get_y      (double x)
{
	return fa->Get_y(this->window_start + fmod(x,this->window_length) );
}

void ChFunction_Repeat::Extimate_x_range (double& xmin, double& xmax)
{
	fa->Extimate_x_range(xmin,xmax);
}

int ChFunction_Repeat::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for children..

	chjs_propdata* mdataA = new chjs_propdata;
	strcpy(mdataA->propname, "fa");
	strcpy(mdataA->label,    mdataA->propname);
	mdataA->haschildren = TRUE;
	mtree->AddTail(mdataA);

	i += this->fa->MakeOptVariableTree(&mdataA->children);

	return i;
}

void ChFunction_Repeat::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << window_start;
	mstream << window_length;
	mstream.AbstractWrite(fa);
}

void ChFunction_Repeat::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> window_start;
	mstream >> window_length;
	if (fa) delete fa; fa=NULL;
	mstream.AbstractReadCreate(&fa);
}

void ChFunction_Repeat::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_REPEAT  \n";

	//***TO DO***
}






//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Recorder
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Recorder> a_registration_recorder;


void ChFunction_Recorder::Copy (ChFunction_Recorder* source)
{
	points.KillAll();
	lastnode = NULL;
	ChRecPoint* mpt;
	for (ChNode<ChRecPoint>* mnode = source->points.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		mpt = new ChRecPoint;
		mpt->x = mnode->data->x;
		mpt->y = mnode->data->y;
		mpt->w = mnode->data->w;
		points.AddTail(mpt);
	}
}

ChFunction* ChFunction_Recorder::new_Duplicate ()
{
	ChFunction_Recorder* m_func;
	m_func = new ChFunction_Recorder;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Recorder::Extimate_x_range (double& xmin, double& xmax)
{
	if (!points.GetTail())
	{
		xmin = 0.0; xmax = 1.2;
		return;
	}
	xmin = points.GetHead()->data->x;
	xmax = points.GetTail()->data->x;
	if (xmin == xmax) xmax = xmin+ 0.5;
}


int ChFunction_Recorder::AddPoint (double mx, double my, double mw)
{
	ChRecPoint* mpt = new ChRecPoint;
	mpt->x = mx;  	mpt->y = my;  	mpt->w = mw;
	double dist;

	if (!points.GetTail())
	{
		points.AddTail(mpt);
		return TRUE;
	}

	for (ChNode<ChRecPoint>* mnode = points.GetTail(); mnode != NULL; mnode= mnode->prev)
	{
		dist = mx - mnode->data->x;
		if (fabs(dist) < CH_RECORDER_EPSILON)
		{
			mnode->data = mpt;	// copy to preexisting node
			return TRUE;
		}
		if (dist > 0.0)
		{
			points.InsertAfter(mnode, mpt);  // insert
			return TRUE;
		}
	}


	points.AddHead(mpt); // was 1st in list
	return TRUE;
}

int ChFunction_Recorder::AddPointClean (double mx, double my, double dx_clean)
{
	ChRecPoint* mpt = new ChRecPoint;
	mpt->x = mx;  	mpt->y = my;  	mpt->w = 0;
	double dist;
	ChNode<ChRecPoint>* msetnode = NULL;

	if (!points.GetHead())
	{
		points.AddHead(mpt);
		lastnode = msetnode = points.GetHead();
		return TRUE;
	}

	if (!lastnode)
		lastnode = points.GetHead();

	if (lastnode->data->x < mx) // 1 forward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->next)
		{
			if (mnode->data->x >= mx)
			{
				lastnode = mnode;
				dist = - mx + mnode->data->x;
				if (fabs(dist) < CH_RECORDER_EPSILON) {
					mnode->data = mpt;
					msetnode = mnode;}	// copy to preexisting node
				else {
					points.InsertBefore(mnode, mpt);
					msetnode = mnode->prev;} // insert
				break;

			}
		}
		if (!msetnode)
		{			// ..oh, it should be tail..
			points.AddTail(mpt);
			msetnode = points.GetTail();
		}
	}
	else						// 2 backward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->prev)
		{
			if (mnode->data->x <= mx)
			{
				lastnode = mnode;
				dist =  mx - mnode->data->x;
				if (fabs(dist) < CH_RECORDER_EPSILON) {
					mnode->data = mpt;	// copy to preexisting node
					msetnode = mnode;}
				else {
					points.InsertAfter(mnode, mpt);  // insert
					msetnode = mnode->next; }
				break;
			}
		}
		if (!msetnode)
		{			// ..oh, it should be head
			points.AddHead(mpt);
			msetnode = points.GetHead();
		}
	}

	lastnode = msetnode;

	// clean on dx
	if (dx_clean >0)
	{
		ChNode<ChRecPoint>* mnode = msetnode->next;
		ChNode<ChRecPoint>* mnextnode = NULL;
		while (mnode != NULL)
		{
			mnextnode = mnode->next;
			if ((mnode->data->x - mx) < dx_clean)
			{
				points.Kill(mnode);
			}
			mnode = mnextnode;
		}
	}

	return TRUE;
}

double ChFunction_Recorder::Get_y      (double x)
{
	double y = 0;

	ChRecPoint p1;
	p1.w = 1;
	p1.x = 0;
	p1.y = 0;
	ChRecPoint p2;
	p2 = p1;
	p2.x = p1.x + 1.0;

	if (points.GetHead() == NULL) return 0.0;

	if (x < points.GetHead()->data->x) return 0.0;
	if (x > points.GetTail()->data->x) return 0.0;

	if (!lastnode) lastnode = points.GetHead();

	if (lastnode->data->x < x) // forward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->next)
		{
			if (mnode->data->x >= x)
			{
				p2 = *mnode->data; lastnode = mnode;
				if (mnode->prev)	{ p1 = *mnode->prev->data; }
				break;
			}
		}
	}
	else	// backward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->prev)
		{
			if (mnode->data->x <= x)
			{
				p1 = *mnode->data;	lastnode = mnode;
				if (mnode->next)	{ p2 = *mnode->next->data; }
				break;
			}
		}
	}


	y = ((x - p1.x)*p2.y + (p2.x -x)*p1.y)/(p2.x - p1.x);

	return y;
}

double ChFunction_Recorder::Get_y_dx   (double x)
{
	double dy = 0;

	ChRecPoint p1;					//    p0...p1..x...p2.....p3
	p1.x = x;	p1.y = 0;	p1.w = 1;
	ChRecPoint p2;
	p2 = p1;	p2.x += 100.0;
	ChRecPoint p0;
	p0 = p1;	p0.x -= 100.0;
	ChRecPoint p3;
	p3 = p1;	p3.x += 200.0;

	if (points.GetHead() == NULL) return 0.0;

	if (x < points.GetHead()->data->x) return 0.0;
	if (x > points.GetTail()->data->x) return 0.0;

	if (!lastnode) lastnode = points.GetHead();

	if (lastnode->data->x < x) // forward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->next)
		{
			if (mnode->data->x >= x)
			{
				p2 = *mnode->data; lastnode = mnode;
				if (mnode->prev) {
					p1 = *mnode->prev->data;
					if (mnode->prev->prev) {
						p0 = *mnode->prev->prev->data;
						if (mnode->next) {
							p3 = *mnode->next->data;
						}else { p3 = p2; p3.x += 1.0;}
					}else { p0 = p1; p0.x -= 1.0;}
				}else { p1 = p2; p1.x -= 1.0;}

				break;
			}
		}
	}
	else	// backward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->prev)
		{
			if (mnode->data->x <= x)
			{
				p1 = *mnode->data;	lastnode = mnode;
				if (mnode->next)  {
					p2 = *mnode->next->data;
					if (mnode->prev)  {
						p0 = *mnode->prev->data;
						if (mnode->next->next)  {
							p3 = *mnode->next->next->data;
						}else { p3 = p2; p3.x += 1.0;}
					}else { p0 = p1; p0.x -= 1.0;}
				}else { p2 = p1; p2.x += 1.0;}

				break;
			}
		}
	}

	double vA = (p1.y -p0.y)/(p1.x -p0.x);
	double vB = (p2.y -p1.y)/(p2.x -p1.x);
	double vC = (p3.y -p2.y)/(p3.x -p2.x);

	double v1 = 0.5* (vA + vB);
	double v2 = 0.5* (vB + vC);

	dy = ((x - p1.x)* v2 + (p2.x -x)* v1)  /  (p2.x - p1.x);

	return dy;
}

double ChFunction_Recorder::Get_y_dxdx (double x)
{
	double ddy = 0;

	ChRecPoint p1;					//    p0...p1..x...p2.....p3
	p1.x = x;	p1.y = 0;	p1.w = 1;
	ChRecPoint p2;
	p2 = p1;	p2.x = p1.x + 100.0;
	ChRecPoint p0;
	p0 = p1;	p0.x = p1.x - 100.0;
	ChRecPoint p3;
	p3 = p1;	p3.x = p1.x + 200.0;

	if (points.GetHead() == NULL) return 0.0;

	if (x < points.GetHead()->data->x) return 0.0;
	if (x > points.GetTail()->data->x) return 0.0;

	if (!lastnode) lastnode = points.GetHead();

	if (lastnode->data->x < x) // forward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->next)
		{
			if (mnode->data->x >= x)
			{
				p2 = *mnode->data; lastnode = mnode;
				if (mnode->prev) {
					p1 = *mnode->prev->data;
					if (mnode->prev->prev) {
						p0 = *mnode->prev->prev->data;
						if (mnode->next) {
							p3 = *mnode->next->data;
						}else { p3 = p2; p3.x += 1.0;}
					}else { p0 = p1; p0.x -= 1.0;}
				}else { p1 = p2; p1.x -= 1.0;}

				break;
			}
		}
	}
	else	// backward search
	{
		for (ChNode<ChRecPoint>* mnode = lastnode; mnode != NULL; mnode= mnode->prev)
		{
			if (mnode->data->x <= x)
			{
				p1 = *mnode->data;	lastnode = mnode;
				if (mnode->next)  {
					p2 = *mnode->next->data;
					if (mnode->prev)  {
						p0 = *mnode->prev->data;
						if (mnode->next->next)  {
							p3 = *mnode->next->next->data;
						}else { p3 = p2; p3.x += 1.0;}
					}else { p0 = p1; p0.x -= 1.0;}
				}else { p2 = p1; p2.x += 1.0;}

				break;
			}
		}
	}

	double vA = (p1.y -p0.y)/(p1.x -p0.x);
	double vB = (p2.y -p1.y)/(p2.x -p1.x);
	double vC = (p3.y -p2.y)/(p3.x -p2.x);

	double a1 = 2.0 * (vB - vA)/(p2.x - p0.x);
	double a2 = 2.0 * (vC - vB)/(p3.x - p1.x);

	ddy = ((x - p1.x)* a2 + (p2.x -x)* a1)  /  (p2.x - p1.x);

	return ddy;
}


void ChFunction_Recorder::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << (int)(points.Count());

	for (ChNode<ChRecPoint>* mnode = points.GetTail(); mnode != NULL; mnode= mnode->prev)
	{
		mstream << mnode->data->x;
		mstream << mnode->data->y;
		mstream << mnode->data->w;
	}
}

void ChFunction_Recorder::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	int mcount;
	mstream >> mcount;

	for (int i = 0; i < mcount; i++)
	{
		ChRecPoint* mpt = new ChRecPoint;
		mstream >> mpt->x;
		mstream >> mpt->y;
		mstream >> mpt->w;
		points.AddHead(mpt);
	}

}

void ChFunction_Recorder::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_RECORDER  \n";

	//***TO DO***
}





//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Oscilloscope
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Oscilloscope> a_registration_oscilloscope;


void ChFunction_Oscilloscope::Copy (ChFunction_Oscilloscope* source)
{
	this->values = source->values;
	this->dx = source->dx;
	this->end_x = source->end_x;
	this->amount = source->amount;
	this->max_amount = source->max_amount;
}

ChFunction* ChFunction_Oscilloscope::new_Duplicate ()
{
	ChFunction_Oscilloscope* m_func;
	m_func = new ChFunction_Oscilloscope;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Oscilloscope::Extimate_x_range (double& xmin, double& xmax)
{
	xmin = this->end_x - this->dx*(this->amount-1);
	xmax = this->end_x;
	if (xmin >= xmax) xmax = xmin+ 0.5;
}


int ChFunction_Oscilloscope::AddLastPoint (double mx, double my)
{
	if (mx < end_x)
		this->Reset();
	this->end_x=mx;
	this->values.push_back(my);
	if (this->amount < this->max_amount)
		this->amount++;
	else
		this->values.pop_front();

	assert(this->values.size()==this->amount);
	return TRUE;
}



double ChFunction_Oscilloscope::Get_y      (double x)
{
	double y = 0;

	double start_x = this->end_x - this->dx*(this->amount-1);
	if (x > end_x) return 0;
	if (x < start_x) return 0;
	
	int i1 = (int)floor( (x - start_x)/this->dx );
	int i2 = i1+1;
	double p1x = start_x + dx*(double)i1;
	double p2x = start_x + dx*(double)i2;
	double p2y,p1y = 0;
	int count = 0;
	std::list<double>::iterator iter = values.begin();
	while(iter != values.end())
	{
		if (count == i1)
		{
			p2y = *iter;
			iter++;
			p1y = *iter;
			break;
		}
		count++;
		iter++;
	}

	y = ((x - p1x)*p2y + (p2x -x)*p1y)/(p2x - p1x);

	return y;
}




void ChFunction_Oscilloscope::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << dx;
	mstream << end_x;
	mstream << max_amount;
	mstream << amount;
	std::list<double>::iterator iter = values.begin();
	while(iter != values.end())
	{
		mstream << *iter;
		iter++;
	}
}

void ChFunction_Oscilloscope::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> dx;
	mstream >> end_x;
	mstream >> max_amount;
	mstream >> amount;
	values.clear();
	int i = 0;
	while(i < amount)
	{
		double mval;
		mstream >> mval;
		this->values.push_back(mval);
	}
}

void ChFunction_Oscilloscope::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_OSCILLOSCOPE \n";

	//***TO DO***
}



//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Sequence
//

ChFseqNode::ChFseqNode(ChFunction* myfx, double mdur)
{
	fx = myfx;
	duration = mdur;
	weight = 1;
	t_start = 0; t_end = t_start+ duration;
	Iy = Iydt = Iydtdt = 0.0;
	y_cont = ydt_cont = ydtdt_cont = FALSE;
}

ChFseqNode::~ChFseqNode()
{
	if (fx) delete fx; fx = NULL;
}

void ChFseqNode::Copy(ChFseqNode* source)
{
	if (fx) delete fx;
	fx = source->fx->new_Duplicate();
	duration = source->duration;
	weight = source->weight;
	t_start = source->t_start;
	t_end = source->t_end;
	Iy = source->Iy;
	Iydt = source->Iydt;
	Iydtdt = source->Iydtdt;
	y_cont = source->y_cont;
	ydt_cont = source->ydt_cont;
	ydtdt_cont = source->ydtdt_cont;
}

void ChFseqNode::SetDuration(double mdur)
{
	duration = mdur;
	if (duration<0)
		duration =0;
	t_end = t_start+duration;
};

void ChFseqNode::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// stream out all member data
	mstream << this->duration;
	mstream << this->weight;
	mstream << this->t_start;
	mstream << this->t_end;
	mstream << this->Iy;
	mstream << this->Iydt;
	mstream << this->Iydtdt;
	mstream << this->y_cont;
	mstream << this->ydt_cont;
	mstream << this->ydtdt_cont;
	mstream.AbstractWrite(this->fx);
}

void ChFseqNode::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// stream in all member data
	mstream >> this->duration;
	mstream >> this->weight;
	mstream >> this->t_start;
	mstream >> this->t_end;
	mstream >> this->Iy;
	mstream >> this->Iydt;
	mstream >> this->Iydtdt;
	mstream >> this->y_cont;
	mstream >> this->ydt_cont;
	mstream >> this->ydtdt_cont;
	if (fx) delete fx; fx=NULL;
	mstream.AbstractReadCreate(&this->fx);
}


/////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Sequence> a_registration_sequence;



ChFunction_Sequence::ChFunction_Sequence ()
{
	start = 0;
}

ChFunction_Sequence::~ChFunction_Sequence ()
{
	functions.KillAll();
}

void ChFunction_Sequence::Copy (ChFunction_Sequence* source)
{
	start = source->start;
	functions.KillAll();
	ChFseqNode* mfxs;
	for (ChNode<ChFseqNode>* mnode = source->functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		mfxs = new ChFseqNode(NULL, 0.0);
		mfxs->Copy(mnode->data);
		functions.AddTail(mfxs);
	}
}

ChFunction* ChFunction_Sequence::new_Duplicate ()
{
	ChFunction_Sequence* m_func;
	m_func = new ChFunction_Sequence;
	m_func->Copy(this);
	return (m_func);
}

int ChFunction_Sequence::InsertFunct (ChFunction* myfx, double duration, double weight, bool c0, bool c1, bool c2, int position)
{
	ChFseqNode* mfxsegment = new ChFseqNode(myfx, duration);
	mfxsegment->y_cont = c0;
	mfxsegment->ydt_cont = c1;
	mfxsegment->ydtdt_cont = c2;
	mfxsegment->weight = weight;

	int inserted = FALSE;
	if (position == 0)
		{ functions.AddHead(mfxsegment); inserted = TRUE;}
	if (position == -1)
		{ functions.AddTail(mfxsegment); inserted = TRUE;}
	if (!inserted)
	{
		int ind = 1;
		for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
		{
			if (ind == position)
				{ functions.InsertAfter(mnode, mfxsegment); inserted = TRUE;  break; }
			ind ++;
		}
	}
	if (!inserted)
		{ functions.AddTail(mfxsegment);}
				// update the continuity offsets and timings
	this->Setup();
	return inserted;
}

int ChFunction_Sequence::KillFunct (int position)
{
	int fcount = functions.Count();
	if (fcount == 0)
		return FALSE;
	if ((position == -1)||(position > fcount))
		{ functions.Kill(functions.GetTail()); return TRUE;}
	if (position == 0)
		{ functions.Kill(functions.GetHead()); return TRUE;}
	functions.Kill(functions.GetNum(position));

	this->Setup();
	return TRUE;
}

ChFunction* ChFunction_Sequence::GetNthFunction (int position)
{
	int fcount = functions.Count();
	if (fcount == 0)
		return NULL;
	if ((position == -1)||(position > fcount))
		{ return functions.GetTail()->data->fx;}
	if (position == 0)
		{ return functions.GetHead()->data->fx;}
	return  functions.GetNum(position)->data->fx;
}

double ChFunction_Sequence::GetNthDuration(int position)
{
	static double default_dur =0.0;
	int fcount = functions.Count();
	if (fcount == 0)
		return default_dur;
	if ((position == -1)||(position > fcount))
		{ return functions.GetTail()->data->duration;}
	if (position == 0)
		{ return functions.GetHead()->data->duration;}
	return  functions.GetNum(position)->data->duration;
}

ChFseqNode* ChFunction_Sequence::GetNthNode(int position)
{
	int fcount = functions.Count();
	if (fcount == 0)
		return NULL;
	if ((position == -1)||(position > fcount))
		{ return functions.GetTail()->data;}
	if (position == 0)
		{ return functions.GetHead()->data;}
	return  functions.GetNum(position)->data;
}

void ChFunction_Sequence::Setup()
{
	double basetime = this->start;
	double lastIy = 0;
	double lastIy_dt = 0;
	double lastIy_dtdt = 0;

	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		mnode->data->t_start = basetime;
		mnode->data->t_end   = basetime + mnode->data->duration;
		mnode->data->Iy		= 0;
		mnode->data->Iydt	= 0;
		mnode->data->Iydtdt = 0;

		if (mnode->data->fx->Get_Type() == FUNCT_FILLET3)	// C0 C1 fillet
		{
			((ChFunction_Fillet3*)mnode->data->fx)->Set_y1(lastIy);
			((ChFunction_Fillet3*)mnode->data->fx)->Set_dy1(lastIy_dt);
			if (mnode->next)
			{
				((ChFunction_Fillet3*)mnode->data->fx)->Set_y2(mnode->next->data->fx->Get_y(0));
				((ChFunction_Fillet3*)mnode->data->fx)->Set_dy2(mnode->next->data->fx->Get_y_dx(0));
			}else
			{
				((ChFunction_Fillet3*)mnode->data->fx)->Set_y2(0);
				((ChFunction_Fillet3*)mnode->data->fx)->Set_dy2(0);
			}
			((ChFunction_Fillet3*)mnode->data->fx)->Set_end(mnode->data->duration);
			mnode->data->Iy = mnode->data->Iydt = mnode->data->Iydtdt = 0;
		}
		else	// generic continuity conditions
		{
			if (mnode->data->y_cont)
				mnode->data->Iy = lastIy - mnode->data->fx->Get_y(0);
			if (mnode->data->ydt_cont)
				mnode->data->Iydt = lastIy_dt - mnode->data->fx->Get_y_dx(0);
			if (mnode->data->ydtdt_cont)
				mnode->data->Iydtdt = lastIy_dtdt - mnode->data->fx->Get_y_dxdx(0);
		}

		lastIy = mnode->data->fx->Get_y(mnode->data->duration) +
				 mnode->data->Iy +
				 mnode->data->Iydt *  mnode->data->duration +
				 mnode->data->Iydtdt *  mnode->data->duration *  mnode->data->duration;
		lastIy_dt =
				 mnode->data->fx->Get_y_dx(mnode->data->duration) +
				 mnode->data->Iydt +
				 mnode->data->Iydtdt *  mnode->data->duration;
		lastIy_dtdt =
				 mnode->data->fx->Get_y_dxdx(mnode->data->duration) +
				 mnode->data->Iydtdt;

		basetime += mnode->data->duration;
	}
}

double ChFunction_Sequence::Get_y      (double x)
{
	double res = 0;
	double localtime;
	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if ((x >= mnode->data->t_start)&&(x < mnode->data->t_end))
		{
			localtime = x - mnode->data->t_start;
			res = mnode->data->fx->Get_y(localtime) +
				  mnode->data->Iy +
				  mnode->data->Iydt *  localtime +
				  mnode->data->Iydtdt *  localtime *  localtime;
		}
	}
	return res;
}

double ChFunction_Sequence::Get_y_dx   (double x)
{
	double res = 0;
	double localtime;
	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if ((x >= mnode->data->t_start)&&(x < mnode->data->t_end))
		{
			localtime = x - mnode->data->t_start;
			res = mnode->data->fx->Get_y_dx(localtime) +
				  mnode->data->Iydt +
				  mnode->data->Iydtdt *  localtime;
		}
	}
	return res;
}

double ChFunction_Sequence::Get_y_dxdx (double x)
{
	double res = 0;
	double localtime;
	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if ((x >= mnode->data->t_start)&&(x < mnode->data->t_end))
		{
			localtime = x - mnode->data->t_start;
			res = mnode->data->fx->Get_y_dxdx(localtime) +
				  mnode->data->Iydtdt;
		}
	}
	return res;
}


double ChFunction_Sequence::Get_weight (double x)
{
	double res = 1.0;
	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if ((x >= mnode->data->t_start)&&(x < mnode->data->t_end))
		{
			res = mnode->data->weight;
		}
	}
	return res;
}

void ChFunction_Sequence::Extimate_x_range (double& xmin, double& xmax)
{
	xmin = start;
	xmax = functions.GetTail()->data->t_end;
	if (xmin == xmax) xmax = xmin + 1.1;
}


int ChFunction_Sequence::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for all children..
	int cnt=1;
	char msubduration[50];
	char msubfunction[50];
	for (ChNode<ChFseqNode>* mnode = this->functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		sprintf(msubduration,"node_n(%d).duration", cnt);

		chjs_propdata* mdataA = new chjs_propdata;
		strcpy(mdataA->propname, msubduration);
		strcpy(mdataA->label,    mdataA->propname);
		mdataA->haschildren = FALSE;
		mtree->AddTail(mdataA);
		i++;

		sprintf(msubfunction,"node_n(%d).fx", cnt);

		chjs_propdata* mdataB = new chjs_propdata;
		strcpy(mdataB->propname, msubfunction);
		strcpy(mdataB->label,    mdataB->propname);
		mdataB->haschildren = TRUE;
		mtree->AddTail(mdataB);

		i += mnode->data->fx->MakeOptVariableTree(&mdataB->children);

		cnt++;
	}

	return i;
}


int ChFunction_Sequence::HandleNumber()
{
	int tot = 1;
	for (ChNode<ChFseqNode>* mnode = this->functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		tot++;
	}
	return tot;
}

int ChFunction_Sequence::HandleAccess(int handle_id, double mx, double my, bool set_mode)
{
	if (handle_id==0)
	{
		if (!set_mode)
			mx = this->Get_start();
		else
			this->Set_start(mx);
		return TRUE;
	}
	int tot = 1;
	for (ChNode<ChFseqNode>* mnode = this->functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		if (handle_id==tot)
		{
			if (!set_mode)
				mx = mnode->data->t_end;
			else
			{
				mnode->data->SetTend(mx);
				this->Setup();
			}
			return TRUE;
		}

	}

	return FALSE;
}


void ChFunction_Sequence::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	int stopID = 0;
	int goID = 1;

	mstream << Get_start();

	for (ChNode<ChFseqNode>* mnode = functions.GetHead(); mnode != NULL; mnode= mnode->next)
	{
		mstream << goID;
		mstream << *mnode->data;
	}
	mstream << stopID;

}

void ChFunction_Sequence::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	double dfoo;
	mstream >> dfoo;		Set_start(dfoo);
	int mgoID;
	mstream >> mgoID;
	while (mgoID == 1)
	{
		ChFseqNode* mynode = new ChFseqNode(NULL, 0.0);
		mstream >> *mynode;
		functions.AddTail(mynode);
		mstream >> mgoID;
	}
}

void ChFunction_Sequence::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_SEQUENCE  \n";

	//***TO DO***
}





//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Matlab

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Matlab> a_registration_matlab;

ChFunction_Matlab::ChFunction_Matlab ()
{
	strcpy(this->mat_command,"x*2+x^2");
}

void ChFunction_Matlab::Copy (ChFunction_Matlab* source)
{
	strcpy(this->mat_command, source->mat_command);
}

ChFunction* ChFunction_Matlab::new_Duplicate ()
{
	ChFunction_Matlab* m_func;
	m_func = new ChFunction_Matlab;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Matlab::Get_y      (double x)
{
	double ret = 0;

	static char m_eval_command[CHF_MATLAB_STRING_LEN+20];

	#ifdef CH_MATLAB
	 // no function: shortcut!
	if (*this->mat_command == NULL) return 0.0;

	 // set string as "x=[x];ans=[mat_command]"
	 sprintf (m_eval_command, "x=%g;ans=%s;", x,this->mat_command);

	 // EVAL string, retrieving y = "ans"
	 ret = ChGLOBALS().Mat_Eng_Eval(m_eval_command);

	#else
	 ret = 0.0;
	#endif

	return ret;
}

void ChFunction_Matlab::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << mat_command;
}

void ChFunction_Matlab::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> mat_command;
}

void ChFunction_Matlab::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_MATLAB  \n";

	//***TO DO***
}






//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Noise

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Noise> a_registration_noise;


ChFunction_Noise::ChFunction_Noise ()
{
	this->amp = 1;
	this->octaves = 2;
	this->amp_ratio = 0.5;
	this->freq = 1;
}

void ChFunction_Noise::Copy (ChFunction_Noise* source)
{
	this->amp = source->amp;
	this->freq = source->freq;
	this->amp_ratio = source->amp_ratio;
	this->octaves = source->octaves;
}

ChFunction* ChFunction_Noise::new_Duplicate ()
{
	ChFunction_Noise* m_func;
	m_func = new ChFunction_Noise;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Noise::Get_y      (double x)
{
	return ChNoise(x,amp,freq,octaves,amp_ratio);
}

void ChFunction_Noise::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << amp;
	mstream << freq;
	mstream << amp_ratio;
	mstream << octaves;
}

void ChFunction_Noise::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> amp;
	mstream >> freq;
	mstream >> amp_ratio;
	mstream >> octaves;
}

void ChFunction_Noise::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_NOISE  \n";

	//***TO DO***
}







} // END_OF_NAMESPACE____


// eof
