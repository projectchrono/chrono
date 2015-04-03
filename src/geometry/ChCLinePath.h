//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_LINEPATH_H
#define CHC_LINEPATH_H



#include <math.h>

#include "ChCLine.h"
#include "core/ChSmartpointers.h"

namespace chrono
{
namespace geometry 
{



#define CH_GEOCLASS_LINEPATH   20

///
/// ARC
///
/// Geometric object representing an sequence of other ChLine objects,
/// assuming they are concatenated properly, to have C0 continuity.
/// 
	
class ChApi ChLinePath : public ChLine 
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChLinePath,ChLine);

public:

		//
		// DATA
		//
	std::vector< ChSmartPtr<ChLine> > lines;
	std::vector< double > end_times;
	std::vector< double > durations;

public:	

		//
		// CONSTRUCTORS
		//

		// Creation by default. 

	ChLinePath ()
	{
	}

	~ChLinePath () {};

	ChLinePath(const ChLinePath & source)
	{
		lines = source.lines;
		end_times = source.end_times;
		durations = source.durations;
	}

	void Copy (const ChLinePath* source)
	{
		ChLine::Copy(source);
		lines = source->lines;
		end_times = source->end_times;
		durations = source->durations;
	}

	ChGeometry* Duplicate () 
	{
		return new ChLinePath(*this); 
	};


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_LINEPATH;};

	virtual int Get_complexity() {return 2;};


			/// Returns curve length. sampling does not matter
	double Length (int sampling) 
	{ 
		double tot = 0;
		for (int i= 0; i< lines.size(); ++i)
		{
			tot += lines[i]->Length(sampling);
		}
		return tot;
	}


				/// Curve evaluation (only parU is used, in 0..1 range)
	virtual void Evaluate(Vector& pos, 
						const double parU, 
						const double parV = 0., 
						const double parW = 0.)
	{
		if (lines.size() == 0)
			return;

		double uA = 0;
		double uB = 0;
		// Search sub line covering the parU 
		// (brute force search.. assuming a limited number of 
		// added lines, it is ok anyway.)
		int i;
		for (i= 0; i< lines.size(); ++i)
		{ 
			if (parU <= end_times[i])
				break;
		}
		uB = end_times[i];
		if (i>0)
			uA = end_times[i-1];

		double local_U = (parU - uA) / durations[i];
		lines[i]->Evaluate(pos, local_U, 0,0);
	}

		/// Return the start point of the line.
	virtual ChVector<> GetEndA() { return (*lines.begin())->GetEndA(); }

		/// Return the end point of the line.
	virtual ChVector<> GetEndB() { return (*lines.end())->GetEndB(); }


		//
		// CUSTOM FUNCTIONS
		//


		/// Get count of sub-lines that have been added:
	size_t GetSubLinesCount() { return lines.size();}

		/// Access the nth line
	ChSmartPtr<ChLine> GetSubLineN (int n) { return lines[n];}

		/// Get the nth line duration
	double GetSubLineDurationN (int n) { return durations[n];}

		/// Set the nth line duration
	void   SetSubLineDurationN (int n, double mduration) 
	{ 
		durations[n] = mduration;

		double last_t = 0;
		if (n>0)
			last_t = end_times[n-1];
		for (int i= n; i< end_times.size(); ++i)
		{ 
			last_t += durations[n];
			end_times[n] = last_t; 
		}
	}

		/// Queue a line (push it back to the array of lines)
	void AddSubLine(   ChSmartPtr<ChLine> mline, //<<< line to add
					double duration = 1)	  //<<< duration of the abscyssa when calling the Evaluate() function
	{ 
		lines.push_back(mline);
		durations.push_back(0);
		end_times.push_back(0);
		SetSubLineDurationN(lines.size()-1, duration);
	}
		/// Queue a line (push it back to the array of lines)
	void AddSubLine( ChLine& mline,			//<<< line to add
					double duration = 1)	//<<< duration of the abscyssa when calling the Evaluate() function
	{ 
		ChSmartPtr<ChLine> pline((ChLine*)mline.Duplicate()); 
		AddSubLine(pline, duration);
	}
	
		/// Insert a line at a specified index  n  in line array.
		/// Note that  n  cannot be higher than GetLineCount(). 
	void InsertSubLine(int n,				  //<<< index of line, 0 is first, etc.
					ChSmartPtr<ChLine> mline, //<<< line to add
					double duration = 1)	  //<<< duration of the abscyssa when calling the Evaluate() function
	{ 
		lines.insert(lines.begin()+n, mline);
		durations.push_back(0);
		end_times.push_back(0);
		// force recompute following end times:
		SetSubLineDurationN(n, duration);
	}

		/// Insert a line at a specified index  n  in line array.
		/// Note that  n  cannot be higher than GetLineCount(). 
	void InsertSubLine(int n,				  //<<< index of line, 0 is first, etc.
					ChLine& mline,			  //<<< line to add
					double duration = 1)	  //<<< duration of the abscyssa when calling the Evaluate() function
	{ 
		ChSmartPtr<ChLine> pline((ChLine*)mline.Duplicate()); 
		InsertSubLine(n, pline, duration);
	}

		/// Erase a line from a specified index  n  in line array.
		/// Note that  n  cannot be higher than GetLineCount().
	void EraseSubLine(int n)	//<<< index of line, 0 is first, etc.
	{ 
		lines.erase(lines.begin()+n);
		durations.erase(durations.begin()+n);
		end_times.pop_back();
		// force recompute all end times:
		if (lines.size())
			SetSubLineDurationN(0, durations[0]);
	}

		/// Tells the duration of the path, sum of the durations of all sub-lines.
		/// This is useful because ifyou use the Evaluate() function on the path, the U
		/// parameter should range between 0 and the max duration.
	double GetPathDuration()
	{
		if (end_times.size())
			return end_times.back();
		return 0;
	}

		/// Shrink or stretch all the durations of the sub-lines so that the
		/// total duration of the path is equal to a specified value.
		/// For example, you can normalize to 1 so you can use Evaluate() with U in 
		/// the 0..1 range like with other lines.
	void SetPathDuration(double mUduration)
	{
		double factor = mUduration / GetPathDuration();
		double last_t = 0;
		for (int i= 0; i< end_times.size(); ++i)
		{ 
			durations[i] *= factor;
			last_t += durations[i];
			end_times[i] = last_t; 
		}
	}

		/// Check if the path is topologically connected,
		/// i.e. if all the sub lines are queued to have C0 continuity
	double GetContinuityMaxError()
	{
		double maxerr= 0;
		for (int i= 1; i< lines.size(); ++i)
		{
			ChSmartPtr< ChLine > prec_line = lines[i-1];
			ChSmartPtr< ChLine > next_line = lines[i];
			double gap = ( prec_line->GetEndB() -  next_line->GetEndA() ).Length();
			if (gap > maxerr)
				maxerr = gap;
		}
		return maxerr;
	}

		//
		// STREAMING
		//

	void StreamOUT(ChStreamOutBinary& mstream)
	{
		// class version number
		mstream.VersionWrite(1);

			// serialize parent class too
		ChLine::StreamOUT(mstream);

			// stream out all member data
		//**TODO**
	}

	void StreamIN(ChStreamInBinary& mstream) 
	{
			// class version number
		int version = mstream.VersionRead();

			// deserialize parent class too
		ChLine::StreamIN(mstream);

			// stream in all member data
		//**TODO**
	}


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif  // END of header


 
	