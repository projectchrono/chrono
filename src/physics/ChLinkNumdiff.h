#ifndef CHLINKNUMDIFF_H
#define CHLINKNUMDIFF_H

///////////////////////////////////////////////////
//
//   ChLinkNumdiff.h
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLinkMasked.h"



namespace chrono
{



///
/// Link for constraints where the jacobians and other 
/// complex terms are automatically computed by numerical 
	/// differentiation. ***EXPERIMENTAL*** (UNDER DEVELOPEMENT)
///
/// The user can inherit custom constraint classes from this
/// class, and the only 'big' function to implement is the
/// ComputeC() function, which must return the residual of the 
/// constraint, given the state of the bodies and the time. that's all.
///
/// Note that numerical differentiation for computing jacobians
/// might be an easy way to write constraints, but in general it
/// is slower (and less precise) than with custom analytical approaches
/// (that's why the ChLinkLock family is not inherited from this 
/// class and prefer to use a custom analytical expression for 
/// jacobians).
///

class ChLinkNumdiff : public ChLinkMasked {

	CH_RTTI(ChLinkNumdiff,ChLinkMasked);

protected:

				//
	  			// DATA
				//

public:
				//
	  			// CONSTRUCTORS
				//
	ChLinkNumdiff ();
	virtual ~ChLinkNumdiff ();
	virtual void Copy(ChLinkNumdiff* source);
	virtual ChLink* new_Duplicate ();


				//
	  			// FUNCTIONS
				//

					///  Forces the system (basically: two connected bodies) into
					/// the specified states/times. Used internally by numerical
					/// differentiation, when computing numerially the jacobian,
					/// by the UpdateState() function. Used by UpdateState() and ComputeK().
					///  Default: the mc matrix is a vector with "Link::GetNumCoords" rows,
					/// with the position states (vect1,quat1,vect1,quat2) of Body1 and Body2.
					/// Note!! It automatically calls, at the end, the
					/// UpdateTime(time)  and 	UpdateRelMarkerCoords()  functions!!!
					///  Most often, no need to give specific implementations if the
					/// q coordinate vector of your constraint equation(s) C(q,t)
					/// represents the position of the two bodies Body1 and Body2, and it's enough.
					/// But you must override it, if your inherited link' "numcoords" aren't 14,
					/// for example if linking 3 bodies, etc -really seldom..-, or body pos & speeds, etc
	virtual void ImposeCoords(ChMatrix<>* mc, double t);

					/// The opposite: returns the current state into the vector mc,
					/// which has number of rows = "GetNumCoords()" (def. 14)
	virtual void FetchCoords(ChMatrix<>* mc);

					/// Same, but returns the _time_derivative_ of current state (the speeds)
	virtual void FetchCoords_dt(ChMatrix<>* mc);

 
					/// >>>> IMPORTANT FUNCTION<<<< 
					/// Given current time and coordinate state, computes
					/// the residual vect. of the constraint equations C.
					///      C = C(q,t)
					///  NOTE!! If your class inherits from this class, often
					/// this is the ONLY function you _must_ provide, together with GetDOC(),
					/// because the base UpdateState() will automatically compute
					/// the jacobians matrix etc. via numerical differentiation
					/// (Default: C = 0, no particular constraint)
	virtual void ComputeC();

					/// Used mostly internally by UpdateState(), to compute Ct term and
					/// and Cq1, Cq2 jacobian matrices at current (q,t), by numerical differentiation. 
					/// However, you may override one of these if you know some fast custom 
					/// analytical expression - otherwise leave these as they are, and 
					/// the numerical differentiation will do the rest.
	virtual void ComputeCt();
	virtual void ComputeCq();


			//
			// UPDATING
			//

					// Override the parent ChLinkMasked UpdateState(), which does nothing,
					// because this version will call ComputeCt, ComputeCq to get the 
					// values of Cq1, Cq2, Qc, Ct. This will happen automatically if one
					// has provided a proper ComputeC() function in his inherited class.
	virtual void UpdateState ();



			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);

					/// Method to allow serialization of transient data in ascii,
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	virtual void StreamOUT(ChStreamOutAscii& mstream) {};

};






} // END_OF_NAMESPACE____

#endif
