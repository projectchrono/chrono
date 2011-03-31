#ifndef CHFUNCT_SEQUENCE_H
#define CHFUNCT_SEQUENCE_H

//////////////////////////////////////////////////
//  
//   ChFunction_Sequence.h
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

#define FUNCT_SEQUENCE	7


/// Node for the list of functions 
/// in a ChFunction_Sequence object.

class ChApi ChFseqNode
{
public:
	ChFunction* fx;
	double duration;
	double weight;
	double t_start;
	double t_end;
	double Iy;
	double Iydt;
	double Iydtdt;
	bool y_cont;
	bool ydt_cont;
	bool ydtdt_cont;

	void SetDuration(double mdur);
	void SetTend(double mt_end) {t_end = mt_end; if (t_end < t_start) t_end= t_start; duration = t_end - t_start;};

	ChFseqNode(ChFunction* myfx, double mdur);
	~ChFseqNode();
	void Copy(ChFseqNode* source);

	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



/// SEQUENCE FUNCTION:
///   y = sequence_of_functions(f1(y), f2(y), f3(y))  
/// All other function types can be inserted into this.
/// This function is very important because very complex motion
/// laws can be created by sequencing many basic ChFunctions.

class ChApi ChFunction_Sequence : public ChFunction
{
	CH_RTTI(ChFunction_Sequence, ChFunction);
private:
	ChList<ChFseqNode> functions;	// the list of sub functions
	double start;					// start time for sequence
public:
	ChFunction_Sequence();
	~ChFunction_Sequence();
	void Copy (ChFunction_Sequence* source);
	ChFunction* new_Duplicate ();
	
	int Get_Type () {return (FUNCT_SEQUENCE);}		


				/// The sequence of functions starts at this x value.
	void	Set_start  (double m_start)  {start = m_start;};
	double	Get_start  ()  {return start;};

				/// Access the list of the sub-functions.
	ChList<ChFseqNode>* Get_list() {return &functions;};

				/// Scans all the seq.of functions and setup the timings and continuity 
				/// offsets, to satisfy all constraints.
				/// This must be called whenever a new function is inserted, or its 
				/// timing and continuity constraints are changed.
	void Setup();

				/// Insert function after the fx with defined "position" index in list.
				///  - If index is higher than available objects, it simply goes to the end. 
				///  - If index = 0 insert at the beginning, 
				///  - If index = -1 insert at the end.
				/// Inserted functions will be deleted automatically when this object will be deleted.
				/// The fx segment has its own 'weight': use 1.0 for default, or different weights 
				/// if you want that Get_weight() will give different results depending on the "x" parameter.
				/// Set c0=true if you want to force C0 continuity with previous function (an offset
				/// will be implicitly added to the function, as y=f(x)+Offset). Same for C1 and C2 continuity,
				/// using c1 and c2 flags.
	int InsertFunct (ChFunction* myfx, double duration, double weight, bool c0, bool c1, bool c2, int position);

				/// Remove and deletes function with defined "position", and returns TRUE. 
				///	 - If position = 0, removes always head (beginning), 
				///  - If position = -1 removes tail (end). 
				///  - If position > max number of current nodes, removes tail anyway, but returns NULL.
	int KillFunct (int position);

				/// Returns the ChFunction with given "position". 
				///  - If position = 0, returns always head (beginning), 
				///  - If position = -1 returns tail (end). 
				///  - If position > max number of current nodes, returns tail fx anyway.
	ChFunction* GetNthFunction (int position);		

				/// As above, but returns the function node (containing function pointer, 
				/// function duration, continuity flags with previous node, etc.) 
	ChFseqNode* GetNthNode(int position);

				/// As above, but returning duration. (return value is reference, so it 
				/// can be also changed later, but remember Setup() for the 
				/// ChFunction_Sequence after you modified this return value by reference ***TO DO***). 
				/// If no function, returns 0.
	double GetNthDuration(int position);

		
	double Get_y      (double x);
	double Get_y_dx   (double x);
	double Get_y_dxdx (double x);

	double Get_weight (double x);

	void Extimate_x_range (double& xmin, double& xmax);


	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);

	int HandleNumber();			
	int HandleAccess(int handle_id, double mx, double my, bool set_mode);

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};




} // END_OF_NAMESPACE____


#endif
