#ifndef TERRAMECH_H
#define TERRAMECH_H

// terramechanics includes
#include "TmAPI.h"
#include "TireAPI.h"
#ifndef _M_X64
#define _M_X64
#endif
#include "cppSharedDataTypes.h"
#include "TM_inputConfig.h"
#include "TM_TractionElement.h"
#include "testUtils.h"
// #include "IrrlichtBuffer.h"
#include "TmIrrWizard.h"
// *********

// chrono includes
#include "physics/CHapidll.h" 
#include "physics/CHsystem.h"
#include "irrlicht_interface/CHbodySceneNode.h"

using namespace chrono;
using namespace irr;
using namespace scene;

// for quick output of TMVector
ostream& operator << (ostream& output, const TMVector& v) {
	output << v.x << "," << v.y << "," << v.z;
	return output;
}
// for quick output of ChQuaternion
ostream& operator << (ostream& output, const TMQuaternion& q) {
	output << q.e0 << "," << q.e1 << "," << q.e2 << "," << q.e3;
	return output;
}

/// helper class, holds onto tire body specific data, IN SAE UNITS!!! [in, radian, lbf]
/// Author: Justin Madsen
class TmQueryElement
{
public:

	// keeps track of a single traction element used for the tm
	// specifically, used for passing data between Terramechanics and Chrono API boundaries
	TmQueryElement() {};

	// constructor for a given cm and orientation
	TmQueryElement(TMVector& cm, Quaternion& quat): pos(cm), rot(quat) {};

	// when all ICs are defined
	TmQueryElement(TMVector& cm, Quaternion& quat,
		TMVector& cm_vel, TMVector& cm_omega): pos(cm), rot(quat), vel(cm_vel), omega(cm_omega)	{};

	~TmQueryElement() {};

	TMVector pos;	// global position
	Quaternion rot; // orientation quaternion
	TMVector vel;	// global velocity
	TMVector omega;	// spin rate, w.r.t. CM coords
	TMVector Force;	// reaction torque
	TMVector Torque;	// reaction force
};

// responsible for making all TM or Tire API function calls
// also contains error flags, persistent data
class TerraMech
{
private:
	// simulation 
	tmReal sim_time;
	tmReal step_size;
	tmReal end_time;	// end of simulation
	size_t num_steps;	// total number of time steps taken
	size_t num_tires;	// number of tires being used

	TM_inputConfig*	tm_input_cfg;		// terramechanics configuration data is read from "...config.tm" file in the ../data directory

	// keep a copy of the tire state for each tire
	std::vector<TmQueryElement> tm_elements;

	// Kinematically drive a tire with a rotation angle
	std::vector<tmReal> theta;

	// flags
	eTMStatus tmstatus;		// tm database error flag
	eTireStatus tireStatus;		// tire-terrain interaction model error flag

		// decide how the geometry surface and terrain queries behave: these are set in tm.config
	int Geom_query_MODE;		// 0 = dynamic vis_grid (follows tire)
								// 1 = static vis_grid (does not move. Careful using this mode, it's slow)
								// 2 = make QT grid and Vis grid the same entity (fastest)
	int Tm_query_MODE;			// 0 = use CPU only for Bous-Cerruti
								// 1 = use GPU only for Bous-Cerruti
								// 2 = use both CPU, GPU, and compare stress values
	bool db_run;	// true if the db is still working on calcs, false once end_time is reached
	bool save_TMoutData;	// save the TM output data? off by default, enabled with the event receiver
	SaveTypes output_types;	// what output data to save with the terrain db

public:
	// buffers go here, so they can be called to read/write Terrain data
//	IrrlichtBuffer* irrBuff;	// read/write to this class when Irr wants to update the scene


	// @param tire_input	initial config. of tire, a single one of these is passed in.
	// @param config_filename	your *.tm file
	//						Tire specific data for each ptr is in a pre-allocated array, 
	//						whose length is the number of tires
	TerraMech(const TractionInput_t& tire_input, const std::string config_filename, const double CEstepSize):  
	  sim_time(0), num_steps(0), num_tires(tire_input.numTires), step_size(CEstepSize)
	{
		for(int tire_idx = 0; tire_idx < tire_input.numTires; tire_idx++) {
			// USING tm-API to Init. Terrain DB & simulation settings
			tmstatus = tm_init(tire_input, tire_idx, config_filename.c_str()); //"../data/config.tm");
			if( tmstatus ) {
				cout << "couldn't init tm: " << tmstatus << endl;
			}
			// as long as tm_init returns OK, set the tire data in the TerraMech object
			TmQueryElement curr_tire(TMVector(tire_input.tire_cm), TMQuaternion(tire_input.tire_Q),
				TMVector(tire_input.tire_cm_v), TMVector(tire_input.tire_cm_w) );
			this->tm_elements.push_back(curr_tire);
			this->theta.push_back(0.0);
		}
		this->tm_input_cfg = (TM_inputConfig*)(getInputConfig(0).inputConfig_ptr);
		// can fill some of the simulation settings now
		this->end_time = 99.9;	//tm_input_cfg.timeEnd;
		this->Geom_query_MODE = tm_input_cfg->GQ_Mode;
		this->Tm_query_MODE = tm_input_cfg->TM_Mode;


		// Irrlicht things
//		irrBuff = new IrrlichtBuffer();
		//	SaveData settings
		if (tm_input_cfg->saveData) {
			output_types = SaveTypes(
#ifdef _DEBUG
				false,				// save soil?
				false, false, false, // ttForces, tireNodes, alignedForce
				false, false,		 // visGrid, QTgrid
				false, false,		// soilCol, soilSlice
				false, false,		// bmp, soilVol
				false	);		// static soil col


	#else
				false,				// save soil?
				false, false, false, // ttForces, tireNodes, alignedForce
				false, false,		 // visGrid, QTgrid
				false, false,		// soilCol, soilSlice
				false, false,		// bmp, soilVol
				false	);		// static soil col
	#endif
		
		set_SaveBools(output_types);	
//		set_static_saveLocation(to_C_vect(tire_input.tire_cm) );
		// turn off all output data while visualizing, but still save the wheel state (pos,vel,F/M) data
		this->save_TMoutData = true;
//		outData->resetSaveBools(false, false, false, false, false, false, false, false, false);
		}

		// db is ready to go. This goes to false when the maximum time is exceeded
		this->db_run = true;
	}

	// @param tire_input	array of initial configs
	// @param config_filename	your *.tm file
	//						Tire specific data for each ptr is in a pre-allocated array, 
	//						whose length is the number of tires
	TerraMech(std::vector<TractionInput_t> tire_input, const std::string config_filename, const double CEstepSize):  
	  sim_time(0), num_steps(0), num_tires(tire_input.size()), step_size(CEstepSize)
	{
		for(int tire_idx = 0; tire_idx < tire_input.size(); tire_idx++) {
			// USING tm-API to Init. Terrain DB & simulation settings
			tmstatus = tm_init(tire_input[tire_idx], tire_idx, config_filename.c_str()); //"../data/config.tm");
			if( tmstatus ) {
				cout << "couldn't init tm: " << tmstatus << endl;
			}
			// as long as tm_init returns OK, set the tire data in the TerraMech object
			TmQueryElement curr_tire(TMVector(tire_input[tire_idx].tire_cm), TMQuaternion(tire_input[tire_idx].tire_Q),
				TMVector(tire_input[tire_idx].tire_cm_v), TMVector(tire_input[tire_idx].tire_cm_w) );
			this->tm_elements.push_back(curr_tire);
			this->theta.push_back(0.0);
		}
		this->tm_input_cfg = (TM_inputConfig*)(getInputConfig(0).inputConfig_ptr);
		// can fill some of the simulation settings now
		this->end_time = 99.9;	//tm_input_cfg.timeEnd;
		this->Geom_query_MODE = tm_input_cfg->GQ_Mode;
		this->Tm_query_MODE = tm_input_cfg->TM_Mode;


		// Irrlicht things
//		irrBuff = new IrrlichtBuffer();
		//	SaveData settings
		if (tm_input_cfg->saveData) {
			output_types = SaveTypes(
#ifdef _DEBUG
				false,				// save soil?
				false, false, false, // ttForces, tireNodes, alignedForce
				false, false,		 // visGrid, QTgrid
				false, false,		// soilCol, soilSlice
				false, false,		// bmp, soilVol
				false	);		// static soil col


	#else
				false,				// save soil?
				false, false, false, // ttForces, tireNodes, alignedForce
				false, false,		 // visGrid, QTgrid
				false, false,		// soilCol, soilSlice
				false, false,		// bmp, soilVol
				false	);		// static soil col
	#endif
		
		set_SaveBools(output_types);	
//		set_static_saveLocation(to_C_vect(tire_input.tire_cm) );
		// turn off all output data while visualizing, but still save the wheel state (pos,vel,F/M) data
		this->save_TMoutData = true;
//		outData->resetSaveBools(false, false, false, false, false, false, false, false, false);
		}

		// db is ready to go. This goes to false when the maximum time is exceeded
		this->db_run = true;
	}

	const tmReal get_simTime() { return this->sim_time; }

	~TerraMech( )
	{
		// shutdown the tire 
		tireStatus = tire_shutdown();
		// and do the same for the terramechanics
		tm_shutdown();
		// delete any irrlicht stuff
//		delete irrBuff;
	}

	// each time step, apply the wheel forces from the contact patch for a rigid wheel
	// directly to the wheel body
	void apply_TM_FT( ChSharedPtr<ChBody>& wheel_body, const int tire_idx ){

		wheel_body->Empty_forces_accumulators();
		// should apply this to the CM location
		wheel_body->Accumulate_force( TerraMech::SAE_to_SI_Force(this->tm_elements[tire_idx].Force), chrono::ChVector<>(0.0,0.0,0.0),1);
		wheel_body->Accumulate_torque( TerraMech::SAE_to_SI_Torque(this->tm_elements[tire_idx].Torque),1);

	}

	// go thru each individual force in the contact patch, add the force/torque via Chrono accumulators
	void apply_TM_FT_accum( ChSharedPtr<ChBody>& wheel_body, const int tire_idx ){
		// use each force from the contact patch individually
		// need the tire pointer to directly use force
		TM_TractionElement* tire_ptr = (TM_TractionElement*)(getTElement(tire_idx).tmTire_ptr);
		// also, empty the accumulator before incrementing
		wheel_body->Empty_forces_accumulators();
		for( int idx = 0; idx < tire_ptr->tireForce.size(); idx++){
			TMVector curr_pos = tire_ptr->contactPos[idx];
			TMVector curr_force = tire_ptr->tireForce[idx];
			chrono::ChVector<> Ch_curr_f = TerraMech::SAE_to_SI_Force(curr_force);
			chrono::ChVector<> Ch_pos = TerraMech::SAE_to_SI_Coords(curr_pos);
			// should be OK to increment the accumulator, in the local ref. frame
			wheel_body->Accumulate_force(Ch_curr_f, Ch_pos,0);
		}
	}

	// update the TM query element from the Chrono rigid body
	// Note: have to transform from SI to SAE units for length, orientation
	//			Also have to deal with y-pos = up to z-negative = up, SI to SAE coord systems
	// replaces tire_displacement:
		//	tire_displacement(num_steps, step_size, 
		//		input_cfg.dz, input_cfg.dx, input_cfg.slipPercent, input_cfg.dia,
		//		theta[t_idx], curr_elem->pos, curr_elem->Eangs, curr_elem->vel, curr_elem->omega );
	void update_TMwheel_fromChrono(ChSharedPtr<ChBody>& chrono_body, TmQueryElement* curr_elem){
		// need to convert each Chrono vect to a TM vect, then do the correct unit conversion
		curr_elem->pos = TerraMech::SI_to_SAE_Coords( chrono_body->GetPos()  );
		curr_elem->vel = TerraMech::SI_to_SAE_Coords( chrono_body->GetPos_dt() );
		// set the element with the right axes. first convert to YPR
		ChVector<> ypr = chrono::Q_to_NasaAngles(curr_elem->rot);
		curr_elem->rot = chrono::Q_from_NasaAngles(ChVector<>(ypr.z, -ypr.x, -ypr.y));
		// same thing for omega: change the axes directly
		ChVector<> local_omega = chrono_body->GetWvel_loc();
		curr_elem->omega = TMVector(local_omega.z, -local_omega.x, -local_omega.y);
	}

	// @brief when no CE body is passed, drive the wheel kinematically
	void do_TM_Step(const double CEsimTime, const double CEstepSize, const TMVector& )
	{
		for(size_t t_idx = 0; t_idx < this->num_tires; t_idx++) {

			// start the tm_query
			// tm_geom_query, tm_CD_query, tm_calcDeform
			tmstatus = tire_timestep_function(sim_time, CEstepSize, t_idx,
				&to_C_vect(this->tm_elements[t_idx].pos), &to_C_quat(this->tm_elements[t_idx].rot),
				&to_C_vect(this->tm_elements[t_idx].vel), &to_C_vect(this->tm_elements[t_idx].omega),
				&to_C_vect(this->tm_elements[t_idx].Force), &to_C_vect(this->tm_elements[t_idx].Torque) );

			if( tmstatus ) {
				cerr << "tire timestep failed, tire #: " << 0 
					<< " code : " << tmstatus << endl;
			}
			// data for the query elements should all have been updated by
			// these two TM API calls;
			// I can kinematically drive things in Chrono, and get rid of
			//	tire_displacement
		}
		// increment sim time, counters and whatnot
		sim_time = CEsimTime;
		num_steps++;
		// save data before going to next time step, if desired
		// save data and output some info to the console
		if ( num_steps  % tm_input_cfg->saveFileIncr == 0) {
			cout << "---- end of Step # " << num_steps << "  , time = " << CEsimTime << ",   SUMMARY:" << endl;
			cout << " ** Tire CM Pos. = " << this->tm_elements[0].pos << "\n ** orientation (YPR) = " <<  TMVector(this->tm_elements[0].rot.Q_to_NasaAngles() ) 
				<< " Vel. = " <<  this->tm_elements[0].vel << endl;
			// using m_configdata like this is unsafe
			cout << " ** Database SUMMARY: \n" << " numContacts = \t" <<  getNumContacts(0) << endl;
			cout << " Tire Spindle Rxn Force,Moment = \n" << this->tm_elements[0].Force
				<< endl << this->tm_elements[0].Torque << endl;
			// save the data to a file (note: some data is persistent through simulation, some is written for each time step)
			cout << endl;
		}

		// this is where I should write the new data to Irrlicht: use a double buffer 
//		irrBuff->writeToBuffer( (TM_TractionElement*)(getTirePtr(0).ptr_Info_t) ); 

		// finally, check to see if we've exceeded the max time, to set a flag that
		//	will tell everything to finish up and shut down
		if( sim_time > end_time)
			db_run = false;
	}

	// @brief using Chrono wheel body data is an INPUT to the TM
	//			so, read in data from this instead of running tire_displacement
	//			Note: pass in ALL the wheels here
	void do_TM_Step(std::vector<ChSharedPtr<ChBody>> mWheels, const double CEsimTime, const double CEstepSize)
	{
		// ensure we pass the right number of Chrono tire to TM tires
		if( mWheels.size() != this->num_tires) {
			cerr << "passed in more Chrono bodies than there are TM tires! " << endl;
			return;
		}

		// update the simulation time
		this->sim_time = CEsimTime;
		for(size_t t_idx = 0; t_idx < this->num_tires; t_idx++) {
			// now, kinematically move the tire to the next configuration
			TmQueryElement* curr_elem = &tm_elements[t_idx];

			// this replaces tire_displacement, e.g. the TM function that
			// kinematically moves the wheel geometry relative to the terrain
			this->update_TMwheel_fromChrono(mWheels[t_idx], curr_elem);

			TMCVector F_out, M_out;
			// tm_geom_query, tm_CD_query, tm_calcDeform are all contained in the high-level
			tmstatus = tire_timestep_function(sim_time, CEstepSize, t_idx,
				&to_C_vect(this->tm_elements[t_idx].pos), &to_C_quat(this->tm_elements[t_idx].rot),
				&to_C_vect(this->tm_elements[t_idx].vel), &to_C_vect(this->tm_elements[t_idx].omega),
				&F_out, &M_out );

			this->tm_elements[t_idx].Force = TerraMech::SAE_to_SI_Force(F_out);
			this->tm_elements[t_idx].Torque = TerraMech::SAE_to_SI_Torque(M_out);

			if( tmstatus ) {
				cerr << "tire timestep failed, tire #: " << 0 
					<< " code : " << tmstatus << endl;
			}
			// data for the query elements should all have been updated by
			// these two TM API calls; apply the force/torque directly.
			// Alternatively, pass in tire ptrs, accumulate force
//			this->apply_TM_FT( mWheels[t_idx], t_idx );

			this->apply_TM_FT_accum( mWheels[t_idx], t_idx);
		}
		// increment sim time, counters and whatnot
		sim_time = CEsimTime;
		num_steps++;
		// save data before going to next time step, if desired
		// save data and output some info to the console
//		if ( num_steps  % tm_input_cfg.saveFileIncr == 0) {
			cout << "---- end of Step # " << num_steps << "  , time = " << sim_time << ",   SUMMARY:" << endl;
			cout << " ** Tire 0 CM/YPR/vel "<< this->tm_elements[0].pos << endl << this->tm_elements[0].rot.Q_to_NasaAngles() << endl
				<< this->tm_elements[0].vel << endl;
			// using m_configdata like this is unsafe
			cout << " ** Database SUMMARY: \n" << " numContacts = \t" <<  getNumContacts(0) << endl;
			cout << " Tire Spindle Rxn Force,Moment = \n" << this->tm_elements[0].Force
				<< endl << this->tm_elements[0].Torque << endl;
			// save the data to a file (note: some data is persistent through simulation, some is written for each time step)
			cout << endl;
//		}

		// this is where I should write the new data to Irrlicht: use a double buffer 
//		irrBuff->writeToBuffer( (TM_TractionElement*)(getTirePtr(0).ptr_Info_t) ); 

		// finally, check to see if we've exceeded the max time, to set a flag that
		//	will tell everything to finish up and shut down
		if( sim_time > end_time)
			db_run = false;
	}

	const TM_inputConfig* get_InputConfig()
	{
		return this->tm_input_cfg;
	}

	// is the tm database still running? true if sim_time <= end_time
	bool run()
	{
		return this->db_run;
	}

	// halt
	void stop()
	{
		this->db_run = false;
	}

	// accessors. These return the raw values, in SAE units
	TMVector get_tire_cm(const int idx){
		return tm_elements[idx].pos;
	}

	TMQuaternion get_tire_rot(const int idx) {
		return tm_elements[idx].rot;
	}

	TMVector get_tire_cm_vel(const int idx) {
		return tm_elements[idx].vel;
	}

	// @brief return the tire error status
	eTireStatus getTireStatus(void) {
		return this->tireStatus;
	}

	// @brief turn the tm error status
	eTMStatus getTmStatus(void) {
		return this->tmstatus;
	}

	// @brief return the latest computed wheel spindle torque
	TMVector getSpindleTorque(const uint64_t tireIdx){
		if( tireIdx > this->num_tires ) {
			cerr << " getSpindleF() error, tire Idx: " << tireIdx << " > numTires " << endl;
		}
		// all the info in tm_elements array is in SAE units, convert to SI
		ChVector<> m_torque = TerraMech::SAE_to_SI_Torque( this->tm_elements[tireIdx].Torque);
		return m_torque;
	}

	// @brief return the latest computed wheel spindle force
	ChVector<> getSpindleForce(const uint64_t tireIdx){
		if( tireIdx > this->num_tires ) {
			cerr << "need to init. tires before querying the force" << endl;
		}
		// all the info in tm_elements array is in SAE units, convert to SI
		ChVector<> m_force = TerraMech::SAE_to_SI_Force( this->tm_elements[tireIdx].Force );
		return m_force;
	}

	bool get_saveTMoutData() {
		return this->save_TMoutData;
	}

	void set_saveTMoutData(bool save) {
		this->save_TMoutData = save;
	}

	// helper functions go here
	// SAE: z- = up, SI: y+ = up
	// going from RH SAE coord sys, can just swap SAE y to SI z. Then, SI y is -z from SAE
	// easy way to remember this: SI to SAE: negative y to get z
	//							  SAE to SI: negative z to get y
	//							  x stays the same

	// length: TM [in] to Ch [m]
	static ChVector<> SAE_to_SI_Coords(const TMVector& vect) {
		ChVector<> out = TMVector();
		out.x = -vect.y * 0.0254;
		out.y = -vect.z * 0.0254;
		out.z = vect.x * 0.0254;
		return out;
	}

	// length: Ch [m] to TM [in]
	static TMVector SI_to_SAE_Coords(const ChVector<>& vect) {
		TMVector out = TMVector();
		tmReal invConv = 1.0/0.0254;
		out.x = vect.z * invConv;
		out.y = -vect.x * invConv;
		out.z = -vect.y * invConv;
		return out;
	}

	// Force: TM [lb] to Ch [N]
	static ChVector<> SAE_to_SI_Force(const TMVector& vect) {
		ChVector<> out = ChVector<>();
		out.x = -vect.y * 6.9;
		out.y = -vect.z * 6.9;
		out.z = vect.x * 6.9;
		return out;
	}

	// Force: Ch [N] to TM [lb]
	static TMVector SI_to_SAE_Force(const ChVector<>& vect) {
		TMVector out = TMVector();
		tmReal invConv = 1.0/6.9;
		out.x = vect.z * invConv;
		out.y = -vect.x * invConv;
		out.z = -vect.y * invConv;
		return out;
	}

	// Torque: [lb-in] to [N-m]
	static ChVector<> SAE_to_SI_Torque(const TMVector& vect) {
		TMVector out = TMVector();
		out.x = -vect.y * 0.112985;
		out.y = -vect.z * 0.112985;
		out.z = vect.x * 0.112985;
		return out;
	}

	// Torque: [N-m] to [lb-in]
	static TMVector SI_to_SAE_Torque(const ChVector<>& vect) {
		TMVector out = TMVector();
		out.x = vect.z * 8.8507;
		out.y = -vect.x * 8.8507;
		out.z = -vect.y * 8.8507;
		return out;
	}

	// just swap axes for unitless stuff
	static ChVector<> SAE_to_SI_axes(const TMVector& vect) {
		ChVector<> out = ChVector<>();
		out.x = -vect.y;
		out.y = -vect.z;
		out.z = vect.x;
		return out;

	}

	// swap axes for SI to SAE axes
	static TMVector SI_to_SAE_axes(const ChVector<>& vect) {
		TMVector out = TMVector();
		out.x = vect.z;
		out.y = -vect.x;
		out.z = -vect.y;
		return out;
	}


};



#endif	// #ifdef ChronoTM_Module_H
