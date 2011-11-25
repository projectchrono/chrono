#ifndef CHCONTACTCONTAINERGPUSIMPLE_H
#define CHCONTACTCONTAINERGPUSIMPLE_H

///////////////////////////////////////////////////
//
//   ChContactContainerGPUsimple.h
//
//   Class for container of many contacts, as CPU
//   typical linked list of ChContactGPUsimple objects
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "physics/ChContactContainerBase.h"
#include <list>
#include "ChApiGPU.h"
namespace chrono
{
	/// Class representing a container of many contacts,
	/// implemented as a typical linked list of ChContactGPUsimple
	/// objects.
	/// This contact container must be used for the preliminar CUDA solver
	/// that was developed by Ale & Dan, but in future will be
	/// replaced by ChContactContainerGPU, and advanced container
	/// that does not use linked lists of cpu objects but rather
	/// keeps all contact data as GPU buffers on the GPU device.

	class ChApiGPU ChContactContainerGPUsimple : public ChContactContainerBase {
		CH_RTTI(ChContactContainerGPUsimple,ChContactContainerBase);

	protected:
		//
		// DATA
		//

		int n_added;

		double load_C; // <- buffered when doing ConstraintsBiLoad_C(), so that the GPU solver can get them because C computation is delegated to Dan's solver preprocessor.
		double load_max_recovery_speed; // "   "
		bool   load_do_clamp;

	public:
		//
		// CONSTRUCTORS
		//

		ChContactContainerGPUsimple ();

		virtual ~ChContactContainerGPUsimple ();

		//
		// FUNCTIONS
		//
		/// Tell the number of added contacts
		virtual int GetNcontacts  () {return n_added;}

		void SetNcontacts  (int contacts) {n_added=contacts;}

		/// Remove (delete) all contained contact data.
		virtual void RemoveAllContacts();

		/// The collision system will call BeginAddContact() before adding
		/// all contacts (for example with AddContact() or similar). Instead of
		/// simply deleting all list of the previous contacts, this optimized implementation
		/// rewinds the link iterator to begin and tries to reuse previous contact objects
		/// until possible, to avoid too much allocation/deallocation.
		virtual void BeginAddContact();

		/// Add a contact between two frames.
		virtual void AddContact(const collision::ChCollisionInfo& mcontact);

		/// The collision system will call BeginAddContact() after adding
		/// all contacts (for example with AddContact() or similar). This optimized version
		/// purges the end of the list of contacts that were not reused (if any).
		virtual void EndAddContact();

		/// Scans all the contacts and for each contact exacutes the ReportContactCallback()
		/// function of the user object inherited from ChReportContactCallback.
		/// Child classes of ChContactContainerBase should try to implement this (although
		/// in some highly-optimized cases as in ChContactContainerGPU it could be impossible to
		/// report all contacts).
		virtual void ReportAllContacts(ChReportContactCallback* mcallback);

		/// Access the C factor, as buffered last time ConstraintsBiLoad_C() executed.
		/// This is used only by the GPU solver because contact jacobian/residual computation is
		/// delegated to Dan's preprocessor in GPU solver.
		double Get_load_C_factor() {return load_C;}
		double Get_load_max_recovery_speed() {return load_max_recovery_speed;}
		bool   Get_load_do_clamp() {return load_do_clamp;}

		/// Tell the number of scalar bilateral constraints (actually, friction
		/// constraints aren't exactly as unilaterals, but count them too)
		virtual int GetDOC_d  () {return n_added * 3;}

		/// In detail, it computes jacobians, violations, etc. and stores
		/// results in inner structures of contacts.
		virtual void Update (double mtime);

		//
		// LCP INTERFACE
		//

		virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
		virtual void ConstraintsBiReset();
		virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
		//virtual void ConstraintsBiLoad_Ct(double factor=1.) {};
		virtual void ConstraintsLoadJacobians();
		virtual void ConstraintsLiLoadSuggestedSpeedSolution();
		virtual void ConstraintsLiLoadSuggestedPositionSolution();
		virtual void ConstraintsLiFetchSuggestedSpeedSolution();
		virtual void ConstraintsLiFetchSuggestedPositionSolution();
		virtual void ConstraintsFetch_react(double factor=1.);
	};

	//////////////////////////////////////////////////////
	//////////////////////////////////////////////////////
} // END_OF_NAMESPACE____

#endif