%{

/* Includes the header in the wrapper code */

#include "chrono/particlefactory/ChRandomShapeCreator.h"
#include "chrono/particlefactory/ChRandomParticlePosition.h"
#include "chrono/particlefactory/ChRandomParticleAlignment.h"
#include "chrono/particlefactory/ChRandomParticleVelocity.h"
#include "chrono/particlefactory/ChParticleProcessor.h"
#include "chrono/particlefactory/ChParticleEventTrigger.h"
#include "chrono/particlefactory/ChParticleProcessEvent.h"
#include "chrono/particlefactory/ChParticleEmitter.h"
#include "chrono/particlefactory/ChParticleRemover.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChMathematics.i"

%shared_ptr(chrono::particlefactory::ChParticleEventTriggerBox)
%shared_ptr(chrono::particlefactory::ChParticleEventTriggerNever)
%shared_ptr(chrono::particlefactory::ChParticleEventTrigger)
%shared_ptr(chrono::particlefactory::ChParticleEventFlowInRectangle)
%shared_ptr(chrono::particlefactory::ChParticleProcessEvent)
%shared_ptr(chrono::particlefactory::ChParticleProcessEventDoNothing)
%shared_ptr(chrono::particlefactory::ChParticleProcessEventRemove)
%shared_ptr(chrono::particlefactory::ChParticleProcessEventCount)
%shared_ptr(chrono::particlefactory::ChParticleProcessEventMassCount)
%shared_ptr(chrono::particlefactory::ChParticleProcessEventMassDistribution)
%shared_ptr(chrono::particlefactory::ChRandomParticlePosition)
%shared_ptr(chrono::particlefactory::ChRandomParticlePositionRectangleOutlet)
%shared_ptr(chrono::particlefactory::ChRandomParticlePositionOnGeometry)
%shared_ptr(chrono::particlefactory::ChRandomParticleAlignment)
%shared_ptr(chrono::particlefactory::ChRandomParticleVelocity)
%shared_ptr(chrono::particlefactory::ChRandomParticleAlignmentUniform)
%shared_ptr(chrono::particlefactory::ChRandomParticleVelocityConstantDirection)
%shared_ptr(chrono::particlefactory::ChRandomParticleVelocityAnyDirection)
%shared_ptr(chrono::particlefactory::ChRandomShapeCreator)
%shared_ptr(chrono::particlefactory::ChRandomShapeCreatorSpheres)
%shared_ptr(chrono::particlefactory::ChRandomShapeCreatorBoxes)
%shared_ptr(chrono::particlefactory::ChRandomShapeCreatorCylinders)
%shared_ptr(chrono::particlefactory::ChRandomShapeCreatorConvexHulls)
%shared_ptr(chrono::particlefactory::ChRandomShapeCreatorShavings)
%shared_ptr(chrono::particlefactory::ChRandomShapeCreatorFromFamilies)
%shared_ptr(chrono::particlefactory::ChRandomShapeCreator::AddBodyCallback)


%ignore chrono::particlefactory::_particle_last_pos;

/* Parse the header file to generate wrappers */
%feature("director") chrono::particlefactory::ChRandomShapeCreator::AddBodyCallback;
%rename("ChRandomShapeCreator_AddBodyCallback") chrono::particlefactory::ChRandomShapeCreator::AddBodyCallback;
%include "../../chrono/particlefactory/ChRandomShapeCreator.h"
%include "../../chrono/particlefactory/ChRandomParticlePosition.h"
%include "../../chrono/particlefactory/ChRandomParticleAlignment.h"
%include "../../chrono/particlefactory/ChRandomParticleVelocity.h"
%include "../../chrono/particlefactory/ChParticleEventTrigger.h"
%include "../../chrono/particlefactory/ChParticleProcessEvent.h"
%include "../../chrono/particlefactory/ChParticleProcessor.h"
%include "../../chrono/particlefactory/ChParticleEmitter.h"
%include "../../chrono/particlefactory/ChParticleRemover.h"



%extend chrono::particlefactory::ChParticleEmitter{
		public:
			void SetParticlesPerSecond(double pps){
			   $self->ParticlesPerSecond() = pps;
			   }

			void SetMassPerSecond(double mps){
			   $self->MassPerSecond() = mps;
			   }

			void SetParticleReservoirAmount(int pra){
			   $self->ParticleReservoirAmount() = pra;
			   }

			void SetMassReservoirAmount(int mra){
			   $self->MassReservoirAmount() = mra;
			   }

			double GetParticlesPerSecond(){
			   double pps = $self->ParticlesPerSecond();
			   return pps;
			   }

			double GetMassPerSecond(){
			   double mps = $self->MassPerSecond();
			   return mps;
			   }

			int GetParticleReservoirAmount(){
			   int pra = $self->ParticleReservoirAmount();
			   return pra;
			   }

			int GetMassReservoirAmount(){
			   int mra = $self->MassReservoirAmount();
			   return mra;
			   }
		};





