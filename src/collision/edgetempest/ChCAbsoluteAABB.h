#ifndef CHC_ABSOLUTEAABB_H
#define CHC_ABSOLUTEAABB_H

//////////////////////////////////////////////////
//  
//   ChCAbsoluteAABB.h
//
//   Header for defining axis-aligned bounding 
//   boxes in absolute space, to be used with
//   the 'sweep and prune' broad-phase
//   collision detection stage.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


namespace chrono 
{
namespace collision 
{



/// Class for end point, to be used for X or Y or Z lists of 
/// the 'sweep and prune' broad-phase collision.

template<class coll_model_type>
class ChIntervalEndpoint
{

  public:

		/// Constructor
    ChIntervalEndpoint(void):m_type(0),m_model(0){};

		/// Initialize
    void init(coll_model_type* model, const unsigned int & type)
				{
					assert(model);
					assert(type==0 || type==1);
					this->m_model = model;
					this->m_type = type;
				};

  public:

	double				m_value;  ///< scalar value of the endpoint.
	coll_model_type*	m_model;   ///< pointer to the body.
	unsigned int		m_type;   ///< start point=0; end point =1;

};





/// Header for defining AABB (axis-aligned bounding 
/// boxes) in absolute space, to be used with
/// the 'sweep and prune' broad-phase
/// collision detection stage. This AABB is made of eight
/// ChIntervalEndpoint() items. 

template<class coll_model_type>
class ChAbsoluteAABB
{
  public:

    ChAbsoluteAABB(void):m_model(0){};

    void init(coll_model_type* amodel)
    {
      if(!m_model)
      {
        m_model = amodel;
        m_beginX.init(amodel,0);
        m_beginY.init(amodel,0);
        m_beginZ.init(amodel,0);
        m_endX.init(amodel,1);
        m_endY.init(amodel,1);
        m_endZ.init(amodel,1);
      }
      assert(m_model);
    };

  public:

    coll_model_type* m_model;   ///< A pointer to the corresponding collision model this AABB encloses.

    ChIntervalEndpoint<coll_model_type> m_beginX;
    ChIntervalEndpoint<coll_model_type> m_beginY;
    ChIntervalEndpoint<coll_model_type> m_beginZ;
    ChIntervalEndpoint<coll_model_type> m_endX;
    ChIntervalEndpoint<coll_model_type> m_endY;
    ChIntervalEndpoint<coll_model_type> m_endZ;

};







} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
