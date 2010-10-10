#ifndef CHDISPLAYTOOLS_H
#define CHDISPLAYTOOLS_H

//////////////////////////////////////////////////
//
//   ChDisplayTools.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Some functions to allow easy display of symbols
//   (contact points, normals, bounding boxes etc.)
//   when using Irrlicht+ChronoEngine for 3D view.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <irrlicht.h>
#include "physics/ChSystem.h"
#include "physics/ChFunction.h"
#include "physics/ChContactContainer.h"
#include "lcp/ChLcpIterativeSolver.h"



namespace irr
{


namespace core
{
/// Utility class to easily convert a Chrono::Engine vector into
/// an Irrlicht vector3df. Simply create an Irrlicht compatible
/// vector as:    myvector=vector3dfCH(mychronovector);
class vector3dfCH : public vector3df
{
public:
	vector3dfCH(chrono::ChVector<>& mch) { X=((f32)mch.x); Y=((f32)mch.y); Z=((f32)mch.z);};
};

}

/// Class with static functions which help with the integration
/// of Chrono::Engine and Irrlicht 3D rendering library.

class ChIrrTools
{
public: 

			/// Function to align an Irrlicht object to a Chrono::Engine 
			/// coordsys:

	static void alignIrrlichtNodeToChronoCsys(scene::ISceneNode* mnode, chrono::ChCoordsys<>& mcoords)
	{
		// Output: will be an Irrlicht 4x4 matrix
			core::matrix4 irrMat;
			
			// Get the rigid body actual rotation, as a 3x3 matrix [A]
			chrono::ChMatrix33<> chMat(mcoords.rot);//GetBody()->GetA();
			
			// Fill the upper 3x3 submatrix with the [A] matrix
			// transposed, since Irrlicht uses the row-major style as in D3D
			irrMat[0] = (irr::f32)chMat.GetElementN(0);
			irrMat[1] = (irr::f32)chMat.GetElementN(3);
			irrMat[2] = (irr::f32)chMat.GetElementN(6);

			irrMat[4] = (irr::f32)chMat.GetElementN(1);
			irrMat[5] = (irr::f32)chMat.GetElementN(4);
			irrMat[6] = (irr::f32)chMat.GetElementN(7);
			
			irrMat[8] = (irr::f32)chMat.GetElementN(2);
			irrMat[9] = (irr::f32)chMat.GetElementN(5);
			irrMat[10]= (irr::f32)chMat.GetElementN(8);

			irrMat[12]= (irr::f32)mcoords.pos.x;
			irrMat[13]= (irr::f32)mcoords.pos.y;
			irrMat[14]= (irr::f32)mcoords.pos.z;

			// Clear the last column to 0 and set low-right corner to 1 
			// as in Denavitt-Hartemberg matrices, transposed.
			irrMat[3] = irrMat[7] = irrMat[11] = 0.0f;
			irrMat[15] = 1.0f;

			// Set position and rotation of node using the 4x4 Irrlicht matrix.
			mnode->setPosition(irrMat.getTranslation());
			mnode->setRotation(irrMat.getRotationDegrees());
	}

			/// Easy-to-use function which draws contact points used 
			/// by a ChSystem in the current Irrlicht viewer (the 
			/// IVideoDriver). Small lines are drawn, of length mlen,
			/// aligned to contact normals.

		enum eCh_ContactsDrawMode{
						 CONTACT_NORMALS = 0,	// draw normals
						 CONTACT_DISTANCES,		// draw lines connecting two contact points
						 CONTACT_FORCES_N,		// draw contact forces (only normal component) applied to 2nd body
						 CONTACT_FORCES,		// draw contact forces applied to 2nd body
						 CONTACT_NONE};			// draw nothing


	static int drawAllContactPoints(chrono::ChSystem& mphysicalSystem, 
							 video::IVideoDriver* driver, 
							double mlen=1.0, 
							eCh_ContactsDrawMode drawtype = CONTACT_NORMALS)
	{ 
			if (drawtype==CONTACT_NONE) 
				return 0;
			if (mphysicalSystem.GetNcontacts()==0)
				return 0;

			driver->setTransform(video::ETS_WORLD, core::matrix4());
			video::SMaterial mattransp;
			mattransp.ZBuffer= false;
			mattransp.Lighting=false;
			driver->setMaterial(mattransp);

			class _draw_reporter_class : public chrono::ChReportContactCallback 
			{
			public:
				virtual bool ReportContactCallback (
								const chrono::ChVector<>& pA,				
								const chrono::ChVector<>& pB,				
								const chrono::ChMatrix33<>& plane_coord,	
								const double& distance,				
								const float& mfriction,			  	
								const chrono::ChVector<>& react_forces,		
								const chrono::ChVector<>& react_torques,	
								chrono::collision::ChCollisionModel* modA,	
								chrono::collision::ChCollisionModel* modB) 
				{
					chrono::ChMatrix33<>& mplanecoord = const_cast<chrono::ChMatrix33<>&>(plane_coord);
					chrono::ChVector<> v1=pA;
					chrono::ChVector<> v2;
					chrono::ChVector<> vn = mplanecoord.Get_A_Xaxis();
					if(drawtype==CONTACT_DISTANCES)
						v2=pB;
					if(drawtype==CONTACT_NORMALS)
						v2=pA + vn * clen;
					if(drawtype==CONTACT_FORCES_N)
						v2=pA + vn * clen * react_forces.x;
					if(drawtype==CONTACT_FORCES)
					{
						v2=pA + (mplanecoord*(react_forces*clen));
					}
					video::SColor mcol;
					if (distance > 0.0) 
						mcol=video::SColor(200,20,255,0);  // green: non penetration
					else
						mcol=video::SColor(200,255,60,60); // red: penetration
					this->cdriver->draw3DLine(core::vector3dfCH(v1), core::vector3dfCH(v2), mcol );
					return true; // to continue scanning contacts
				}
				// Data 
				video::IVideoDriver* cdriver;

				double clen;
				eCh_ContactsDrawMode drawtype;
			};

			_draw_reporter_class my_drawer;

			my_drawer.cdriver = driver;
			my_drawer.clen = mlen;
			my_drawer.drawtype = drawtype;

			// scan all contacts
			mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_drawer);

			return 0;
	}

			/// Easy-to-use function which draws contact informations 
			/// as labels at the contact point

	enum eCh_ContactsLabelMode{
						 CONTACT_DISTANCES_VAL,		// print distance
						 CONTACT_FORCES_VAL,		// print contact forces modulus
						 CONTACT_FORCES_N_VAL,		// print contact forces (only normal component)
						 CONTACT_FORCES_T_VAL,		// print contact forces (only tangential component)
						 CONTACT_TORQUES_VAL,		// print contact torques modulus
						 CONTACT_TORQUES_S_VAL,		// print contact torques modulus (only spinning component)
						 CONTACT_TORQUES_R_VAL,		// print contact torques modulus (only rolling component)
						 CONTACT_NONE_VAL};			// print nothing


	static int drawAllContactLabels(chrono::ChSystem& mphysicalSystem, 
							irr::IrrlichtDevice* device,  
							eCh_ContactsLabelMode labeltype = CONTACT_FORCES_N_VAL,
							video::SColor mcol = video::SColor(255,255,255,255) )
	{ 
			if (labeltype==CONTACT_NONE_VAL) 
				return 0;
			if (mphysicalSystem.GetNcontacts()==0)
				return 0;

			class _label_reporter_class : public chrono::ChReportContactCallback 
			{
			public:
				virtual bool ReportContactCallback (
								const chrono::ChVector<>& pA,				
								const chrono::ChVector<>& pB,				
								const chrono::ChMatrix33<>& plane_coord,	
								const double& distance,				
								const float& mfriction,			  	
								const chrono::ChVector<>& react_forces,		
								const chrono::ChVector<>& react_torques,	
								chrono::collision::ChCollisionModel* modA,	
								chrono::collision::ChCollisionModel* modB) 
				{
					char buffer[25];
					core::vector3df mpos((irr::f32)pA.x, (irr::f32)pA.y, (irr::f32)pA.z);
					core::position2d<s32> spos = this->cdevice->getSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
					gui::IGUIFont* font = this->cdevice->getGUIEnvironment()->getBuiltInFont();
					if (labeltype==CONTACT_DISTANCES_VAL)
					{
						sprintf(buffer,"% 5.3g", distance ); 
					}
					if (labeltype==CONTACT_FORCES_N_VAL)
					{
						sprintf(buffer,"% 5.3g", react_forces.x); 
					}
					if (labeltype==CONTACT_FORCES_T_VAL)
					{
						sprintf(buffer,"% 5.3g", chrono::ChVector<>(0,react_forces.y,react_forces.z).Length() ); 
					}
					if (labeltype==CONTACT_FORCES_VAL)
					{
						sprintf(buffer,"% 5.3g", chrono::ChVector<>(react_forces).Length() ); 
					}
					if (labeltype==CONTACT_TORQUES_VAL)
					{
						sprintf(buffer,"% 5.3g", chrono::ChVector<>(react_torques).Length() ); 
					}
					if (labeltype==CONTACT_TORQUES_S_VAL)
					{
						sprintf(buffer,"% 5.3g", react_torques.x); 
					}
					if (labeltype==CONTACT_TORQUES_R_VAL)
					{
						sprintf(buffer,"% 5.3g",  chrono::ChVector<>(0,react_torques.y,react_torques.z).Length() ); 
					}
					font->draw(core::stringw(buffer).c_str(), 
						core::rect<s32>(spos.X-15,spos.Y, spos.X+15, spos.Y+10), this->ccol);
					return true; // to continue scanning contacts
				}
				// Data 
				irr::IrrlichtDevice* cdevice;
				eCh_ContactsLabelMode labeltype;
				video::SColor ccol;
			};

			_label_reporter_class my_label_rep;

			my_label_rep.cdevice = device;
			my_label_rep.ccol = mcol;
			my_label_rep.labeltype = labeltype;

			// scan all contacts
			mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_label_rep);

			return 0;
	}

			/// Easy-to-use function which draws collision objects bounding boxes 
			/// for rigid bodies - if they have a collision shape.

	static int drawAllBoundingBoxes(chrono::ChSystem& mphysicalSystem, 
							video::IVideoDriver* driver)
	{ 
			if (mphysicalSystem.GetNbodiesTotal()==0)
				return 0;

			driver->setTransform(video::ETS_WORLD, core::matrix4());
			video::SMaterial mattransp;
			mattransp.ZBuffer= true;
			mattransp.Lighting=false;
			driver->setMaterial(mattransp);

			chrono::ChSystem::IteratorBodies myiter = mphysicalSystem.IterBeginBodies();
			while (myiter != mphysicalSystem.IterEndBodies())
			{
				if ((*myiter)->GetCollisionModel())
				{
					video::SColor mcol;
					if ((*myiter)->GetSleeping()) 
						mcol=video::SColor(70,0,50,255);  // blue: sleeping
					else
						mcol=video::SColor(70,30,200,200);  // cyan: not sleeping
					chrono::ChVector<> hi = chrono::VNULL;
					chrono::ChVector<> lo = chrono::VNULL;
					(*myiter)->GetCollisionModel()->GetAABB(lo,hi);
					driver->draw3DLine(core::vector3dfCH(lo), core::vector3dfCH(chrono::ChVector<>(hi.x,lo.y,lo.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(lo), core::vector3dfCH(chrono::ChVector<>(lo.x,hi.y,lo.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(lo), core::vector3dfCH(chrono::ChVector<>(lo.x,lo.y,hi.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(hi), core::vector3dfCH(chrono::ChVector<>(hi.x,hi.y,lo.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(hi), core::vector3dfCH(chrono::ChVector<>(lo.x,hi.y,hi.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(hi), core::vector3dfCH(chrono::ChVector<>(hi.x,lo.y,hi.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(chrono::ChVector<>(lo.x,lo.y,hi.z)), core::vector3dfCH(chrono::ChVector<>(hi.x,lo.y,hi.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(chrono::ChVector<>(lo.x,lo.y,hi.z)), core::vector3dfCH(chrono::ChVector<>(lo.x,hi.y,hi.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(chrono::ChVector<>(lo.x,hi.y,lo.z)), core::vector3dfCH(chrono::ChVector<>(lo.x,hi.y,hi.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(chrono::ChVector<>(lo.x,hi.y,lo.z)), core::vector3dfCH(chrono::ChVector<>(hi.x,hi.y,lo.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(chrono::ChVector<>(hi.x,lo.y,lo.z)), core::vector3dfCH(chrono::ChVector<>(hi.x,lo.y,hi.z)), mcol );
					driver->draw3DLine(core::vector3dfCH(chrono::ChVector<>(hi.x,lo.y,lo.z)), core::vector3dfCH(chrono::ChVector<>(hi.x,hi.y,lo.z)), mcol );
				}
				++myiter;
			}
			return 0;
	}




	static void drawHUDviolation(video::IVideoDriver* driver, IrrlichtDevice* mdevice,
								chrono::ChSystem& asystem, 
								int mx=10, int my=290, int sx=300, int sy=100, double spfact=100.0, double posfact=500.0
					)
	{
			if (asystem.GetLcpSolverType()!=chrono::ChSystem::LCP_SIMPLEX)
			{
				chrono::ChLcpIterativeSolver* msolver_speed = (chrono::ChLcpIterativeSolver*)asystem.GetLcpSolverSpeed();
				chrono::ChLcpIterativeSolver* msolver_stab  = (chrono::ChLcpIterativeSolver*)asystem.GetLcpSolverStab();
				msolver_speed->SetRecordViolation(true);
				msolver_stab->SetRecordViolation(true);

				core::rect<s32> mrect(mx, my, mx+sx, my+sy);
				driver->draw2DRectangle(video::SColor(100,200,200,230),
						mrect);
				for (unsigned int i=0; i<msolver_speed->GetViolationHistory().size(); i++)
				{
					driver->draw2DRectangle(video::SColor(90,255,0,0),
						core::rect<s32>(mx+i*4,       sy+my -(int)(spfact*msolver_speed->GetViolationHistory()[i]), 
						                mx+(i+1)*4-1, sy+my),
										&mrect);
				}
				for (unsigned int i=0; i<msolver_speed->GetDeltalambdaHistory().size(); i++)
				{
					driver->draw2DRectangle(video::SColor(100,255,255,0),
						core::rect<s32>(mx+i*4,       sy+my -(int)(spfact*msolver_speed->GetDeltalambdaHistory()[i]), 
						                mx+(i+1)*4-2, sy+my),
										&mrect);
				}
				for (unsigned int i=0; i<msolver_stab->GetViolationHistory().size(); i++)
				{
					driver->draw2DRectangle(video::SColor(90,0,255,0),
						core::rect<s32>(mx+sx/2+i*4,       sy+my -(int)(posfact*msolver_stab->GetViolationHistory()[i]), 
						                mx+sx/2+(i+1)*4-1, sy+my),
										&mrect);
				}
				for (unsigned int i=0; i<msolver_stab->GetDeltalambdaHistory().size(); i++)
				{
					driver->draw2DRectangle(video::SColor(100,0,255,255),
						core::rect<s32>(mx+sx/2+i*4,       sy+my -(int)(posfact*msolver_stab->GetDeltalambdaHistory()[i]), 
						                mx+sx/2+(i+1)*4-2, sy+my),
										&mrect);
				}
				if (mdevice->getGUIEnvironment())
				{
					gui::IGUIFont* font = mdevice->getGUIEnvironment()->getBuiltInFont();
					if (font)
					{
						char buffer[100];
						font->draw(L"LCP speed violation", 
						 core::rect<s32>(mx+sx/2-100,my, mx+sx, my+10), video::SColor(200,100,0,0));
						sprintf(buffer,"%g",sy/spfact); 
						font->draw(core::stringw(buffer).c_str(), 
						 core::rect<s32>(mx,my, mx+30, my+10), video::SColor(200,100,0,0));
						font->draw(L"LCP position violation", 
						 core::rect<s32>(mx+sx-100,my, mx+sx, my+10),   video::SColor(200,0,100,0));
						sprintf(buffer,"%g",sy/posfact); 
						font->draw(core::stringw(buffer).c_str(), 
						 core::rect<s32>(mx+sx/2,my, mx+sx/2+10, my+10), video::SColor(200,0,100,0));
					}
				}
			}
	}

	
	static void drawChFunction( IrrlichtDevice* mdevice,
								chrono::ChFunction* fx, 
								double xmin= 0, double xmax =1,
								double ymin=-1, double ymax =1,
								int mx=10, int my=290, int sx=300, int sy=100
					)
	{
			irr::video::IVideoDriver* driver = mdevice->getVideoDriver();

			if (!fx) return;

			core::rect<s32> mrect(mx, my, mx+sx, my+sy);
				driver->draw2DRectangle(video::SColor(100,200,200,230),
						mrect);
			
			if (mdevice->getGUIEnvironment())
			{
				gui::IGUIFont* font = mdevice->getGUIEnvironment()->getBuiltInFont();
				if (font)
				{
					char buffer[100];
					sprintf(buffer,"%g",ymax);
					font->draw(core::stringw(buffer).c_str(), 
					 core::rect<s32>(mx,my, mx+sx, my+10), video::SColor(200,100,0,0));
					sprintf(buffer,"%g",ymin);
					font->draw(core::stringw(buffer).c_str(), 
					 core::rect<s32>(mx,my+sy, mx+sx, my+sy+10), video::SColor(200,100,0,0));
					if ((ymin <0 )&&(ymax>0))
					{
						int yzero =  my+sy- (int) ((( -ymin)/(ymax-ymin)) * (double) sy )  ;
						driver->draw2DLine(irr::core::position2d<s32>(mx,yzero), 
									   irr::core::position2d<s32>(mx+sx, yzero), 
									   video::SColor(90,255,255,255));
						font->draw(core::stringw(buffer).c_str(), 
							core::rect<s32>(mx,my+yzero, mx+sx, my+yzero+10), video::SColor(200,100,0,0));
					}
				}
			}

			int prevx = 0; int prevy = 0;
			for (int ix = 0; ix < sx; ix++)
			{
				double x = xmin + (xmax-xmin)*((double)(ix))/(double)(sx);
				double y = fx->Get_y(x);
				int py = my+sy- (int) (((y -ymin)/(ymax-ymin)) * (double) sy )  ;
				int px = mx+ix;
				if(ix>0)
					driver->draw2DLine(irr::core::position2d<s32>(px,py), 
									   irr::core::position2d<s32>(prevx, prevy), 
									   video::SColor(200,255,0,0));
				prevx = px; prevy=py;
			}

			
	}



			/// Easy-to-use function to draw segment lines in 3D space, 
			/// with given colour.

	static void drawSegment(video::IVideoDriver* driver,
					chrono::ChVector<> mstart,
					chrono::ChVector<> mend,
					video::SColor mcol = video::SColor(255,0,0,0),
					bool use_Zbuffer = false
					)
	{ 
			driver->setTransform(video::ETS_WORLD, core::matrix4());
			video::SMaterial mattransp;
			mattransp.ZBuffer= use_Zbuffer;
			mattransp.Lighting=false;
			driver->setMaterial(mattransp);
			driver->draw3DLine(core::vector3dfCH(mstart), core::vector3dfCH(mend), mcol );
	}

			/// Easy-to-use function to draw a polyline in 3D space, given 
			/// the array of points as a std::vector.

	static void drawPolyline(video::IVideoDriver* driver,
					  std::vector< chrono::ChVector<> > mpoints,
					  video::SColor mcol = video::SColor(255,0,0,0),
					  bool use_Zbuffer = false
					)
	{ 
		// not very efficient, but enough as an example..
		if (mpoints.size()<2) return;
		for (unsigned int i =0; i<mpoints.size()-1; i++)
		{
			drawSegment(driver, mpoints[i], mpoints[i+1], mcol, use_Zbuffer);
		}
	}


			/// Easy-to-use function to draw a circle line in 3D space, with given 
			/// colour. Specify the center as coordsys position. Orientation as 
			/// coordsys quaternion (default in xy plane)

	static void drawCircle(video::IVideoDriver* driver,
					double radius,
					chrono::ChCoordsys<> mpos = chrono::CSYSNORM,
					video::SColor mcol = video::SColor(255,0,0,0),
					int mresolution = 36,
					bool use_Zbuffer = false
					)
	{ 
			driver->setTransform(video::ETS_WORLD, core::matrix4());
			video::SMaterial mattransp;
			mattransp.ZBuffer= false;
			mattransp.Lighting=false;
			driver->setMaterial(mattransp);
			double phaseA = 0; double phaseB = 0;
			for (int iu=1; iu<=mresolution; iu++)
			{
				phaseB= chrono::CH_C_2PI*(double)iu/(double)mresolution;
				chrono::ChVector<> V1(radius*cos(phaseA), radius*sin(phaseA), 0);
				chrono::ChVector<> V2(radius*cos(phaseB), radius*sin(phaseB), 0);
				drawSegment(driver,mpos.TrasformLocalToParent(V1), mpos.TrasformLocalToParent(V2), mcol, use_Zbuffer );
				phaseA=phaseB;
			}
	}


			/// Easy-to-use function to draw a spring in 3D space, with given 
			/// colour. Specify the radius, the end points in absolute space, the resolution (i.e.
			/// the number of segments approximating the helix) and the number of turns. 

	static void drawSpring(video::IVideoDriver* driver,
					double radius,
					chrono::ChVector<> start,
					chrono::ChVector<> end,
					video::SColor mcol = video::SColor(255,0,0,0),
					int mresolution = 65,
					double turns = 5,
					bool use_Zbuffer = false
					)
	{ 
			chrono::ChMatrix33<> rel_matrix;
			chrono::ChVector<> dist = end-start;
			chrono::ChVector<> Vx,Vy,Vz;
			double length = dist.Length();
			chrono::ChVector<> dir = chrono::Vnorm(dist);
			chrono::ChVector<> singul(chrono::VECT_Y);
			chrono::XdirToDxDyDz(&dir, &singul, &Vx,  &Vy, &Vz);
			rel_matrix.Set_A_axis(Vx,Vy,Vz);
			chrono::ChQuaternion<> Q12 = rel_matrix.Get_A_quaternion();
			chrono::ChCoordsys<> mpos(start,Q12);

			driver->setTransform(video::ETS_WORLD, core::matrix4());
			video::SMaterial mattransp;
			mattransp.ZBuffer= false;
			mattransp.Lighting=false;
			driver->setMaterial(mattransp);
			double phaseA  = 0; double phaseB  = 0;
			double heightA = 0; double heightB = 0;
			for (int iu=1; iu<=mresolution; iu++)
			{
				phaseB= turns*chrono::CH_C_2PI*(double)iu/(double)mresolution;
				heightB= length*((double)iu/(double)mresolution);
				chrono::ChVector<> V1(heightA, radius*cos(phaseA), radius*sin(phaseA));
				chrono::ChVector<> V2(heightB, radius*cos(phaseB), radius*sin(phaseB));
				drawSegment(driver,mpos.TrasformLocalToParent(V1), mpos.TrasformLocalToParent(V2), mcol, use_Zbuffer );
				phaseA=phaseB;
				heightA=heightB;
			}
	}

			/// Easy-to-use function to draw grids in 3D space, with given 
			/// orientation, colour and spacing.

	static void drawGrid(video::IVideoDriver* driver,
					 double ustep=0.1, double vstep=0.1,
					 int nu=20, int nv=20,
					 chrono::ChCoordsys<> mpos = chrono::CSYSNORM,
					 video::SColor mcol = video::SColor(50,80,110,110),
					 bool use_Zbuffer = false
					)
	{ 
			driver->setTransform(video::ETS_WORLD, core::matrix4());
			video::SMaterial mattransp;
			mattransp.ZBuffer= true;
			mattransp.Lighting=false;
			driver->setMaterial(mattransp);
			for (int iu=-nu/2; iu<=nu/2; iu++)
			{
				chrono::ChVector<> V1(iu*ustep, vstep*(nv/2), 0);
				chrono::ChVector<> V2(iu*ustep,-vstep*(nv/2), 0);
				drawSegment(driver, mpos.TrasformLocalToParent(V1),mpos.TrasformLocalToParent(V2), mcol, use_Zbuffer);
			}
			for (int iv=-nv/2; iv<=nv/2; iv++)
			{
				chrono::ChVector<> V1(ustep*(nu/2), iv*vstep, 0);
				chrono::ChVector<> V2(-ustep*(nu/2),iv*vstep, 0);
				drawSegment(driver, mpos.TrasformLocalToParent(V1),mpos.TrasformLocalToParent(V2), mcol, use_Zbuffer);
			}		
	}





}; // end of class with tools



} // END_OF_NAMESPACE____

#endif // END of ChDisplayTools.h

