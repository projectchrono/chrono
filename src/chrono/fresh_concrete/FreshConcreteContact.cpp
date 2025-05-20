// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bahar Ayhan
// =============================================================================
//
//  Contact force model for discrete fresh concrete
//  Details about the model can be found from the paper of "Ramyar, Elham, and Gianluca Cusatis"
//  Ramyar, Elham, and Gianluca Cusatis. "Discrete Fresh Concrete Model for Simulation of Ordinary, 
//                                        Self-Consolidating, and Printable Concrete Flow." 
//											Journal of Engineering Mechanics 148.2 (2022): 04021142.
//
//			    Material Parameters
//		 	  	float ENm		: mortar to mortar and mortar to aggregate stiffness
//				float ENa		: aggregate to aggregate stiffness
//				float h			: thickness off mortar layer around an aggregate 		
//				float alpha		: parameter for compressive contact  
//				float beta		: parameter for tensile contact
//				float np 		: np=1 newtonian fluid, np<1 shear-thinning and np>1 shear-thickening 
//				float sgmTmax 	: tensile strength of mortar
//				float sgmTau0 	: shear yield stress
//				float kappa0  	: is a constant 
//				float eta_inf 	: mortar plastic viscosity
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/physics/ChContactSMC.h"

//#include "chrono/core/ChDistribution.h"
#include "chrono/core/ChRandom.h"

//#include "chrono/collision/ChCollisionSystem.h"
//#include "chrono/collision/ChCollisionSystemBullet.h" 
 

#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <boost/numeric/odeint.hpp>

using namespace chrono;

///////////////////////data type definition ///////////////////////////////////////////////////////
class MyStateVar {
    public:
    
    MyStateVar() {};
    MyStateVar(double mradA, double mradB, ChVector3d mstrain, double mlambda)
    : radA(mradA), radB(mradB), strain(mstrain), mquaternion(QUNIT), lambda(mlambda)
    {};
    virtual ~MyStateVar() {}
    
    public:
    double step_time=0;
    float radA=1;
    float radB=1;
    float lambda= 0.;
    ChVector3d strain={0,0,0};
    ChQuaternion<> mquaternion;
    
};

//////////////////////////////////////////////////////////////////////////////
std::unordered_map<std::string, MyStateVar> map_contact_info;

std::unordered_map<int, std::vector<double>> ParticleThixoInfo;

//////////////////////////////////////////////////////////////////////////////
template <typename T>
T AveragOfVector(const std::vector<T>& vec) {
    T sum = 0;
    for (const T& element : vec) {
        sum += element;
    }
    return sum/vec.size();
}
//////////////////////////////////////////////////////////////////////////////

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//////////////////////////////////////////////////////////////////////////////
template<typename T>
T rungeKutta4(std::function<T(T, T, T, T, T, T)> f, T y0, T t0, T beta, T gammadot, T m, T Tcr, T h, int steps) {    
    T t = t0;
    T y = y0;
    h=h/steps;	
    for (int i = 0; i < steps; ++i) {
    	//std::cout << "t " << t << " gammadot1 : " << gammadot << " y1: " << y << std::endl;
        T f1 = h * f(t, y, beta, gammadot, m, Tcr);
        T f2 = h * f(t + h / 2, y + f1 / 2, beta, gammadot, m, Tcr);
        T f3 = h * f(t + h / 2, y + f2 / 2, beta, gammadot, m, Tcr);
        T f4 = h * f(t + h, y + f3, beta, gammadot, m, Tcr);

        y = y + (f1 + 2 * f2 + 2 * f3 + f4) / 6;
        //std::cout << "t " << t << " gammadot2 : " << gammadot << " y2: " << y << std::endl;
        
        t = t + h;
    }   
    return y;
}
//////////////////////////////////////////////////////////////////////////////

double fode(double t, double y) {
    return t * t - y; // Example ODE: y' = t^2 - y
}
//////////////////////////////////////////////////////////////////////////////
double dfloc(double t, double y, double beta, double gammadot,  double m=1, double T=300.){ 
	if(y<1e-8)
		y=1e-8;
    return (1./T/pow(y,m) -beta*gammadot*y);
}

//////////////////////////////////////////////////////////////////////////////

template <class RealA>
void XdirToDxDyDz(const ChVector3<RealA>& Vxdir,
                  const ChVector3<RealA>& Vsingular,
                  ChVector3<RealA>& Vx,
                  ChVector3<RealA>& Vy,
                  ChVector3<RealA>& Vz) {
    ChVector3<RealA> mVnull(0, 0, 0);
    double zlen;

    if (Vequal(Vxdir, mVnull))
        Vx = ChVector3<RealA>(1, 0, 0);
    else
        Vx = Vnorm(Vxdir);

    Vz = Vcross(Vx, Vsingular);
    zlen = Vlength(Vz);

    // If close to singularity, change reference vector
    if (zlen < 0.0001) {
        ChVector3<> mVsingular;
        if (std::abs(Vsingular.z()) < 0.9)
            mVsingular = ChVector3<RealA>(0, 0, 1);
        if (std::abs(Vsingular.y()) < 0.9)
            mVsingular = ChVector3<RealA>(0, 1, 0);
        if (std::abs(Vsingular.x()) < 0.9)
            mVsingular = ChVector3<RealA>(1, 0, 0);
        Vz = Vcross(Vx, mVsingular);
        zlen = Vlength(Vz);  // now should be nonzero length.
    }

    // normalize Vz
    Vz = Vmul(Vz, 1.0 / zlen);
    // compute Vy
    Vy = Vcross(Vz, Vx);
}


/// Class for fuller curve distribution of concrete aggregates
class ChApi ChConcreteDistribution : public ChDistribution {
  public:
    ChConcreteDistribution(double mminD, double mmaxD, double mcement, double mWtoC, 
						   double mAtoC, double mmortar_layer) : minD(mminD), maxD(mmaxD), cement(mcement), WtoC(mWtoC), AtoC(mAtoC), mortar_layer(mmortar_layer) {}

    /// Compute a random value whose probability is defined by the distribution,
    /// that is a value between min and max.
    virtual double GetRandom() override {
		//double va=1+c/rho_c+(WC*c)/rho_w+vair;
		//double va0=(1-(d0/da)**nF)*va;
		double q=3.-nF;
		double di=minD*pow( (1- ::chrono::ChRandom::Get() * ( 1.- pow((minD/maxD),q)) ), (-1./q) );	
		//double di=minD*pow( ( 1-::chrono::ChRandom() * ( 1- pow((0.5),q)) ), (-1./q) );
		return (di+2.*mortar_layer);
	};

  private:
    double minD;
    double maxD;
	double cement;
	double WtoC;
	double AtoC;
	double rho_c=3150;
	double rho_w=1000;
	double vair=0.03;
	double nF=0.5;	
	double mortar_layer=3;
};




class ChMaterialFCM {

   public:
    float ENm=4.0E-2;
	float ENa=100;
	float mortar_h=3.0;		
	float alpha=0.25;
	float beta=0.5;
	float np=1.0;
	float sgmTmax=9.E-3;
	float sgmTau0=5.E-4;
	float kappa0=100.;
	float eta_inf=10.E-6;
	float flocbeta=1.0e-2;	
	float flocm=1.0;
	float flocTcr=1000;
	float lambda_init=1.0e-8;
	bool SmoothOnFlag = false;
	bool ThixOnFlag = false;

  public:   
     // ODE system as a member variable
    struct ThixotropyOdeSys {
        float& T;
        float& m;
        float& beta;
        float gammadot;        

        ThixotropyOdeSys(float& T, float& m, float& beta)
            : T(T), m(m), beta(beta), gammadot(0.0) {}

        // Define the system of ODEs: y' = 1/(T * y^m) - beta * gammadot * y
        void operator()(const std::vector<double>& y, std::vector<double>& dydt, double t) {
        	//std::cout<<"time: "<<t<<"\t T: "<<T<<"\tm: "<<m<<"\tbeta: "<<beta<<"\tgammadot: "<<gammadot<<"\t"<<"lambda: "<<y[0]<<std::endl;
            dydt[0] = 1.0 / (T * std::pow(y[0], m)) - beta * gammadot * y[0];
        }
        
        // Update gammadot (time-varying)
        void updateGammadot(double val) {
            gammadot = val;  // Example: sinusoidal variation
        }        
        
    };

    // ODE system and stepper as member variables
    ThixotropyOdeSys thixSystem = ThixotropyOdeSys(flocTcr, flocm, flocbeta);
    boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper;
    
     /// Construct a material.
    ChMaterialFCM(float mENm,
		float mENa,
		float mmortar_h,		
		float malpha,
		float mbeta,
		float mnp,
		float msgmTmax,
		float msgmTau0,
		float mkappa0,
		float meta_inf)
		: ENm(mENm),  ENa(mENa), mortar_h(mmortar_h),	alpha(malpha), beta(mbeta), np(mnp), 
		sgmTmax(msgmTmax), sgmTau0(msgmTau0), kappa0(mkappa0), eta_inf(meta_inf) { };
    
    
    ChMaterialFCM(){  };

    // Destructor declared:
    ~ChMaterialFCM(){};



    /// Getter and Setter for material parameters.
    float Get_ENm() const { return ENm; }  
    void Set_ENm(float mENm) { ENm=mENm; }
    //
    float Get_ENa() const { return ENa; }  
    void Set_ENa(float mENa) { ENa=mENa; }
    //
    float Get_mortar_h() const { return mortar_h; }  
    void Set_mortar_h(float mmortar_h) { mortar_h=mmortar_h; }
    //
    float Get_alpha() const { return alpha; }  
    void Set_alpha(float malpha) { alpha=malpha; }
    //
    float Get_beta() const { return beta; }  
    void Set_beta(float mbeta) { beta=mbeta; }
    //
    float Get_np() const { return np; }  
    void Set_np(float mnp) { np=mnp; }
    //
    float Get_sgmTmax() const { return sgmTmax; }  
    void Set_sgmTmax(float msgmTmax) { sgmTmax=msgmTmax; }
    //
    float Get_sgmTau0() const { return sgmTau0; }  
    void Set_sgmTau0(float msgmTau0) { sgmTau0=msgmTau0; }
    //
    float Get_kappa0() const { return kappa0; }  
    void Set_kappa0(float mkappa0) { kappa0=mkappa0; }
    //
    float Get_eta_inf() const { return eta_inf; }  
    void Set_eta_inf(float meta_inf) { eta_inf=meta_inf; }
    //
    float Get_flocbeta() const { return flocbeta;}  
    void Set_flocbeta(float mflocbeta) { flocbeta=mflocbeta;}
    //
    float Get_flocm() const { return flocm; }  
    void Set_flocm(float mflocm) { flocm=mflocm; }
    //
    float Get_flocTcr() const { return flocTcr; }  
    void Set_flocTcr(float mflocTcr) { flocTcr=mflocTcr; }
    //	
    float Get_lambda_init() const { return lambda_init; }  
    void Set_lambda_init(float mlambda_init) { lambda_init=mlambda_init; }
    //	
    bool Get_SmoothOnFlag() const { return SmoothOnFlag; }  
    void Set_SmoothOnFlag(bool mSmoothOnFlag) { SmoothOnFlag=mSmoothOnFlag; }
	//
	bool Get_ThixOnFlag() const { return ThixOnFlag; }  
    void Set_ThixOnFlag(bool mThixOnFlag) { ThixOnFlag=mThixOnFlag; }
	
	double CalculateEffDensity(ChSystem& sys, double specVol, double rho_0){		
		double totVol=0;
		for (auto body:sys.GetBodies()){
			if (body->IsFixed() || body->GetCollisionModel()->GetShapeInstance(0).first->GetType()!=0 )
				continue;
			auto shape=body->GetCollisionModel()->GetShapeInstance(0).first;
			auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);             	
			double radius=shape_sphere->GetRadius();
			totVol+=1.3333333333333333333*CH_PI*radius*radius*radius;
		}
		return rho_0*specVol/totVol;
	}
	
	void ModifyDensity(ChSystem& sys, double rho){
		double radius;
		double vol;
		double mass;
		for (auto body:sys.GetBodies()){
			if (body->IsFixed() || body->GetCollisionModel()->GetShapeInstance(0).first->GetType()!=0 )
				continue;
			auto shape=body->GetCollisionModel()->GetShapeInstance(0).first;
			auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);             	
			double radius=shape_sphere->GetRadius();
			mass=1.3333333333333333333*CH_PI*radius*radius*radius*rho;
			body->SetInertiaXX((2.0 / 5.0) * mass * pow(radius, 2) * ChVector3d(1, 1, 1));
			body->SetMass(mass);
			//body->SetDensity(rho);
		}		
	}
	
	
};


// -----------------------------------------------------------------------------
// Class for overriding composite material laws
// -----------------------------------------------------------------------------
class CustomCompositeMaterial : public ChContactMaterialCompositionStrategy {
  public:
    virtual float CombineFriction(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineCohesion(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineRestitution(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineDamping(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineCompliance(float a1, float a2) const { return std::min<float>(a1 , a2); }

    virtual float CombineAdhesionMultiplier(float a1, float a2) const { return std::min<float>(a1, a2); }
    virtual float CombineStiffnessCoefficient(float a1, float a2) const { return std::min<float>(a1 , a2); }
    virtual float CombineDampingCoefficient(float a1, float a2) const { return std::min<float>(a1 , a2); }  
    
};
//////////////////////////////////////////////////////////////////////////////////////////////////

ChWrenchd CalculateForceTorque_ORG(
        const ChSystemSMC& sys,                    ///< containing sys
        const ChVector3d& normal_dir,              ///< normal contact direction (expressed in global frame)
        const ChVector3d& p1,                      ///< most penetrated point on obj1 (expressed in global frame)
        const ChVector3d& p2,                      ///< most penetrated point on obj2 (expressed in global frame)
        const ChVector3d& vel1,                    ///< velocity of contact point on obj1 (expressed in global frame)
        const ChVector3d& vel2,                    ///< velocity of contact point on obj2 (expressed in global frame)
        const ChContactMaterialCompositeSMC& mat,  ///< composite material for contact pair
        double delta,                              ///< overlap in normal direction
        double eff_radius,                         ///< effective radius of curvature at contact
        double mass1,                              ///< mass of obj1
        double mass2,                              ///< mass of obj2
        ChContactable* objA,                       ///< pointer to contactable obj1
        ChContactable* objB                        ///< pointer to contactable obj2
    ) {
        // Relative velocity at contact
        ChVector3d relvel = vel2 - vel1;
        double relvel_n_mag = relvel.Dot(normal_dir);
        ChVector3d relvel_n = relvel_n_mag * normal_dir;
        ChVector3d relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();	
        // Calculate effective mass
        double eff_mass = mass1 * mass2 / (mass1 + mass2);
	//
	//
	double E_eff=100.0;	
        //double eff_mass = objA->GetContactableMass() * objB->GetContactableMass() /
        //                      (objA->GetContactableMass() + objB->GetContactableMass());
        double Sn = 2 * E_eff * std::sqrt(eff_radius * delta);
        double loge = std::log(mat.cr_eff);
        double beta = loge / std::sqrt(loge * loge + CH_PI * CH_PI);
        double kn = (2.0 / 3) * Sn;
        double kt = kn;        
        // Calculate the magnitudes of the normal and tangential contact forces
        //double kn = mat.kn;
        //double kt = mat.kt;
        double gn = eff_mass * mat.gn;
        double gt = eff_mass * mat.gt;
        //std::cout<<"kn: "<<kn<<" kt: "<<kt<<"\n";
        // Tangential displacement (magnitude)
        double dT = sys.GetStep();
        double delta_t = relvel_t_mag * dT;

        double forceN = kn * delta - gn * relvel_n_mag;
        double forceT = kt * delta_t + gt * relvel_t_mag;

        // Coulomb law
        forceT = std::min<double>(forceT, mat.mu_eff * std::abs(forceN));
	//std::cout<<"mat.mu_eff: "<<mat.mu_eff<<std::endl;
        // Accumulate normal and tangential forces
        ChVector3d force = forceN * normal_dir;
        if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
            force -= (forceT / relvel_t_mag) * relvel_t;

        // for torque do nothing (this could be used to simulate rolling or spinning friction, if needed)
        ChVector3d torque = VNULL;

        return {force, torque};
    }





float ChGetShapeDimension(const chrono::ChCollisionModel::ShapeInstance& shape_instance){
		const auto& shape = shape_instance.first;        
		float radius=5.0;
        switch (shape->GetType()) {
            case ChCollisionShape::Type::SPHERE: {            	
                auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
                radius = shape_sphere->GetRadius();               
                break;
            }
            case ChCollisionShape::Type::ELLIPSOID: {            	
                auto shape_ell = std::static_pointer_cast<ChCollisionShapeEllipsoid>(shape);
                auto haxes = shape_ell->GetSemiaxes(); 
                radius = std::min(std::min(haxes.x(), haxes.y()), haxes.z());                 
                break;
            }
            case ChCollisionShape::Type::BOX: {            	
                auto shape_box = std::static_pointer_cast<ChCollisionShapeBox>(shape);
                auto len = shape_box->GetHalflengths();
		radius = std::min(std::min(len.x(), len.y()), len.z());                
                break;
            }
            case ChCollisionShape::Type::CYLINDER: {            	
                auto shape_cylinder = std::static_pointer_cast<ChCollisionShapeCylinder>(shape);
                auto height = shape_cylinder->GetHeight();
                radius = shape_cylinder->GetRadius();                
                break;
            }            
            case ChCollisionShape::Type::CAPSULE: {
            	std::cout<<"CAPSULE\t";
                auto shape_capsule = std::static_pointer_cast<ChCollisionShapeCapsule>(shape);
                auto height = shape_capsule->GetHeight();
                radius = shape_capsule->GetRadius();                
                break;
            }
            case ChCollisionShape::Type::CYLSHELL: {
                std::cout<<"CYLSHELL\t";
                auto shape_cylshell = std::static_pointer_cast<ChCollisionShapeCylindricalShell>(shape);
                auto height = shape_cylshell->GetHeight();
                radius = shape_cylshell->GetRadius();               
                break;
            }            
            case ChCollisionShape::Type::POINT: {            	
                auto shape_point = std::static_pointer_cast<ChCollisionShapePoint>(shape);
                auto radius = shape_point->GetRadius();                
                break;
            }  
            case ChCollisionShape::Type::TRIANGLEMESH: {             	
                auto shape_obj = std::static_pointer_cast<ChCollisionShapePoint>(shape);
                auto radius = shape_obj->GetRadius();                
                break;
            }           
            default:
                // Shape type not supported
                break;
        }
		
		return radius;
}



//////////////////////////////////////////////////////////////////////////////////////////////////
// -----------------------------------------------------------------------------
// Class for overriding the default SMC contact force calculation
// -----------------------------------------------------------------------------
class FCContactForce : public ChSystemSMC::ChContactForceTorqueSMC {
  public:
    // Demonstration only.	
	
   virtual ChWrenchd CalculateForceTorque(
        const ChSystemSMC& sys,             ///< containing sys
        const ChVector3d& normal_dir,       ///< normal contact direction (expressed in global frame)
        const ChVector3d& p1,               ///< most penetrated point on obj1 (expressed in global frame)
        const ChVector3d& p2,               ///< most penetrated point on obj2 (expressed in global frame)
        const ChVector3d& vel1,             ///< velocity of contact point on obj1 (expressed in global frame)
        const ChVector3d& vel2,             ///< velocity of contact point on obj2 (expressed in global frame)
        const ChContactMaterialCompositeSMC& mat,  ///< composite material for contact pair
        double delta,                       ///< overlap in normal direction
        double eff_radius,                  ///< effective radius of curvature at contact
        double mass1,                       ///< mass of obj1
        double mass2,                       ///< mass of obj2
        ChContactable* objA,                ///< pointer to contactable obj1
        ChContactable* objB                 ///< pointer to contactable obj2
    ) const override {
    		//
		// Get current_time
		//
		auto current_time=sys.GetChTime();
		
		//
		// Get material properties
		//
		float mortar_layer=this->material->Get_mortar_h();
		float ENm=this->material->Get_ENm();
		float ENa=this->material->Get_ENa();			
		float alpha=this->material->Get_alpha();
		float beta=this->material->Get_beta();
		float np=this->material->Get_np();
		float sgmTmax=this->material->Get_sgmTmax();
		float sgmTau0=this->material->Get_sgmTau0();
		float kappa0=this->material->Get_kappa0();
		float eta_inf=this->material->Get_eta_inf();
		float flocbeta=this->material->Get_flocbeta();	
		float flocm=this->material->Get_flocm();
		float flocTcr=this->material->Get_flocTcr();
		//printf("%f \t %f \t %f \t %f \t %f \t %f \t %f \t %f10 \t %f \t %f \n", mortar_layer, ENm, ENa, alpha, beta, 
		//	np, sgmTmax, sgmTau0, kappa0, eta_inf);
		
		//
		float h=mortar_layer;
		float R1=5000.0;
		float R2=5000.0; 
		double Rmin, Rmax, delta_rmin;
		double Lij, L0ij;
		//Material parameters of viscous part which should be defined inside contactinfo
		double eta0=kappa0*eta_inf;
		double Deps0=sgmTau0/eta0;
		
		
		//
    	//Get state variables form the map defined globally
    	//    	
		auto bodyA=dynamic_cast<ChBody*>(objA);
		auto bodyB=dynamic_cast<ChBody*>(objB);
		
		
		std::vector<unsigned int> ID={bodyA->GetIndex(),bodyB->GetIndex()};
		std::string mykey;
		
		if (ID[0]>ID[1]){			
		       mykey=std::to_string(ID[0])+"_"+std::to_string(ID[1]);		      
		}else{
		       mykey=std::to_string(ID[1])+"_"+std::to_string(ID[0]);		       
		}
		//
    	// Get state variables relevant to this contact	
		//
    		ChVector3d statevar=map_contact_info[mykey].strain;
    		double lambda0=map_contact_info[mykey].lambda;
			//
			// ==== SIGMA-t influenced by Flocculation ======
			sgmTmax=this->material->Get_sgmTmax()*(1.+lambda0);
    		//    		
    		if (lambda0==0 ){
    			lambda0=material->Get_lambda_init();    			
    			auto val0=ParticleThixoInfo[ID[0]];
    			auto val1=ParticleThixoInfo[ID[1]];
    			double avg0, avg1;
    			//std::cout << "lambda-1 " << lambda0 << std::endl;
			//
    			if(val0.size()){
    				avg0 = AveragOfVector(val0); //std::accumulate(val0.begin(), val0.end(), 0)/ double(val0.size());
    			}else{
    				avg0 = lambda0;
    			}
    			//std::cout << "lambda-2 " << lambda0 << std::endl;
    			//    			
			//
    			if(val1.size()){
    				avg1 = AveragOfVector(val1); //std::accumulate(val1.begin(), val1.end(), 0)/double(val1.size());
    			}else{
    				avg1 = lambda0;
    			}  
    			//std::cout << "lambda-3 " << lambda0 << std::endl;  			
    			//    			
    			lambda0=std::min(avg0, avg1);
    			map_contact_info[mykey].lambda=lambda0;
    			//std::cout << "lambda-4 " << map_contact_info[mykey].lambda <<  "key " << mykey << std::endl;
				map_contact_info[mykey].step_time=current_time;
    		}
		//
		// initialize force values
		//
		ChVector3d force=(0,0,0);		
		//
		// If delta>h, objects are saparated
		//	
		if (delta<0){
			ChVector3d force = (0,0,0);
			ChVector3d torque = (0,0,0);
			//return std::make_pair(-force, torque);
			return {force, torque};
		}
		//
		// Get local frame
		//
		//
		ChVector3d Vx, Vy, Vz;
		XdirToDxDyDz(normal_dir, VECT_Y, Vx, Vy, Vz);
        //std::cout<<" Vy: "<<Vy<<" Vz: "<<Vz<<"\t";				
		
		//
		// Get the dimension of the object
		//	
		auto shapeA=bodyA->GetCollisionModel()->GetShapeInstance(0).first->GetType();
		auto shapeB=bodyB->GetCollisionModel()->GetShapeInstance(0).first->GetType();
		R1 = ChGetShapeDimension(bodyA->GetCollisionModel()->GetShapeInstance(0));	
		R2 = ChGetShapeDimension(bodyB->GetCollisionModel()->GetShapeInstance(0));
		
		/*if (bodyA->GetCollisionModel()->GetShapeInstance(0).first->GetType()==0){        
			//R1=bodyA->GetCollisionModel()->GetShapeDimensions(0)[0];
			//R1=5;//bodyA->GetCollisionModel()->GetRadius();
			R1 = ChGetShapeDimension(bodyA->GetCollisionModel()->GetShapeInstance(0));	
			//auto shape=bodyA->GetCollisionModel()->GetShapeInstance(0).first;
			//auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
            		//R1 = shape_sphere->GetRadius();				
		}
		
		if (bodyB->GetCollisionModel()->GetShapeInstance(0).first->GetType()==0){        
			//R2=bodyB->GetCollisionModel()->GetShapeDimensions(0)[0];
			//R2=5;//bodyA->GetCollisionModel()->GetRadius();
			//auto shape=bodyB->GetCollisionModel()->GetShapeInstance(0).first;
			//auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
            		//R2 = shape_sphere->GetRadius();	
            		R2 = ChGetShapeDimension(bodyB->GetCollisionModel()->GetShapeInstance(0));			
		}*/
		if (shapeA==ChCollisionShape::Type::SPHERE & shapeB==ChCollisionShape::Type::SPHERE){
			/// Concrete - Concrete Interaction
			// Contact of two DFC spheres
			// center to center distance between 2 objects	
			L0ij=R1+R2-h;
			Lij=abs(R1+R2-delta);
			Rmin=std::min(R1, R2);	
			Rmax=std::max(R1, R2);
			delta_rmin=delta/2*(2.0*Rmax-delta)/Lij;
			
		}else if((shapeA==ChCollisionShape::Type::SPHERE & shapeB==ChCollisionShape::Type::CYLINDER) || 
				(shapeB==ChCollisionShape::Type::SPHERE & shapeA==ChCollisionShape::Type::CYLINDER) ) { /// Concrete - Fiber Interaction
			// Contact of concrete sphere and cylindrical fiber object
			// center to center distance between 2 objects
			h=mortar_layer/2;
			//sgmTmax=this->material->Get_sgmTmax()/2;
			L0ij=R1+R2-h;
			Lij=abs(R1+R2-delta);
			Rmin=std::min(R1, R2);	
			Rmax=std::max(R1, R2);	
			delta_rmin=delta/2;
			
		}else if ( (shapeA==ChCollisionShape::Type::SPHERE & shapeB!=ChCollisionShape::Type::SPHERE) || 
			   	(shapeB==ChCollisionShape::Type::SPHERE & shapeA!=ChCollisionShape::Type::SPHERE) ) { /// COncrete-container interaction
			h=mortar_layer/2;
			//sgmTmax=this->material->Get_sgmTmax()/2;
			L0ij=R1+R2-h;
			Lij=abs(R1+R2-delta);
			if (shapeA==ChCollisionShape::Type::SPHERE){
				Rmin=R1;
			}else {
				Rmin=R2;
			}
			//Rmin=std::min(R1, R2);	
			Rmax=std::max(R1, R2);	
			delta_rmin=delta;
			
		}else if (shapeA==ChCollisionShape::Type::TRIANGLEMESH && shapeB==ChCollisionShape::Type::TRIANGLEMESH) {
					
			// contact of DFC sphere with outer wall surface			
			ChWrenchd res=default_contact_algorithm.CalculateForceTorque(
							sys,                     ///< containing sys
							normal_dir,              ///< normal contact direction (expressed in global frame)
							p1,                      ///< most penetrated point on obj1 (expressed in global frame)
							p2,                      ///< most penetrated point on obj2 (expressed in global frame)
							vel1,                    ///< velocity of contact point on obj1 (expressed in global frame)
							vel2,                    ///< velocity of contact point on obj2 (expressed in global frame)
							mat,  					 ///< composite material for contact pair
							delta,                   ///< overlap in normal direction
							eff_radius,              ///< effective radius of curvature at contact
							mass1,                   ///< mass of obj1
							mass2,                   ///< mass of obj2
							objA,                    ///< pointer to contactable obj1
							objB                     ///< pointer to contactable obj2
							);
			
			return res;
				
			
		}else{ /// Fiber-container or another object- container interaction
			h=mortar_layer/2;
			//sgmTmax=this->material->Get_sgmTmax()/2;
			L0ij=R1+R2-h;
			Lij=abs(R1+R2-delta);
			Rmin=std::min(R1, R2);	
			Rmax=std::max(R1, R2);	
			delta_rmin=delta;
		
		}
		
		/*if (bodyA->GetCollisionModel()->GetShapeInstance(0).first->GetType()!=0 || 
						bodyB->GetCollisionModel()->GetShapeInstance(0).first->GetType()!=0){ 
			// contact of DFC sphere with outer wall surface
			
			h=mortar_layer/2;
			//sgmTmax=this->material->Get_sgmTmax()/2;
			Rmin=std::min(R1, R2);	
			Rmax=std::max(R1, R2);
			L0ij=2.0*Rmin-h;
			Lij=abs(2.0*Rmin-delta);	
			delta_rmin=delta;
			
		}else{
			// Contact of two DFC spheres
			// center to center distance between 2 objects	
			L0ij=R1+R2-h;
			Lij=abs(R1+R2-delta);
			Rmin=std::min(R1, R2);	
			Rmax=std::max(R1, R2);
			delta_rmin=delta/2*(2.0*Rmax-delta)/Lij;
		}
		*/
		
		//
		// Calculate contact plane rotation
		//		
		/*auto ref_rot=map_contact_info[mykey].mquaternion;
		ChMatrix33<> A0(ref_rot);	
	    ChMatrix33<> Aabs;
	    ChQuaternion<> abs_rot;
	    //
	    ChVector3d mXele = normal_dir;
	    ChVector3d myele = (bodyA->GetFrameRefToAbs().GetA().GetAxisY() + 
							bodyB->GetFrameRefToAbs().GetA().GetAxisY()).GetNormalized();
	    Aabs.Set_A_Xdir(mXele, myele);
	    abs_rot = Aabs.Get_A_quaternion();			
		//
		ChQuaternion<> q_delta=(abs_rot %  ref_rot.GetConjugate());
		//
		// update quaternion
		//
		//std::cout<<"current_time: "<<current_time<<" ref_rot : "<<ref_rot<< " abs_rot : "<<abs_rot<<"\t"<<normal_dir<<std::endl;
		//std::cout<<"q_delta  "<<q_delta<<std::endl;
		map_contact_info[mykey].mquaternion=abs_rot;*/
		//
		// Calculate contact area
		//		
		
		
		double ai=abs(Rmin-delta_rmin);			
		double radius2=Rmin*Rmin-ai*ai;
		double contact_area=CH_PI*radius2;
		
		//
		// modify penetration according to initial overlap
		//
		double delta_new=delta-h;
		//
		//	
        // Relative velocities at contact at midplane (p0=(p1+p2)/2) 	
		//
		//			
		ChVector3d p0=(p1+p2)/2;		
		/*
		ChVector3d vcA=bodyA->GetFrameCOMToAbs().GetPosDt();
		ChVector3d vcB=bodyB->GetFrameCOMToAbs().GetPosDt();		
		//
		ChVector3d wcA=bodyA->GetFrameCOMToAbs().GetAngVelLocal();
		ChVector3d wcB=bodyB->GetFrameCOMToAbs().GetAngVelLocal();
		//
		ChVector3d velA=vcA+Vcross( bodyA->GetPos()-p0, wcA);
		ChVector3d velB=vcB+Vcross( bodyB->GetPos()-p0, wcB);
		*/				
		//ChVector3d m_p1_loc = bodyA->Point_World2Body(p0);
		ChVector3d m_p1_loc = bodyA->GetFrameRefToAbs().TransformPointParentToLocal(p0);
		ChVector3d velA=bodyA->PointSpeedLocalToParent(m_p1_loc)/time_scale_val;
		
		//ChVector3d m_p2_loc = bodyB->Point_World2Body(p0);
		ChVector3d m_p2_loc = bodyB->GetFrameRefToAbs().TransformPointParentToLocal(p0);
		ChVector3d velB=bodyB->PointSpeedLocalToParent(m_p2_loc)/time_scale_val;
		/*
		printf("  %10.6f  %10.6f  %10.6f\n ", vcA.x(), vcA.y(), vcA.z());
		printf("  %10.6f  %10.6f  %10.6f\n ", vcB.x(), vcB.y(), vcB.z());
		printf("  %10.6f  %10.6f  %10.6f\n ", wcA.x(), wcA.y(), wcA.z());
		printf("  %10.6f  %10.6f  %10.6f\n ", wcB.x(), wcB.y(), wcB.z());		
				
		printf("VA:  %10.6f  %10.6f  %10.6f\n ", velA.x(), velA.y(), velA.z());
		printf("VB:  %10.6f  %10.6f  %10.6f\n ", velB.x(), velB.y(), velB.z());	
		
		printf("VA2:  %10.6f  %10.6f  %10.6f\n ", velA2.x(), velA2.y(), velA2.z());
		printf("VB2:  %10.6f  %10.6f  %10.6f\n ", velB2.x(), velB2.y(), velB2.z());	
		
		printf("V1:  %10.6f  %10.6f  %10.6f\n ", vel1.x(), vel1.y(), vel1.z());
		printf("V2:  %10.6f  %10.6f  %10.6f\n ", vel2.x(), vel2.y(), vel2.z());
		*/
		//
        ChVector3d relvel = velB - velA;
        double relvel_n_mag = relvel.Dot(normal_dir);
        ChVector3d relvel_n = relvel_n_mag * normal_dir;
        ChVector3d relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();		
		//
		// Calculate displacement increment in normal and tangential direction
		//
		double dT = sys.GetStep();
		double delta_t = relvel_t_mag * dT;
		double delta_n = relvel_n_mag * dT;			
        ChVector3d v_delta_t = relvel_t * dT;
		//
		//  Calculate the strain increment in each local direction
		//
		double depsN=delta_n/Lij;
		double delta_M = relvel.Dot(Vy)*dT; //v_delta_t.Dot(Vy);
		double delta_L = relvel.Dot(Vz)*dT; //v_delta_t.Dot(Vz);
		double depsM=delta_M/Lij;
		double depsL=delta_L/Lij;		
		// 
		//
		//
		double epsA=log(1-h/(L0ij));			
		double epsN=log(Lij/L0ij);		
		//double epsN=statevar[0]+depsN;			
		double epsM=statevar[1]+depsM;
		double epsL=statevar[2]+depsL;
		//
		double epsT=pow((epsM*epsM+epsL*epsL),0.5);
		double epsQ=pow(epsN*epsN+alpha*epsT*epsT,0.5);
		//
		//
		//
		statevar[0]=epsN; statevar[1]=epsM;	 statevar[2]=epsL;		
		map_contact_info[mykey].strain=statevar;
		map_contact_info[mykey].step_time=current_time;
		//
		//
		//
		if (epsN<0){
			////////////////////////////////////////////////////////////
			// Compressive contact;
			////////////////////////////////////////////////////////////
			double sgmN=0;
			double sgmM=0;
			double sgmL=0;	
			double sgmT=0;
			//
			//double k= 10/mortar_layer;
			//ENm= ENm + (ENa - ENm) / (1. + exp(-k * (delta_new -0.5* mortar_layer)));
			//
			//std::cout << "Penetration " << delta_new << " ENm= " << ENm << std::endl;
			double stot;
			if (epsN>=epsA){				
				//stot=epsQ*ENm;	
				sgmN=epsN*ENm;
				map_contact_info[mykey].strain[1]=0; map_contact_info[mykey].strain[2]=0;
			}else{
				sgmN=(epsA)*ENm+(epsN-epsA)*ENa;
				sgmM=alpha*ENa*epsM;
				sgmM = sgn(epsM)*std::min<double>(abs(sgmM), mat.mu_eff * std::abs(sgmN))*0;
				sgmL=alpha*ENa*epsL;
				sgmL = sgn(epsL)* std::min<double>(abs(sgmL), mat.mu_eff * std::abs(sgmN))*0;
				//std::cout<<"delta_new: "<<delta_new <<" epsQ: "<<epsQ<<"  epsA: "<<epsA<<"  stot "<<stot<<"\n";
				//exit(0);
				/*
				stot=((epsA)*ENm+(epsQ-epsA)*ENa);
				
				if (epsQ!=0){
					sgmN=stot*epsN/epsQ;
					sgmT=alpha*stot*epsT/epsQ;
					sgmT = std::min<double>(sgmT, mat.mu_eff * std::abs(sgmN));
					if (epsT!=0){
						sgmM=sgmT*epsM/epsT;
						sgmL=sgmT*epsL/epsT;
					}
					//std::cout<<"sgmN: "<<sgmN<<"  sgmT: "<<sgmT<<"  sgmM: "<<sgmM<<"  sgmL: "<<sgmL<<std::endl;
				}
				*/
			}		
			
						
			//std::cout<<"delta_new: "<<delta_new<<" epsN: "<<epsN<< " epsQ: "<<epsQ<<" stot: "<<stot<<" sgmN: "<<sgmN<<std::endl;
			//////////////////////////////////////////////////////////
			//
			// Viscous stresses
			//		
			//////////////////////////////////////////////////////////
			//
			//depsN=relvel_n_mag/Lij*dT;
			//depsT=relvel_t_mag/Lij*dT;
			// calculate strain rates
			double vdepsN=depsN/dT;
			double vdepsM=depsM/dT;
			double vdepsL=depsL/dT;
			double vDeps=pow((beta*vdepsN*vdepsN+vdepsM*vdepsM+vdepsL*vdepsL),0.5);
			//
			//std::cout << "Lij : " << Lij << " vdeps_Comp : " << vDeps << std::endl;
			double eta;
			double lambda=lambda0;
			//std::cout<<"Lambda0: "<<lambda0<<std::endl;
			if (this->material->ThixOnFlag) {
				std::vector<double> lmd={lambda0};
				this->material->thixSystem.updateGammadot(vDeps);
				boost::numeric::odeint::integrate_adaptive(this->material->stepper, this->material->thixSystem, lmd, 
									current_time, current_time+dT, dT);
				lambda=lmd[0];
				//std::cout<<"lambda: "<<lambda<<"\n";
				map_contact_info[mykey].lambda=lambda;
			}
			
			if(this->material->SmoothOnFlag){	
				//double lambda = rungeKutta4<double>(dfloc, lambda0, current_time, flocbeta, vDeps, flocm, flocTcr, dT, 1);
				//map_contact_info[mykey].lambda=lambda;
				//lambda=0;
				if (vDeps>0){				
					eta=(1.- exp(-1.0E+3*vDeps/Deps0) ) * ( eta_inf + sgmTau0 * (1.0+lambda)/vDeps );				
				}else{
					eta= eta0*(1.0+lambda); //sgmTau0 * (1.0+lambda)/Deps0;
				}
			}else{
				//double lambda = rungeKutta4<double>(dfloc, lambda0, current_time, flocbeta, vDeps, flocm, flocTcr, dT, 1);
				//lambda=0;
				//map_contact_info[mykey].lambda=lambda;
				if(vDeps<=Deps0) {
					eta=eta0;
				}else{
					eta=eta_inf*pow(abs(vDeps),np-1.)+sgmTau0*(1.0+lambda)/vDeps;
				}	
				
			}	
			
			double sgmN_vis=beta*eta*vdepsN;
			double sgmM_vis=eta*vdepsM;
			double sgmL_vis=eta*vdepsL;
			double sgmT_vis=pow(sgmM_vis*sgmM_vis+sgmL_vis*sgmL_vis,0.5);
			//
			//	
			//
			double forceN=contact_area*(sgmN+sgmN_vis);	
			double forceM=contact_area*(sgmM+sgmM_vis);
			double forceL=contact_area*(sgmL+sgmL_vis);
			//std::cout<<"epsN "<<epsN<<" sgmN "<<sgmN<<std::endl;
			//force[0]=forceN;force[1]=contact_area*(sgmM-sgmM_vis);force[2]=contact_area*(sgmL-sgmL_vis);
			force=forceN*normal_dir+forceM*Vy+forceL*Vz;
			//std::cout<<" force "<<force<<std::endl;
			/*
			force = -forceN * normal_dir;		
			if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
				force -= (forceT / relvel_t_mag) * relvel_t;			 
            */			
			//return -force;
			
		
		} else{
			////////////////////////////////////////////////////////////
			// Tensile contact
			////////////////////////////////////////////////////////////
						
			//////////////////////////////////////////////////////////
			//
			// Calculate Stress from material stiffness
			//
			//////////////////////////////////////////////////////////
			double sgmN=ENm*epsN;
			double sgmM=0;
			double sgmL=0;
			if (sgmN>sgmTmax)
				sgmN=sgmTmax;
			//
			//////////////////////////////////////////////////////////
			//
			// Viscous stresses
			//		
			//////////////////////////////////////////////////////////
			//			
			// calculate strain rates
			double vdepsN=depsN/dT;
			double vdepsM=depsM/dT;
			double vdepsL=depsL/dT;
			double vDeps=pow((beta*vdepsN*vdepsN+vdepsM*vdepsM+vdepsL*vdepsL),0.5);
			//
			//std::cout << "Lij : " << Lij << " vdeps_Tension : " << vDeps << std::endl;
			double eta;
			double lambda=lambda0;
			if (this->material->ThixOnFlag) {
				std::vector<double> lmd={lambda0};
				this->material->thixSystem.updateGammadot(vDeps);
				boost::numeric::odeint::integrate_adaptive(this->material->stepper, this->material->thixSystem, lmd, 
									current_time, current_time+dT, dT);
				lambda=lmd[0];
				//std::cout<<"lambda: "<<lambda<<"\n";
				map_contact_info[mykey].lambda=lambda;
			}
			if(this->material->SmoothOnFlag){	
			    //std::cout<<"lmd: "<<lmd[0]<<std::endl;
				//double lambda = rungeKutta4<double>(dfloc, lambda0, current_time, flocbeta, vDeps, flocm, flocTcr, dT, 1);
				//map_contact_info[mykey].lambda=lambda;
				//lambda=0;
				if (vDeps>0){				
					eta=(1.- exp(-1.0E+3*vDeps/Deps0) ) * ( eta_inf + sgmTau0 * (1.0+lambda)/vDeps );				
				}else{
					eta= eta0*(1.0+lambda); //sgmTau0 * (1.0+lambda)/Deps0;
				}
			}else{
				//double lambda = rungeKutta4<double>(dfloc, lambda0, current_time, flocbeta, vDeps, flocm, flocTcr, dT, 1);
				//lambda=0;
				//map_contact_info[mykey].lambda=lambda;
				if(vDeps<=Deps0) {
					eta=eta0;
				}else{
					eta=eta_inf*pow(abs(vDeps),np-1.)+sgmTau0*(1.0+lambda)/vDeps;
				}	
				
			}	
			//std::cout<<"relvel"<<relvel<<" Deps0: "<<Deps0<<" vDeps"<<vDeps<<" eta: "<<eta<<std::endl;
			double sgmN_vis=beta*eta*vdepsN;
			double sgmM_vis=eta*vdepsM;
			double sgmL_vis=eta*vdepsL;
			//double sgmT=pow(sgmM*sgmM+sgmL*sgmL,0.5);
			//////////////////////////////////////////////////////////
			//
			// Combine Viscous stresses and stiffness stresses and calculate forces
			//		
			//////////////////////////////////////////////////////////				
			//exit(0);
			double forceN=contact_area*(sgmN+sgmN_vis);	
			double forceM=contact_area*(sgmM+sgmM_vis);
			double forceL=contact_area*(sgmL+sgmL_vis);
			//std::cout<<"epsN "<<epsN<<" sgmN "<<sgmN<<std::endl;
			//double forceT = sgmT * contact_area;
			//force[0]=forceN;force[1]=contact_area*(sgmM-sgmM_vis);force[2]=contact_area*(sgmL-sgmL_vis);
			force=forceN*normal_dir+forceM*Vy+forceL*Vz;
			/*			
			force = -forceN * normal_dir;
			if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
				force -= (forceT / relvel_t_mag) * relvel_t;			
			*/
			//return -force;
		}
		ChVector3d torque = Vcross((p1-p0), -force);
        //return std::make_pair(-force, torque);
		return {-force, torque};
        
    }
	
	std::shared_ptr<ChMaterialFCM> Get_Material() const { return material; }  
    void Set_Material(std::shared_ptr<ChMaterialFCM> mmat) { material=mmat; }
    
	public:	
	
	std::shared_ptr<ChMaterialFCM> material;
	chrono::ChDefaultContactForceTorqueSMC default_contact_algorithm;
	double time_scale_val=1.0;
	
};


//////////////////////////////////////////////////////////////////////////////////////////////////

class MyContactContainer : public ChContactContainerSMC {
  public:
    MyContactContainer() {}
    ~MyContactContainer() {}
    // Traverse the list contactlist_6_6
    
    
    void IterOnContactList(double& current_time, double& AverageLambda, double& IE) {
	int num_contact = 0;
	double lambda;
	IE=0;
	AverageLambda=0;
	//double AverageLambda=0;
	std::unordered_map<std::string, MyStateVar> updated_map_contact_info;
        auto iter = contactlist_6_6.begin();        
        while (iter != contactlist_6_6.end()) {
            ChVector3d p1 = (*iter)->GetContactP1();
            ChVector3d p2 = (*iter)->GetContactP2();
            ChVector3d force = (*iter)->GetContactForce();
            ChVector3d dp=p1-p2;
            IE=IE+std::abs(force.x()*dp.x()+force.y()*dp.y()+force.z()*dp.z());
            double CD = (*iter)->GetContactDistance();
            ChBody* bodyA = dynamic_cast<ChBody*>((*iter)->GetObjA());
	    ChBody* bodyB = dynamic_cast<ChBody*>((*iter)->GetObjB());
	    ///
	    ///
	    ///
            std::vector<unsigned int> ID={bodyA->GetIndex(),bodyB->GetIndex()};
            std::string mykey;
            ///
            ///
            if (ID[0]>ID[1]){
            	mykey=std::to_string(ID[0])+"_"+std::to_string(ID[1]);
            	//std::cout<<mykey<<std::endl;
            }else{
            	mykey=std::to_string(ID[1])+"_"+std::to_string(ID[0]);
            	//std::cout<<mykey<<std::endl;
            }
            //
            lambda=map_contact_info[mykey].lambda;
            if(lambda>0)
            	AverageLambda += lambda;
            //
            //std::cout<<"mykey: "<<mykey<<"  dist: "<< CD <<" lambda: "<<map_contact_info[mykey].lambda<<std::endl;
            updated_map_contact_info[mykey]=map_contact_info[mykey];
            ///
            ///
            num_contact++;
            ++iter;
        }
		
	map_contact_info=updated_map_contact_info;
	AverageLambda = AverageLambda/num_contact;	
        
    }
    
    //min average for each pairs
    void IterOnContactList(double& current_time, double& IE) {
	int num_contact = 0;	
	IE=0;
	double Lambda=0;
	ParticleThixoInfo.clear();
	std::unordered_map<std::string, MyStateVar> updated_map_contact_info;
        auto iter = contactlist_6_6.begin();        
        while (iter != contactlist_6_6.end()) {
            ChVector3d p1 = (*iter)->GetContactP1();
            ChVector3d p2 = (*iter)->GetContactP2();
            ChVector3d force = (*iter)->GetContactForce();
            ChVector3d dp=p1-p2;
            IE=IE+std::abs(force.x()*dp.x()+force.y()*dp.y()+force.z()*dp.z());
            double CD = (*iter)->GetContactDistance();
            ChBody* bodyA = dynamic_cast<ChBody*>((*iter)->GetObjA());
	    ChBody* bodyB = dynamic_cast<ChBody*>((*iter)->GetObjB());
	    ///
	    ///
	    ///
            std::vector<unsigned int> ID={bodyA->GetIndex(),bodyB->GetIndex()};
            std::string mykey;
            ///
            ///
            if (ID[0]>ID[1]){
            	mykey=std::to_string(ID[0])+"_"+std::to_string(ID[1]);
            	//std::cout<<mykey<<std::endl;
            }else{
            	mykey=std::to_string(ID[1])+"_"+std::to_string(ID[0]);
            	//std::cout<<mykey<<std::endl;
            }
            //
            Lambda=map_contact_info[mykey].lambda;
            if(Lambda>=0){
            	ParticleThixoInfo[ID[0]].push_back(Lambda);
            	ParticleThixoInfo[ID[1]].push_back(Lambda);
            }
            //
            //std::cout<<"mykey: "<<mykey<<"  dist: "<< CD <<" lambda: "<<map_contact_info[mykey].lambda<<std::endl;
            updated_map_contact_info[mykey]=map_contact_info[mykey];
            ///
            ///
            num_contact++;
            ++iter;
        }
		
	map_contact_info=updated_map_contact_info;
	/*
	for (auto it = ParticleThixoInfo.begin(); it != ParticleThixoInfo.end(); ++it) {
        	std::cout << "Key: " << it->first << ", Value: ";
		for (const auto& val : it->second) {
		    std::cout << val << " ";
		}
		std::cout <<"\n";
    	} 
    	*/   			
        
    }

    
    
    

	int TrackHardContact(double& h_layer) {
		int num_hard_contact = 0;
		double max_CD=0;
        auto iter = contactlist_6_6.begin();        
        while (iter != contactlist_6_6.end()) {           
            double CD = (*iter)->GetContactDistance(); 
			if (CD<max_CD)
				max_CD=CD;
            if (CD<=-2*h_layer) 
				num_hard_contact++;
            ++iter;
        }	
		//std::cout<<"  max_CD "<<max_CD<<"\n";
        return num_hard_contact;
		
    }	
 
    
};


// -----------------------------------------------------------------------------
// Callback class for contact reporting
// -----------------------------------------------------------------------------
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(FILE* fptr, std::shared_ptr<ChBody> objA, std::shared_ptr<ChBody> objB) : m_objA(objA), m_objB(objB), m_fptr(fptr) {}

  private:
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector3d& cforce,
                                 const ChVector3d& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
		//
		//
		auto bodyA=dynamic_cast<ChBody*>(modA);
		auto bodyB=dynamic_cast<ChBody*>(modB);
		ChVector3d pcA=bodyA->GetFrameCOMToAbs().GetPos();
		ChVector3d pcB=bodyB->GetFrameCOMToAbs().GetPos();
		
		//fprintf(m_fptr,"center of sphere A: %10.6f  %10.6f  %10.6f", pcA.x(), pcA.y(), pcA.z());
		//fprintf(m_fptr,"  center of sphere B: %10.6f  %10.6f  %10.6f", pcB.x(), pcB.y(), pcB.z());
		fprintf(m_fptr," %10.6f  %10.6f  %10.6f ", pcA.x(), pcA.y(), pcA.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pcB.x(), pcB.y(), pcB.z());
		
		ChVector3d pcA0=bodyA->GetFrameRefToAbs().GetPos();
		ChVector3d pcB0=bodyB->GetFrameRefToAbs().GetPos();
		
		ChVector3d vcA=bodyA->GetFrameCOMToAbs().GetPosDt();
		ChVector3d vcB=bodyB->GetFrameCOMToAbs().GetPosDt();
		//fprintf(m_fptr,"  vel of sphere A: %10.6f  %10.6f  %10.6f", vcA.x(), vcA.y(), vcA.z());
		//fprintf(m_fptr,"  vel of sphere B: %10.6f  %10.6f  %10.6f", vcB.x(), vcB.y(), vcB.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", vcA.x(), vcA.y(), vcA.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", vcB.x(), vcB.y(), vcB.z());
		
		ChVector3d wcA=bodyA->GetFrameCOMToAbs().GetAngVelLocal();
		ChVector3d wcB=bodyB->GetFrameCOMToAbs().GetAngVelLocal();		
		//fprintf(m_fptr,"  angular vel of sphere A: %10.6f  %10.6f  %10.6f", wcA.x(), wcA.y(), wcA.z());
		//fprintf(m_fptr,"  angular vel of sphere B: %10.6f  %10.6f  %10.6f", wcB.x(), wcB.y(), wcB.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", wcA.x(), wcA.y(), wcA.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", wcB.x(), wcB.y(), wcB.z());
		//
		//
		
        // Check if contact involves objA
        if (modA == m_objA.get()) {
            //fprintf(m_fptr,"  A contact on sphere 1 at pos: %10.6f  %10.6f  %10.6f", pA.x(), pA.y(), pA.z());
			fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pA.x(), pA.y(), pA.z());
        } else if (modB == m_objA.get()) {
            //fprintf(m_fptr,"  B contact on sphere 1 at pos: %10.6f  %10.6f  %10.6f", pB.x(), pB.y(), pB.z());
			fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pB.x(), pB.y(), pB.z());
        }

        // Check if contact involves objB
        if (modA == m_objB.get()) {
            //fprintf(m_fptr,"  A contact on sphere 2 at pos: %10.6f  %10.6f  %10.6f", pA.x(), pA.y(), pA.z());
			fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pA.x(), pA.y(), pA.z());
        } else if (modB == m_objB.get()) {
            //fprintf(m_fptr,"  B contact on sphere 2 at pos: %10.6f  %10.6f  %10.6f", pB.x(), pB.y(), pB.z());
			fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pB.x(), pB.y(), pB.z());
        }
				
		
        const ChVector3d& nrm = plane_coord.GetAxisX();
		const ChVector3d& nrt1 = plane_coord.GetAxisY();
		const ChVector3d& nrt2 = plane_coord.GetAxisZ();
        //fprintf(m_fptr,"  nrm: %10.6f, %10.6f  %10.6f", nrm.x(), nrm.y(), nrm.z());
        //fprintf(m_fptr,"  frc: %12.8e  %12.8e  %12.8e", cforce.x(), cforce.y(), cforce.z());		
        ////printf("  trq: %7.3f, %7.3f  %7.3f", ctorque.x(), ctorque.y(), ctorque.z());
        //fprintf(m_fptr,"  penetration: %12.8e   eff. radius: %7.3f\n", distance, eff_radius);
		
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", nrm.x(), nrm.y(), nrm.z());
		//fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", nrt1.x(), nrt1.y(), nrt1.z());
		//fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", nrt2.x(), nrt2.y(), nrt2.z());
        fprintf(m_fptr,"  %12.8e  %12.8e  %12.8e ", cforce.x(), cforce.y(), cforce.z());        
        fprintf(m_fptr,"  %12.8e   %7.3f\n", distance, eff_radius);

        return true;
    }

    std::shared_ptr<ChBody> m_objA;
    std::shared_ptr<ChBody> m_objB;
	FILE *m_fptr;
};



 
 // Function to write particle positions and radii to a VTK file
void WriteParticlesVTK(ChSystem& sys, const std::string& filename, double h_layer, bool isAgregateOut) {
     // Get the number of particles
     auto body_list= sys.GetBodies();
     
	 std::vector<std::shared_ptr<ChBody>> body_list_new;
	 for (auto body:body_list){
	         if (body->GetCollisionModel() && !body->IsFixed() && body->GetCollisionModel()->GetShapeInstance(0).first->GetType()==0) {
		 	body_list_new.push_back(body);
		 }
	 }
			
	int num_particles = body_list_new.size();
	
     // Create the VTK file and write the header
     std::ofstream vtk_file(filename);
     vtk_file << "# vtk DataFile Version 3.0\n";
     vtk_file << "vtk output\n";
     vtk_file << "ASCII\n";
     vtk_file << "DATASET UNSTRUCTURED_GRID\n";

     // Write the particle positions
     vtk_file << "POINTS " << num_particles << " float\n";
     for (int i = 0; i < num_particles; i++) {
         ChVector3f pos = body_list_new[i]->GetPos();
         vtk_file << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
     }

     // Write the particle IDs
     vtk_file << "\nCELLS " << num_particles << " " << num_particles * 2 << "\n";
     for (int i = 0; i < num_particles; i++) {
         vtk_file << "1 " << i << "\n";
     }

     // Write the cell types
     vtk_file << "\nCELL_TYPES " << num_particles << "\n";
     for (int i = 0; i < num_particles; i++) {
         vtk_file << "1\n";
     }

     // Write the particle radii
     vtk_file << "\nPOINT_DATA " << num_particles << "\n";
     vtk_file << "SCALARS radius float 1\n";
     vtk_file << "LOOKUP_TABLE default\n";
     for (int i = 0; i < num_particles; i++) {
		auto shape=body_list_new[i]->GetCollisionModel()->GetShapeInstance(0).first;
		auto shape_sphere = std::static_pointer_cast<ChCollisionShapeSphere>(shape);
        double Radius = shape_sphere->GetRadius();	
		if (isAgregateOut)
			vtk_file << (Radius-h_layer) << "\n";
		else
			vtk_file << Radius << "\n";
     }

     // Write the particle velocities
     vtk_file << "\nVECTORS velocity float\n";
     for (int i = 0; i < num_particles; i++) {
         ChVector3f vel = body_list_new[i]->GetPosDt();;
         vtk_file << vel.x() << " " << vel.y() << " " << vel.z() << "\n";
     }
	 
	 // Write the particle angular velocities
     vtk_file << "\nVECTORS angular_velocity float\n";
     for (int i = 0; i < num_particles; i++) {
         ChVector3f w = body_list_new[i]->GetFrameCOMToAbs().GetAngVelLocal();
         vtk_file << w.x() << " " << w.y() << " " << w.z() << "\n";
     }

     // Close the file
     vtk_file.close();
}
