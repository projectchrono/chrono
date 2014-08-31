#ifndef CHPAC2002_DATA_H
#define CHPAC2002_DATA_H

#include <string>

namespace chrono{

// all the sections are declared first
typedef struct model{
  std::string property_file_format;
  int use_mode;
  double vxlow;
  double longvl;
  std::string tyreside;
} model;

typedef struct dimension{
  double unloaded_radius;
  double width;
  double aspect_ratio;
  double rim_radius;
  double rim_width;
} dimension;

typedef struct shape{
  std::vector<double> radial;
  std::vector<double> width;
} shape;

typedef struct vertical{
  double vertical_stiffness;  // vertical stiffness
  double vertical_damping;    // vertical damping
  double breff; // low load stiffness e.r.r. ???
  double dreff; // peak value e.r.r.
  double freff; // high load stiffness e.r.r.
  double fnomin;// nominal wheel load
} vertical;

typedef struct long_slip_range{
  double kpumin;  // min valid slip
  double kpumax;  // max valid slip
} long_slip_range;

typedef struct slip_angle_range{
  double alpmin;  // min. valid slip angle
  double alpmax;  // max valid slip angle
} slip_angle_range;

typedef struct inclination_angle_range{
  double cammin;  // min valid camber angle (rads)
  double cammax;  // max
} inclination_angle_range;

typedef struct vertical_force_range{
  double fzmin; // min. allowable wheel load
  double fzmax; // max
} vertical_force_range;

typedef struct scaling_coefficients{
  double lfzo;  // scale factor, rated load
  double lcx;   // " ", Fx shape
  double lmux;  // " ", Fx peak friction coef.
  double lex;   // Fx curvature
  double lkx;   // Fx slip stiffness
  double lhx;   // Fx horizontal shift
  double lvx;   // Fx vertical shift
  double lgax;  // Fx camber factor
  double lcy;   // Fy shape factor
  double lmuy;  // Fy peak friction
  double ley;   // Fy curvature
  double lky;   // Fy cornering stiffness
  double lhy;   // Fy horizontal shift
  double lvy;   // Fy vertical shift
  double lgay;  // Fy camber factor
  double ltr;   // Peak pneumatic trail
  double lres;  // residual torque offset
  double lgaz;  // Mz camber factor
  double lxal;  // alpha influence on Fx
  double lyka;  // alpha influence on Fy
  double lvyka; // kappa induced Fy
  double ls;    // moment arm, Fx
  double lsgkp; // relaxation length, Fx
  double lsgal; // relaxation length, Fy
  double lgyr;  // gyroscopic torque
  double lmx;   // overturning couple
  double lvmx;  // vertical shift, Mx
  double lmy;   // rolling resistance torque
} scaling_coefficients;

typedef struct longitudinal_coefficients{
  double pcx1;  // shape factor C,fx
  double pdx1;  // long. friction Mux at Fz,nom
  double pdx2;  // variation of friction Mux w/ load
  double pdx3;  // " w/ camber
  double pex1;  // Long. curvature E,fx at Fz,nom
  double pex2;  // variation of curvature E,fx w/ load
  double pex3;  // " w/ load^2
  double pex4;  // Curvature E,fx while driving
  double pkx1;  // long. slip stiff K,fx/Fz @ Fz,nom
  double pkx2;  // variation " w/ load
  double pkx3;  // exponent " w/ load
  double phx1;  // horizontal shift S,hx @ Fz,nom
  double phx2;  // variation of S,hx w/ load
  double pvx1;  // vertical shift S,vx/Fz @ Fz,nom
  double pvx2;  // variation of shift S,vx/Fz w/ load
  double rbx1;  // slope factor for combined slip Fx reduction
  double rbx2;  // variation of slope Fx reduction w/ kappa
  double rcx1;  // shape factor for combined slip Fx reduction
  double rex1;  // curvature factor, combined Fx
  double rex2;  // ", w/ load
  double rhx1;  // shift factor for combined slip Fx reduction
  double ptx1;  // relaxation length sigkap0/Fz @ Fz,nom
  double ptx2;  // variation of " w/ load
  double ptx3;  // variation of " w/ exponent of load
} longitudinal_coefficients;


typedef struct overturning_coefficients {
  double qsx1;  // lateral force induced overturning moment
  double qsx2;  // camber induced overturning couple
  double qsx3;  // Fy induced overturning couple
} overturning_coefficients;

typedef struct lateral_coefficients{
  double pcy1;  // shape factor C,Fy
  double pdy1;  // lateral friction Muy
  double pdy2;  // variation of friction Muy w/ load
  double pdy3;  // " w/ camber^2
  double pey1;  // lateral curvature E,fy @ Fz,nom
  double pey2;  // variation of curvature E,fy w/ load
  double pey3;  // zero order camber dep. on curvature E,fy
  double pey4;  // variation of curvature E,fy w/ camber
  double pky1;  // max. val of stiffness K,fy/Fz,nom
  double pky2;  // load @ which K,fy reaches maximum value
  double pky3;  // variation of K,fy/Fz,nom w/ camber
  double phy1;  // horizontal shift S,hy @ Fz,nom
  double phy2;  // variation of shift S,hy w/ load
  double phy3;  // " w/ camber
  double pvy1;  // vertical shift in X,vy/Fz @ Fz,nom
  double pvy2;  // variation of shift S,vy/Fz w/ load
  double pvy3;  // " w/ camber
  double pvy4;  // " w/ camber And load
  double rby1;  // slope for combined Fy reduction
  double rby2;  // variation of slope Fy reduction w/ alpha
  double rby3;  // shift term, alpha in Fy slope reduction
  double rcy1;  // shape factor for combined Fy reduction
  double rey1;  // curvature factor, combined Fy
  double rey2;  // " w/ load
  double rhy1;  // shift for combined Fy reduction
  double rhy2;  // " w/ load
  double rvy1;  // kappa induced side force X,vyk/Muy*Fz @ Fz,nom
  double rvy2;  // variation of " w/ load
  double rvy3;  // " w/ camber
  double rvy4;  // " w/ alpha
  double rvy5;  // " w/ kappa
  double rvy6;  // " w/ arctan(kappa)
  double pty1;  // peak val of relaxation length SigAlpha,0/R,0
  double pty2;  // val of Fz/Fz,nom where SigAlpha,0 is extreme
} lateral_coefficients;

typedef struct rolling_coefficients{
  double qsy1;  // rolling resistance, torque
  double qsy2;  // rolling resistance dep. on Fx
  double qsy3;  // " dep. on speed
  double qsy4;  // " dep. on speed^4
} rolling_coefficients;

typedef struct aligning_coefficients{
  double qbz1;  // trail slope B,pt @ Fz,nom
  double qbz2;  // variation of slope B,pt w/ load
  double qbz3;  // " w/ load^2
  double qbz4;  // " w/ camber
  double qbz5;  // " w/ ||camber||
  double qbz9;  // slope Br of residual torque M,zr
  double qbz10; // "
  double qcz1;  // shape C,pt for pneumatic trail
  double qdz1;  // peak trail D,pt'' = D,pt*(Fz/Fz,nom*R,0)
  double qdz2;  // variation of peak D,pt'' w/ load
  double qdz3;  // " w/ camber
  double qdz4;  // " w/ camber^2
  double qdz6;  // peak residual torque D,mr'' = D,mr/(Fz*R,0)
  double qdz7;  // variation of peak factor D,mr'' w/ load
  double qdz8;  // " w/ camber
  double qdz9;  // " w/ camber and load
  double qez1;  // trail curvature E,pt @ Fz,nom
  double qez2;  // variation of curvature E,pt w/ load
  double qez3;  // " w/ load^2
  double qez4;  // " w/ sign(Alpha-t)
  double qez5;  // variation of E,pt w/ camber and sign(Alpha-t)
  double qhz1;  // trial horizontal shift S,ht @ Fz,nom
  double qhz2;  // variation of shift S,ht w/ load
  double qhz3;  // " w/ camber
  double qhz4;  // " w/ camber and load
  double ssz1;  // nom. val of s/R,0, effect of Fx on Mz
  double ssz2;  // variation of distance x/R,0 w/ Fy/Fz,nom
  double ssz3;  // " w/ camber
  double ssz4;  // " w/ load and camber
  double qtz1;  // gyration torque constant
  double mbelt; // belt mass
} aligning_coefficients;

// collect all the subsections into the master struct
typedef struct Pac2002_data{
  struct model model;
  struct dimension dimension;
  struct shape shape;
  struct vertical vertical;
  struct long_slip_range long_slip_range;
  struct slip_angle_range slip_angle_range;
  struct inclination_angle_range inclination_angle_range;
  struct vertical_force_range vertical_force_range;
  struct scaling_coefficients scaling_coefficients;
  struct longitudinal_coefficients longitudinal_coefficients;
  struct overturning_coefficients overturning_coefficients;
  struct lateral_coefficients lateral_coefficients;
  struct rolling_coefficients rolling_coefficients;
  struct aligning_coefficients aligning_coefficients;
} Pac2002_data;

} // end namespace chrono
#endif