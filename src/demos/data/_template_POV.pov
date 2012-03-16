//
// Script to be executed with POV-Ray for rendering a frame of a Chrono::Engine generated animation.
// Most often, in POV-Ray, you run the .INI file, that calls this file multiple times to render
// the many frames of a complete animation.
//
// Plase do not modify the _template_POV.pov , unless you know exactly what you are
// doing, because _template_POV.pov is used by default to build POV scenes by the postprocessing
// unit of Chrono::Engine


#include "debug.inc"
#include "colors.inc" 
#include "textures.inc"           
           
// 
// Function to generate a rotation matrix from a C::E quaternion data   
// 


#macro quatRotation(q)
    #local aa = q.x*q.x;
    #local ab = q.x*q.y;
    #local ac = q.x*q.z;
    #local ad = q.x*q.t;
    #local bb = q.y*q.y;
    #local bc = q.y*q.z;
    #local bd = q.y*q.t;
    #local cc = q.z*q.z;
    #local cd = q.z*q.t;
    #local dd = q.t*q.t;
    #local qq = aa+bb+cc+dd;
    matrix <(aa+bb-cc-dd)/qq, 2*(bc+ad)/qq, 2*(bd-ac)/qq,
                2*(bc-ad)/qq, (aa-bb+cc-dd)/qq, 2*(cd+ab)/qq,
                2*(bd+ac)/qq, 2*(cd-ab)/qq, (aa-bb-cc+dd)/qq,
                0, 0, 0>
#end

#declare apx=0;
#declare apy=0;
#declare apz=0;
#declare aq0=0;
#declare aq1=0;
#declare aq2=0;
#declare aq3=0;  



// 
// Defaults   
// 


		
global_settings { 
        ambient_light rgb<4,4,4> 
}  


background { rgb<1,1,1> }
    

    
        
//     
// --- Default materials, textures, etc.
// 

    

#declare PebbleFinish =
finish {
	ambient 0.07 diffuse 1
	roughness 0.02
	brilliance 2   
}
  
#declare HopperFinish =
finish {
	ambient 0.1 diffuse 1
	specular 0.8 roughness 0.02
	brilliance 2 
	/* 
             reflection {
              0.3                      // minimal reflection value (for variable reflection)
              1.0                        // reflection amount (maximum for variable reflection)
              fresnel on               // realistic variable reflection
              falloff 1.0              // falloff exponent for variable reflection
              exponent 1.0             // influence surface reflection characteristic
              metallic 1.0             // tint reflection in surface color
            }
        */
}
       
       
#macro tx_def()
  pigment{color rgb <1,1,1>} 
#end
  

//     
// --- Light sources
// 

    

  /* 
// An area light (creates soft shadows)
// WARNING: This special light can significantly slow down rendering times!
light_source {
  0*x                 // light's position (translated below)
  color rgb 1.5       // light's color
  area_light
  <0.6, 0, 0> <0, 0.6, 0> // lights spread out across this distance (x * z)
  4, 4                // total number of lights in grid (4x*4z = 16 lights)
  adaptive 0          // 0,1,2,3...
  jitter              // adds random softening of light
  circular            // make the shape of the light circular
  orient              // orient light
  translate <2, 6, -4>   // <x y z> position of light
}
 */   
    
          
         
//
// Macros for showing useful geometries
//

// Macro for showing the COG as a dot 
          
#macro sh_COG(apx, apy, apz, arad)
sphere { 
 <apx,apy,apz>, arad        
 pigment {color rgbt <1,1,0,0> }
 no_shadow
}
#end   
     
// Macro for showing a coordsystem for a COG (with a sphere in center)
     
#macro sh_csysCOG(apx, apy, apz, aq0, aq1, aq2, aq3, asize)
union {
 sphere { 
  <0,0,0>, asize/8        
  pigment {color rgbt <1,1,0,0> }
 }
 cylinder { 
  <0,0,0>, <asize,0,0>, asize/16        
  pigment {color rgbt <1,0,0,0> }
 }
 cylinder { 
  <0,0,0>, <0,asize,0>, asize/16        
  pigment {color rgbt <0,1,0,0> }
 }
 cylinder { 
  <0,0,0>, <0,0,asize>, asize/16        
  pigment {color rgbt <0,0,1,0> }
 }
 quatRotation(<aq0, aq1, aq2, aq3>) 
 translate  <apx, apy, apz>
 no_shadow 
}
#end 

// Macro for showing a coordsystem for a frame (with a cube in center)
     
#macro sh_csysFRM(apx, apy, apz, aq0, aq1, aq2, aq3, asize)
union {
 box { 
  <-asize/8,-asize/8,-asize/8>,<asize/8,asize/8,asize/8>         
  pigment {color rgbt <0.5,0.5,0.6,0> }
 }
 cylinder { 
  <0,0,0>, <asize,0,0>, asize/12        
  pigment {color rgbt <1,0,0,0> }
 }
 cylinder { 
  <0,0,0>, <0,asize,0>, asize/12        
  pigment {color rgbt <0,1,0,0> }
 }
 cylinder { 
  <0,0,0>, <0,0,asize>, asize/12        
  pigment {color rgbt <0,0,1,0> }
 }
 quatRotation(<aq0, aq1, aq2, aq3>) 
 translate  <apx, apy, apz>
 no_shadow 
}
#end


// -----------------------------------------------------------------------------
// OBJECTS TO BE RENDERED ARE AUTOMATICALLY INSERTED AFTER THIS LINE
// THANKS TO THE POSTPROCESSING UNIT OF CHRONO::ENGINE. YOU SHOULD NOT NEED TO
// MODIFY WHAT FOLLOWS.


          
     
                
