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

		
global_settings { 
        ambient_light rgb<4,4,4> 
}  


background { rgb<1,1,1> }
    
   
// from side right,   
camera {               
       right x*image_width/image_height  
       location <1, 1.8, -1>
       direction 2*z 
       look_at <0, 0, 0>  
       angle 36.6
}    
 
/*
// from front  
camera { 
        orthographic 
        location <0, 0.3, -6>  
        look_at <0, 0.3, -2>  
        angle 14
} 

// from side  
camera { 
        orthographic 
        location <8, 0.3, 0>  
        look_at <0, 0.3,  -0.3>  
        angle 14
} 
  */
  
/*
// from top        
camera { 
        orthographic 
        location <0, 6, 0>  
        look_at <0, 0, 0> 
        angle 10
}
*/
    
        
//     
// --- Materials, textures, etc.
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
   
#declare ContactFinish =
finish {
	ambient 0.3 diffuse 0.5
	specular 0.0 roughness 0
	brilliance 0   
} 
#declare ContactFinishB =
finish {
	ambient 0.07 diffuse 1
	specular 0.8 roughness 0.02
	brilliance 2  
}
  

//     
// --- Light sources
// 

    

light_source { <3,6,-4>,  color rgb<1.2,1.2,1.2>} 

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
// --- The background checkered grid 0.5 m
// 

  /*
union {
 plane{y,0} 
 plane{-x,2.0} 
 //pigment{rgb<1,1,1>}
 no_shadow
 //finish { ambient 0.8 }
 pigment{checker rgb<1,1,1>, rgb<0.75,0.75,0.85>} 
 scale <0.5, 0.5, 0.5>   
}       
       
    */    

      
// -----------------------------------------------------------------------------
// OBJECTS TO BE RENDERED ARE AUTOMATICALLY INSERTED AFTER THIS LINE
// THANKS TO THE POSTPROCESSING UNIT OF CHRONO::ENGINE. YOU SHOULD NOT NEED TO
// MODIFY WHAT FOLLOWS.


          
     
                
