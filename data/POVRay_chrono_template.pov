//
// Script to be executed with POV-Ray for rendering a frame of a Chrono::Engine generated animation.
// Most often, in POV-Ray, you run the .INI file, that calls this file multiple times to render
// the many frames of a complete animation.
//
// Do not modify this file , unless you know exactly what you are doing, because this is the default
// script used to build POV scenes by the Chrono::Postprocess module.

#version 3.7;

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
    
  
#default {
  pigment {rgb <1,1,1>}
}
      
        
// A macro to create grids


#macro Raster(RScale, RLine, lcolor)
pigment{
   gradient x scale RScale
   color_map{
     [0.000   color lcolor]
     [0+RLine color lcolor]
     [0+RLine color rgbt<1,1,1,1>]
     [1-RLine color rgbt<1,1,1,1>]
     [1-RLine color lcolor]
     [1.000   color lcolor]
            }
       } 
#end

#macro Grid(Scale,
            Line_thickness,
            Line_color,
            Plane_color)
plane{<0,1,0>, 0
      texture{ pigment{color Plane_color } } 
      texture{ Raster(Scale,  Line_thickness*0.5, Line_color) } 
      texture{ Raster(Scale,  Line_thickness*0.5, Line_color) rotate<0,90,0> }
      no_shadow 
     } 
#end
     
// Use as:
// Grid(0.05,0.04, rgb<0.5,0.5,0.5>, rgbt<1,1,1,1>)



 
      
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
       
       
       
// Macros for showing contact points / contact forces  

#declare ContactFinishA =
finish {
	ambient 0.07 diffuse 1
	specular 0.8 roughness 0.02
	brilliance 2  
}
#declare ContactFinishB =
finish {
	ambient 1 diffuse 0
	specular 0.0 roughness 0.0
	brilliance 0  
}

#declare apx=0;
#declare apy=0;
#declare apz=0;     
#declare anx=0;
#declare any=0;
#declare anz=0;  
#declare afx=0;
#declare afy=0;
#declare afz=0;
#declare draw_contacts_info =0; // if =1 draw only normal force component
   
#macro make_contact(apx, apy, apz, anx, any, anz,  afx, afy, afz)  
        #local vdir = vnormalize(<afx,afy,afz>);
        #local astrengthval = vlength(<afx,afy,afz>); 
        #if(draw_contacts_info=1)  
             #local fnormalcomp= vdot(<afx, afy, afz>,<anx, any, anz>);
             #local vdir = <anx, any, anz>;
             #local astrengthval = fnormalcomp;
        #end
        #local asize = astrengthval * contacts_scale;
        
        #local acolormap = (1/(contacts_colormap_endscale-contacts_colormap_startscale))*(astrengthval-contacts_colormap_startscale);
        #local acolormapclamp =  max(min( acolormap, 1 ),-1);
        #local acolor= contacts_defaultcolor;
        #if(contacts_do_colormap)
                #if(acolormapclamp>0.5) 
                        #local acolor=rgb<(acolormapclamp-0.5)*2, 1-(acolormapclamp-0.5)*2 ,0>; 
                #else
                        #local acolor=rgb<0, acolormapclamp*2 , 1-acolormapclamp*2>;
                #end
        #end     
        #if (asize > contacts_maxsize) 
             #local asize = contacts_maxsize;
             #local acolor=rgb<1,1,0.5>; 
        #end

        #if (draw_contacts_asspheres=1)  
                #if(contacts_scale_mode=1)
                       #local arad = asize;
                #else
                       #local arad = contacts_width;
                #end
                sphere { 
                        <apx,apy,apz>, arad 
                        pigment{acolor}
                        finish {ContactFinishB}
                        no_shadow
                } 
        #end
        #if (draw_contacts_ascylinders=1)
                #if(contacts_scale_mode=1)
                       #local arad = contacts_width;
                       #local alen = asize;
                #else   
                  #if(contacts_scale_mode=2)
                       #local arad = asize; 
                       #local alen = contacts_maxsize;  
                  #else 
                       #local arad = contacts_width; 
                       #local alen = contacts_maxsize;
                  #end
                #end
                cylinder {
                        <apx,apy,apz> -vdir*alen, // <apx-alen*anx, apy-alen*any, apz-alen*anz>,
                        <apx,apy,apz> +vdir*alen, 
                        arad
                        pigment{acolor} 
                        finish {ContactFinishB}
                        no_shadow
                }
        #end
#end
     
        
        
/*       
#macro tx_def()
  pigment{color rgb <1,1,1>} 
#end
*/
  

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
        
        
        
// -----------------------------------------------------------------------------
// OBJECTS TO BE RENDERED ARE AUTOMATICALLY INSERTED AFTER THIS LINE
// THANKS TO THE POSTPROCESSING UNIT OF CHRONO::ENGINE. YOU SHOULD NOT NEED TO
// MODIFY WHAT FOLLOWS.


          
     
                
