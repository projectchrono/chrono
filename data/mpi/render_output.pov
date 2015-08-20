// 
// Render data dumped from the MPI demo "demo_domains.exe" (that must be
// previously run with mpiexec or wmpiexec)
//
// It simply renders the domain boxes and the AABB of the items in the domains
//



#include "debug.inc"
#include "colors.inc" 

        
        
   
   
        
//     
// --- User defined variables (modify them to your needs)
// 
   
        
#declare r_eps=1e-8; 

#declare draw_domains=1; 

#declare draw_domains_all=1;    
#declare draw_domain_onlyN=8;   // if draw_domains_all=0 here set the domain ID to render alone
  
#declare draw_items=1;
#declare draw_items_as_spheres=1;
#declare draw_items_as_cages=0;
#declare draw_items_as_boxes=0;

#declare cage_domain_thick=0.03;
#declare cage_domain_maxsize=200;
#declare cage_domain_clipping=15;
#declare cage_item_thick=0.02;  

#declare contacts_color= rgb<0.0,0.9,0.2>;         

#declare data_dir ="output/"                                 
                                  
//     
// --- Global settings
// 

  


  
    

global_settings { 
        ambient_light rgb<4,4,4> 
}  


background { rgb<1,1,1> }
    
   
// from side right,   

camera {               
       right x*image_width/image_height  
       location <40, 30, 70>*0.3
       direction 50*z 
       look_at <0, -1, 0>  
       angle 20
}    

/*
// from front  
camera { 
        orthographic 
        location <18, 0, 0>  
        look_at <0, 0, 0>  
        angle 14
} 
*/

/*
// from front closeup to nozzle
camera { 
        //orthographic 
        location <0, 0.03, -1.8>  
        look_at <0, 0.03, 0>  
        angle 4
}     
*/

// from top (to see violations of contacts against cylinder  
/* 
camera { 
        orthographic 
        location <0, 30, 0>  
        look_at <0, 0, 0> 
        angle 22
}
 */    
        
//     
// --- Materials, textures, etc.
// 

    

#declare SphereFinish =
finish {
	ambient 0.07 diffuse 1
	specular 0.8 roughness 0.02
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
   

#declare DomainTransparentFinish =
finish {
	ambient 0.02 diffuse 1
	specular 0.8 roughness 0.02
	brilliance 0   
}

//     
// --- Light sources
// 

    

light_source { <100,300,100>,  color rgb<0.8,0.8,0.8>} 

/*
// An area light (creates soft shadows)
// WARNING: This special light can significantly slow down rendering times!
light_source {
  0*x                 // light's position (translated below)
  color rgb 1.5       // light's color
  area_light
  <0.6, 0, 0> <0, 0.6, 0> // lights spread out across this distance (x * z)
  2, 2                // total number of lights in grid (4x*4z = 16 lights)
  adaptive 0          // 0,1,2,3...
  jitter              // adds random softening of light
  circular            // make the shape of the light circular
  orient              // orient light
  translate <4, 20, -20>   // <x y z> position of light
} 

*/
    
                                         

     
#macro make_cage(aminx, aminy, aminz, amaxx, amaxy, amaxz, cage_domain_thick, cage_domain_thock)
                        box{<aminx-cage_domain_thock,   aminy-cage_domain_thock,        aminz-cage_domain_thock >
                            <aminx+cage_domain_thick,   aminy+cage_domain_thick,        amaxz+cage_domain_thock >
                            }
                        box{<aminx-cage_domain_thock,   aminy-cage_domain_thock,        aminz-cage_domain_thock >
                            <aminx+cage_domain_thick,   amaxy+cage_domain_thock,        aminz+cage_domain_thick >
                            }            
                        box{<aminx-cage_domain_thock,   aminy-cage_domain_thock,        aminz-cage_domain_thock >
                            <amaxx+cage_domain_thock,   aminy+cage_domain_thick,        aminz+cage_domain_thick >
                            } 
                        box{<amaxx+cage_domain_thock,   amaxy+cage_domain_thock,        amaxz+cage_domain_thock >
                            <amaxx-cage_domain_thick,   amaxy-cage_domain_thick,        aminz-cage_domain_thock >
                            }
                        box{<amaxx+cage_domain_thock,   amaxy+cage_domain_thock,        amaxz+cage_domain_thock >
                            <amaxx-cage_domain_thick,   aminy-cage_domain_thock,        amaxz-cage_domain_thick >
                            }
                        box{<amaxx+cage_domain_thock,   amaxy+cage_domain_thock,        amaxz+cage_domain_thock >
                            <aminx-cage_domain_thock,   amaxy-cage_domain_thick,        amaxz-cage_domain_thick >
                            }                      
                        box{<aminx-cage_domain_thock,   aminy-cage_domain_thock,        amaxz+cage_domain_thock >
                            <amaxx+cage_domain_thock,   aminy+cage_domain_thick,        amaxz-cage_domain_thick >
                            }
                        box{<aminx-cage_domain_thock,   aminy-cage_domain_thock,        amaxz+cage_domain_thock >
                            <aminx+cage_domain_thick,   amaxy+cage_domain_thock,        amaxz-cage_domain_thick >
                            }      
                        box{<aminx-cage_domain_thock,   amaxy+cage_domain_thock,        aminz-cage_domain_thock >
                            <aminx+cage_domain_thick,   amaxy-cage_domain_thick,        amaxz+cage_domain_thock >
                            }
                        box{<aminx-cage_domain_thock,   amaxy+cage_domain_thock,        aminz-cage_domain_thock >     
                            <amaxx+cage_domain_thock,   amaxy-cage_domain_thick,        aminz+cage_domain_thick >
                            }   
                        box{<amaxx+cage_domain_thock,   aminy-cage_domain_thock,        aminz-cage_domain_thock >
                            <amaxx-cage_domain_thick,   aminy+cage_domain_thick,        amaxz+cage_domain_thock >   
                            }      
                        box{<amaxx+cage_domain_thock,   aminy-cage_domain_thock,        aminz-cage_domain_thock >
                            <amaxx-cage_domain_thick,   amaxy+cage_domain_thock,        aminz+cage_domain_thick >  
                            }
#end
   
       
       

//     
// --- The cage of the domains
// 


#if (draw_domains=1)    
        //#declare domains_data_file = concat(data_dir,"domains", str(frame_number,-4,0), ".dat")
        #declare domains_data_file = concat(data_dir,"domains.dat") 
        
        #warning concat("---- LOADING DOMAINS FILE : ",  domains_data_file, "\n")
             
        #declare aid=0;
        #declare aminx=0;
        #declare aminy=0;
        #declare aminz=0;
        #declare amaxx=0;
        #declare amaxy=0;
        #declare amaxz=0;
        
        #fopen MyDomFile domains_data_file read 
        
        #while (defined(MyDomFile))
                #read (MyDomFile, aid, aminx, aminy, aminz, amaxx, amaxy, amaxz)    
                #if (aminx < -cage_domain_maxsize)
                         #declare aminx = -cage_domain_maxsize;
                #end
                #if (aminy < -cage_domain_maxsize)
                         #declare aminy = -cage_domain_maxsize;
                #end
                #if (aminz < -cage_domain_maxsize)
                         #declare aminz = -cage_domain_maxsize;
                #end
                #if (amaxx > cage_domain_maxsize)
                         #declare amaxx = cage_domain_maxsize;
                #end
                #if (amaxy > cage_domain_maxsize)
                         #declare amaxy = cage_domain_maxsize;
                #end
                #if (amaxz > cage_domain_maxsize)
                         #declare amaxz = cage_domain_maxsize;
                #end   
                #if ((draw_domains_all) | (draw_domain_onlyN=aid))
                  intersection{          
                        box{<-cage_domain_clipping,  -cage_domain_clipping, -cage_domain_clipping >
                             <cage_domain_clipping,   cage_domain_clipping,  cage_domain_clipping >   }
                        union{
                                make_cage(aminx, aminy, aminz, amaxx, amaxy, amaxz, cage_domain_thick*0.5, cage_domain_thick*0.5)
                             }
                        no_shadow 
                        pigment{rgb<0.7,0.4,0.4>}
                        finish {SphereFinish}
                       }
                #if (draw_domains_all=0)
                        box{<aminx,   aminy,  aminz >
                            <amaxx,   amaxy,  amaxz > 
                             no_shadow pigment{rgbt<0.6,0.9,0.9,0.84>} }   
                #end
                #end
        #end
#end   

         

#if (draw_items=1)    
        #declare items_data_file = concat(data_dir,"pos", str(frame_number,-4,0), ".dat")
       
        
        #warning concat("---- LOADING AABB FILE : ",  items_data_file, "\n")
             
        #declare aid=0;
        #declare aitemid=0;
        #declare atype=0;
        #declare aminx=0;
        #declare aminy=0;
        #declare aminz=0;
        #declare amaxx=0;
        #declare amaxy=0;
        #declare amaxz=0;
        
        #fopen MyItemFile items_data_file read 
        
        #while (defined(MyItemFile))
                #read (MyItemFile, aid, aitemid, atype, aminx, aminy, aminz, amaxx, amaxy, amaxz)    
                #if (aminx < -cage_domain_maxsize)
                         #declare aminx = -cage_domain_maxsize;
                #end
                #if (aminy < -cage_domain_maxsize)
                         #declare aminy = -cage_domain_maxsize;
                #end
                #if (aminz < -cage_domain_maxsize)
                         #declare aminz = -cage_domain_maxsize;
                #end
                #if (amaxx > cage_domain_maxsize)
                         #declare amaxx = cage_domain_maxsize;
                #end
                #if (amaxy > cage_domain_maxsize)
                         #declare amaxy = cage_domain_maxsize;
                #end
                #if (amaxz > cage_domain_maxsize)
                         #declare amaxz = cage_domain_maxsize;
                #end 
                #declare  cage_offset_in  = cage_item_thick; 
                #declare  cage_offset_out = 0; 
                #declare  sphererad = 0.1; // (amaxx-aminx)*0.5;
                #if (atype = 1)
                         #declare cage_offset_out = cage_item_thick*0.1; // to show both master/slave cages not overlapping
                         #declare cage_offset_in  = cage_item_thick*1.1;  
                         #declare sphererad = sphererad*1.05;
                #end
                #if ((draw_domains_all) | (draw_domain_onlyN=aid)) 
                  union{      
                        #if (draw_items_as_cages =1)
                                make_cage(aminx, aminy, aminz, amaxx, amaxy, amaxz, cage_offset_in, cage_offset_out)        
                        #end 
                        #if ((draw_items_as_boxes =1) | ((aitemid > 10000)&(aid=0)) )
                                box{<aminx,   aminy,  aminz >
                                    <amaxx,   amaxy,  amaxz > }        
                        #end
                        #if ((draw_items_as_spheres =1) & (aitemid < 10000))
                                sphere{<(aminx+amaxx)*0.5,   (aminy+amaxy)*0.5,  (aminz+amaxz)*0.5 >, sphererad }           
                        #end 
                        #if (atype = 0)
                                pigment{rgb<0.8,0.8,0.8>}
                        #end 
                        #if (atype = 1)
                                pigment{rgb<0.9,0.2,0.2>}
                        #end
                        #if (atype = 2)
                                pigment{rgb<0.2,0.9,0.2>}
                        #end
                        #if (atype = 3)
                                pigment{rgb<0.9,0.9,0.0>}
                        #end
                        //no_shadow 
                        finish {SphereFinish}
                      } 
                #end
        #end
#end     

          
     
      