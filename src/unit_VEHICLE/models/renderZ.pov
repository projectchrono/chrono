// ============================================================================================
// Render simulation data
// 
// DATA ASSUMED TO BE SPECIFIED IN A RIGHT-HAND-FRAME WITH Z UP
// ============================================================================================
#version 3.7;
global_settings { assumed_gamma 1 }
global_settings { ambient_light rgb<1, 1, 1> }
           
           
// ============================================================================================
// OPTIONS
// ============================================================================================

// -------------------------------------------------------           
// Render a sequence of frames or a single one  
    
//#declare fnum=abs(frame_number); 
#declare fnum = 1;

#declare datafile = concat("POVRAY/data_", str(fnum,-3,0), ".dat")


// -------------------------------------------------------           
// Render objects?
#declare render_objects = true;

// Render static objects?
#declare render_static = false;

//Render links?
#declare render_links = true;
                        
                        
// -------------------------------------------------------
// Dimensions for rendering links           
#declare revolute_radius = 0.02;
#declare revolute_halflen = 0.04;                
 
#declare spherical_radius = 0.035;                
                                              
                                              
// -------------------------------------------------------           
// Draw global frame?
#declare draw_global_frame = false;
#declare global_frame_radius = 0.01;
#declare global_frame_len = 10;

// Draw body frames?
#declare draw_body_frame = false;
#declare body_frame_radius = 0.003;
#declare body_frame_len = 0.75;

// Draw shape frames?
#declare draw_object_frame = false;
#declare object_frame_radius = 0.006;
#declare object_frame_len = 0.5;


// -------------------------------------------------------           
// Perspective camera?
#declare cam_perspective = true;
       
// Camera location and look-at (RIGHT-HAND-FRAME with Z up) 
#declare cam_loc    = <-3, -3, 3>;
#declare cam_lookat = <0, 0, 0>;
   
   
// Camera presets

                
// Vehicle top-down                  
//#declare cam_loc    = <-0.3, 0, 4.5>;
//#declare cam_lookat = <-0.3, 0, 0>;
                  
 
// Front-left top-down                  
//#declare cam_loc    = <-2, -0.7, 1.5>;
//#declare cam_lookat = <-2, -0.7, 0>;
 
                  
// Behind vehicle (settled)
//#declare cam_loc    = <3.5, -2.5, 2.3>;
//#declare cam_lookat = <0, 0, 0.6>;

 
// Front-left suspension                               
//#declare dir = <-2,-1,-2>;
//#declare cam_loc    = <-1, 0, 2.3> + 0.2*dir;
//#declare cam_lookat = <-3, -1, 0.3> + 0.2*dir;
                               
                               
// -------------------------------------------------------           
// Render environment?
#declare draw_environment = true;   


// -------------------------------------------------------           
#declare draw_shadows = true;
                                            
                                            
                                            
// ============================================================================================
// Includes
// ============================================================================================ 
#include "shapes.inc"  
#include "strings.inc"            
#include "textures.inc"              
#include "colors.inc"
  
   
// ============================================================================================
// Macro definitions
// ============================================================================================     

// --------------------------------------------------------------------------------------------          
// Convert a LEFT-HAND-FRAME quaternion into a (4x3) Povray transformation matrix
//
#macro QToMatrix(Q)
  // Use: matrix <M[0].x,M[0].y,M[0].z,M[1].x,M[1].y,M[1].z,M[2].x,M[2].y,M[2].z,M[3].x,M[3].y,M[3].z>
  #local X2 = Q.x + Q.x;
  #local Y2 = Q.y + Q.y;
  #local Z2 = Q.z + Q.z;
  #local XX = Q.x * X2;
  #local XY = Q.x * Y2;
  #local XZ = Q.x * Z2;
  #local YY = Q.y * Y2;
  #local YZ = Q.y * Z2;
  #local ZZ = Q.z * Z2;
  #local TX = Q.t * X2;
  #local TY = Q.t * Y2;
  #local TZ = Q.t * Z2;
  array[4] {<1.0 - (YY + ZZ),XY + TZ,XZ - TY>,<XY - TZ,1.0 - (XX + ZZ),YZ + TX>,<XZ + TY,YZ - TX,1.0 - (XX + YY)>,<0,0,0>}
#end

#macro QMatrix(Q)
  // Use quaternion directly as an object modifier
  #local M = QToMatrix(Q)
  transform { matrix <M[0].x,M[0].y,M[0].z,M[1].x,M[1].y,M[1].z,M[2].x,M[2].y,M[2].z,M[3].x,M[3].y,M[3].z> }
#end
           
// --------------------------------------------------------------------------------------------          
// Set RIGHT-HAND-FRAME location and orientation as an object modifier. 
//
// Input:
//    L  -  location vector <x,y,z>
//    Q  -  orientation quaternion <e0, e1, e2, e3> 
//
// NOTE: L and Q are assumed to be expressed in a right-hand frame with Z up. 
// 
// The conversion from right-hand frames with Z up (input) to left-hand frames with Y up (povray)
// requires:
//    (1) swapping y and z
//    (2) changing the sign of the rotation angle. 
//
#macro position(L, Q)
    #local pL = <L.x, L.z, L.y>;  
    #local pQ = <Q.y,Q.t,Q.z,-Q.x>;  // i.e., <e1,e3,e2,-e0>   
    QMatrix(pQ) 
    translate pL
#end     
     
// --------------------------------------------------------------------------------------------          
// Create a RIGHT-HAND-FRAME quaternion from an angle and an axis
//
#macro Q_from_AngAxis(a, axis)
  #local a2 = a/2;
  #local sina2 = sin(a2);
  #local e0 = cos(a2); 
  #local e1 = sina2 * axis.x;
  #local e2 = sina2 * axis.y;
  #local e3 = sina2 * axis.z;  
  #local Q = <e0,e1,e2,e3>;
  Q
#end

// --------------------------------------------------------------------------------------------          
// Render a RIGHT-HAND-FRAME with Z up
//    
#macro XYZframe(len, rad)
   union {
        cylinder{<0,0,0>, <len,  0,  0>, rad pigment{color rgb<1,0,0>} no_shadow}
        cylinder{<0,0,0>, < 0,  0, len>, rad pigment{color rgb<0,1,0>} no_shadow}
        cylinder{<0,0,0>, < 0, len,  0>, rad pigment{color rgb<0,0,1>} no_shadow}
   }
#end
 
 
// --------------------------------------------------------------------------------------------          
// Render a mesh at specified location and with specified orientation
//
// NOTES:
//   (1) It is assumed that a file [mesh_name].inc exists in the search path
//   (2) This file must define a macro "mesh_name()" which returns a mesh object
//       the input argument is the color for the mesh
//   (3) It is assumed that the mesh vertices are specified in a RIGHT-HAND-FRAME
//       with Z up.
//
// Use as:
//    object {
//       MyMesh(mesh_name, pos, rot, col)
//       [object_modifiers]
//    }
//
#macro MyMesh(mesh_name, pos, rot)
    Parse_String(concat("#include \"", mesh_name,"\""))
    Parse_String(concat(mesh_name, "() position(pos,rot)"))
#end   
  
  
                                                                  
// ============================================================================================     
    
// Set a color of the background (sky)
background { color rgb <1, 1, 1> }
       
// Convert camera location and look_at to LEFT-HAND-FRAME with Y up  
#declare cloc = <cam_loc.x, cam_loc.z, cam_loc.y>;
#declare clookat = <cam_lookat.x, cam_lookat.z,  cam_lookat.y>;     
       
// Perspective (default, not required) camera
camera { 
    #if (cam_perspective)
        perspective   
    #else
        orthographic
    #end
    location cloc
    look_at  clookat  
    angle 60
   //right     x*image_width/image_height  // aspect
   // direction z                        // direction and zoom
   // angle 67                           // field (overides direction zoom)
}
                              
// Create a regular point light source                              
#if (draw_environment = false) 

  #if (draw_shadows)
    light_source { 
      1000*(cloc - clookat)  // behind the camera
      color rgb <1,1,1>      // light's color
      translate <0, 1500, 0>
    }         
  #else
    light_source { 
      1000*(cloc - clookat)  // behind the camera
      color rgb <1,1,1>      // light's color
      translate <0, 1500, 0> 
      shadowless
    }         
  #end

#end


// ============================================================================================     

// Read datafile
#warning concat("LOADING DATA FILE : ",  datafile, "\n")
#fopen MyDataFile datafile read 
                               
#read (MyDataFile, numBodies, numObjects, numLinks)

        // ---------------------------------------------
        // RENDER BODY FRAMES
        // ---------------------------------------------

#for (i, 1, numBodies)
    #read (MyDataFile, id, active, ax, ay, az, e0, e1, e2, e3)
    #if (draw_body_frame & (active | render_static))
       object {
            XYZframe(body_frame_len, body_frame_radius) 
            position(<ax,ay,az>,<e0,e1,e2,e3>)  
       }
    #end
#end

        // ---------------------------------------------
        //    RENDER OBJECTS (VISUAL ASSETS)
        // ---------------------------------------------
                                           
#for (i, 1, numObjects)                               
              
    #read (MyDataFile, id, active, ax, ay, az, e0, e1, e2, e3, shape)  
                 
    #if (draw_object_frame & (active | render_static))
       object {
            XYZframe(object_frame_len, object_frame_radius) 
            position(<ax,ay,az>,<e0,e1,e2,e3>)  
       }
    #end
          
    #switch (shape)
                       
        // sphere -------------  
        #case (0)
            # read (MyDataFile, ar)  
            #if (render_objects & (active | render_static))
			    sphere {
				    <0,0,0>, ar
					position(<ax,ay,az>,<e0,e1,e2,e3>)
					pigment {color rgbt <.8, 0.6, 0.6,0> }
					finish {diffuse 1 ambient 0.0 specular .05 } 
				}  
            #end                                             
        #break     
              
        // box ----------------
        #case (2)
            #read (MyDataFile, hx, hy, hz)
			#if (render_objects & (active | render_static))  
				box {   
				    <-hx, -hz, -hy>, 
					<hx, hz, hy>     
					position(<ax,ay,az>,<e0,e1,e2,e3>)
					pigment {color rgbt <0, .7, 0.3, 0>}
					finish {diffuse 1 ambient 0.0 specular .05 } 
				}   
			#end
                                                                      
        #break
              
        // cylinder --------------
        #case (3)
            #read (MyDataFile, ar, p1x, p1y, p1z, p2x, p2y, p2z)
			#if (render_objects & (active | render_static))
				cylinder {
			        <p1x,p1z,p1y>, <p2x,p2z,p2y>, ar      
					pigment {color rgbt <1, 0.9, 0.9, 0> transmit 0}
					position(<ax,ay,az>,<e0,e1,e2,e3>)     
					finish {diffuse 1 ambient 0.0 specular .05 }
				}   
			#end
        #break
         
        // rounded cylinder --------------
        #case (10)
            #read (MyDataFile, ar, hl, sr)
			#if (render_objects & (active | render_static))
				object {
			        Round_Cylinder(<0,0,hl + sr>, <0,0,-hl - sr>, ar+sr, sr, 0)     
					pigment {color rgbt <0.2, 0.4, 0.9, 0> }
					position(<ax,ay,az>,<e0,e1,e2,e3>)     
					finish {diffuse 1 ambient 0.0 specular .05 }
				}   
			#end
        #break

        // capsule ------------
        #case (7)
            #read (MyDataFile, ar, hl)  
			#if (render_objects & (active | render_static))
				sphere_sweep {
			        linear_spline
					2
					<0,0,-hl>,ar,<0,0,hl>,ar
					pigment {Candy_Cane scale 0.1}
					position(<ax,ay,az>,<e0,e1,e2,e3>)     
					finish {diffuse 1 ambient 0.0 specular .05 }
				}
			#end
        #break  
        
        // mesh ----------------
        #case (5)
            #read (MyDataFile, mesh_name)
			#if (render_objects & (active | render_static))
			    #warning concat("Mesh name: ", mesh_name, "\n")
				object {
			        MyMesh(mesh_name, <ax,ay,az>,<e0,e1,e2,e3>)     
			    }
			#end
        #break  
           
    #end  // switch (shape)     
     
       
#end  // for objects      
 
        // ---------------------------------------------
        //    RENDER LINKS
        // ---------------------------------------------

#if (render_links)                                           
#for (i, 1, numLinks)                               
              
    #read (MyDataFile, link)
   
	#switch (link) 
	    // Spherical ------
	    #case (1)
	        #read (MyDataFile, px, py, pz)
	        sphere {
			    <px,pz,py>, spherical_radius
			    pigment {color Red }}
	    #break
	    
		// Revolute -------
		#case (8)
			#read (MyDataFile, px, py, pz, dx, dy, dz)
            cylinder {
                <px-revolute_halflen*dx,  pz-revolute_halflen*dz, py-revolute_halflen*dy>, 
                <px+revolute_halflen*dx,  pz+revolute_halflen*dz, py+revolute_halflen*dy>, 
                revolute_radius   
                pigment {color Blue}}
		#break

		// Linkspring ------
		#case (25)
			#read (MyDataFile, p1x, p1y, p1z, p2x, p2y, p2z)
			cylinder {<p1x,p1z,p1y>, <p2x,p2z,p2y>, 2 * object_frame_radius  pigment {color Red }} 
		#break

		// LinkEngine ------
		//#case (31)
		//
		//#break

		// Distance constraint -------
		#case (37)
			#read (MyDataFile, p1x, p1y, p1z, p2x, p2y, p2z)
			cylinder {<p1x,p1z,p1y>, <p2x,p2z,p2y>, object_frame_radius  pigment {color Black }}

		#break

	#end  // switch (link)
    
#end  // for links
#end  // if (render_links)   
   
#fclose MyDataFile
          
          
// Draw axes (RHF with z up)  
#if (draw_global_frame)
XYZframe(global_frame_len, global_frame_radius) 
#end 
      
 
#if (draw_environment) 
 
    // sun --------------------------------------------------------------- 
    # if (draw_shadows)
       light_source{<1500,2500,-2500>*100 color rgb<1,1,1>} 
    #else
       light_source{<1500,2500,-2500>*100 color rgb<1,1,1> shadowless} 
    #end        
    
    light_source{<1500,2500, 2500>*100 color rgb<0.5,0.5,0.5> shadowless}

    // sky ---------------------------------------------------------------

    // optional with ground fog
    sky_sphere{ pigment{ color rgb<0.15,0.28,0.75>*0.5}   } 
    // ground fog at the horizon -----------------------------------------
    fog{ fog_type   2
        distance   500
        color      rgb<1,1,1>*0.9
        fog_offset 0.1
        fog_alt    30
        turbulence 1.8
    } //---------------------------------------------------------------
            

/*
    // without ground fog
    sky_sphere{
        pigment{ gradient y
          color_map{
          [0.0 color rgb<1,1,1>             ]
          [0.3 color rgb<0.18,0.28,0.75>*0.6]
          [1.0 color rgb<0.15,0.28,0.75>*0.5] }
          scale 1.05
          translate<0,-0.05,0>
        }
    }  
*/

    // spherical cloud layer --------------------------------------------
    #declare R_planet = 6000000 ;
    #declare R_sky    = R_planet + 2000 ;

    sphere{ <0, -R_planet, 0>, R_sky  hollow
       
        texture{ pigment{ bozo turbulence 0.75
                          octaves 6  omega 0.7 lambda 2  phase 0.00 //0.15*clock
                         color_map {
                          [0.00 color rgb <0.95, 0.95, 0.95> ]
                          [0.05 color rgb <1, 1, 1>*1.25 ]
                          [0.15 color rgb <0.85, 0.85, 0.85> ]
                          [0.55 color rgbt <1, 1, 1, 1>*1 ]
                          [1.00 color rgbt <1, 1, 1, 1>*1 ]
                         } // end color_map 
                         translate< 3, 0,-1>
                         scale <0.3, 0.4, 0.2>*3
                       } // end pigment
                              
                 #if (version = 3.7 )  finish {emission 1 diffuse 0}
                 #else                 finish { ambient 1 diffuse 0}
                 #end 
                 scale 3000
               } // end interior texture
        // no_shadow 
    }

    // ground ------------------------------------------------------------
    sphere{ <0, -R_planet, 0>, R_planet+0.05  
 
         texture{ pigment{color rgb<0.35,0.65,0.0>*0.8}
                  normal {bumps 0.75 scale 0.015}
                } // end of texture
    }
    //--------------------------------------------------------------------
    
#end

