// ============================================================================================
// Render simulation data
// 
// DATA ASSUMED TO BE SPECIFIED IN A RIGHT-HAND-FRAME WITH Z UP
// ============================================================================================
#version 3.7;
global_settings { assumed_gamma 1 }

  
// Render a sequence of frames or a single one      
//#declare fnum=abs(frame_number); 
#declare fnum = 1;

#declare datafile = concat("POVRAY/data_", str(fnum,-3,0), ".dat")


// Render objects?
#declare render_objects = true;

// Render static objects?
#declare render_static = false;


// Draw global frame?
#declare draw_global_frame = true;
#declare global_frame_radius = 0.01;
#declare global_frame_len = 10;


// Draw shape frames?
#declare draw_object_frame = false;
#declare object_frame_radius = 0.01;
#declare object_frame_len = 1.25;

// Perspective camera?
#declare cam_perspective = false;
       
// Camera location and look-at (RIGHT-HAND-FRAME with Z up)
#declare cam_loc    = <-5, 0, 1>;
#declare cam_lookat = <0, 0, 1>;

     
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
//   (2) This file must define a macro "mesh_name(col)" which returns a mesh object
//       the input argument is the color for the mesh
//   (3) It is assumed that the mesh vertices are specified in a RIGHT-HAND-FRAME
//       with Z up.
//
#macro RenderMesh(mesh_name, pos, rot, col)
    Parse_String(concat("#include \"", mesh_name,"\""))
    Parse_String(concat("object{", mesh_name, "(col) position(pos,rot)}"))
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
   //right     x*image_width/image_height  // aspect
   // direction z                        // direction and zoom
   // angle 67                           // field (overides direction zoom)
}

// Create a regular point light source 
light_source { 
    1000*(cloc - clookat)  // behind the camera
    color rgb <1,1,1>      // light's color
    //translate <-10, 1500, 2>
}

// ============================================================================================     

// Read datafile
#warning concat("LOADING DATA FILE : ",  datafile, "\n")
#fopen MyDataFile datafile read 
                               
#read (MyDataFile, numObjects, numLinks)
                                     
        // ---------------------------------------------
        //    RENDER OBJECTS (VISUAL ASSETS)
        // ---------------------------------------------
                                           
#for (i, 1, numObjects)                               
              
    #read (MyDataFile, id, active, ax, ay, az, e0, e1, e2, e3, shape)  
    
    //#warning concat("  read id: ", str(id,0,0), "  shape: ", str(shape,0,0), "\n")
             
    #if (draw_object_frame)
       object {
            XYZframe(object_frame_len, object_frame_radius) 
            position(<ax,ay,az>,<e0,e1,e2,e3>)  
       }
    #end
          
    #switch (shape)
                       
        // sphere -------------  
        #case (0)
            # read (MyDataFile, ar)  
            #if (render_objects)
				#if (active)    
					sphere {
					   <0,0,0>, ar
					   position(<ax,ay,az>,<e0,e1,e2,e3>)
					   pigment {color rgbt <.8, 0.6, 0.6,0> }
					   finish {diffuse 1 ambient 0.0 specular .05 } 
					}  
				#else
					#if (render_static)
					sphere {
					   <0,0,0>, ar
					   position(<ax,ay,az>,<e0,e1,e2,e3>)
					   pigment {color rgbt <1, 0.9, 0.9, 0> }
					   finish {diffuse 1 ambient 0.0 specular .05 } 
					}
					#end
				#end   
            #end                                             
        #break     
              
        // box ----------------
        #case (2)
            #read (MyDataFile, hx, hy, hz)
			#if (render_objects)   
				#if (active)    
					box {   
					   <-hx, -hz, -hy>, 
					   <hx, hz, hy>     
					   position(<ax,ay,az>,<e0,e1,e2,e3>)
					   pigment {color rgbt <0, .7, 0.3, 0>}
					   finish {diffuse 1 ambient 0.0 specular .05 } 
					}   
				#else
					#if (render_static)
					box {   
					   <-hx, -hz, -hy>, 
					   <hx, hz, hy>     
					   position(<ax,ay,az>,<e0,e1,e2,e3>)
					   pigment {color rgbt <1, 0.9, 0.9, 0> transmit 0.6}
					   finish {diffuse 1 ambient 0.0 specular .05 } 
					} 
					#end   
				#end
			#end
                                                                      
        #break
              
        // cylinder --------------
        #case (3)
            #read (MyDataFile, ar, hl)
			#if (render_objects)
				#if (active)
					cylinder {
					   <0,0,hl>, <0,0,-hl>, ar      
					   pigment {color rgbt <1, 0.9, 0.9, 0> }
					   position(<ax,ay,az>,<e0,e1,e2,e3>)     
					   finish {diffuse 1 ambient 0.0 specular .05 }
					}   
				#else
					#if (render_static)
					cylinder {
					   <0,0,hl>, <0,0,-hl>, ar      
					   pigment {color rgbt <1, 0.9, 0.9, 0> }
					   position(<ax,ay,az>,<e0,e1,e2,e3>)     
					   finish {diffuse 1 ambient 0.0 specular .05 }
					}
					#end
				#end
			#end
        #break
         
        // rounded cylinder --------------
        #case (10)
            #read (MyDataFile, ar, hl, sr)
			#if (render_objects)
				#if (active)
					object {
					   Round_Cylinder(<0,0,hl + sr>, <0,0,-hl - sr>, ar+sr, sr, 0)     
					   pigment {color rgbt <0.2, 0.4, 0.9, 0> }
					   position(<ax,ay,az>,<e0,e1,e2,e3>)     
					   finish {diffuse 1 ambient 0.0 specular .05 }
					}   
				#else
					#if (render_static)
					object {
					   Round_Cylinder(<0,0,hl + sr>, <0,0,-hl - sr>, ar+sr, sr, 0)      
					   pigment {color rgbt <1, 0.9, 0.9, 0> }
					   position(<ax,ay,az>,<e0,e1,e2,e3>)     
					   finish {diffuse 1 ambient 0.0 specular .05 }
					}
					#end
				#end
			#end
        #break

        // capsule ------------
        #case (7)
            #read (MyDataFile, ar, hl)  
			#if (render_objects)
				#if (active)    
					sphere_sweep {
					   linear_spline
					   2
					   <0,0,-hl>,ar,<0,0,hl>,ar
					   pigment {Candy_Cane scale 0.1}
					   position(<ax,ay,az>,<e0,e1,e2,e3>)     
					   finish {diffuse 1 ambient 0.0 specular .05 }
					}
				#else
					#if (render_static)
					sphere_sweep {
					   linear_spline
					   2
					   <0,0,-hl>,ar,<0,0,hl>,ar
					   pigment {color rgbt<1, 0.9, 0.9, 0>}
					   position(<ax,ay,az>,<e0,e1,e2,e3>)     
					   finish {diffuse 0.5 ambient 0.2 specular .05}
					}    
					#end
				#end
			#end
        #break  
        
        // mesh ----------------
        #case (5)
            #read (MyDataFile, mesh_name)
			#if (render_objects)
				#if (active)  
					#warning concat("Mesh name: ", mesh_name, "\n")
					RenderMesh(mesh_name, <ax,ay,az>,<e0,e1,e2,e3>, rgb<0.9,0.5,0.6>)
				#else
					#if (render_static)
					RenderMesh(mesh_name, <ax,ay,az>,<e0,e1,e2,e3>, rgb<1,0.9,0.9>)
					#end
				#end
			#end
        #break  
           
    #end  // switch (shape)     
     
       
#end  // for objects      
 
        // ---------------------------------------------
        //    RENDER OBJECTS (VISUAL ASSETS)
        // ---------------------------------------------
                                           
#for (i, 1, numLinks)                               
              
    #read (MyDataFile, link)
   
	#switch (link)
		// Revolute -------
		#case (8)
			#read (MyDataFile, px, py, pz, dx, dy, dz)

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

#fclose MyDataFile
          
          
// Draw axes (RHF with z up)  
#if (draw_global_frame)
XYZframe(global_frame_len, global_frame_radius) 
#end 
      
 
// Draw ground plane    
//plane {
//y,0 
//pigment {color rgbt <.5, .5, .5,0> }
//finish {diffuse 0.5 ambient 0.2 reflection {0.2, 1.0 fresnel on}}
//hollow on
//}
// 
//plane {
//x,9 
//pigment {color rgbt <.5, .5, .5,0> }
//finish {diffuse 0.5 ambient 0.2}
//hollow on
//}
//
//plane {
//z,4 
//pigment {color rgbt <.5, .5, .5,0> }
//finish {diffuse 0.5 ambient 0.2}
//hollow on
//}

