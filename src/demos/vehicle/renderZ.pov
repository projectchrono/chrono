// ============================================================================================
// Render simulation data
// 
// DATA ASSUMED TO BE SPECIFIED IN A RIGHT-HAND-FRAME WITH Z UP
// ============================================================================================

#version 3.7;
global_settings { assumed_gamma 1 }
global_settings { ambient_light rgb<1, 1, 1> }


// ============================================================================================
//
// OPTIONS
//
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
 
#declare spring_radius = 0.02;
  
#declare distance_cnstr_radius = 0.007;

#declare prismatic_halflen = 1;  
#declare prismatic_radius = 0.015;

// -------------------------------------------------------
// Draw global frame?
#declare draw_global_frame = false;
#declare global_frame_radius = 0.01;
#declare global_frame_len = 10;

// Draw body frames?
#declare draw_body_frame = false;
#declare body_frame_radius = 0.004;
#declare body_frame_len = 0.3;


// -------------------------------------------------------------
// CAMERA SETTINGS (perspective/ortographic and location/lookat)
//
// RIGHT-HAND FRAME WITH Z UP
//
// Note: For a vehicle-following camera, overwrite these settings
//       in the "CAMERA" section towards the bottom of this script.
//       This feature requires that body information was saved in
//       the data files (the chassis is assumed to be the first body)


#declare cam_perspective = true;
#declare cam_loc    = <5, -3, 3>;
#declare cam_lookat = <-1, 1, 0>;

  
         // Vehicle view from front
//#declare cam_perspective = true;
//#declare cam_loc    = <3.5, 1, 2>;
//#declare cam_lookat = <-1, -1, 0>;
 
         // Vehicle view from rear
//#declare cam_perspective = true;
//#declare cam_loc    = <-3, 2, 2>;
//#declare cam_lookat = <1, -1.5, 0>;
 
         // Vehicle top-down
//#declare cam_perspective = false;
//#declare cam_loc    = <0, 0, 4.2>;
//#declare cam_lookat = <0, 0, 0>;


         // Front-left suspension (from rear-right)
//#declare cam_perspective = true;
//#declare cam_loc    = <0.4, 0.2, 2>;
//#declare cam_lookat = <4.2, 1.4, 0>;

         // Front-left top-down
//#declare cam_perspective = false;
//#declare cam_loc    = <1.8, 0.7, 1.5>;
//#declare cam_lookat = <1.8, 0.7, 0>;

         // Front-left back view
//#declare cam_perspective = false;
//#declare cam_loc    = <0.8, 0.7, 1>;
//#declare cam_lookat = <1.8, 0.7, 1>;


         // Front-right suspension (from rear-right)
//#declare cam_perspective = true;
//#declare cam_loc    = <0.8, -1.2, 1.5>;
//#declare cam_lookat = <4, -0.2, 0>;

         // Front-right top-down
//#declare cam_perspective = false;
//#declare cam_loc    = <1.8, -0.7, 1.5>;
//#declare cam_lookat = <1.8, -0.7, 0>;
 
         // Front-right back view
//#declare cam_perspective = false;
//#declare cam_loc    = <0.8, -0.7, 1>;
//#declare cam_lookat = <1.8, -0.7, 1>;


         // Rear-left suspension (from rear)
//#declare cam_perspective = true;
//#declare cam_loc    = <-3, 0.2, 1.5>;
//#declare cam_lookat = <1.2, 1.4, 0>;

         // Rear-left top-down
//#declare cam_perspective = false;
//#declare cam_loc    = <-1.6, 0.7, 1.5>;
//#declare cam_lookat = <-1.6, 0.7, 0>;


         // Rear-right suspension (from rear)
//#declare cam_perspective = true;
//#declare cam_loc    = <-2.5, -1.2, 1.3>;
//#declare cam_lookat = <1, -0.2, 0>;

         // Rear-right top-down
//#declare cam_perspective = false;
//#declare cam_loc    = <-1.6, -0.7, 1.5>;
//#declare cam_lookat = <-1.6, -0.7, 0>;


         // Steering mechanism (from rear-right)
//#declare cam_perspective = true;
//#declare cam_loc    = <0.2, -0.8, 2>;
//#declare cam_lookat = <4.2, 1.4, 0>;


// -------------------------------------------------------
// Render environment?
#declare draw_environment = true;


// -------------------------------------------------------
#declare draw_shadows = true;


// ============================================================================================  
// 
// INCLUDES
//
// ============================================================================================   

#include "shapes.inc"
#include "strings.inc"
#include "textures.inc"
#include "colors.inc"

// ============================================================================================ 
//
// MACRO DEFINITIONS
//
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
   #local White_texture = texture {pigment{color rgb<1,1,1>}   finish{phong 1}}
   #local Red_texture   = texture {pigment{color rgb<0.8,0,0>} finish{phong 1}}
   #local Green_texture = texture {pigment{color rgb<0,0.8,0>} finish{phong 1}}
   #local Blue_texture  = texture {pigment{color rgb<0,0,0.8>} finish{phong 1}}  
   
   union {  
        // X-axis
        cylinder{
          <0,0,0>, <len,  0,  0>, rad
          texture{checker texture{Red_texture} texture{White_texture} scale len/10 translate<0,rad,rad>}
          no_shadow
        }  
        cone{
          <len, 0, 0>, 4*rad, <len + 8*rad, 0, 0>, 0
          texture{Red_texture}
          no_shadow
        } 
              
        // Y-axis
        cylinder{
          <0,0,0>, < 0,  0, len>, rad
          texture{checker texture{Green_texture} texture{White_texture} scale len/10 translate<rad,rad,0>}
          no_shadow
        }   
        cone{
          <0, 0, len>, 4*rad, <0, 0, len + 8*rad>, 0
          texture{Green_texture}
          no_shadow
        }
            
        // Z-axis
        cylinder{
          <0,0,0>, < 0, len,  0>, rad
          texture{checker texture{Blue_texture} texture{White_texture} scale len/10 translate<rad,0,rad>}
          no_shadow
        } 
        cone{
          <0, len, 0>, 4*rad, <0, len + 8*rad, 0>, 0
          texture{Blue_texture}
          no_shadow
        }
   }
#end


// --------------------------------------------------------------------------------------------
// Render a mesh at specified location and with specified orientation
//
// NOTES:
//   (1) It is assumed that a file [mesh_name].inc exists in the search path
//   (2) This file must define a macro "mesh_name()" which returns a mesh object
//   (3) The mesh vertices are specified in a RIGHT-HAND-FRAME with Z up.
//
// Use as:
//    object {
//       MyMesh(mesh_name, pos, rot)
//       [object_modifiers]
//    }
//
#macro MyMesh(mesh_name, pos, rot)
    Parse_String(concat("#include \"", mesh_name,"\""))
    Parse_String(concat(mesh_name, " position(pos,rot)"))
#end   


// --------------------------------------------------------------------------------------------          
// Render a curve at specified location and with specified orientation
//
// NOTES:
//    (1) It is assumed that a file [curve_name].inc exists in the search path
//    (2) This file muct define a macro "curve_name()" which return an object
//
// Use as:
//    object {
//       MyCurve(curve_name, pos, rot)
//       [object_modifiers]
//    }
//
#macro MyCurve(curve_name, pos, rot)
    Parse_String(concat("#include \"", curve_name,"\""))
    Parse_String(concat(curve_name, " position(pos,rot)"))
#end


// ============================================================================================
//
// READ DATA AND RENDER SCENE
//
// ============================================================================================

// Open data file
#warning concat("LOADING DATA FILE : ",  datafile, "\n")
#fopen MyDataFile datafile read 

// Read first line: number of bodies, visualization objects, and links
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

    // Save the location of body 0 as 'chassis_loc'.
    // This can be used to move the camera relative to this body (see "CAMERA" section below)
    #if (id = 0) 
       #declare chassis_loc = <ax, ay, az>;
    #end
#end

        // ---------------------------------------------
        //    RENDER OBJECTS (VISUAL ASSETS)
        // ---------------------------------------------

#for (i, 1, numObjects)

    #read (MyDataFile, id, active, ax, ay, az, e0, e1, e2, e3, cR, cG, cB, shape)

    #switch (shape)

        // sphere -------------  
        #case (0)
            # read (MyDataFile, ar)  
            #if (render_objects & (active | render_static))
               sphere {
                 <0,0,0>, ar
                 position(<ax,ay,az>,<e0,e1,e2,e3>)
                 pigment {color rgbt <cR, cG, cB, 0> }
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
                pigment {color rgbt <cR, cG, cB, 0>}
                finish {diffuse 1 ambient 0.0 specular .05 } 
              }
           #end
        #break

        // cylinder --------------
        #case (3)
            #read (MyDataFile, ar, p1x, p1y, p1z, p2x, p2y, p2z)
            #if (p1x = p2x & p1y = p2y & p1z = p2z) 
                 #warning concat("DEGENERATE CYLINDER : ",  str(id,-3,0), "\n")
            #end
            #if (render_objects & (active | render_static))
               cylinder {
                 <p1x,p1z,p1y>, <p2x,p2z,p2y>, ar
                 pigment {color rgbt <cR, cG, cB, 0> transmit 0}
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
                 pigment {color rgbt <cR, cG, cB, 0> }
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
                pigment {color rgbt <cR, cG, cB, 0> }
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
               object { MyMesh(mesh_name, <ax,ay,az>,<e0,e1,e2,e3>) }
            #end
        #break

        // Bezier curve ----------
        #case (12)
             #read (MyDataFile, curve_name) 
             #if (render_objects)
                #warning concat("Curve name: ", curve_name, "\n")
                object { MyCurve(curve_name, <ax,ay,az>,<e0,e1,e2,e3>) }
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
            pigment{Bronze2}
        }
        #break
  
        // Revolute -------
        #case (0)
            #read (MyDataFile, px, py, pz, dx, dy, dz)
            cylinder {
                  <px-revolute_halflen*dx,  pz-revolute_halflen*dz, py-revolute_halflen*dy>, 
                  <px+revolute_halflen*dx,  pz+revolute_halflen*dz, py+revolute_halflen*dy>, 
                  revolute_radius   
                  pigment{Bronze2}
            }
        #break
  
        // Prismatic -------
        #case (2)
            #read (MyDataFile, px, py, pz, dx, dy, dz)
            cylinder {
                  <px-prismatic_halflen*dx,  pz-prismatic_halflen*dz, py-prismatic_halflen*dy>, 
                  <px+prismatic_halflen*dx,  pz+prismatic_halflen*dz, py+prismatic_halflen*dy>, 
                  prismatic_radius
                  pigment {color Bronze2}
              }
        #break
  
        // Universal ----
        #case (3)
            #read (MyDataFile, px, py, pz, ux, uy, uz, vx, vy, vz)
            cylinder {
                  <px-revolute_halflen*ux,  pz-revolute_halflen*uz, py-revolute_halflen*uy>, 
                  <px+revolute_halflen*ux,  pz+revolute_halflen*uz, py+revolute_halflen*uy>, 
                  revolute_radius   
                  pigment{Bronze2}
            }  
            cylinder {
                  <px-revolute_halflen*vx,  pz-revolute_halflen*vz, py-revolute_halflen*vy>, 
                  <px+revolute_halflen*vx,  pz+revolute_halflen*vz, py+revolute_halflen*vy>, 
                  revolute_radius
                  pigment{Bronze2}
            }
        #break
  
        // Linkspring ------
        #case (6)
            #read (MyDataFile, p1x, p1y, p1z, p2x, p2y, p2z)
            cylinder {
              <p1x,p1z,p1y>, <p2x,p2z,p2y>, spring_radius
              ////pigment {color Orange }
              texture{Peel scale revolute_halflen/2}
            } 
            //sphere {<p1x,p1z,p1y> 1.01 * spring_radius pigment{Orange}} 
            //sphere {<p2x,p2z,p2y> 1.01 * spring_radius pigment{Orange}} 
        #break
    
        // LinkspringCB ------
        #case (7)
            #read (MyDataFile, p1x, p1y, p1z, p2x, p2y, p2z)
            cylinder {
              <p1x,p1z,p1y>, <p2x,p2z,p2y>, spring_radius 
              ////pigment {color Orange } 
              texture{Peel scale revolute_halflen/2}
            }
            //sphere {<p1x,p1z,p1y> 1.01 * spring_radius pigment{Orange}} 
            //sphere {<p2x,p2z,p2y> 1.01 * spring_radius pigment{Orange}} 
        #break
  
        // LinkEngine ------
        #case (5)
            #read (MyDataFile, px, py, pz, dx, dy, dz)
            cylinder {
                  <px-revolute_halflen*dx,  pz-revolute_halflen*dz, py-revolute_halflen*dy>, 
                  <px+revolute_halflen*dx,  pz+revolute_halflen*dz, py+revolute_halflen*dy>, 
                  spring_radius   
                  pigment{Scarlet}
            }
        #break
  
        // Distance constraint -------
        #case (4)
            #read (MyDataFile, p1x, p1y, p1z, p2x, p2y, p2z)
            cylinder {
              <p1x,p1z,p1y>, <p2x,p2z,p2y>, distance_cnstr_radius
              pigment {color DarkSlateGray }
            }
        #break
  
    #end  // switch (link)
      
  #end  // for links
#end  // if (render_links)

// Done processing data file.
#fclose MyDataFile

// ---------------------------------------
// Draw axes (RHF with z up)

#if (draw_global_frame)
    XYZframe(global_frame_len, global_frame_radius)
#end 


// ============================================================================================
//
// CAMERA
//
// ============================================================================================

// ---------------------------------------------------------------------------------------------
// If desired, overwrite camera location and look_at point, using for example, the location
// of body 0 (chassis_loc); nothe that this assumed body information was saved in the data file.
// Remember to use a right-hand frame coordinate frame with Z up.

// Follow vehicle (behind)
//#declare cam_loc = <chassis_loc.x - 10, -130, 3>;
//#declare cam_lookat = <chassis_loc.x + 10, -120, 0>;

// Follow vehicle (front)
//#declare cam_loc = <chassis_loc.x + 8, -130, 3>;
//#declare cam_lookat = <chassis_loc.x - 8, -120, -2>;

// Follow vehicle (from a fixed location)
//#declare cam_loc = <-63, -135, 3>;
//#declare cam_lookat = <chassis_loc.x, chassis_loc.y, -2>;

// Follow vehicle (front suspension view, from front)
//#declare cam_loc = <chassis_loc.x + 4, chassis_loc.y - 2, 0.5>;
//#declare cam_lookat = <chassis_loc.x - 4, chassis_loc.y + 2, 0>;



// ---------------------------------------------------------------------------------------------
// Convert camera location and look_at to LEFT-HAND-FRAME with Y up
#declare cloc = <cam_loc.x, cam_loc.z, cam_loc.y>;
#declare clookat = <cam_lookat.x, cam_lookat.z,  cam_lookat.y>;

// Create the camera
camera { 
    #if (cam_perspective)
        perspective   
    #else
        orthographic
    #end
    location cloc
    look_at  clookat  
    angle 60
}


// ============================================================================================     
//
// ENVIRONMENT
//
// ============================================================================================     

#if (draw_environment) 
  
    // Lights
    # if (draw_shadows)
       light_source{<1500,2500,-2500>*100 color rgb<1,1,1>} 
    #else
       light_source{<1500,2500,-2500>*100 color rgb<1,1,1> shadowless} 
    #end        
    
    light_source{<1500,2500, 2500>*100 color rgb<0.5,0.5,0.5> shadowless}
   
    // Sky with fog
    sky_sphere{ pigment{ color rgb<0.15,0.28,0.75>*0.5}   } 

    fog{ fog_type   2
        distance   1000
        color      rgb<1,1,1>*0.9
        fog_offset 0.1
        fog_alt    30
        turbulence 1.8
    }           

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

    // Spherical cloud layer
    #declare R_planet = 6000000 ;
    #declare R_sky    = R_planet + 2000 ;

    sphere{
      <0, -R_planet, 0>, R_sky  hollow   
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

    // Ground
    sphere{
      <0, -R_planet, 0>, R_planet+0.05  
      //texture{ pigment{color rgb<0.35,0.65,0.0>*0.8} normal {bumps 0.75 scale 0.015} }
      texture{ pigment{color rgb<0.45,0.55,0.45>*0.8} normal {bumps 0.75 scale 0.015} }
    }
    
#else

  // Set a color of the background (sky)
  background { color rgb <1, 1, 1> }
   
   // Create a regular point light source behind the camera
  #if (draw_shadows)
    light_source { 
      1500*(cloc - clookat)
      color rgb <1,1,1>
      translate <0, 1500, 0>
    }         
  #else
    light_source { 
      1500*(cloc - clookat)
      color rgb <1,1,1>
      translate <0, 1500, 0> 
      shadowless
    }
  #end

#end // if (draw_environment)
