//==================================================
//POV-Ray Materials list
//==================================================
//This file has been created by PoseRay v3.13.23.587
//3D model to POV-Ray/Moray Converter.
//Author: FlyerX
//Email: flyerx_2000@yahoo.com
//Web: https://sites.google.com/site/poseray/
//==================================================
//Files needed to run the POV-Ray scene:
//hmmwv_chassis_POV_main.ini (initialization file - open this to render)
//hmmwv_chassis_POV_scene.pov (scene setup of cameras, lights and geometry)
//hmmwv_chassis_POV_geom.inc (geometry)
//hmmwv_chassis_POV_mat.inc (materials)
// 
 
//Macros used by PoseRay to manipulate pigments
//October 06, 2010
//By FlyerX 

#macro pigment_gray(p1)
//converts to grayscale
#local PR1 = function {pigment{p1}}
#local PR_Gray=function (x,y,z) {0.2126*PR1(x,y,z).red+0.7152*PR1(x,y,z).green+0.0722*PR1(x,y,z).blue}
function{PR_Gray(x,y,z)}
#end  

#macro pigment_multiply(p1,p2)
//multiplies two pigments
#local PR1 = function {pigment{p1}}
#local PR2 = function {pigment{p2}}
#local PR_FRed=function (x,y,z) {PR1(x,y,z).red*PR2(x,y,z).red}
#local PR_FGrn=function (x,y,z) {PR1(x,y,z).green*PR2(x,y,z).green}
#local PR_FBlu=function (x,y,z) {PR1(x,y,z).blue*PR2(x,y,z).blue} 
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgb 0][1 rgb <1,0,0>*3]}]  //add each color and then divide by 3
    [function{PR_FGrn(x,y,z)} color_map{[0 rgb 0][1 rgb <0,1,0>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgb 0][1 rgb <0,0,1>*3]}]    
  }
#end       

#macro pigment_transmit(p1,tr)
//sets the transparency to tr
#local PR1 = function {pigment{p1}} 
#local PR_FRed=function (x,y,z) {PR1(x,y,z).red}
#local PR_FGrn=function (x,y,z) {PR1(x,y,z).green}
#local PR_FBlu=function (x,y,z) {PR1(x,y,z).blue} 
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgbft <0,0,0,0,tr>][1 rgbft <1,0,0,0,tr/3>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgbft <0,0,0,0,tr>][1 rgbft <0,1,0,0,tr/3>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgbft <0,0,0,0,tr>][1 rgbft <0,0,1,0,tr/3>*3]}]   
  }
#end  

#macro pigment_process(p1,p2,tr)
//multiplies two pigments and sets the transparency to tr
#local PR1 = function {pigment{p1}}
#local PR2 = function {pigment{p2}}
#local PR_FRed=function (x,y,z) {PR1(x,y,z).red*PR2(x,y,z).red}
#local PR_FGrn=function (x,y,z) {PR1(x,y,z).green*PR2(x,y,z).green}
#local PR_FBlu=function (x,y,z) {PR1(x,y,z).blue*PR2(x,y,z).blue} 
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgbft <0,0,0,0,tr>][1 rgbft <1,0,0,0,tr/3>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgbft <0,0,0,0,tr>][1 rgbft <0,1,0,0,tr/3>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgbft <0,0,0,0,tr>][1 rgbft <0,0,1,0,tr/3>*3]}]    
  }
#end   


#macro pigment_add(p1,p2)
//adds two pigments - values are clamped from 0 to 1
#local PR1 = function {pigment{p1}}
#local PR2 = function {pigment{p2}}
#local PR_FRed=function (x,y,z) {clamp(PR1(x,y,z).red+PR2(x,y,z).red,0,1)}
#local PR_FGrn=function (x,y,z) {clamp(PR1(x,y,z).green+PR2(x,y,z).green,0,1)}
#local PR_FBlu=function (x,y,z) {clamp(PR1(x,y,z).blue+PR2(x,y,z).blue,0,1)}
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgb 0][1 rgb <1,0,0>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgb 0][1 rgb <0,1,0>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgb 0][1 rgb <0,0,1>*3]}]
  }
#end 

#macro pigment_substract(p1,p2)
//substracts two pigments - values are clamped from 0 to 1
#local PR1 = function {pigment{p1}}
#local PR2 = function {pigment{p2}}
#local PR_FRed=function (x,y,z) {clamp(PR1(x,y,z).red-PR2(x,y,z).red,0,1)}
#local PR_FGrn=function (x,y,z) {clamp(PR1(x,y,z).green-PR2(x,y,z).green,0,1)}
#local PR_FBlu=function (x,y,z) {clamp(PR1(x,y,z).blue-PR2(x,y,z).blue,0,1)}
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgb 0][1 rgb <1,0,0>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgb 0][1 rgb <0,1,0>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgb 0][1 rgb <0,0,1>*3]}]
  }
#end

#macro pigment_power(p1,n)
//raises a pigment to a power - works with images with no mask (jpeg, bmp...)
#local PR1 = function {pigment{p1}}
#local PR_FRed=function (x,y,z) {pow(PR1(x,y,z).red,n)}
#local PR_FGrn=function (x,y,z) {pow(PR1(x,y,z).green,n)}
#local PR_FBlu=function (x,y,z) {pow(PR1(x,y,z).blue,n)}
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgb 0][1 rgb <1,0,0>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgb 0][1 rgb <0,1,0>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgb 0][1 rgb <0,0,1>*3]}]
  }
#end 


#macro pigment_step(p1,s)
//makes a pigment black if color < s or white otherwise
#local PR1 = function {pigment{p1}}
#local PR_FRed=function (x,y,z) {0.5*(sgn(PR1(x,y,z).red-s)+1)}
#local PR_FGrn=function (x,y,z) {0.5*(sgn(PR1(x,y,z).green-s)+1)}
#local PR_FBlu=function (x,y,z) {0.5*(sgn(PR1(x,y,z).blue-s)+1)}
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgb 0][1 rgb <1,0,0>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgb 0][1 rgb <0,1,0>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgb 0][1 rgb <0,0,1>*3]}]
  }
#end  

#macro pigment_luminance(p1)
//calculate luminance from RGB colors
#local PR1 = function {pigment{p1}}
#local PR_FRed=function (x,y,z) {0.2126*PR1(x,y,z).red + 0.7152*PR1(x,y,z).green + 0.0722*PR1(x,y,z).blue}
#local PR_FGrn=function (x,y,z) {0.2126*PR1(x,y,z).red + 0.7152*PR1(x,y,z).green + 0.0722*PR1(x,y,z).blue}
#local PR_FBlu=function (x,y,z) {0.2126*PR1(x,y,z).red + 0.7152*PR1(x,y,z).green + 0.0722*PR1(x,y,z).blue}
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgb 0][1 rgb <1,0,0>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgb 0][1 rgb <0,1,0>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgb 0][1 rgb <0,0,1>*3]}]
  }
#end  

#macro pigment_invert(p1)
//inverts the color -> 1-RGB
#local PR1 = function {pigment{p1}}
#local PR_FRed=function (x,y,z) {1-PR1(x,y,z).red}
#local PR_FGrn=function (x,y,z) {1-PR1(x,y,z).green}
#local PR_FBlu=function (x,y,z) {1-PR1(x,y,z).blue}
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgb 0][1 rgb <1,0,0>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgb 0][1 rgb <0,1,0>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgb 0][1 rgb <0,0,1>*3]}]
  }
#end  

#macro pigment_mix(p1,p2,xv)
//mixes two pigments on a sliding scale xv from 0 to 1 
#local PR1 = function {pigment{p1}}
#local PR2 = function {pigment{p2}}
#local PR_FRed=function (x,y,z) {PR1(x,y,z).red*(1-xv)+PR2(x,y,z).red*xv}
#local PR_FGrn=function (x,y,z) {PR1(x,y,z).green*(1-xv)+PR2(x,y,z).green*xv}
#local PR_FBlu=function (x,y,z) {PR1(x,y,z).blue*(1-xv)+PR2(x,y,z).blue*xv}
average pigment_map{
    [function{PR_FRed(x,y,z)} color_map{[0 rgb 0][1 rgb <1,0,0>*3]}]
    [function{PR_FGrn(x,y,z)} color_map{[0 rgb 0][1 rgb <0,1,0>*3]}]
    [function{PR_FBlu(x,y,z)} color_map{[0 rgb 0][1 rgb <0,0,1>*3]}]
  }
#end

//=======================================

#macro power_ramp(xv,xa,xb,ya,yb,pw)
//generates a float with a power ramp from (xa,ya) to (xb,yb) with power pw and argument xv. 
//ramp slope is zero at xb
  ((ya-yb)/pow((xa-xb),pw))*pow((xv-xb),pw)+yb
#end

#macro power_ramp2(xv,xa,xb,ya,yb,pw)
//generates a float with a power ramp from (xa,ya) to (xb,yb) with power pw and argument xv. 
//ramp slope is zero at xa
  ((yb-ya)/pow((xb-xa),pw))*pow((xv-xa),pw)+ya
#end
 
//==================================================
//Map definitions
 
//==================================================
//Material definitions
#declare Wshld_Frame_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Body_Drvr_Side_=
#declare PR_DIFFUSE=pigment {color srgb <0.82,0.71,0.51> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Body_Psngr_Side_=
#declare PR_DIFFUSE=pigment {color srgb <0.82,0.71,0.51> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Hood_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Infinite_Black_=
#declare PR_DIFFUSE=pigment {color srgb <0.00,0.00,0.00> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Firewall_=
#declare PR_DIFFUSE=pigment {color srgb <0.82,0.71,0.51> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Front_End_=
#declare PR_DIFFUSE=pigment {color srgb <0.73,0.81,0.00> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Rear_End_=
#declare PR_DIFFUSE=pigment {color srgb <0.73,0.81,0.00> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Lower_Chassis_=
#declare PR_DIFFUSE=pigment {color srgb <0.19,0.19,0.19> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Exterior_Tan_=
#declare PR_DIFFUSE=pigment {color srgb <0.82,0.71,0.51> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Exhaust_Pipe_=
#declare PR_DIFFUSE=pigment {color srgb <0.59,0.59,0.59> transmit 0/100 }


#declare F1=finish{specular albedo 0.5019608 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Interior_=
#declare PR_DIFFUSE=pigment {color srgb <0.75,0.75,0.75> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Chrome_Reflector_=
#declare PR_DIFFUSE=pigment {color srgb <0.80,0.80,0.80> transmit 0/100 }


#declare F1=finish{specular albedo 0.5019608 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare BrakeLite_Lenses_=
#declare PR_DIFFUSE=pigment {color srgb <0.38,0.00,0.00> transmit 15/100 }


#declare F1=finish{specular albedo 0.4772549 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Seat_Cushions_=
#declare PR_DIFFUSE=pigment {color srgb <0.46,0.45,0.33> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Flat_Mirror_=
#declare PR_DIFFUSE=pigment {color srgb <0.55,0.55,0.55> transmit 0/100 }


#declare F1=finish{specular albedo 0.5019608 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Window_Glass_=
#declare PR_DIFFUSE=pigment {color srgb <0.25,0.26,0.24> transmit 76/100 }


#declare F1=finish{specular albedo 0.5019608 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Gunmetal_=
#declare PR_DIFFUSE=pigment {color srgb <0.39,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Trnsmssn_Cover_=
#declare PR_DIFFUSE=pigment {color srgb <0.30,0.29,0.29> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Steering_Wheel_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.38,0.31> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Tailgate_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare PsngrSide_Door_2_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare PsngrSide_Door_1_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Dash_Panel_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Instruments_=
#declare PR_DIFFUSE=pigment {color srgb <0.38,0.38,0.38> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Hatch_Cover_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare DrvrSide_Door_1_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare DrvrSide_Door_2_=
#declare PR_DIFFUSE=pigment {color srgb <0.40,0.40,0.40> transmit 0/100 }


#declare F1=finish{specular albedo 0.4509804 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
#declare Headlight_Lenses_=
#declare PR_DIFFUSE=pigment {color srgb <0.44,0.44,0.44> transmit 80/100 }


#declare F1=finish{specular albedo 0.5019608 //Max highlight intensity: 0 -> 1
                   roughness 0.00286312 //surface roughness: 0.0005 -> 1
                   phong albedo 0 phong_size 0 //Phong specular disabled
                   
                   ambient <0,0,0> emission srgb <0.00,0.00,0.00> //ambient RGB color: 0->1 for each component
                   diffuse albedo 0.6 
                   reflection{0 } conserve_energy //reflection amount: 0 -> 1
                   }


#declare T1=texture{
                   pigment{uv_mapping PR_DIFFUSE }
                            finish{F1}
                    }


material{
         texture{/*uv_mapping*/
                 T1
                }
}
//==================================================
