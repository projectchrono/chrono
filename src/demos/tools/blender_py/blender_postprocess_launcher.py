import os
import blend_pproc_render
######### SET THESE AS ARGUMENTS

# where to look for the meshes. Use multiple paths if there are more folders to look into
meshes_prefixes = ["C:/codes/Chrono_synchrono_debug/src/data/vehicle/sedan/", 'C:/codes/Chrono_synchrono_debug/src/data/simpleRural/' ]

out_dir = os.path.dirname(os.path.realpath(__file__)) + '/rendered_images/'

#datadir = os.path.dirname(os.path.realpath(__file__))
datadir = 'C:/codes/NSF_project/projectlets_buid/Release/SERVER/POVRAY/'

# resolution: 'HIGH',  'MID', 'LOW' 
res = 'LOW'#'LOW'

camera_mode = "Follow" # 'Fixed' 'Lookat'

# If true, sky is added
use_sky = True
# camera position (unused in follow mode)
camera_pos = (-93.7839, 1.3150, 0.5082)
# Camera target data
targ_bodyid = 0 
targ_shapetypeid = 5 
targ_name = 'sedan_chassis_vis'
# distance from camera target used by "follow", unnused in Lookat and Fixed
camera_dist = (-15.5,-5,1.5)
light_loc=(10, 50, 50)
light_energy=53000
axis_up = 'Z'

blend_pproc_render.bl_render(meshes_prefixes, out_dir, datadir, res, camera_mode, use_sky, camera_pos, targ_bodyid, 
                             targ_shapetypeid, targ_name, camera_dist, axis_up, light_loc, light_energy) 

