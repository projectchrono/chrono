import os
import blend_pproc_render

"""  SET THESE AS ARGUMENTS  """

"""  SET THESE AS ARGUMENTS  """

# where to look for the meshes. Use multiple paths if there are more folders to look into
meshes_prefixes = ["C:/Users/benatti/codes/synchrono/src/data/vehicle/sedan/", 'C:/Users/benatti/codes/synchrono/src/data/simpleRural/' ]
# directory where the generated images are saved
out_dir = os.path.dirname(os.path.realpath(__file__)) + '/rendered_images/'

# paths where the post-process outputs (.dat, .chpf) are looked for. There might be multiple files in different directories
# defining different bodies in the same timestep (cosimulation) but the prefixes must be the same
# e.g. : path1/001.dat, path1/001.chpf, path2/001.dat will be processed together.
# Please, in follow and lookat modes the tracked object must be in the first element of the list
#datadir = os.path.dirname(os.path.realpath(__file__))
datadir = ['./', 'C:/Users/benatti/codes/blender/NADS/dat_files/']

# resolution: 'HIGH',  'MID', 'LOW': divide by 1,4,16 the default HD resolution of 3840x2160
res = 'MID'#'LOW'

# Camera options: 'Follow', 'Fixed', 'Lookat'
camera_mode = "Fixed" # 'Fixed' 'Lookat'

# If true, sky is added
use_sky = True
# camera position (unused in follow mode)
camera_pos = (200,0,200)

# Camera target data: some keys might be unused depending on the type
target = dict([
    # ID of the body to target (lookat and follow modes).
    ('bodyid' , 0),
    # ID of the shape on the body to target (lookat and follow modes). The body might have many shapes
    ('shapetypeid' , 5),
    # name of the mesh on the body to target (lookat and follow modes). The body might have many vis meshes
    ('name' , 'sedan_chassis_vis'),
    # Point to look at. Used only by Fixed
    ('position', (0,0,-10)),
    # Distance, relative to the target, from which the camera is looking at. Only in Follow mode
    ('distfrom', (-15.5,-5,1.5))
])
# point light origin
light_loc=(10, 50, 50)
# light intensity
light_energy=53000
# 'up' axis
axis_up = 'Z'

blend_pproc_render.bl_render(meshes_prefixes, out_dir, datadir, res, camera_mode, use_sky, camera_pos, target, axis_up, light_loc, light_energy)

