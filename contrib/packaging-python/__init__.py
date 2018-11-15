import sys
import os

dir_path = os.path.dirname(os.path.realpath(__file__))

# for Windows: assume all .pyd, .dll binaries are in these dir, so:
sys.path.append(dir_path) 
sys.path.append(dir_path + '/bin') 