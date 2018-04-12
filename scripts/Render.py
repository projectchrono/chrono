#!/usr/bin/env python2
import os
from os import walk
import sys
from glob import glob

fps = 60  # TODO
width = 1024  # TODO
height = 1024  # TODO
rad = 25984

if len(sys.argv) < 2:
    sys.exit(1)

dirname = os.path.abspath(os.getcwd()) + '/' + sys.argv[1]
print(dirname)

os.chdir(dirname)

file_names = []
# CSV fields (and header)
fieldstring = 'x,y,z,vx,vy,vz,absv,nTouched\n'

os.chdir(dirname)

for filename in glob("*.csv"):
    if filename[-4:] == ".csv":
        file_names.append(filename)


for i in range(len(file_names) - 1, 0, -1):
    if file_names[i][0] == '.':
        del file_names[i]

# for name in file_names:
#     print(name)

# TODO: Precondition: ONE csv file for the entire frame
# Generate an image for each file
for i in range(len(file_names)):
    # csv infile
    infile = open(file_names[i], 'r')

    # xml outfile
    xmlfilename = str(file_names[i])[:len(file_names[i]) - 4] + '.xml'
    print "Writing to", xmlfilename
    outfile = open(xmlfilename, 'w')

    outfile.write('<?xml version=\"1.0\" encoding=\"utf-8\"?>\n')
    outfile.write('<scene version=\"0.6.0\">\n')

    # Add Camera
    outfile.write('\t<sensor type=\"perspective\">\n')
    outfile.write('\t\t<string name=\"fovAxis\" value=\"smaller\"/>\n')
    outfile.write('\t\t<transform name=\"toWorld\">\n')
    outfile.write('\t\t\t<lookAt origin=\"-12708787, -12708787, 5338924\" target=\"0, 0, 0\" up=\"0, 0, 1\"/>\n')
    outfile.write('\t\t</transform>\n')
    outfile.write('\t\t<float name=\"fov\" value=\"39\"/>\n')
    outfile.write('\t\t<sampler type=\"ldsampler\">\n')
    outfile.write('\t\t\t<integer name=\"sampleCount\" value="\64\"/>\n')
    outfile.write('\t\t</sampler>\n')
    outfile.write('\t\t<film type=\"ldrfilm\">\n')
    outfile.write('\t\t\t<integer name=\"width\" value=\"1024\"/>\n')
    outfile.write('\t\t\t<integer name=\"height\" value=\"1024\"/>\n')
    outfile.write('\t\t\t<rfilter type="gaussian"/>\n')
    outfile.write('\t\t</film>\n')
    outfile.write('\t</sensor>\n')

    # Add light source
    # outfile.write('\t<shape type=\"sphere\">\n')
    # outfile.write('\t\t<emitter type=\"area\">\n')
    # outfile.write('\t\t\t<spectrum name=\"radiance\" value=\"20\"/>\n')
    # # outfile.write('\t\t<point name=\"position\" x=\"-12708787\" y=\"-12708787\" z=\"5338924\"/>\n')
    # outfile.write('\t\t</emitter>\n')
    # outfile.write('\t</sphere>\n')

    # # Add scene box
    # outfile.write('\t<shape type=\"obj\">\n')
    # outfile.write('\t\t<string name=\"filename\" value=\"Meshes/Box.obj\"/>\n')
    # outfile.write('\t</shape>\n')

    # Create a sphere entry for each body
    print "Reading from", file_names[i]
    lines = infile.readlines()
    for j in range(1, len(lines), 1):
        line = lines[j]
        # print line
        tokens = line.split(',')
        try:
            if (len(tokens) > 3):
                # printw
                outfile.write('\t<shape type=\"sphere\">\n')
                outfile.write('\t\t<float name=\"radius\" value=\"' + '1' + '\"/>\n')
                outfile.write('\t\t<transform name=\"toWorld\" >\n')
                outfile.write('\t\t\t<translate x=\"' + str(float(tokens[0]) / rad) +
                              '\" y=\"' + str(float(tokens[1]) / rad) + '\" z=\"' + str(float(tokens[2]) / rad) + '\"/>\n')
                outfile.write('\t\t</transform>\n')
                outfile.write('\t\t<bsdf type=\"plastic\">')
                outfile.write('\t\t\t<srgb name=\"diffuseReflectance\" value=\"#7A5230\"/>\n')
                outfile.write('\t\t</bsdf>\n')
                outfile.write('\t</shape>\n')
        except IndexError:
            print "Index error, line is ", line
            print "Tokens is ", tokens

    outfile.write('</scene>\n')
    outfile.close()

    # # Use Mitsuba to render an exr output file
    # os.system('source /home/nic/pkg/mitsuba-git/src/mitsuba/setpath.sh; /home/nic/pkg/mitsuba-git/src/mitsuba/build/release/mitsuba/mitsuba ' + xmlfilename)
    #
    # # Comment the following line to keep xml files for manual tweeking
    # os.system('rm ' + xmlfilename)
    #
    # os.system('rm mitsuba.*')

# TODO: Merge exr files into a video
# os.system('ffmpeg -r ' + str(fps) + ' -f image2 -s ' + str(width) + 'x' + str(height) +
#           ' -i T%04d.png -crf 15 -vcodec libx264 -pix_fmt yuv420p video.mp4')
