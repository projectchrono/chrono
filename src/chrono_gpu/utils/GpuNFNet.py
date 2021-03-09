# Import pandas and math libraries
import pandas as pd
import math
import numpy as np
import sys

numOfFiles = int(sys.argv[1]);
path = str(sys.argv[2]);    # This path should be a folder

path = path + '/'

for i in range(numOfFiles):
    # csv1-Particle contact data (normal force magnitude, friction force vector)
    # csv2-Particle location data (x,y,z coordinates of the particles along with ordered numbers)
    if i<10:
        csv1 = pd.read_csv("contact00000"+str(i)+".csv")    #change contact file name
    elif i<100:
        csv1 = pd.read_csv("contact0000"+str(i)+".csv")    #change contact file name
    elif i<1000:
        csv1 = pd.read_csv("contact000"+str(i)+".csv")    #change contact file name
    elif i<10000:
        csv1 = pd.read_csv("contact00"+str(i)+".csv")    #change contact file name
    elif i<100000:
        csv1 = pd.read_csv("contact0"+str(i)+".csv")    #change contact file name
    else:
    	csv1 = pd.read_csv("contact"+str(i)+".csv")    #change contact file name


    if i<10:
        csv2 = pd.read_csv("step00000"+str(i)+".csv")    #change contact file name
    elif i<100:
        csv2 = pd.read_csv("step0000"+str(i)+".csv")    #change contact file name
    elif i<1000:
        csv2 = pd.read_csv("step000"+str(i)+".csv")    #change contact file name
    elif i<10000:
        csv2 = pd.read_csv("step00"+str(i)+".csv")    #change contact file name
    elif i<100000:
        csv2 = pd.read_csv("step0"+str(i)+".csv")    #change cont file name
    else:
        csv2 = pd.read_csv("step"+str(i)+".csv")    #change contact file name

    csv2.insert(loc=0, column='I', value=np.arange(len(csv2)))

    # combine particles i
    csv_out = pd.merge(csv1, csv2, left_on = 'bi', right_on = 'I', how = 'left')

    # rename coordinates of particles i
    csv_out = csv_out.rename(columns = {"x": "xi",
                     "y":"yi",
                     "z":"zi"})

    # combine particles j
    csv_out_2 = pd.merge(csv_out, csv2, left_on = ' bj', right_on = 'I', how = 'left')

    # clean redundancy
    #csv_out_2 = csv_out_2.drop('I_y', axis=1)
    #csv_out_2 = csv_out_2.drop('absv_x', axis=1)
    #csv_out_2 = csv_out_2.drop('wx_x', axis=1)
    #csv_out_2 = csv_out_2.drop('wy_x', axis=1)
    #csv_out_2 = csv_out_2.drop('wz_x', axis=1)
    #csv_out_2 = csv_out_2.drop('absv_y', axis=1)
    #csv_out_2 = csv_out_2.drop('wx_y', axis=1)
    #csv_out_2 = csv_out_2.drop('wy_y', axis=1)
    #csv_out_2 = csv_out_2.drop('wz_y', axis=1)
    #csv_out_2 = csv_out_2.drop(' mx', axis=1)
    #csv_out_2 = csv_out_2.drop(' my', axis=1)
    #csv_out_2 = csv_out_2.drop(' mz', axis=1)
    # rename coordinates of particles j, clean redundancy
    csv_out_2 = csv_out_2.rename(columns = {"I_x": "I",
                     "x":"xj",
                     "y":"yj",
                    "z":"zj"})

    # add mean x-coord of i and j
    csv_out_2['meanX'] = (csv_out_2['xi']+csv_out_2['xj'])/2
    # add mean y-coord of i and j
    csv_out_2['meanY'] = (csv_out_2['yi']+csv_out_2['yj'])/2
    # add mean z-coord of i and j
    csv_out_2['meanZ'] = (csv_out_2['zi']+csv_out_2['zj'])/2

    # add scale
    csv_out_2['scale'] = (((csv_out_2['xj']-csv_out_2['xi'])*(csv_out_2['xj']-csv_out_2['xi'])+(csv_out_2['yj']-csv_out_2['yi'])*(csv_out_2['yj']-csv_out_2['yi'])+(csv_out_2['zj']-csv_out_2['zi'])*(csv_out_2['zj']-csv_out_2['zi']))**(1/2))

    csv_out_2 = csv_out_2.drop('I', axis=1)

    #path=r'/home/jason/Gran/Test_Folder/repose/Output/'

    # save the first post processing file
    csv_out_2.to_csv(path+"normal_network_postprocessing"+str(i)+".csv", encoding='utf-8', index=False)

    print(csv_out_2)

    # calculate friction force magnitude
    csv_out_2['f_Mag'] = (csv_out_2[' fx']*csv_out_2[' fx']+csv_out_2[' fy']*csv_out_2[' fy']+csv_out_2[' fz']*csv_out_2[' fz'])**(1/2)

    # clean redundancy
    #csv_out_2.drop(['xi', 'yi','zi','xj','yj','zj'], axis=1)

    # save the second post processing file
    csv_out_2.to_csv(path+"friction_network_postprocessing"+str(i)+".csv", encoding='utf-8', index=False)
