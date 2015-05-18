# -*- coding: utf-8 -*-
"""
Created on Wed May 06 11:00:53 2015

@author: newJustin
"""

import ChronoTrack_pandas as CT
import pylab as py
      
if __name__ == '__main__':
    
    # logger
    import logging as lg
    
    lg.basicConfig(fileName = 'logFile.log', level=lg.WARN, format='%(message)s')
    # default font size

    import matplotlib
    font = {'size' : 14}
    matplotlib.rc('font', **font)       
    
    #  **********************************************************************    
    #  ===============   USER INPUT   =======================================
    # data dir, end w/ '/'
    # data_dir = 'D:/Chrono_github_Build/bin/outdata_M113/'
    data_dir = 'E:/Chrono_github_Build/bin/outdata_M113/'    
    
    '''    
    # list of data files to plot
    chassis = 'M113_chassis.csv'
    gearSubsys = 'M113_Side0_gear.csv'
    idlerSubsys = 'M113_Side0_idler.csv'
    # ptrainSubsys = 'test_driveChain_ptrain.csv'
    shoe0 = 'M113_Side0_shoe0.csv'
    '''
    chassis = 'M113_400_200__chassis.csv'
    gearSubsys = 'M113_400_200__Side0_gear.csv'
    idlerSubsys = 'M113_400_200__Side0_idler.csv'
    # ptrainSubsys = 'test_driveChain_ptrain.csv'
    shoe0 = 'M113_400_200__Side0_shoe0.csv'    

    data_files = [data_dir + chassis, data_dir + gearSubsys, data_dir + idlerSubsys, data_dir + shoe0]
    handle_list = ['chassis','gear','idler','shoe0']
    # handle_list = ['Gear','idler','ptrain','shoe0','gearCV','idlerCV','rollerCV','gearContact','shoeGearContact']

    
    
    '''
    gearCV = 'test_driveChain_GearCV.csv'
    idlerCV = 'test_driveChain_idler0CV.csv'
    rollerCV = 'test_driveChain_roller0CV.csv'
    gearContact = 'test_driveChain_gearContact.csv'
    shoeGearContact = 'test_driveChain_shoe0GearContact.csv'
    '''
    
    # data_files = [data_dir + gearSubsys, data_dir + idlerSubsys, data_dir + ptrainSubsys, data_dir + shoe0, data_dir + gearCV, data_dir + idlerCV, data_dir + rollerCV, data_dir + gearContact, data_dir+shoeGearContact]
    # handle_list = ['Gear','idler','ptrain','shoe0','gearCV','idlerCV','rollerCV','gearContact','shoeGearContact']

    
    # list of data files for gear/pin comparison plots    
    #  Primitive gear geometry
    '''
    gear = 'driveChain_P_gear.csv'
    gearContact = 'driveChain_P_gearContact.csv'
    shoe = 'driveChain_P_shoe0.csv'
    shoeContact = 'driveChain_P_shoe0GearContact.csv'
    ptrain = 'driveChain_P_ptrain.csv'    
    
    
    #  Collision Callback gear geometry     
    gear = 'driveChain_CC_gear.csv'
    gearContact = 'driveChain_CC_gearContact.csv'
    shoe = 'driveChain_CC_shoe0.csv'
    shoeContact = 'driveChain_CC_shoe0GearContact.csv'
    ptrain = 'driveChain_CC_ptrain.csv'    
    
    
    data_files = [data_dir+gear, data_dir+gearContact, data_dir+shoe, data_dir+shoeContact, data_dir+ptrain]
   
    handle_list = ['Gear','gearContact','shoe0','shoeGearContact','ptrain']
    '''
 
 
    # construct the panda class for the DriveChain, file list and list of legend
    M113_Chain0 = CT.ChronoTrack_pandas(data_files, handle_list)
    
    # set the time limits. tmin = -1 will plot the entire time range
    tmin = 1.0
    tmax = 8.0
    
    
    #0) plot the chassis
    M113_Chain0.plot_chassis(tmin, tmax)    
    
    # 1) plot the gear body info
    M113_Chain0.plot_gear(tmin, tmax)
    
    
    # 2) plot idler body info, tensioner force
    M113_Chain0.plot_idler(tmin,tmax)

    '''
    # 3) plot powertrain info
    M113_Chain0.plot_ptrain()    
    '''
    
    # 4) plot shoe 0 body info, and pin 0 force/torque
    M113_Chain0.plot_shoe(tmin,tmax)
    
    '''
    # 5) plot gear Constraint Violations
    M113_Chain0.plot_gearCV(tmin,tmax)
    
    # 6) plot idler Constraint Violations
    M113_Chain0.plot_idlerCV(tmin,tmax)
    
    # 7) plot roller Constraint Violations
    M113_Chain0.plot_rollerCV(tmin,tmax)
    
    # 8) from the contact report callback function, gear contact info
    M113_Chain0.plot_gearContactInfo(tmin,tmax)

    # 9)  from shoe-gear report callback function, contact info
    M113_Chain0.plot_shoeGearContactInfo(tmin,tmax)
    '''
    
    # 10) track shoe trajectory: rel-X vs. rel-Y
    M113_Chain0.plot_trajectory(tmin,tmax)

    py.show()