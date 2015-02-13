# -*- coding: utf-8 -*-
"""
Created on Friday, 2/13/15

@author: Justin Madsen, 2015
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import pylab as py


class DriveChain_panda:
    '''
    @class: loads, manages and plots output from any number of output files from 
            Chrono Track Vehicle Toolkit
    '''
    def __init__(self,filename_list, handle_list):
        '''
        Input:
            filename_list:  .csv path + filename list of all the file names
            handle_list:    handle identifiers for each file, for legend and getting a handle to different dataframes
        Appends:
            _filename_list  full name of csv output files to read from
            _DF_list        contains data frames for each loaded output data file
            _handle_list    string, to access a DF handle
        '''
        # first file is the steady state magic formula output
        if( len(filename_list) != len(handle_list)):
            print 'must have same length input arrays'
            return
        
        self._filename_list = []   
        self._DF_list = []      
        self._handle_list = []
        self._dict = {} # for associating the string handle to the DF
        self._nFiles = len(filename_list)
        
        # append members with input lists, create the dict for lookup
        for i in range(0, self._nFiles):
            self._filename_list.append(filename_list[i])
            DF_curr = pd.read_csv(filename_list[i], header=0, sep=',')   # index_col=0,
            self._DF_list.append(DF_curr)
            self._handle_list.append(handle_list[i])
            # associate the handle string with the index of the DF in the list
            self._dict[handle_list[i]] = i
    
    
    # to manage lots of different input files and their associated data frames, 
    #   Access handles to the _DF_list members by specifying the corresponding 
    #   as a dictionary
    def _getDFhandle(self, str_handle):
        return self._DF_list[self._dict[str_handle]]
    
    # plot gear body info    
    def plot_gear(self,tmin=-1,tmax=-1):
        # plot pos, vel, omega (pitch,yaw,roll)
        fig, axarr = plt.subplots(3,sharex=True)
        
        # pull the data from the DF for the gear
        inDat = ['time','x','y','z','Vx','Vy','Vz','Wx','Wy','Wz']
        # create a dataframe for the desired data, by getting ahandle to the loaded DF for the gear
        DF = pd.DataFrame(self._getDFhandle('Gear'), columns = inDat)
        
        # create the 3 subplots
        DF.plot(ax = axarr[0], linewidth=1.5, x='time', y=['x','y','z'])
        DF.plot(ax = axarr[1], linewidth=1.5, x='time', y=['Vx','Vy','Vz'])
        DF.plot(ax = axarr[2], linewidth=1.5, x='time', y=['Wx','Wy','Wz'])
        
        # label axes
        if(tmin>0):
            # make sure tmax is larger than tmin
            if(tmax>tmin):
                axarr[0].set_xlim([tmin,tmax])
            else:
                # tmin still used, through tend
                axarr[0].set_xlim([tmin,DF['time'][-1]])
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('position [m]')
        axarr[1].set_ylabel('velocity [m/s]')
        axarr[2].set_ylabel('rot. vel. [rad/sec]')
        
        # set the legend
        axarr[0].legend()
        axarr[1].legend(loc='best')
        axarr[2].legend(loc='best')
        axarr[0].set_title('Gear body')
        
        
    # plot idler body info, tensioner force
    def plot_idler(self):
        # plot pos, vel, omega
        figA, axarrA = plt.subplots(3,sharex=True)
        
        #and F_tensioner
        FigB, axarrB = plt.subplots(1)
        
        # data headers to pull from
        inDat = ['time','x','y','z','Vx','Vy','Vz','Wx','Wy','Wz','F_tensioner','F_k','F_c']      
        
        # data frame from the headers
        DFid = pd.DataFrame(self._getDFhandle('idler'), columns = inDat)
        
        # crate 3 subplots for the body data
        DFid.plot(ax = axarrA[0], linewidth=1.5, x='time', y=['x','y','z'])
        DFid.plot(ax = axarrA[1], linewidth=1.5, x='time', y=['Vx','Vy','Vz'])
        DFid.plot(ax = axarrA[2], linewidth=1.5, x='time', y=['Wx','Wy','Wz'])
        
        # first plot label axes 
        axarrA[0].set_xlabel('time [s]')
        axarrA[0].set_ylabel('position [m]')
        axarrA[1].set_ylabel('velocity [m/s]')
        axarrA[2].set_ylabel('rot vel [rad/sec]')
        # first plot, legend
        axarrA[0].legend()
        axarrA[1].legend()
        axarrA[2].legend()
        axarrA[0].set_title('idler body')
        
        # create the tensioner force plot       
        DFid.plot(ax = axarrB, linewidth=1.5, x='time', y=['F_tensioner','F_k','F_c'])
        
        # second plot label axes
        axarrB.set_xlabel('time [s]')
        axarrB.set_ylabel('tensioner Force [N]')
        # second plot legend
        axarrB.legend()
        axarrB.set_title('tensioner')
        
    # plot powertrain info
    def plot_ptrain(self):
        # plot mot speed, mot torque, and output shaft torque
        fig, axarr = plt.subplots(2,sharex=True)
        
        #data headers
        inDat = ['time','motSpeed','motT','outT']
        # data frame from headers
        DFpt = pd.DataFrame(self._getDFhandle('ptrain'), columns = inDat)
        
        # create a subplot for speed, and torues
        DFpt.plot(ax = axarr[0], linewidth=1.5, x='time', y=['motSpeed'])
        DFpt.plot(ax = axarr[1], linewidth=1.5, x='time', y=['motT','outT'])
        
        # labels, legend
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('rot vel [rad/s]')
        axarr[1].set_ylabel('torque [N-m]')
        axarr[0].legend()
        axarr[1].legend()
        
    # plot shoe 0 body info, and pin 0 force/torque
    def plot_shoe(self, shoe_idx):
        # a plot for the shoe body,
        figA, axarrA = plt.subplots(4,sharex=True)
        #  pin rxn forces
        figB, axarrB = plt.subplots(3,sharex=True)
        # pin rxn torques
        figC, axarrC = plt.subplots(3,sharex=True)
        
        # data headers
        inDat = ['time','x','y','z','Vx','Vy','Vz','Ax','Ay','Az','Wx','Wy','Wz','Fx','Fy','Fz','Tx','Ty','Tz']
        # get the data frame from the shoe
        DFs = pd.DataFrame(self._getDFhandle('shoe0'), columns=inDat)
        
        # create 4 subplots for the shoe body
        DFs.plot(ax = axarrA[0], linewidth=1.5, x='time', y=['x','y','z'])
        DFs.plot(ax = axarrA[1], linewidth=1.5, x='time', y=['Vx','Vy','Vz'])
        DFs.plot(ax = axarrA[2], linewidth=1.5, x='time', y=['Ww','Wy','Wz'])
        DFs.plot(ax = axarrA[3], linewidth=1.5, x='time', y=['Ax','Ay','Az'])
        
        # first plot label axes 
        axarrA[0].set_xlabel('time [s]')
        axarrA[0].set_ylabel('position [m]')
        axarrA[1].set_ylabel('velocity [m/s]')
        axarrA[2].set_ylabel('rot vel [rad/sec]')
        axarrA[3].set_ylabel('accel [m/s2]')
        # first plot, legend
        axarrA[0].legend()
        axarrA[1].legend()
        axarrA[2].legend()
        axarrA[3].legend()
        axarrA[0].set_title('shoe 0 body')
        
        # create the second plot, pin reaction forces
        DFs.plot(ax = axarrB[0], linewidth=1.5, x='time', y=['Fx'])
        DFs.plot(ax = axarrB[1], linewidth=1.5, x='time', y=['Fy'])
        DFs.plot(ax = axarrB[2], linewidth=1.5, x='time', y=['Fz'])
        
        # second plot label axes, legend
        axarrB[0].set_xlabel('time [s]')
        axarrB[0].set_ylabel('Fx [N]')
        axarrB[1].set_ylabel('Fy [N]')
        axarrB[2].set_ylabel('Fz [N]')
        # first plot, legend
        axarrB[0].legend()
        axarrB[1].legend()
        axarrB[2].legend()
        axarrB[0].set_title('inter-shoe# ' + str(shoe_idx) + ' revolute reaction forces')
        
        # create third plot, pin reaction torques
        DFs.plot(ax = axarrC[0], linewidth=1.5, x='time', y=['Tx'])
        DFs.plot(ax = axarrC[1], linewidth=1.5, x='time', y=['Ty'])
        DFs.plot(ax = axarrC[2], linewidth=1.5, x='time', y=['Tz'])
        
        # second plot label axes, legend
        axarrC[0].set_xlabel('time [s]')
        axarrC[0].set_ylabel('Tx [N-m]')
        axarrC[1].set_ylabel('Ty [N-m]')
        axarrC[2].set_ylabel('Tz [N-m]')
        # first plot, legend
        axarrC[0].legend()
        axarrC[1].legend()
        axarrC[2].legend()
        axarrC[0].set_title('inter-shoe# ' + str(shoe_idx) + ' revolute reaction torques')
    
    # plot gear Constraint Violations
    def plot_gearCV(self):
        # plot gear revolute constraint violations
        fig, axarr = plt.subplots(2,sharex=True)
        
        #data headers
        inDat = ['time','x','y','z','rx','ry']
        # data frame from headers
        DFpt = pd.DataFrame(self._getDFhandle('gearCV'), columns = inDat)
        
        # create a subplot for speed, and torues
        DFpt.plot(ax = axarr[0], linewidth=1.5, x='time', y=['x','y','z'])
        DFpt.plot(ax = axarr[1], linewidth=1.5, x='time', y=['rx','ry'])
        
        # labels, legend
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('pos. coord violation ')
        axarr[1].set_ylabel('rot. coord violation ')
        axarr[0].legend()
        axarr[1].legend()
        axarr[0].set_title('Gear rev. Constraint Violtion')
        
    # plot idler Constraint Violations  
    def plot_idlerCV(self, idler_idx):
        # plot two pos. and two rot. CVs
        fig, axarr = plt.subplots(2,sharex=True)
        
        #data headers
        inDat = ['time','y','z','rx','ry']
        # data frame from headers
        DFpt = pd.DataFrame(self._getDFhandle('idlerCV'), columns = inDat)
        
        # create a subplot for speed, and torues
        DFpt.plot(ax = axarr[0], linewidth=1.5, x='time', y=['y','z'])
        DFpt.plot(ax = axarr[1], linewidth=1.5, x='time', y=['rx','ry'])
        
        # labels, legend
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('pos. coord violation ')
        axarr[1].set_ylabel('rot. coord violation ')
        axarr[0].legend()
        axarr[1].legend()
        axarr[0].set_title('Idler Constraint Violtion')
        
    # plot roller Constraint Violations
    def plot_rollerCV(self, roller_idx):
        # plot roller revolute constraint violations
        fig, axarr = plt.subplots(2,sharex=True)
        
        #data headers
        inDat = ['time','x','y','z','rx','ry']
        # data frame from headers
        DFpt = pd.DataFrame(self._getDFhandle('rollerCV'), columns = inDat)
        
        # create a subplot for speed, and torues
        DFpt.plot(ax = axarr[0], linewidth=1.5, x='time', y=['x','y','z'])
        DFpt.plot(ax = axarr[1], linewidth=1.5, x='time', y=['rx','ry'])
        
        # labels, legend
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('pos. coord violation ')
        axarr[1].set_ylabel('rot. coord violation ')
        axarr[0].legend()
        axarr[1].legend()
        axarr[0].set_title('Roller rev. Constraint Violtion')
    
    
    

      
if __name__ == '__main__':
    
    # logger
    import logging as lg
    lg.basicConfig(fileName = 'logFile.log', level=lg.WARN, format='%(message)s')
    # default font size
    font = {'size' : 14}
    matplotlib.rc('font', **font)       
    
    #  **********************************************************************    
    #  ===============   USER INPUT   =======================================
    # laptop data dir
    # data_dir = 'E:/Chrono-T_Build/bin/Release/'
    # desktop data dir
    data_dir = 'D:/Chrono_github_Build/bin/outdata_driveChain/'
    
    # list of data files to plot
    gearSubsys = 'test_driveChain_gear.csv'
    idlerSubsys = 'test_driveChain_idler.csv'
    ptrainSubsys = 'test_driveChain_ptrain.csv'
    shoe0 = 'test_driveChain_shoe0.csv'
    gearCV = 'test_driveChain_GearCV.csv'
    idlerCV = 'test_driveChain_idler0CV.csv'
    rollerCV = 'test_driveChain_roller0CV.csv'
    
    data_files = [data_dir + gearSubsys, data_dir + idlerSubsys, data_dir + ptrainSubsys, data_dir + shoe0, data_dir + gearCV, data_dir + idlerCV, data_dir + rollerCV]
    handle_list = ['Gear','idler','ptrain','shoe0','gearCV','idlerCV','rollerCV']
    
 
    # construct with file list and list of legend
    Chain = DriveChain_panda(data_files,handle_list)
    
    # 1) plot the gear body info
    Chain.plot_gear()
    
    # 2) plot idler body info, tensioner force
    Chain.plot_idler()

    # 3) plot powertrain info
    Chain.plot_ptrain()    
    
    # 4) plot shoe 0 body info, and pin 0 force/torque
    Chain.plot_shoe(0)
    
    # 5) plot gear Constraint Violations
    Chain.plot_gearCV()
    
    # 6) plot idler Constraint Violations
    Chain.plot_idlerCV(0)
    
    # 7) plot roller Constraint Violations
    Chain.plot_rollerCV(0)
    

py.show()