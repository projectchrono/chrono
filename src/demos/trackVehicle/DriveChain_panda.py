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
    def __init__(self,filename_list, leg_list):
        '''
        Input:
            filename_list:  .csv path + filename list of all the file names
            leg_list:       legend identifiers for each file
        Appends:
            _filename_list  Reads .csv for each file, appends the DF.
            _DF_list
            _leg_list
        '''
        # first file is the steady state magic formula output
        if( len(filename_list) != len(leg_list)):
            print 'must have same length input arrays'
            return
        
        self._filename_list = []
        self._DF_list = []
        self._leg_list = []
        self._nFiles = len(filename_list)
        
        for i in range(0, self._nFiles):
            self._filename_list.append(filename_list[i])
            DF_curr = pd.read_csv(filename_list[i], header=0, sep=',')   # index_col=0,
            self._DF_list.append(DF_curr)
            self._leg_list.append(leg_list[i])
    
    
    # plot gear body info    
    def plot_gear(self):
        arg = 2
        
    # plot idler body info, tensioner force
    def plot_idler(self):
        arg = 3
        
    # plot powertrain info
    def plot_ptrain(self):
        arg = 4
        
    # plot shoe 0 body info, and pin 0 force/torque
    def plot_shoe(self):
        arg =  5
    
    # plot gear Constraint Violations
    def plot_gearCV(self):
        arg = 6
        
    # plot idler Constraint Violations  
    def plot_idlerCV(self, idler_idx):
        arg = 7
        
    # plot roller Constraint Violations
    def plot_rollerCV(self, roller_idx):
        arg = 8
    
    
    
    
    # @brief put the input values on the RHS y-axis
    # returns the secondary y-axis
    def _plot_inputs_twinx(self, ax, DF):
        # create a 2nd axis for the steer and shaker inputs
        ax_RHS = ax.twinx()
        ax_RHS.plot(DF['time'], DF['steer'], 'r--',linewidth = 1.5, label = 'steer')
        ax_RHS.plot(DF['time'], DF['postDisp_L'], 'g--',linewidth = 1.5, label = 'postDisp_L')
        ax_RHS.plot(DF['time'], DF['postDisp_R'], 'b--',linewidth = 1.5, label = 'postDisp_R')
        return ax_RHS
    
    # @brief plot the inputs to the suspension test rig (steer and post displacement)       
    def plot_input(self):
        # figF = plt.figure()
        
        df_in_list = []
        inDat = ['time','steer','postDisp_L','postDisp_R']
        cols = ['steer','postDisp_L','postDisp_R']
        # add files to the DF list
        for i in range(0,self._nFiles):
            df_temp = pd.DataFrame(self._DF_list[i], columns= inDat)
            df_in_list.append(df_temp)
            

        # plot first file normally
        ax = df_in_list[0].plot(linewidth=1.5,x='time',y=cols)  
        # overlay files 2 to nFiles
        for p in range(1,self._nFiles):
             ax.plot(df_in_list[p]['time'], df_in_list[p]['steer'], linewidth = 1.5, label = 'steer')
             ax.plot(df_in_list[p]['time'], df_in_list[p]['postDisp_L'], linewidth = 1.5, label = 'PostDisp_L')
             ax.plot(df_in_list[p]['time'], df_in_list[p]['postDisp_R'], linewidth = 1.5, label = 'PostDisp_R')
        
        ax.set_xlabel('time [s]')
        ax.set_ylabel('steer [-], displacement [in]')
        ax.legend(loc='upper right')
        ax.set_title('test rig inputs')  
        
    # @brief plot the kingpin angle and offset, with the inputs overlayed on each
    def plot_kingpin(self,file_number=0):
        fig, axarr = plt.subplots(2,sharex=True)
        
        inDat = ['time','steer','postDisp_L','postDisp_R','KA_L','KA_R','Koff_L','Koff_R']
        # add files to the DF list
        DF = pd.DataFrame(self._DF_list[file_number], columns= inDat)
            
        DF.plot(ax = axarr[0], linewidth=1.5,x='time',y=['KA_L','KA_R'])
        ax_rhs_top = self._plot_inputs_twinx(axarr[0],DF)
        DF.plot(ax = axarr[1], linewidth=1.5,x='time',y=['Koff_L','Koff_R'])
        ax_rhs_bot = self._plot_inputs_twinx(axarr[1],DF)
    
        # label axes, legends
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('Kingpin angle [deg]')
        axarr[1].set_ylabel('offset [inches]')
        ax_rhs_top.set_ylabel('steer [-], displacement [in]')
        ax_rhs_bot.set_ylabel('steer [-], displacement [in]')
        
        axarr[0].legend(loc='upper left')
        axarr[1].legend(loc='lower left')
        # ax_rhs_top.legend(loc='lower right')
        ax_rhs_bot.legend(loc='upper right')
        axarr[0].set_title('Kingpin angle and offset')  
        
         
    # @brief plot the caster angle and offset
    def plot_caster(self,file_number=0):
        fig, axarr = plt.subplots(2,sharex=True)
        
        inDat = ['time','steer','postDisp_L','postDisp_R','CA_L','CA_R','Coff_L','Coff_R']
        # add files to the DF list
        DF = pd.DataFrame(self._DF_list[file_number], columns= inDat)
            
        DF.plot(ax = axarr[0], linewidth=1.5,x='time',y=['CA_L','CA_R'])
        ax_rhs_top = self._plot_inputs_twinx(axarr[0],DF)
        DF.plot(ax = axarr[1], linewidth=1.5,x='time',y=['Coff_L','Coff_R'])
        ax_rhs_bot = self._plot_inputs_twinx(axarr[1],DF)
    
        # label axes, legends
        axarr[0].set_xlabel('time [s]')
        axarr[0].set_ylabel('Caster angle [deg]')
        axarr[1].set_ylabel('offset [inches]')
        ax_rhs_top.set_ylabel('steer [-], displacement [in]')
        ax_rhs_bot.set_ylabel('steer [-], displacement [in]')
        
        axarr[0].legend(loc='upper left')
        axarr[1].legend(loc='lower left')
        # ax_rhs_top.legend(loc='lower right')
        ax_rhs_bot.legend(loc='center right')
        axarr[0].set_title('Caster angle and offset')  
        
    # @brief plot the toe angle
    def plot_toe(self,file_number=0):
        # fig = plt.figure()
        
        inDat = ['time','steer','postDisp_L','postDisp_R','TA_L','TA_R']
        # add files to the DF list
        DF = pd.DataFrame(self._DF_list[file_number], columns= inDat)
            
        axis = DF.plot(linewidth=1.5,x='time',y=['TA_L','TA_R'])
        ax_rhs = self._plot_inputs_twinx(axis,DF)
    
        # label axes, legends
        axis.set_xlabel('time [s]')
        axis.set_ylabel('Toe angle [deg]')

        axis.set_title('Toe angle')
        ax_rhs.set_ylabel('steer [-], displacement [in]')        
        axis.legend(loc='upper left')
        ax_rhs.legend(loc='center right')

      
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
    idlerCV = 'test_driveChain_idlerCV0.csv'
    rollerCV = 'test_rollerCV0.csv'
    
    data_files = [data_dir + gearSubsys, data_dir + idlerSubsys, data_dir + ptrainSubsys, data_dir + shoe0, data_dir + gearCV, data_dir + idlerCV, data_dir + rollerCV]
    leg_list = ['Gear','idler','ptrain','shoe0','gearCV','idlerCV','rollerCV']
    
 
    # construct with file list and list of legend
    Chain = DriveChain_panda(data_files,leg_list)
    
    # plot the gear body info
    Chain.plot_gear()
    # plot idler body info, tensioner force
    Chain.plot_idler()
    # plot powertrain info
    Chain.plot_ptrain()
    # plot shoe 0 body info, and pin 0 force/torque
    Chain.plot_shoe(0)
    
    # plot Constraint Violations
    Chain.plot_gearCV()
    Chain.plot_idlerCV(0)
    Chain.plot_rollerCV(0)
    


py.show()