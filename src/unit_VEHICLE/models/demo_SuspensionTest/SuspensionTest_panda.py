# -*- coding: utf-8 -*-
"""
Created on Wed 10/8/14

post process output from HMMWV9 vehicle model's pactire

@author: Justin Madsen, 2014
"""

# Note: add Chrono-T/tests/pacTest   to your directory
import PacTire_panda as tire
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import pylab as py
    


class SuspensionTest_panda:
    '''
    @class: loads, manages and plots output from any number of output files from 
            ChronoT class SuspensionTest
    '''
    def __init__(self,filename_list, leg_list):
        '''
        Input:
            filename_list:  .csv path + filename list of all the file names
            leg_list:       legend identifiers for each file
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
     
    # @brief plot the inputs to the suspension test rig (steer and post displacement)       
    def plot_input(self):
        figF = plt.figure()
        
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
        ax.set_ylabel('steer [-], displacement [m]')
        ax.legend(loc='upper left')
        ax.set_title('test rig inputs')  
        
     
      
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
    data_dir = 'C:/Users/newJustin/Google Drive/shareWith_Radu/PacTire/suspensionTest_data/'
    
    steer_data = 'log_test_SuspensionTester.csv'
 
    # construct with file list and list of legend
    sTest = SuspensionTest_panda([data_dir+steer_data], ["dPost = 0"])
    sTest.plot_input()
    
    # sTest.plot_kingpin()
    # sTest.plot_caster()
    # sTest.plot_toe()
    # sTest.plot_spring()

py.show()