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
    


class PacTire_vehicle_panda:
    '''
    @class: loads, manages and plots output from a 4 pacTires on a vehicle 
    '''
    def __init__(self,filename_list,tire_names):
        '''
        Input:
            filename_list:  .csv path + filename of pactire output
            tire_names:     name of tires, must be same length as filename_list
        '''
        # first file is the steady state magic formula output
        if( len(filename_list) != len(tire_names)):
            print 'must have same length input arrays'
            return
        
        self._filenames = []
        self._DF = []
        self._tires = []
        self._num_tires = len(filename_list)
        
        for i in range(0, self._num_tires):
            self._filenames.append(filename_list[i])
            DF_curr = pd.read_csv(filename_list[i], header=0, sep=',')   # index_col=0,
            self._DF.append( DF_curr )
            self._tires.append(tire_names[i])
            
    # @brief plot forces, moments vs. kappa for one of the tires in the array
    def plot_kappa(self, tire_num = 0, titleStr = ''):
        if(tire_num > self.__num_tires):
            print 'specified tire_num is out of range'
            return
        # plot Fx, Fy
        figF = plt.figure()
        df_F = pd.DataFrame(self._DF[tire_num], columns = ['kappa','Fxc','Fyc'])
        axF = df_F.plot(linewidth=1.5,x='kappa',y=['Fxc','Fyc'])   # ,'Fz'])
        axF.set_xlabel(r'$\kappa $')
        axF.set_ylabel('Force [N]')
        axF.legend(loc='best')
        axF.set_title(r'$\kappa $' + titleStr)
        
        # plot Mx, My, Mzc
        figM = plt.figure()
        df_M = pd.DataFrame(self._DF[tire_num], columns = ['kappa','Mx','My','Mzc'])
        axM = df_M.plot(linewidth=1.5,x='kappa',y=['Mx','My','Mzc'])
       
        axM.set_xlabel(r'$\kappa $')
        axM.set_ylabel('Moment [N-m]')
        axM.legend(loc='upper left')
        axM.set_title(r'$\kappa $, ' + titleStr)
    
    # @brief plot forces, moments vs. alpha
    def plot_alpha(self, tire_num = 0, titleStr = ''):
        if(tire_num > self.__num_tires):
            print 'specified tire_num is out of range'
            return
        # plot the forces vs alpha
        figF = plt.figure()
        df_F = pd.DataFrame(self._DF[tire_num], columns = ['alpha','Fxc','Fyc'])
        axF = df_F.plot(linewidth=1.5,x='alpha',y=['Fxc','Fyc']) 
       
        axF.set_xlabel(r'$\alpha $[deg]')
        axF.set_ylabel('Force [N]')
        axF.legend(loc='best')
        axF.set_title(r'$\alpha $,' + titleStr)
        
        # plot moments vs. alpha
        figM = plt.figure()
        df_M = pd.DataFrame(self._DF[tire_num], columns = ['alpha','Mx','My','Mzc'])
        axM = df_M.plot(linewidth=2.0,x='alpha',y=['Mx','My','Mzc'])
        
        axM.set_xlabel(r'$\alpha $[deg]')
        axM.set_ylabel('Moment [N-m]')
        axM.legend(loc='best')
        axM.set_title(r'$\alpha $, combined slip')
        
    # @brief plot reactions vs. time
    def plot_time(self, tire_num = 0, titleStr = 'time vs. '):
        if(tire_num > self._num_tires):
            print 'specified tire_num is out of range'
            return
        # plot the forces vs time
        figF = plt.figure()
        df_F = pd.DataFrame(self._DF[tire_num], columns = ['time','Fxc','Fyc'])
        axF = df_F.plot(linewidth=1.5,x='time',y=['Fxc','Fyc']) 
       
        axF.set_xlabel('time [sec]')
        axF.set_ylabel('Force [N]')
        axF.legend(loc='best')
        axF.set_title(titleStr)
        
        # plot moments vs. time
        figM = plt.figure()
        df_M = pd.DataFrame(self._DF[tire_num], columns = ['time','Mx','My','Mzc'])
        axM = df_M.plot(linewidth=2.0,x='time',y=['Mx','My','Mzc'])
        
        axM.set_xlabel(r'$time $[sec]')
        axM.set_ylabel('Moment [N-m]')
        axM.legend(loc='best')
        axM.set_title(titleStr)        
    
        # check the displacements, u, v vs. time
        fig_uv = plt.figure()
        df_uv = pd.DataFrame(self._DF[tire_num], columns = ['time','u','valpha','du','dvalpha','vgamma','dvgamma'])
        ax_uv = df_uv.plot(linewidth=1.5, x='time', y=['u','valpha','vgamma'])
        ax_uv.set_xlabel('time [sec]')
        ax_uv.legend(loc='best')
        ax_uv.set_title(titleStr)
        
        fig_duv = plt.figure()
        ax_duv = df_uv.plot(linewidth=2.0,x='time',y=['du','dvalpha','dvgamma'])
        ax_duv.set_xlabel('time [sec]')
        ax_duv.legend(loc='best')
        ax_duv.set_title(titleStr)
        
        
    '''
    # @brief plot what you please
    # @param x_col the column name of the x-series data frame
    # @param y_col_list list of col names of y-series data frames
    # Usage: tire.plot_custom('Fx',['Fy','Fyc'])
    # NOTE: y_col_list should be of the same units
    def plot_custom(self,x_col, y_col_list,fig_title='custom plot'):
        fig = plt.figure()
        df_cols = []
        df_cols = list(y_col_list)
        df_cols.append(x_col)
        df_sy = pd.DataFrame(self._m_df, columns = df_cols)
        ax = df_sy.plot(linewidth=1.5, x=x_col, y=y_col_list)
        ax.set_xlabel(x_col)
        
        ax.legend(loc='best')
        ax.set_title(fig_title)

    # @brief similar as above, but y_col_2 will be on a second y-axis
    def plot_custom_two(self,x_col, y_col_1, y_secondary, fig_title='custom plot 2'):
        fig = plt.figure()
        df_cols = [x_col , y_col_1 , y_secondary]
        df_sy = pd.DataFrame(self._m_df, columns = df_cols)
        ax = df_sy.plot(linewidth=1.5, x= x_col, secondary_y= y_secondary)
        ax.set_xlabel(x_col)
        ax.set_ylabel(y_col_1)
        ax.right_ax.set_ylabel(y_secondary)
        
        ax.legend(loc='best')
        ax.set_title(fig_title)    
    '''
    
if __name__ == '__main__':
    
    # logger
    import logging as lg
    lg.basicConfig(fileName = 'logFile.log', level=lg.WARN, format='%(message)s')
    # default font size
    font = {'size' : 14}
    matplotlib.rc('font', **font)       
    
    # laptop data dir
    
    # desktop data dir
    data_dir = 'D:/Chrono-T_Build/bin/Release/'
    
    # create a tire using the single tire python plotting code
    tire_LF = tire.PacTire_panda(data_dir + 'test_HMMWV9_pacTire_FL.csv')
    
    tire_LF.plot_custom('time',['m_Fz'],'vertical Force [N]')
    tire_LF.plot_custom('time',['Fxc','Fyc'],'Fx, Fy [N]')
    
    tire_LF.plot_custom('time',['kappa'],'longitudinal slip [-]')
    tire_LF.plot_custom('time',['alpha'],'lateral slip [deg]')
    
    # tire_LF.plot_combined_kappa()
    # tire_LF.plot_combined_alpha()
    
    # use the new python code for plotting any/all of the pacTire outputs on a vehicle
    # there are 4 pacTire files associated with a HMMWV model
    dat_FL = 'test_HMMWV9_pacTire_FL.csv'
    dat_FR = 'test_HMMWV9_pacTire_FR.csv'
    dat_RL = 'test_HMMWV9_pacTire_RL.csv'
    dat_RR = 'test_HMMWV9_pacTire_RR.csv'
    vehicle_output = PacTire_vehicle_panda([data_dir+dat_FL, data_dir+dat_FR, data_dir+dat_RL, data_dir+dat_RR],
                                           ['FL','FR','RL','RR'])
    
    # plot w.r.t. to time
    vehicle_output.plot_time(0,'Front Left')
    #vehicle_output.plot_time(1,'Front Right')
    #vehicle_output.plot_time(2,'Rear Left')
    vehicle_output.plot_time(3,'Rear Right')
    
    # plot w.r.t. long slip
    
    # plot w.r.t. lateral slip angle

py.show()