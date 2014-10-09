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
        
        self._fileanmes = []
        self._DF = []
        self._tires = []
        for i in range(0,len(filename_list)):
            self._filenames.append(filename_list[i])
            DF_curr = pd.read_csv(filename_list[i], header=0, sep=',')   # index_col=0,
            self._DF.append( DF_curr )
            self._tires.append(tire_names[i])
            
    # @brief plot combined forces, moments vs. kappa
    def plot_kappa(self):
        figF = plt.figure()
        df_sy = pd.DataFrame(self._m_df, columns = ['kappa','Fxc','Fyc','m_Fz'])
        axF = df_sy.plot(linewidth=1.5,x='kappa',y=['Fxc','Fyc'])   # ,'Fz'])
        # check to see if adams data is avaiable
        if( adams_Fx_tab_filename != "none"):
             # load in the adams junk data, Fx vs. kappa %
            dfx_adams = pd.read_table(adams_Fx_tab_filename,sep='\t',header=0)
            dfx_adams['longitudinal slip'] = dfx_adams['longitudinal slip'] / 100.0
            axF.plot(dfx_adams['longitudinal slip'],dfx_adams['longitudinal force'],'r--',linewidth=1.5,label="Fx Adams")
        if( adams_Fy_tab_filename != "none"):
            # load in adams tabular data for Fy vs. alpha [deg]
            dfy_adams = pd.read_table(adams_Fy_tab_filename,sep='\t',header=0)
            axF.plot(dfy_adams['slip_angle'],dfy_adams['lateral_force'],'g--',linewidth=1.5,label="Fy Adams")
        
        # plot transient slip output?
        if( self._use_transient_slip):
            # Fx here
            df_T = pd.DataFrame(self._m_df_T, columns = ['kappa','Fxc','Fyc'])
            axF.plot(df_T['kappa'], df_T['Fxc'],'k-*',linewidth=1.0,label='Fxc transient')
            # axF.plot(df_T['kappa'], df_T['Fyc'],'c--',linewidth=1.5,label='Fyc transient')
        axF.set_xlabel(r'$\kappa $')
        axF.set_ylabel('Force [N]')
        axF.legend(loc='best')
        axF.set_title(r'$\kappa $, combined slip')
        
        figM = plt.figure()
        df_M = pd.DataFrame(self._m_df, columns = ['kappa','Mx','My','Mzc'])
        axM = df_M.plot(linewidth=1.5,x='kappa',y=['Mx','My','Mzc'])
        # chceck for adams tabular data        
        if( adams_Mz_tab_filename != "none"):
            dfmz_adams = pd.read_table(adams_Mz_tab_filename,sep='\t',header=0)
            dfmz_adams['longitudinal_slip'] = dfmz_adams['longitudinal_slip'] / 100.0
            axM.plot(dfmz_adams['longitudinal_slip'], dfmz_adams['aligning_moment'],'r--',linewidth=1.5,label="Mz Adams")
    
        # plot transient slip output for Mz?
        if( self._use_transient_slip):
            # Mzc here
            df_T_M = pd.DataFrame(self._m_df_T, columns = ['kappa','Mzc','Fyc','Fxc','Mzx','Mzy'])
            axM.plot(df_T_M['kappa'], df_T_M['Mzc'],'k-*',linewidth=1.0,label='Mzc transient')
            # overlay the components of the moment from the x and y forces, respectively
            axM.plot(df_T_M['kappa'], df_T_M['Mzx'],'y--',linewidth=1.5,label='Mz,x')
            axM.plot(df_T_M['kappa'], df_T_M['Mzy'],'b--',linewidth=1.5,label='Mz,y')
            # overlay the forces, to see what is happening on those curves when
            # Mz deviates from validation data values
            '''
            ax2 = axM.twinx()
            ax2.plot(df_T_M['kappa'], df_T_M['Fxc'],'g-.',linewidth=1.5,label='Fxc transient')
            ax2.plot(df_T_M['kappa'], df_T_M['Fyc'],'c-.',linewidth=1.5,label='Fyc transient')
            ax2.set_ylabel('Force [N]')
            ax2.legend(loc='lower right')
            '''
        axM.set_xlabel(r'$\kappa $')
        axM.set_ylabel('Moment [N-m]')
        axM.legend(loc='upper left')
        
        axM.set_title(r'$\kappa $, combined slip')
    
    # @brief plot Fy combined vs. kappa and alpha
    def plot_combined_alpha(self,adams_Fx_tab_filename="none",adams_Fy_tab_filename="none",adams_Mz_tab_filename="none"):
        figF = plt.figure()
        df_sy = pd.DataFrame(self._m_df, columns = ['alpha','Fxc','Fyc','Fz'])
        axF = df_sy.plot(linewidth=1.5,x='alpha',y=['Fxc','Fyc'])   # ,'Fz'])
        # check to see if adams data is avaiable
        if( adams_Fx_tab_filename != "none"):
             # load in the adams junk data, Fx vs. kappa %
            dfx_adams = pd.read_table(adams_Fx_tab_filename,sep='\t',header=0)
            dfx_adams['longitudinal slip'] = dfx_adams['longitudinal slip'] / 100.0
            axF.plot(dfx_adams['longitudinal slip'],dfx_adams['longitudinal force'],'r--',linewidth=1.5,label="Fx Adams")
        if( adams_Fy_tab_filename != "none"):
            # load in adams tabular data for Fy vs. alpha [deg]
            dfy_adams = pd.read_table(adams_Fy_tab_filename,sep='\t',header=0)
            axF.plot(dfy_adams['slip_angle'],dfy_adams['lateral_force'],'g--',linewidth=1.5,label="Fy Adams")
      
        if( self._use_transient_slip):
            # Fy here
            df_T = pd.DataFrame(self._m_df_T, columns = ['alpha','Fyc'])
            axF.plot(df_T['alpha'], df_T['Fyc'],'k-*',linewidth=1.0,label='Fyc transient')
            
        axF.set_xlabel(r'$\alpha $[deg]')
        axF.set_ylabel('Force [N]')
        axF.legend(loc='best')
        axF.set_title(r'$\alpha $ , combined slip')
        
        figM = plt.figure()
        df_M = pd.DataFrame(self._m_df, columns = ['alpha','Mx','My','Mzc'])
        axM = df_M.plot(linewidth=2.0,x='alpha',y=['Mx','My','Mzc'])
         # chceck for adams tabular data
        if( adams_Mz_tab_filename != "none"):
            dfmz_adams = pd.read_table(adams_Mz_tab_filename,sep='\t',header=0)
            axM.plot(dfmz_adams['slip_angle'], dfmz_adams['aligning_moment'],'r--',linewidth=1.5,label="Mz Adams")
            
        # plot transient slip output
        if( self._use_transient_slip):
            # Fx here
            df_T_M = pd.DataFrame(self._m_df_T, columns = ['alpha','Mzc'])
            axM.plot(df_T_M['alpha'], df_T_M['Mzc'],'k-*',linewidth=1.0,label='Mzc transient')
            
        axM.set_xlabel(r'$\alpha $[deg]')
        axM.set_ylabel('Moment [N-m]')
        axM.legend(loc='best')
        axM.set_title(r'$\alpha $, combined slip')
    
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
    
    
if __name__ == '__main__':
    
    # logger
    import logging as lg
    lg.basicConfig(fileName = 'logFile.log', level=lg.WARN, format='%(message)s')
    # default font size
    font = {'size' : 14}
    matplotlib.rc('font', **font)       
    
    # directory location on my laptop
    
    # desktop 
    dir_ChronoT = 'D:/Chrono-T_Build/bin/Release/'
    
    # create the 4 tires
    
    tire_LF = tire.PacTire_panda(dir_ChronoT + 'test_HMMWV9_pacTire_FL.csv')
    
    tire_LF.plot_custom('time',['m_Fz'],'vertical Force [N]')
    tire_LF.plot_custom('time',['Fxc','Fyc'],'vertical Force [N]')
    
    tire_LF.plot_custom('time',['kappa'],'longitudinal slip [-]')
    tire_LF.plot_custom('time',['alpha'],'lateral slip [deg]')
    
    tire_LF.plot_combined_kappa()
    tire_LF.plot_combined_alpha()
    

py.show()