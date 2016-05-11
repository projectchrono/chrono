# -*- coding: utf-8 -*-
"""
Created on Mon 9/2/14

Using python pandas to post process csv output from Pacejka Tire model

@author: Justin Madsen, 2014
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import pylab as py

class PacTire_panda:
    '''
    @class: loads, manages and plots various output from Pacejka Tire model
    '''

    def __init__(self,fileName_steadyState,fileName_transient="none"):
        '''
        Input:
        	filename: full filename w/ input data in csv form
        '''
        # first file is the steady state magic formula output
        self._m_filename_SS = fileName_steadyState
        # optional: compare against the transient slip output
        if( fileName_transient=="none"):
            self._use_transient_slip = False
        else:
            self._use_transient_slip = True
            self._m_filename_T = fileName_transient
            df_T = pd.read_csv(self._m_filename_T, header=0, sep=',')   # index_col=0,
            self._m_df_T = df_T
            
        # Header Form:
        # time,kappa,alpha,gamma,kappaP,alphaP,gammaP,Vx,Vy,Fx,Fy,Fz,Mx,My,Mz,Fxc,Fyc,Mzc
        df = pd.read_csv(self._m_filename_SS, header=0, sep=',')    # index_col=0, 
        self._m_df = df
        
    
    # @brief plot Forces, moments, pure slip vs. kappa
    def plot_kappa_FMpure(self,adams_Fx_tab_filename="none",adams_Mz_tab_filename="none"):
        # force v. kappa
        figF = plt.figure()
        df_kappa = pd.DataFrame(self._m_df, columns = ['kappa','Fx','Fz','Fxc'])
        
        # always create the plot, axes handle
        axF = df_kappa.plot(linewidth=2.0,x='kappa',y=['Fxc','Fz'])
        axF.set_xlabel(r'$\kappa $ ')
        axF.set_ylabel('Force [N]')
        axF.set_title(r'$\kappa *$, pure long. slip')
        if( adams_Fx_tab_filename != "none"):
            # load in the adams junk data, Fx vs. kappa %
            dfx_adams = pd.read_table(adams_Fx_tab_filename,sep='\t',header=0)
            dfx_adams['Longitudinal Slip'] = dfx_adams['Longitudinal Slip'] / 100.0
            axF.plot(dfx_adams['Longitudinal Slip'],dfx_adams['Longitudinal Force'],'r--',linewidth=1.5,label="Fx Adams")
            # plt.legend(loc='best')
        # compare transient slip output also?
        if( self._use_transient_slip):
            df_ts = pd.DataFrame(self._m_df_T, columns = ['kappa','Fx','Fxc'])
            axF.plot(df_ts['kappa'],df_ts['Fx'],'c--',linewidth=1.0,label='Fx transient')
            axF.plot(df_ts['kappa'],df_ts['Fxc'],'k-*',linewidth=1.0,label='Fxc transient')
        axF.legend(loc='best')
        
        figM = plt.figure()
        df_kappaM = pd.DataFrame(self._m_df, columns = ['kappa','Mx','My','Mz','Mzc'])
        axM = df_kappaM.plot(linewidth=2.0,x='kappa',y=['Mx','My','Mzc'])
        if( adams_Mz_tab_filename != "none"):
            dfmz_adams = pd.read_table(adams_Mz_tab_filename,sep='\t',header=0)
            dfmz_adams['longitudinal_slip'] = dfmz_adams['longitudinal_slip']/100.
            axM.plot(dfmz_adams['longitudinal_slip'], dfmz_adams['aligning_moment'],'r--',linewidth=1.5,label="Mz Adams")
        if( self._use_transient_slip):
            df_tsM = pd.DataFrame(self._m_df_T, columns = ['kappa','Mzc','Mzx','Mzy','M_zrc','t','s'])
            axM.plot(df_tsM['kappa'],df_tsM['Mzc'],'k-*',linewidth=1.0,label='Mzc transient')
            axM.plot(df_tsM['kappa'], df_tsM['Mzx'],'b--',linewidth=2,label='Mz,x')
            axM.plot(df_tsM['kappa'], df_tsM['Mzy'],'g--',linewidth=2,label='Mz,y')
            axM.plot(df_tsM['kappa'], df_tsM['M_zrc'],'y--',linewidth=2,label='M_zrc')
            axM2 = axM.twinx()
            axM2.plot(df_tsM['kappa'], df_tsM['t'],'b.',linewidth=1.0,label='t trail')
            axM2.plot(df_tsM['kappa'], df_tsM['s'],'c.',linewidth=1.0,label='s arm')
            axM2.set_ylabel('length [m]')
            axM2.legend(loc='lower right')            
            
        axM.set_xlabel(r'$\kappa $ ')
        axM.set_ylabel('Moment [N-m]')
        axM.legend(loc='best')
        axM.set_title(r'$\kappa $, pure long. slip')
        
    # @brief plot Forces, Moments, pure slip vs. alpha
    def plot_alpha_FMpure(self,adams_Fy_tab_filename="none",adams_Mz_tab_filename="none"):
        figF = plt.figure()
        df_sy = pd.DataFrame(self._m_df, columns = ['alpha','Fy','Fz','Fyc'])
        axF = df_sy.plot(linewidth=2.0,x='alpha', y=['Fyc','Fz'])# y=['Fy','Fyc','Fz'])
        if( adams_Fy_tab_filename != "none"): 
            # load in adams tabular data for Fy vs. alpha [deg]                             
            dfy_adams = pd.read_table(adams_Fy_tab_filename,sep='\t',header=0)
            axF.plot(dfy_adams['slip_angle'],dfy_adams['lateral_force'],'r--',linewidth=1.5,label="Fy Adams")
        # compare transient slip output
        if(self._use_transient_slip):
            # already have this in df_T
            df_ts = pd.DataFrame(self._m_df_T, columns =   ['alpha','Fyc','Fy'] )
            # axF.plot(df_ts['alpha'], df_ts['Fy'],'c-*',linewidth=2.0,label="Fy Transient")
            axF.plot(df_ts['alpha'], df_ts['Fyc'],'k-*',linewidth=1.0,label="Fyc Transient")
        axF.set_xlabel(r'$\alpha $[deg]')
        axF.set_ylabel('Force [N]')
        axF.legend(loc='best')
        axF.set_title(r'$\alpha $, pure lateral slip')
        
        figM = plt.figure()
        df_M = pd.DataFrame(self._m_df, columns = ['alpha','Mx','My','Mz','Mzc'])
        axM = df_M.plot(linewidth=2.0,x='alpha',y=['Mx','My','Mzc']) # ,'Mz'])
        if( adams_Mz_tab_filename != "none"):
            dfmz_adams = pd.read_table(adams_Mz_tab_filename,sep='\t',header=0)
            axM.plot(dfmz_adams['slip_angle'], dfmz_adams['aligning_moment'],'r--',linewidth=1.5,label="Mz Adams")
        # also plot transient slip outputs
        if(self._use_transient_slip):
            # already have this in df_T
            df_tsM = pd.DataFrame(self._m_df_T, columns = ['alpha','Mz','Mzc','MP_z','M_zr','t','s'] )
            # axM.plot(df_tsM['alpha'], df_tsM['Mz'],'k-*',linewidth=1.0,label="Mz Transient")
            axM.plot(df_tsM['alpha'], df_tsM['Mzc'],'k-*',linewidth=1.0,label="Mzc Transient")    
            axM.plot(df_tsM['alpha'], df_tsM['MP_z'],'g--',linewidth=2,label="MP_z")
            axM.plot(df_tsM['alpha'], df_tsM['M_zr'],'y--',linewidth=2,label="M_zr")
            axM2 = axM.twinx()
            axM2.plot(df_tsM['alpha'], df_tsM['t'],'b.',linewidth=1.0,label='t trail')
            axM2.plot(df_tsM['alpha'], df_tsM['s'],'c.',linewidth=1.0,label='s arm')
            axM2.legend(loc='lower right')
            axM2.set_ylabel('length [m]')
            
        axM.set_xlabel(r'$\alpha  $[deg]')
        axM.set_ylabel('Moment [N-m]')
        axM.legend(loc='best')
        axM.set_title(r'$\alpha $, pure lateral slip')
        
    # @brief plot combined forces, moments vs. kappa
    def plot_combined_kappa(self, adams_Fx_tab_filename="none",adams_Fy_tab_filename="none",adams_Mz_tab_filename="none"):
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
            df_T_M = pd.DataFrame(self._m_df_T, columns = ['kappa','Mzc','Fyc','Fxc','Mzx','Mzy','M_zrc','t','s'])
            axM.plot(df_T_M['kappa'], df_T_M['Mzc'],'k-*',linewidth=1.0,label='Mzc transient')
            # overlay the components of the moment from the x and y forces, respectively
            axM.plot(df_T_M['kappa'], df_T_M['Mzx'],'b--',linewidth=2,label='Mz,x')
            axM.plot(df_T_M['kappa'], df_T_M['Mzy'],'g--',linewidth=2,label='Mz,y')
            axM.plot(df_T_M['kappa'], df_T_M['M_zrc'],'y--',linewidth=2,label='M_zrc')
            axM2 = axM.twinx()
            axM2.plot(df_T_M['kappa'], df_T_M['t'],'b.',linewidth=1.0,label='t trail')
            axM2.plot(df_T_M['kappa'], df_T_M['s'],'c.',linewidth=1.0,label='s arm')
            axM2.set_ylabel('length [m]')
            axM2.legend(loc='lower right')
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
    def plot_custom(self,x_col, y_col_list,fig_title='custom plot',compare_transient=False,compare_adams_file="none"):
        fig = plt.figure()
        df_cols = []
        df_cols = list(y_col_list)
        df_cols.append(x_col)
        df_sy = pd.DataFrame(self._m_df, columns = df_cols)
        ax = df_sy.plot(linewidth=1.5, x=x_col, y=y_col_list)        
        if(self._use_transient_slip and compare_transient):
            df_fxfy = pd.DataFrame(self._m_df_T, columns = df_cols)
            ax.plot(df_fxfy[x_col], df_fxfy[y_col_list[0]], 'k-', linewidth=2,label='transient')

        
        if( compare_adams_file != "none"):
            dfy_adams = pd.read_table(compare_adams_file, sep='\t', header=0)
            time = py.arange(0,8,8.0/len(dfy_adams['slip_angle'] ))
            angToRad = 3.14159/180.0
            ax.plot(time, dfy_adams['slip_angle']*angToRad,'r--',linewidth=2,label="Adams")
        

        ax.set_xlabel(x_col)
        ax.legend(loc='best')    
        ax.set_title(fig_title)

    # @brief similar as above, but y_col_2 will be on a second y-axis
    def plot_custom_two(self,x_col, y_col_1, y_secondary, figTitle ='custom2 plot'):
        fig = plt.figure()
        df_cols = [x_col , y_col_1 , y_secondary]
        df_sy = pd.DataFrame(self._m_df, columns = df_cols)
        ax = df_sy.plot(linewidth=1.5, x= x_col, y= y_col_1, label = y_col_1)
        ax_2 = ax.twinx()
        ax_2.plot(df_sy[x_col], df_sy[y_secondary], 'c--', linewidth = 1.5, label = y_secondary)
       
        ax.set_xlabel(x_col)
        ax.legend(loc='best')
        ax_2.legend(loc='upper right')
        ax.set_title(figTitle)
        
    #  @brief plot (Fx,Fy)
    def plot_FxFy(self, adams_Fx_tab_filename="none",adams_Fy_tab_filename="none"):
        fig = plt.figure()
        df_ss = pd.DataFrame(self._m_df, columns = ['Fxc','Fyc'])
        ax = df_ss.plot(linewidth=1.5, x='Fxc', y='Fyc')        
        if(self._use_transient_slip):
            df_t = pd.DataFrame(self._m_df_T, columns = ['Fxc','Fyc'])
            ax.plot(df_t['Fxc'], df_t['Fyc'], 'k*-', linewidth=1.5,label='transient')

        if( adams_Fx_tab_filename != "none"):
            dfx_adams = pd.read_table(adams_Fx_tab_filename,sep='\t',header=0)
            dfy_adams = pd.read_table(adams_Fy_tab_filename,sep='\t',header=0)
            ax.plot(dfx_adams['longitudinal force'], dfy_adams['lateral_force'],'r--',linewidth=2,label="Adams")
            
        ax.set_xlabel(r'$F_x $[N]')
        ax.set_ylabel(r'$F_y $ [N]')
        ax.legend(loc='best')    
        ax.set_title('combined slip')
        
    # @brief plot (Fx, Mz)
    def plot_FxMz(self, adams_Fx_name="none", adams_Fy_name="none"):
        fig = plt.figure()
        df_ss = pd.DataFrame(self._m_df, columns = ['Fxc','Mzc'])
        ax = df_ss.plot(linewidth=1.5, x='Fxc', y='Mzc')        
        if(self._use_transient_slip):
            df_t = pd.DataFrame(self._m_df_T, columns = ['Fxc','Mzc'])
            ax.plot(df_t['Fxc'], df_t['Mzc'], 'k*-', linewidth=1.5,label='transient')

        if( adams_Fx_name != "none"):
            dfx_adams = pd.read_table(adams_Fx_name,sep='\t',header=0)
            dfy_adams = pd.read_table(adams_Fy_name,sep='\t',header=0)
            ax.plot(dfx_adams['longitudinal force'], dfy_adams['aligning_moment'],'r--',linewidth=2,label="Adams")

        ax.set_xlabel(r'$F_x $[N]')
        ax.set_ylabel(r'$M_z $ [Nm]')
        ax.legend(loc='best')    
        ax.set_title('combined slip')        
    
    
    # @brief plot reactions vs. alpha between similar runs w/ different gamma
    # assuming tire_gamma0 is the zero (or lower) of the two gamma value
    def plot_gammaComparison(self, tire_gamma0, gamma_val = 10):
        
        figFy = plt.figure()
        df_G = pd.DataFrame(self._m_df_T, columns = ['alpha','Fyc','gammaP'])
        axFy = df_G.plot(linewidth=2.0,x='alpha', y=['Fyc'])
        # compare to Fy from other tire
        df_G0 = pd.DataFrame(tire_gamma0._m_df_T, columns =   ['alpha','Fyc','gammaP'] )
        axFy.plot(df_G0['alpha'], df_G0['Fyc'],'k-*',linewidth=1.0,label=r'Fy, $\gamma$ =0')
        axFy2 = axFy.twinx()
        axFy2.plot(df_G['alpha'], df_G['gammaP'], 'r--', linewidth = 2.0, label=r'$\gamma $P ')
        
        axFy.set_xlabel(r'$\alpha $[deg]')
        axFy.set_ylabel('Force [N]')
        axFy.legend(loc='best')
        axFy2.legend(loc='lower left')
        axFy.set_title(r'Fy vs. $\alpha $, pure lateral slip')
        
        # plot self, which is assumed to have results for a non-zero gamme = gamma_val
        figMz = plt.figure()
        df_Mz = pd.DataFrame(self._m_df_T, columns = ['alpha','Mzc','Mzx','Mzy','M_zrc','t','s'])
        axMz = df_Mz.plot(linewidth=2.0, x='alpha',y=['Mzc'])
        # overlay the components of the moment from the x and y forces, respectively
        # when gamma is nonzero
        axMz.plot(df_Mz['alpha'], df_Mz['Mzx'],'b--',linewidth=2,label='Mz,x')
        axMz.plot(df_Mz['alpha'], df_Mz['Mzy'],'g--',linewidth=2,label='Mz,y')
        axMz.plot(df_Mz['alpha'], df_Mz['M_zrc'],'y--',linewidth=2,label='M_zrc')
        # trailing and side arms in meters
        axMz2 = axMz.twinx()
        axMz2.plot(df_Mz['alpha'], df_Mz['t'],'b.',linewidth=1.0,label='t trail')
        axMz2.plot(df_Mz['alpha'], df_Mz['s'],'c.',linewidth=1.0,label='s arm')

        # overlay plot for input PacTire_panda object, results when gamma = 0
        df_MzG0 = pd.DataFrame(tire_gamma0._m_df_T, columns = ['alpha','Mzc','Mzx','Mzy','M_zrc','t','s'] )
        axMz.plot(df_MzG0['alpha'], df_MzG0['Mzc'],'k-*',linewidth=1.0,label=r'Mzc, $\gamma $=0')        
        # also overlay components for the gamma = 0 case
        axMz.plot(df_MzG0['alpha'], df_MzG0['Mzx'],'b-',linewidth=1,label=r'Mz,x, $\gamma $=0')
        axMz.plot(df_MzG0['alpha'], df_MzG0['Mzy'],'g-',linewidth=1,label=r'Mz,y,  $\gamma $=0')
        axMz.plot(df_MzG0['alpha'], df_MzG0['M_zrc'],'y-',linewidth=1,label=r'M_zrc,  $\gamma $=0')
        axMz2.plot(df_MzG0['alpha'], df_MzG0['t'],'b-',linewidth=1.0,label='t trail, $\gamma $=0')
        axMz2.plot(df_MzG0['alpha'], df_MzG0['s'],'c-',linewidth=1.0,label='s arm, $\gamma $=0')

        axMz2.set_ylabel('length [m]')
        axMz2.legend(loc='lower right')
     
        
        axMz.set_xlabel(r'$\alpha  $[deg]')
        axMz.set_ylabel('Moment [N-m]')
        axMz.legend(loc='best')
        axMz.set_title(r'Mz vs. $\alpha $, pure lateral slip, $\gamma$='+str(gamma_val) +' deg.')
        
        
    
if __name__ == '__main__':
    # logger
    import logging as lg
    lg.basicConfig(fileName = 'logFile.log', level=lg.WARN, format='%(message)s')
    # default font size
    font = {'size' : 16}
    matplotlib.rc('font', **font)       
    
    # directory location on my laptop
    #dir_ChronoT = 'E:/Chrono-T_Build/bin/Debug/'
    #dir_Adams = 'C:/Users/justin/Google Drive/shareWith_Radu/PacTire/ACar_validation_tests/'
    
    # desktop 
    dir_ChronoT = 'D:/Chrono-T_Build/bin/Debug/'
    dir_Adams = 'C:/Users/newJustin/Google Drive/shareWith_Radu/PacTire/ACar_validation_tests/'
    
    
    
    # ****************** STEADY STATE SLIP    

    '''    
    # pure longitudinal slip case. (alpha = 0)
    longSlip = PacTire_panda(dir_ChronoT + "test_pacTire_pureLongSlip.csv")
    # validate pure Fx, Fy, Mz, vs. kappa    
    longSlip.plot_kappa_FMpure(dir_Adams + "pureLong_Fx.tab", dir_Adams + "pureLong_Mz.tab")
    '''                           
    
    
    '''
    # make two custom plots. Function usage depends on y-series units
    #       longSlip.plot_custom('Fx',['Fy','Fz'])
    #       longSlip.plot_custom_two('Fx','Fy','Mz')
    
    # pure lateral slip case, free rolling tire (kappa ~= 0)
    latSlip = PacTire_panda(dir_ChronoT + "test_pacTire_pureLatSlip.csv")
    latSlip.plot_alpha_FMpure(dir_Adams + "pureLat_Fy.tab",
                              dir_Adams + "pureLat_Mz.tab")
    # Fyc should = Fy for pure lateral slip case
    latSlip.plot_custom('alpha',['Fxc','Fyc','Fy'],'pure lateral slip')
    
    # overturning and rolling resistance moments
    # latSlip.plot_custom('alpha',['Mx','My'])
    # latSlip.plot_custom('alpha',['Mx'])
    '''    
    
    
    '''
    # combined slip case, (-a_lim,a_lim) and (-k_lim,k_lim) for alpha, kappa, respectively
    combinedSlip = PacTire_panda(dir_ChronoT + "test_pacTire_combinedSlip.csv")
    combinedSlip.plot_combined_kappa(dir_Adams + "combined_Fx.tab",
                                     "none",dir_Adams + "combined_Mz.tab")
    combinedSlip.plot_combined_alpha("none",dir_Adams + "combined_Fy.tab")
    
    # combinedSlip.plot_custom('Fxc',['Fyc'])
    
    # compare overturning and rolling resistance moments to pure lateral slip case
    # combinedSlip.plot_custom('alpha',['Mx','My'],'combined slip')
    '''
    
    
    # *************   TRANSIENT SLIP
    
    # pure longitudinal slip case
    longSlip = PacTire_panda(dir_ChronoT + 'test_pacTire_pureLongSlip.csv',
                            dir_ChronoT + 'test_pacTire_pureLongSlip_transient.csv')
    # plot rea;ctions vs. kappa
    longSlip.plot_kappa_FMpure(dir_Adams + 'pureLong_Fx.tab',
                               dir_Adams + 'pureLong_Mz.tab')
    
    # pure lateral slip case, gamma = 0
    latSlip = PacTire_panda(dir_ChronoT + "test_pacTire_pureLatSlip.csv",
                        dir_ChronoT + "test_pacTire_pureLatSlip_transient.csv")
    # plot reactions vs. alpha
    latSlip.plot_alpha_FMpure(dir_Adams + "pureLat_Fy.tab",
                              dir_Adams + "pureLat_Mz.tab")    
    
    # pure lateral slip, gamma = 10 deg.
    latGammaSlip = PacTire_panda(dir_ChronoT + "test_pacTire_pureLatSlipGamma.csv",
                                 dir_ChronoT + "test_pacTire_pacLatSlipGamma_transient.csv")
    # plot reactions vs. alpha, compared to the previous tire that had gamma = 0
    latGammaSlip.plot_gammaComparison(latSlip)
    
    # combined slip
    combinedSlip = PacTire_panda(dir_ChronoT + "test_pacTire_combinedSlip.csv",
                                 dir_ChronoT + "test_pacTire_combinedSlip_transient.csv")
                                 
    # plot some reactions vs. kappa
    combinedSlip.plot_combined_kappa(dir_Adams + "combined_Fx.tab",
                                     "none", dir_Adams + "combined_Mz.tab")
    # and others vs. alpha
    combinedSlip.plot_combined_alpha("none",dir_Adams + "combined_Fy.tab")
    
    # combinedSlip.plot_custom('Fxc',['Fyc'])
    
    # more random things
    # combinedSlip.plot_custom('alpha',['Mx','My'],'combined slip')
    
    # combinedSlip.plot_custom('Fxc',['Fyc'],'combined slip',True)
    combinedSlip.plot_FxFy(dir_Adams + "combined_Fx.tab",dir_Adams + "combined_Fy.tab")
    combinedSlip.plot_FxMz(dir_Adams + "combined_Fx.tab",dir_Adams + "combined_Mz.tab")
    
    # DEBUGGING
    combinedSlip.plot_custom('time',['alphaP'],'alpha',True, dir_Adams + "combined_Fy.tab")

    py.show()