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
        axM.legend(loc='upper center')
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
        df_uv = pd.DataFrame(self._DF[tire_num], columns = ['time','u','valpha','du','dvalpha','vgamma','dvgamma','Vx','Vy'])
        ax_uv = df_uv.plot(linewidth=1.5, x='time', y=['u','valpha','vgamma'])
        ax_uv_2 = ax_uv.twinx()
        ax_uv_2.plot(df_uv['time'], df_uv['Vx'], 'c--', linewidth = 1.5, label = 'Vx')
        ax_uv_2.plot(df_uv['time'], df_uv['Vy'], 'b--', linewidth = 1.5, label = 'Vy')
        ax_uv.set_xlabel('time [sec]')
        ax_uv.legend(loc='best')
        ax_uv_2.legend(loc='upper right')
        ax_uv.set_title(titleStr)
        
        # plot integrated ODE values for each slip displacement
        fig_duv = plt.figure()
        ax_duv = df_uv.plot(linewidth=2.0,x='time',y=['du','dvalpha','dvgamma'])
        ax_duv_2 = ax_duv.twinx()
        ax_duv_2.plot(df_uv['time'], df_uv['Vx'], 'c--', linewidth = 1.5, label = 'Vx')
        ax_duv_2.plot(df_uv['time'], df_uv['Vy'], 'b--', linewidth = 1.5, label = 'Vy')
        ax_duv.set_xlabel('time [sec]')
        ax_duv.legend(loc='best')
        ax_duv_2.legend(loc='upper center')
        ax_duv.set_title(titleStr)
        
        # plot the calculated transient slips
        fig_slip = plt.figure()
        df_slip = pd.DataFrame(self._DF[tire_num], columns = ['time','alphaP','kappaP','gammaP','Vx','Vy'])
        ax_slip = df_slip.plot(linewidth=1.5, x='time', y=['alphaP','gammaP'])
        ax_slip_2 = ax_slip.twinx()
        ax_slip_2.plot(df_slip['time'], df_slip['kappaP'], 'r', linewidth = 1.5, label = r"$\kappa$'")
        ax_slip_2.plot(df_slip['time'], df_slip['Vx'], 'c--', linewidth = 1.5, label = 'Vx')
        ax_slip_2.plot(df_slip['time'], df_slip['Vy'], 'b--', linewidth = 1.5, label = 'Vy')
        ax_slip.set_xlabel('time [sec]')
        ax_slip.legend(loc='upper left')
        ax_slip_2.legend(loc = 'upper right')
        ax_slip.set_title(titleStr)
        
        # forward velocity, spin
        fig_vel = plt.figure()
        df_vel = pd.DataFrame(self._DF[tire_num], columns = ['time','Vx','omega'])
        ax_vel = df_vel.plot(linewidth=1.5, x='time', y=['Vx','omega'])
        ax_vel.set_xlabel('time [sec]')
        ax_vel.legend(loc='best')
        ax_vel.set_title(titleStr)
        
        # plot radius vs. m_F_z on second axis
        fig_rad = plt.figure()
        df_rad = pd.DataFrame(self._DF[tire_num], columns = ['time','R0','R_l','Reff','m_Fz'])
        ax_rad = df_rad.plot(linewidth=1.5, x='time', y=['R0','R_l','Reff'])
        ax_rad.set_xlabel('time [sec]')
        ax_rad2 = ax_rad.twinx()
        ax_rad2.plot(df_rad['time'], df_rad['m_Fz'], 'r--', linewidth=1.5, label='m_Fz')
        ax_rad.legend(loc='best')
        ax_rad2.legend(loc='lower right')
        
    # @brief a subplot for each forces, all 4 tires, vs. time    
    def plotall_forces(self,plot_global=False):
        # plot the forces vs time
        fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
        data_cols = []
        if(plot_global):
            data_cols = ['time','Fxc','Fyc','m_Fz','FX','FY','FZ']
        else:
            data_cols = ['time','Fxc','Fyc','m_Fz']
        df_FL = pd.DataFrame(self._DF[0], columns = data_cols)
        df_FR = pd.DataFrame(self._DF[1], columns = data_cols)
        df_RL = pd.DataFrame(self._DF[2], columns = data_cols)
        df_RR = pd.DataFrame(self._DF[3], columns = data_cols)
        df_list = [df_FL, df_FR, df_RL, df_RR]
        marker_list = ['k-','k--','r-','r--']
        leg_list = ['FL','FR','RL','RR']
        
        # plot Fx, Fy and m_Fz
        for i in range(4):
            ax1.plot(df_list[i]['time'], df_list[i]['Fxc'], marker_list[i], linewidth=1.5, label=leg_list[i])
            ax2.plot(df_list[i]['time'], df_list[i]['Fyc'], marker_list[i], linewidth=1.5, label=leg_list[i])
            ax3.plot(df_list[i]['time'], df_list[i]['m_Fz'], marker_list[i], linewidth=1.5, label=leg_list[i])
       
        ax1.set_title('Forces, all tires')
        ax3.set_xlabel('time [sec]')
        ax1.set_ylabel('Fxc [N]')
        ax2.set_ylabel('Fyc [N]')
        ax3.set_ylabel('Fz [N]')
        ax1.legend(loc='upper right')
        ax2.legend(loc='upper right')
        ax3.legend(loc='upper right')
        ax1.grid(True)
        ax2.grid(True)
        ax3.grid(True)
        
        # a new plot for the total global reactions applied to the wheel rigid body
        if( plot_global ):
            figG, (axG1, axG2, axG3) = plt.subplots(3, sharex=True)
            # plot Fx, Fy and m_Fz
            for i in range(4):
                axG1.plot(df_list[i]['time'], df_list[i]['FX'], marker_list[i], linewidth=1.5, label=leg_list[i]+' global')
                axG2.plot(df_list[i]['time'], df_list[i]['FY'], marker_list[i], linewidth=1.5, label=leg_list[i]+' global')
                axG3.plot(df_list[i]['time'], df_list[i]['FZ'], marker_list[i], linewidth=1.5, label=leg_list[i]+' global')
               
                axG1.set_title('Forces, all tires')
                axG3.set_xlabel('time [sec]')
                axG1.set_ylabel('Fx [N]')
                axG2.set_ylabel('Fy [N]')
                axG3.set_ylabel('Fz [N]')
                axG1.legend(loc='best')
                axG2.legend(loc='best')
                axG3.legend(loc='best')
                axG1.grid(True)
                axG2.grid(True)
                axG3.grid(True)        
            

    # @brief a subplot for each forces, pure Fx and Fy    
    def plotall_pureForces(self):
        # plot the forces vs time
        fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
        data_cols = ['time','Fx','Fy','Mz']
        df_FL = pd.DataFrame(self._DF[0], columns = data_cols)
        df_FR = pd.DataFrame(self._DF[1], columns = data_cols)
        df_RL = pd.DataFrame(self._DF[2], columns = data_cols)
        df_RR = pd.DataFrame(self._DF[3], columns = data_cols)
        df_list = [df_FL, df_FR, df_RL, df_RR]
        marker_list = ['k-','k--','r-','r--']
        leg_list = ['FL','FR','RL','RR']
        
        # plot Fx, Fy and m_Fz
        for i in range(4):
            ax1.plot(df_list[i]['time'], df_list[i]['Fx'], marker_list[i], linewidth=1.5, label=leg_list[i])
            ax2.plot(df_list[i]['time'], df_list[i]['Fy'], marker_list[i], linewidth=1.5, label=leg_list[i])
            ax3.plot(df_list[i]['time'], df_list[i]['Mz'], marker_list[i], linewidth=1.5, label=leg_list[i])
       
        ax1.set_title('Pure Forces, all tires')
        ax3.set_xlabel('time [sec]')
        ax1.set_ylabel('Fx [N]')
        ax2.set_ylabel('Fy [N]')
        ax3.set_ylabel('Mz [N-m]')
        ax1.legend(loc='best')
        ax2.legend(loc='best')
        ax3.legend(loc='best')
        ax1.grid(True)
        ax2.grid(True)
        ax3.grid(True)
        
        
    # @brief a subplot for each moment, all 4 tires, vs. time    
    def plotall_moments(self,plot_global=False):
        # plot the forces vs time
        fig, (ax1, ax2, ax3) = plt.subplots(3, sharex=True)
        data_cols = []
        if(plot_global):
            data_cols = ['time','Mx','My','Mzc','MX','MY','MZ']
        else:
            data_cols = ['time','Mx','My','Mzc']
            
        df_FL = pd.DataFrame(self._DF[0], columns = data_cols)
        df_FR = pd.DataFrame(self._DF[1], columns = data_cols)
        df_RL = pd.DataFrame(self._DF[2], columns = data_cols)
        df_RR = pd.DataFrame(self._DF[3], columns = data_cols)
        df_list = [df_FL, df_FR, df_RL, df_RR]
        marker_list = ['k-','k--','r-','r--']
        leg_list = ['FL','FR','RL','RR']
        
        # plot Mx, My, Mzc, in tire contact frame
        for i in range(4):
            ax1.plot(df_list[i]['time'], df_list[i]['Mx'], marker_list[i], linewidth=1.5, label=leg_list[i])
            ax2.plot(df_list[i]['time'], df_list[i]['My'], marker_list[i], linewidth=1.5, label=leg_list[i])
            ax3.plot(df_list[i]['time'], df_list[i]['Mzc'], marker_list[i], linewidth=1.5, label=leg_list[i])
       
        ax1.set_title('Moments, all tires')
        ax3.set_xlabel('time [sec]')
        ax1.set_ylabel('Mx [N-m]')
        ax2.set_ylabel('My [N-m]')
        ax3.set_ylabel('Mzc [N-m]')
        ax1.legend(loc='upper right')
        ax2.legend(loc='upper right')
        ax3.legend(loc='upper right')
        ax1.grid(True)
        ax2.grid(True)
        ax3.grid(True)   
        
        
        # a new plot for the total global reactions applied to the wheel rigid body
        if( plot_global ):
            figG, (axG1, axG2, axG3) = plt.subplots(3, sharex=True)
            
            # plot MX, MY, MZ, caps indicate global coords
            for i in range(4):
                axG1.plot(df_list[i]['time'], df_list[i]['MX'], marker_list[i], linewidth=1.5, label=leg_list[i]+' global')
                axG2.plot(df_list[i]['time'], df_list[i]['MY'], marker_list[i], linewidth=1.5, label=leg_list[i]+' global')
                axG3.plot(df_list[i]['time'], df_list[i]['MZ'], marker_list[i], linewidth=1.5, label=leg_list[i]+' global')
               
                axG1.set_title('Forces, all tires')
                axG3.set_xlabel('time [sec]')
                axG1.set_ylabel('MX [Nm]')
                axG2.set_ylabel('MY [Nm]')
                axG3.set_ylabel('MZ [Nm]')
                axG1.legend(loc='best')
                axG2.legend(loc='best')
                axG3.legend(loc='best')
                axG1.grid(True)
                axG2.grid(True)
                axG3.grid(True)        
        
        
    # @brief detailed subplot for each wheel Mz, one for each of the 4 tires, vs. time.
    # NOTE: terms of Mzc are always expressed on the wheel side of the tire parameter file.
    # e.g., Mz,x Mz,y and Mz,r are the same for the left/right tires, but Mzc
    # switches signs to give the correct symmetry for Mz
    def plotall_Mz_detailed(self):
        # plot the forces vs time
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
        df_FL = pd.DataFrame(self._DF[0], columns = ['time','Mzc','Mzx','Mzy','M_zrc','t','s'])
        df_FR = pd.DataFrame(self._DF[1], columns = ['time','Mzc','Mzx','Mzy','M_zrc','t','s'])
        df_RL = pd.DataFrame(self._DF[2], columns = ['time','Mzc','Mzx','Mzy','M_zrc','t','s'])
        df_RR = pd.DataFrame(self._DF[3], columns = ['time','Mzc','Mzx','Mzy','M_zrc','t','s'])
        
        df_list = [df_FL, df_FR, df_RL, df_RR]
        ax_list = [ax1, ax2, ax3, ax4]
        leg_list = ['FL','FR','RL','RR']
        
        # plot Mzc, Mzx, Mzy, M_zrc, t and s for each tire here
        for i in range(4):
            ax_list[i].plot(df_list[i]['time'], df_list[i]['Mzc'],'k-',linewidth=2,label=leg_list[i]+' Mzc')
            ax_list[i].plot(df_list[i]['time'], df_list[i]['Mzx'],'b--',linewidth=2,label=leg_list[i]+' Mz,x')
            ax_list[i].plot(df_list[i]['time'], df_list[i]['Mzy'],'r--',linewidth=2,label=leg_list[i]+' Mz,y')
            ax_list[i].plot(df_list[i]['time'], df_list[i]['M_zrc'],'y--',linewidth=2,label=leg_list[i]+' Mz,r')
            ax_list[i].legend(loc= 'upper left')
            ax_list[i].set_ylabel('Moment [N-m]')
            ax_list[i].grid(True)
            '''
            axM2 = ax_list[i].twinx()
            axM2.plot(df_list[i]['time'], df_list[i]['t'],'g-',linewidth=1.5,label=leg_list[i]+' t trail')
            axM2.plot(df_list[i]['time'], df_list[i]['s'],'c-',linewidth=1.5,label=leg_list[i]+' s arm')
            axM2.set_ylabel('length [m]')
            axM2.legend(loc='upper right')        
            '''
        ax1.set_title('Mzc =(Mz,x + Mz,y + Mz,r)*m_sameSide')
        ax4.set_xlabel('time [sec]')       
        
    # @brief plot input slips, and the 4 main reactions (Fxc, Fyc, My, Mzc)  
    # plot input slips to the force/moment equations (alphaP, kappaP, gammaP)
    # plot the kinematic slips from rigid body info (alpha, kappa, gamma)
    def plotall_slips(self, plot_kinematic = False):    
        # plot the forces vs time
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
        data_cols = []
        if( plot_kinematic):
            data_cols = ['time','Fxc','Fyc','My','Mzc','alpha','kappa','gamma','Vx','omega']
        else:
            data_cols = ['time','Fxc','Fyc','My','Mzc','alphaP','kappaP','gammaP','Vx','omega']
            
        df_FL = pd.DataFrame(self._DF[0], columns = data_cols)
        df_FR = pd.DataFrame(self._DF[1], columns = data_cols)
        df_RL = pd.DataFrame(self._DF[2], columns = data_cols)
        df_RR = pd.DataFrame(self._DF[3], columns = data_cols)            
        
        df_list = [df_FL, df_FR, df_RL, df_RR]
        ax_list = [ax1, ax2, ax3, ax4]
        leg_list = ['FL','FR','RL','RR']
        
        # plot Mzc, Mzx, Mzy, M_zrc, t and s for each tire here
        for i in range(4):
            if(plot_kinematic):
                ax_list[i].plot(df_list[i]['time'], df_list[i]['alpha'],'b-',linewidth=1.5,label=leg_list[i]+r" $\alpha$")
                ax_list[i].plot(df_list[i]['time'], df_list[i]['kappa'],'r-',linewidth=1.5,label=leg_list[i]+r" $\kappa$")
                ax_list[i].plot(df_list[i]['time'], df_list[i]['gamma'],'g-',linewidth=1.5,label=leg_list[i]+r" $\gamma$")
            else:
                ax_list[i].plot(df_list[i]['time'], df_list[i]['alphaP'],'b-',linewidth=1.5,label=leg_list[i]+r" $\alpha$'")
                ax_list[i].plot(df_list[i]['time'], df_list[i]['kappaP'],'r-',linewidth=1.5,label=leg_list[i]+r" $\kappa$'")
                ax_list[i].plot(df_list[i]['time'], df_list[i]['gammaP'],'g-',linewidth=1.5,label=leg_list[i]+r" $\gamma$'")
            axM2 = ax_list[i].twinx()
            axM2.plot(df_list[i]['time'], df_list[i]['Vx'],'k--',linewidth=2.0,label=leg_list[i]+' Vx')
            # ax_list[i].plot(df_list[i]['time'], df_list[i]['Vx'],'k--',linewidth=2.0,label=leg_list[i]+' Vx')
            axM2.plot(df_list[i]['time'], df_list[i]['omega'],'b--',linewidth=2.0,label=leg_list[i]+r' $\omega$')
            axM2.legend(loc= 'upper right')
            axM2.set_ylabel('[m/s], [rad/s]')
            
            ax_list[i].grid(True)
            ax_list[i].set_ylabel('slip [-]')
            ax_list[i].legend(loc='upper left')        
    
        ax1.set_title('Slips and reactions for each tire')
        ax4.set_xlabel('time [sec]')       
        
        
    # @brief plot contributions to bessel link: kappa = u_sigma - u_bessel
    def plotall_bessel_kappa(self):    
        # plot the forces vs time
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
        data_cols = ['time','du','u_Bessel','u_sigma','kappaP']

        df_FL = pd.DataFrame(self._DF[0], columns = data_cols)
        df_FR = pd.DataFrame(self._DF[1], columns = data_cols)
        df_RL = pd.DataFrame(self._DF[2], columns = data_cols)
        df_RR = pd.DataFrame(self._DF[3], columns = data_cols)            
        
        df_list = [df_FL, df_FR, df_RL, df_RR]
        ax_list = [ax1, ax2, ax3, ax4]
        leg_list = ['FL','FR','RL','RR']
        
        # plot x contributions
        for i in range(4):
            ax_list[i].plot(df_list[i]['time'], df_list[i]['kappaP'],'k-',linewidth=2.0,label=leg_list[i]+r" $\kappa$'")
            ax_list[i].plot(df_list[i]['time'], df_list[i]['u_sigma'],'r-',linewidth=1.5,label=leg_list[i]+' u_sigma')
            ax_list[i].plot(df_list[i]['time'], df_list[i]['u_Bessel'],'b--',linewidth=2.0,label=leg_list[i]+' u_Bessel')
            
            ax_list[i].set_ylabel('slip [-]')
            ax_list[i].grid(True)
            ax_list[i].legend(loc='upper right')        
    
        ax1.set_title('kappaP = u_sigma - u_Bessel - u_tow')
        ax4.set_xlabel('time [sec]')               
        
        
    # @brief plot contributions to bessel link: alpha = v_sigma - v_bessel - v_tow 
    def plotall_bessel_alpha(self):    
        # plot the forces vs time
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
        data_cols = ['time','dvalpha','v_Bessel','v_sigma','alphaP']

        df_FL = pd.DataFrame(self._DF[0], columns = data_cols)
        df_FR = pd.DataFrame(self._DF[1], columns = data_cols)
        df_RL = pd.DataFrame(self._DF[2], columns = data_cols)
        df_RR = pd.DataFrame(self._DF[3], columns = data_cols)            
        
        df_list = [df_FL, df_FR, df_RL, df_RR]
        ax_list = [ax1, ax2, ax3, ax4]
        leg_list = ['FL','FR','RL','RR']
        
        # plot x contributions
        for i in range(4):
            ax_list[i].plot(df_list[i]['time'], df_list[i]['alphaP'],'k-',linewidth=2.0,label=leg_list[i]+r" $\alpha$'")
            ax_list[i].plot(df_list[i]['time'], df_list[i]['v_sigma'],'r-',linewidth=1.5,label=leg_list[i]+' v_sigma')
            ax_list[i].plot(df_list[i]['time'], df_list[i]['v_Bessel'],'b--',linewidth=2.0,label=leg_list[i]+' v_Bessel')
            # axM2 = ax_list[i].twinx()
            # axM2.plot(df_list[i]['time'], df_list[i]['dvalpha'],'y--',linewidth=2.0,label=leg_list[i]+' I_dv')
            # axM2.legend(loc= 'upper right')
            ax_list[i].set_ylabel('slip [-]')
            ax_list[i].grid(True)
            ax_list[i].legend(loc='upper left')        
    
        ax1.set_title('alphaP = v_sigma - v_Bessel')
        ax4.set_xlabel('time [sec]')               
   
   
    # @brief plot pneumatic trail, t, and side moment arm, s, for each tire
    def plotall_ts(self):
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
        data_cols = ['time','t','s']

        df_FL = pd.DataFrame(self._DF[0], columns = data_cols)
        df_FR = pd.DataFrame(self._DF[1], columns = data_cols)
        df_RL = pd.DataFrame(self._DF[2], columns = data_cols)
        df_RR = pd.DataFrame(self._DF[3], columns = data_cols)            

        df_list = [df_FL, df_FR, df_RL, df_RR]
        ax_list = [ax1, ax2, ax3, ax4]
        leg_list = ['FL','FR','RL','RR']

        # plot x contributions
        for i in range(4):
            ax_list[i].plot(df_list[i]['time'], df_list[i]['t'],'k-',linewidth=1.5,label=leg_list[i]+ " t")
            ax_list[i].plot(df_list[i]['time'], df_list[i]['s'],'r-',linewidth=1.5,label=leg_list[i]+ "  s")
            ax_list[i].set_ylabel('disp. [m]')
            ax_list[i].grid(True)
            ax_list[i].legend(loc='upper left')        
    
        ax1.set_title('M_z_y = Fy * -t,  M_z_x = Fx * s')
        ax4.set_xlabel('time [sec]')          
       
       
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
    data_dir = 'D:/Chrono-T_Build/bin/Release/'
    
    dat_FL = 'test_HMMWV9_pacTire_FL.csv'
    dat_FR = 'test_HMMWV9_pacTire_FR.csv'
    dat_RL = 'test_HMMWV9_pacTire_RL.csv'
    dat_RR = 'test_HMMWV9_pacTire_RR.csv'
    
    #  ********************************************************************** 
    
    # create a tire using the single tire python plotting code
    # tire_LF = tire.PacTire_panda(data_dir + dat_FL)
    
    # tire_LF.plot_custom_two('time','m_Fz','Vx','FL')
    # tire_LF.plot_custom('time',['Fxc','Fyc'],'Fx, Fy [N]')
    # tire_LF.plot_custom('time',['kappa'],'longitudinal slip [-]')
    # tire_LF.plot_custom('time',['alpha'],'lateral slip [deg]')
    
    # tire_LF.plot_combined_kappa()
    # tire_LF.plot_combined_alpha()
    
    # use the new python code for plotting any/all of the pacTire outputs on a vehicle
    # there are 4 pacTire files associated with a HMMWV model

    vehicle_output = PacTire_vehicle_panda([data_dir+dat_FL, data_dir+dat_FR, data_dir+dat_RL, data_dir+dat_RR],
                                           ['FL','FR','RL','RR'])
    
    # plot w.r.t. to time
    # vehicle_output.plot_time(0,'Front Left')
    # vehicle_output.plot_time(1,'Front Right')
    # vehicle_output.plot_time(2,'Rear Left')
    # vehicle_output.plot_time(3,'Rear Right')
    
    # overlay all 4 tires for slip, reactions calculated.
    vehicle_output.plotall_slips()
    vehicle_output.plotall_forces(False)     # also plot global forces?
    vehicle_output.plotall_moments(False)    # also plot global moments?
    vehicle_output.plotall_Mz_detailed()
    vehicle_output.plotall_bessel_kappa()
    vehicle_output.plotall_bessel_alpha()
    # vehicle_output.plotall_ts()
    
    # vehicle_output.plotall_pureForces()


py.show()