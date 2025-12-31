# -*- coding: utf-8 -*-
"""
Created on Thu Jan 10 11:01:21 2019

@author: SB
"""

import pychrono as chrono
import chtrain_ant
import chtrain_pendulum

def Init(env_name, render):
       if env_name=='ChronoAnt':
              return chtrain_ant.Model(render)
                     
       elif env_name=='ChronoPendulum':
              return chtrain_pendulum.Model(render)
       else: 
              print('Unvalid environment name')
                            
       