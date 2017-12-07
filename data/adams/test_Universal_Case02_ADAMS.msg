
-------------------------------------------------------------------------------
|        ..          ..                                                       |
|       ####*.      ####.                                                     |
|         `###*.       `#.                 MSC.Software                       |
|            `###.       #.                ------------                       |
|               `##.     ##                                                   |
|                 `##.   ##                                                   |
|                   `## ##'                A d a m s   C++   S o l v e r      |
|                     ####                 [Build:         2012.1.0-CL127566] |
|                     ###.                 Version:        2012               |
|                    ###`#.                Patch Version:  None               |
|                  .###  `#                                                   |
|                 ####    `#                                                  |
|                ####      #.                                                 |
|              #####       `#              http://www.mscsoftware.com         |
|            .#####                                                           |
|           #####'                                                            |
|           ####                                                              |
|                                                                             |
|    *********************************************************************    |
|    *                                                                   *    |
|    *                      MSC.Software Corporation                     *    |
|    *                                                                   *    |
|    *                             A d a m s                             *    |
|    *                                                                   *    |
|    *          Automatic Dynamic Analysis of Mechanical Systems         *    |
|    *                                                                   *    |
|    *********************************************************************    |
|    *                                                                   *    |
|    *                         LEGAL INFORMATION                         *    |
|    *                                                                   *    |
|    *  Warning:  This computer  program is protected  by copyright law  *    |
|    *  and  international  treaties.    Unauthorized  reproduction  or  *    |
|    *  distribution  of this  computer program, or any  portion of it,  *    |
|    *  may result in severe civil and criminal penalties.               *    |
|    *                                                                   *    |
|    *  Copyright (C) 2011 MSC.Software Corporation and  its licensors.  *    |
|    *                        All rights reserved.                       *    |
|    *                                                                   *    |
|    *  This computer program may contain certain  third-party software  *    |
|    *  that is protected by  copyright and licensed from  MSC.Software  *    |
|    *  suppliers.   Portions of this software are owned  by UGS  Corp.  *    |
|    *  (C) Copyright 1997.   All Rights Reserved.   Portions, (C) 1998  *    |
|    *  Spacetec IMC Corporation ("Spacetec").                           *    |
|    *                                                                   *    |
|    *  The  MSC.Software logo,  MSC,  MSC Adams,  MD Adams,  Adams and  *    |
|    *  Adams/  are trademarks or registered trademarks of MSC.Software  *    |
|    *  Corporation  or its  subsidiaries in  the United States  and/or  *    |
|    *  other   countries.   FLEXlm  is  a   registered   trademark  of  *    |
|    *  Macrovision Corporation. Parasolid is a registered trademark of  *    |
|    *  UGS  Corp.   All  other trademarks are  the property  of  their  *    |
|    *  respective owners.                                               *    |
|    *                                                                   *    |
|    *********************************************************************    |
|                                                                             |
|-----------------------------------------------------------------------------|
|                                                                             |
|    *********************************************************************    |
|    *                                                                   *    |
|    *   Local Time: 2014-11-18 09:13:55                                 *    |
|    *    Host Name: Romney                                              *    |
|    *        Uname: MS Windows NT                                       *    |
|    *      OS Info: 6.1                                                 *    |
|    *         User: AdamsCar                                            *    |
|    *  Customer ID: 56BD3C53-F9BCF7F4                                   *    |
|    *  Current dir: C:\mtaylor\2014_JointValidation\ADAMS_Files_InProcess*    |
|    *                                                                   *    |
|    *********************************************************************    |
|                                                                             |
-------------------------------------------------------------------------------

 command: FILE/MODEL=test_Universal_Case02.adm, OUTPUT_PREFIX=test_Universal_Case02_ADAMS 

-------------------------------------------------------------------------------
|                                 Model Title                                 |
|=============================================================================|
|                                                                             |
|                ADAMS/View model name: test_Universal_Case02                 |
|                                                                             |
-------------------------------------------------------------------------------

 command: output/noseparator
 command: integrator/gstiff, &
 command: 	error = 1.0e-4, hmax=1e-5
 command: simulate/transient, &
 command: 	end=5.0, dtout=1.0E-002


  Begin Simulation

  ****** Performing Dynamic Simulation using Gstiff I3 Integrator ******
   The system is modelled with INDEX-3 DAEs.
   The integrator is GSTIFF, CORRECTOR = original
   Integration error =    1.000000E-04

   Simulation      Step        Function    Cumulative   Integration     CPU
      Time         Size       Evaluations  Steps Taken    Order        time
   ___________  ___________   ___________  ___________   _________    ________
   0.00000E+00  5.00000E-07            0            0       1            0.02 
   5.00000E-01  1.00000E-05       130795        50005       2            0.86 
   1.00000E+00  1.00000E-05       230846       100005       1            1.59 
   1.50000E+00  1.00000E-05       320729       150005       1            2.26 
   2.00000E+00  1.00000E-05       414804       200005       1            2.93 
   2.50000E+00  1.00000E-05       502635       250005       1            3.56 
   3.00000E+00  1.00000E-05       589059       300005       1            4.21 
   3.50000E+00  1.00000E-05       681878       350005       1            4.91 
   4.00000E+00  1.00000E-05       772649       400005       1            5.62 
   4.50000E+00  1.00000E-05       856630       450005       1            6.29 
   5.00000E+00  1.00000E-05       948347       500005       1            7.02 

  End Simulation
 command: stop


Finished -----
Elapsed time = 12.75s,  CPU time = 7.02s,  55.05%
