
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
|    *   Local Time: 2014-11-18 09:14:09                                 *    |
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

 command: FILE/MODEL=test_Spherical_Case02.adm, OUTPUT_PREFIX=test_Spherical_Case02_ADAMS 

-------------------------------------------------------------------------------
|                                 Model Title                                 |
|=============================================================================|
|                                                                             |
|                ADAMS/View model name: test_Spherical_Case02                 |
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
   0.00000E+00  5.00000E-07            0            0       1            0.00 
   5.00000E-01  1.00000E-05       140942        50005       2            0.73 
   1.00000E+00  1.00000E-05       286863       100005       2            1.45 
   1.50000E+00  1.00000E-05       430389       150005       2            2.14 
   2.00000E+00  1.00000E-05       573685       200005       2            2.87 
   2.50000E+00  1.00000E-05       713974       250005       2            3.56 
   3.00000E+00  1.00000E-05       860539       300005       2            4.27 
   3.50000E+00  1.00000E-05      1004962       350005       2            4.99 
   4.00000E+00  1.00000E-05      1147618       400005       2            5.71 
   4.50000E+00  1.00000E-05      1294091       450005       2            6.44 
   5.00000E+00  1.00000E-05      1442306       500005       2            7.16 

  End Simulation
 command: stop


Finished -----
Elapsed time = 7.48s,  CPU time = 7.16s,  95.66%
