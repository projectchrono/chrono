REM 
REM NVCC compile script - 
REM This is a workaround, since nvcc cannot be called from
REM Microsoft 'nmake', so this script is called instead and
REM all goes fine.
REM

echo NVCC executing...

nvcc %nvcc_par% 

echo ..NVCC done
