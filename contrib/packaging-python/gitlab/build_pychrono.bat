@echo on
echo Process begins
call "%CONDA_INSTALL_LOCN%"\Scripts\activate.bat
call conda install --yes anaconda-client
call conda uninstall --yes conda-build
call conda install --yes conda-build=3.18.11
call conda install --yes -c intel mkl-devel
Rem needed for libiomp5md.lib
Rem conda install -c intel openmp
call conda install -c conda-forge swig=4.0.2 --yes
call conda install -c dlr-sc opencascade --yes
call conda install mkl --yes
call conda install cmake --yes
call conda install jinja2 --yes
Rem call conda install ninja --yes
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat"
Rem CMAKE config output is redirected to a file otherwise it gets truncated due to depth
call conda build purge-all timeout /t 240
call conda build .\contrib\packaging-python\conda --python=3.8 --no-remove-work-dir
call anaconda --token "%ANACONDA_TOKEN%" upload "%CONDA_INSTALL_LOCN%"\conda-bld\win-64\pychrono*.bz2 --force --label main  >> "%LOG_DIR%"\condauploadlog.txt 2>&1
call conda build purge-all  timeout /t 240
call conda build .\contrib\packaging-python\conda --python=3.7 --no-remove-work-dir
call anaconda --token "%ANACONDA_TOKEN%" upload "%CONDA_INSTALL_LOCN%"\conda-bld\win-64\pychrono*.bz2 --force --label main  >> "%LOG_DIR%"\condauploadlog.txt 2>&1
call conda build purge  timeout /t 240
call conda build .\contrib\packaging-python\conda --python=3.6 --no-remove-work-dir
call anaconda --token "%ANACONDA_TOKEN%" upload "%CONDA_INSTALL_LOCN%"\conda-bld\win-64\pychrono*.bz2 --force --label main  >> "%LOG_DIR%"\condauploadlog.txt 2>&1
echo End Reached
