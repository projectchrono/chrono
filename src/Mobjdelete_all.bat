REM 
REM Execute this script to delete .o files 
REM before a complete rebuild
REM

echo DELETING ALL .O  OBJECT FILES....

nmake /f "make-chrono_lib" clean

echo DELETED ALL .O and .OBJ OBJECT FILES!
