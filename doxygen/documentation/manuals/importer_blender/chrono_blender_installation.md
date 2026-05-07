Install Chrono Blender {#chrono_blender_installation}
==========================


1.  Install [Blender](http://www.blender.org). Currently, only two versions of Blender are supported: 4.4 and 5.0
	
2.  Start Blender. 

3.  Open the **Edit>Preferences>Add-ons** menu panel <br>
 
    ![](http://projectchrono.org/assets/manual/blender_install.jpg)

4.  Press the **Install from Disk...**  button (in the drop-down menu at top-right) and select the `chrono_import.py` from the subdirectory in the Chrono source tree corresponding to the Blender version:
    - `chrono/src/importer_blender/for_blender_4.4/chrono_import.py`, or
    - `chrono/src/importer_blender/for_blender_5.0/chrono_import.py`.
	
5.  Make sure the Chrono importer add-on is enabled (set the checkmark in the list of add-ons)
	
6.  Close the Blender **Preferences** window.

Once the Chrono importer add-on is installed and enabled, a new option is available as **File>Import>Chrono import**.

For details on using the Chrono Blender add-on, see the [Chrono::Blender](@ref introduction_chrono_blender) page.
