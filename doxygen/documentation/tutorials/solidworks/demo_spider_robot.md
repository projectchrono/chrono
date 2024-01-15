Make a spider robot in SolidWorks and simulate it {#tutorial_chrono_solidworks_demo_spider_robot}
==========================

This demo is about the simulation of a crawling spider robot with six legs,
where we control the motion of the legs with 18 actuators.

![](http://projectchrono.org/assets/manual/Tutorial_spider_robot.jpg)

In deail, one performs the following steps: 
- use SolidWorks to make a 3D CAD model of the crawling robot, 
- export it as a .pyfile using the Chrono::SolidWorks add-in; ex. use the name **spider\_robot.py**
- create a Python program, ex. use the name **demo_SW\_spider_robot.py**, using the functions of [PyChrono](@ref pychrono_introduction) to load and simulate spider\_robot.py.

The CAD model and the **demo_SW\_spider_robot.py** program come with the Chrono SolidWorks add-in installation, but are also directly available [here](https://github.com/projectchrono/chrono-solidworks/tree/master/to_put_in_app_dir/examples/spider_robot) 


Note that a part that shows as *M-410iB-300 -1/ArmBase\<1\>* in the GUI of SolidWorks, becomes *M-410iB-300 -1/ArmBase-1* for the Python side; i.e. the \<N\> suffix becomes -N.