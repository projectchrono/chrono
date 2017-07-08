Apply loads   (demo_FEA_loads.cpp)  {#tutorial_demo_FEA_loads}
================================== 


Tutorial that teaches how to use the 
[FEA module](group__fea__module.html)
to add loads to finite element models.

Loads are added via ChLoad objects.

There are various ready-to-use ChLoad objects, but here you can also learn how to define new ChLoad classes for custom loads.

Optionally the ChLoader classes can be used to automate some tasks, most noticeably the Gauss integration of distributed loads.


\include demo_FEA_loads.cpp