Chrono GPU module tutorials {#tutorial_table_of_content_chrono_gpu}
===============================

The Chrono distribution contains several demos for granular dynamics with the [GPU module](group__gpu__module.html).

Chrono::Gpu provides support for GPU-accelerated granular dynamics via the penalty-based discrete element method (*aka SMC*).

Granular dynamics demos:

- demo_GPU_ballCosim
    
    A simple co-simulation example: A large sphere represented as a triangular mesh interacting with granular material. The dynamics of the sphere are performed by `ChSystem`, with contact forces generated from the `Chrono::GPU` system. 
        
- demo_GPU_fixedTerrain

    A terrain initialization utility that generates fixed particles on a surface mesh of the terrain and load additional particles from the above. 
    
- demo_GPU_movingBoundary

    A settled container of granular material is shoved to the side by a moving boundary plane.
    
-  demo_GPU_mixer

    A cylindrical container with a rotating blade that mixes granular material. 

-  demo_GPU_repose

    Granular material flows through a hopper funnel and forms a stable pile, demonstrating the angle of repose.
    