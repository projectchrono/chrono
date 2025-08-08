Install the GPU module   {#module_gpu_installation}
===============================

[TOC]

Chrono::Gpu is an optional module that enables a GPU solver specialized for simulating large systems of granular materials with a penalty-based Discrete Element Method.

## Features

The **GPU module** allows users to construct a dynamic system consisting of spherical elements

Here are the main features:
* a variety of friction models
    * frictionless (optimized)
    * single-step pseudo history
    * multi-step history tracking
* a variety of explicit time integrators
    * forward Euler
    * extended Taylor
    * centered difference
    * [Chung](https://onlinelibrary.wiley.com/doi/abs/10.1002/nme.1620372303)
* single-GPU scaling up to 700 million frictionless elements or 200 million full-history frictional elements
* triangular meshes (`obj` format) in order to facilitate co-simulation with a more full-featured solver (such as the ChSystem)

## Requirements

- To **build** applications based on this module you must have CUDA installed
- To **run** applications based on this module you need
    - an NVIDIA GPU
    - Linux or Windows
- This module has been build/tested on both Windows 11 and Linux (Ubuntu 22.04, Fedora 40, and Arch Linux) with CUDA 12.3 and 12.8.

## Building instructions
   
1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must add the following steps:
   
2. Set the `CH_ENABLE_MODULE_GPU` as 'on', then press 'Configure' (to refresh the variable list) 
	 
3. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Look at the [API section](group__gpu__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_gpu) to learn how to use the functions of this module.

## MacOS support

This module cannot be built for MacOS, Nvidia GPU hardware and CUDA are unsupported.
