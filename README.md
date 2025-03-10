## gdrigsystem
[WIP] Beam system inspired by Rigs of Rods and BeamNG.drive physics, packed as a Godot 4 addon.

Sample Godot project using this addon [here](https://github.com/michal2229/godot_gdrigsystem_project).

### description

Beam physics for each rig is implemented independently from Godot physics system. It then interfaces with it using a set of ShapeCast3D nodes, which detect collisions with the Godot colliders and with other rigs.
Currently there is a ShapeCast3D at each rig node, which is not optimal in terms of performance and behavior (large shapecasts needed to prevent rigs merging, which makes them float).

Internal beam physics is integrated using Radau IIA method. 

The addon consists of the inner lib (rigsystem_lib/) and outer godot interface (gdrigsystem.cpp). It is written in C++ and uses GDExtension to interface with Godot.

The gdrigsystem.cpp part is registered as a Godot class inherited from godot::Node, and is to be used as a child of a rig scene.
It uses rigsystem_lib part for computation and serves as a glue between it and Godot.

The rigsystem_lib part is where the physics is implemented. Currently only Radau IIA is implemented as an integrator. 
It is designed to be used also outside of Godot extension, as it does not use any Godot stuff - there are a sample apps (app_test, app_perf_test) that use rigsystem_lib as a dynamic dependency. 


### build
Build steps for usage with Godot:
- clone the repo with --recursive flag
- apply the patch in godot-cpp dir: godot-cpp_enable_cpp2b.patch (you should have C++23 capable compiler; scripts configured for llvm/clang)
- run ./build_all.sh if on Linux, for other platforms please refer to [Godot GDExtension docs](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/gdextension_cpp_example.html) (if you have ANDROID_HOME env var defined, it will also try to build for Android; check [docs about compiling for Android](https://docs.godotengine.org/en/stable/contributing/development/compiling/compiling_for_android.html) to set up the SDK)
- you should have it built now, it is ready to be placed in your Godot project

Build steps for standalone rigsystem_lib usage:
- run ./build_librigsystem_so.sh  (you need meson and ninja)
- sample apps are included (app_test - renders pinned tower structure in ASCII and shows basic perf info, see gif below; app_perf_test - playground regarding vectorisation and cache perf, prints some iter timings using various approaches to data layout and computation methods), will be built alongside the library

<div align="center">
<img src="https://raw.githubusercontent.com/michal2229/gdrigsystem/refs/heads/main/res/test_app_tower_00.gif" width="574" />
</div>


### todo
- [x] refine the GDRigSystem api - more flexibility in terms of rig definition (functions callable from GDScript for managing nodes and connections instead of enumerating other node's children, function to import definition from YAML file, etc.)
- [ ] refine the rigsystem_lib api - cleaner use of resources, modular integrator for runtime switching, etc.
- [ ] implement more integrators (implicit Euler, Runge–Kutta of various orders, etc.)
- [x] set up a buildsystem and make scripts for building rigsystem_lib standalone 
- [x] make a C++ app for more robust testing and profiling the rigsystem_lib without Godot
- [ ] better interfacing with Godot physics system - more performance, better behavior, ability to deform a mesh, etc.
- [ ] maybe some visual rig designer, maybe as a godot game/app, with option for exporting to a file
- [ ] other beam types and nodes, as in Rigs of Rods/BeamNG.drive, for modelling useful stuff
- [ ] ability to pin nodes to rigidbodies
- [ ] make it better
- [ ] rewrite in Rust (jk)

