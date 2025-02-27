#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")

# For reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

env.Append(CCFLAGS=[ "-std=c++20", "-O3", "-g3", "-flto", "-ffast-math", "-Wextra", "-Wall", "-Wno-unused-parameter"])
env.Append(LINKFLAGS=["-std=c++20", "-O3", "-g3", "-flto", "-ffast-math"])

if env["platform"] not in ["android"]:
    env.Append(CCFLAGS=[ "-mtune=native", "-mavx2" ])
    env.Append(LINKFLAGS=[ "-mtune=native", "-mavx2" ])

# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=[
    "src/rigsystem_lib/subprojects/yaml-cpp/include/",
    "src/rigsystem_lib/src/",
    "src/", 
])

sources = [
    "src/register_types.cpp",
    "src/gdrigsystem.cpp",
    "src/rigsystem_lib/src/rigsystem_common.cpp",
]
# Add OpenMP flags based on platform.
#if env["platform"] == "macos":
#    env.Append(CXXFLAGS=["-Xpreprocessor", "-fopenmp"])
#    env.Append(LINKFLAGS=["-lomp"])
#elif env["platform"] in ["linux", "windows"]:
#    env.Append(CXXFLAGS=["-fopenmp"])
#    env.Append(LINKFLAGS=["-fopenmp"])

# print(env.Dump())

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "bin/libgdrigsystem.{}.{}.framework/libgdrigsystem.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
elif env["platform"] == "ios":
    if env["ios_simulator"]:
        library = env.StaticLibrary(
            "bin/libgdrigsystem.{}.{}.simulator.a".format(env["platform"], env["target"]),
            source=sources,
        )
    else:
        library = env.StaticLibrary(
            "bin/libgdrigsystem.{}.{}.a".format(env["platform"], env["target"]),
            source=sources,
        )
else:
    library = env.SharedLibrary(
        "bin/libgdrigsystem{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
