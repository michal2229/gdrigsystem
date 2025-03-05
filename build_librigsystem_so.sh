#!/usr/bin/env bash

set -eux
reset

export CXX=clang++ 
#export CXX=g++ 


pushd src/rigsystem_lib

rm -rfv ./builddir

meson setup builddir
cd builddir
meson compile


popd

