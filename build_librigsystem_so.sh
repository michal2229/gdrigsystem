#!/usr/bin/env bash

set -eux
reset

export CXX=clang++ 


pushd src/rigsystem_lib

rm -rfv ./builddir

meson setup builddir
cd builddir
meson compile


popd

