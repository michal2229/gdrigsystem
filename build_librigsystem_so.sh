#!/usr/bin/env bash

set -eux

export CXX=clang++ 


pushd src/rigsystem_lib


meson setup builddir
cd builddir
meson compile


popd

