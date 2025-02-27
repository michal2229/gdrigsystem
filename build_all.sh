#!/usr/bin/env bash

set -eux

./build_godot-cpp.sh  $@
./build_extension.sh  $@
./build_librigsystem_so.sh

