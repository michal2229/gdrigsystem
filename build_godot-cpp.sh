#!/usr/bin/env bash

set -eux

pushd godot-cpp

scons platform=linux target=template_debug dev_build=yes use_llvm=yes  $@
scons platform=linux target=template_debug use_llvm=yes  $@
scons platform=linux target=template_release use_llvm=yes  $@

if [ -n "${ANDROID_HOME}" ]; then
	scons platform=android target=template_debug  $@
	scons platform=android target=template_release  $@
fi

popd

