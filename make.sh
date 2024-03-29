#!/bin/bash

script="$(readlink -f $0)"
script_dir="$(dirname $script)"

. "$script_dir/config.sh.in"


#rm -rf "$build_dir" "$install_dir"

mkdir -p $eeros_build_dir
pushd $eeros_build_dir
cmake -DCMAKE_TOOLCHAIN_FILE=$toolchain_file -DCMAKE_INSTALL_PREFIX=$install_dir -DCMAKE_BUILD_TYPE=Release $eeros_source_dir
make
make install
popd

if [ ! -z ${flink_source_dir+x} ]; then
  mkdir -p $flink_build_dir
  pushd $flink_build_dir
  cmake -DCMAKE_TOOLCHAIN_FILE=$toolchain_file -DCMAKE_INSTALL_PREFIX=$install_dir -DCMAKE_BUILD_TYPE=Release $flink_source_dir
  make
  make install
  popd
fi

if [ ! -z ${flink_eeros_source_dir+x} ]; then
  mkdir -p $flink_eeros_build_dir
  pushd $flink_eeros_build_dir
  cmake -DCMAKE_TOOLCHAIN_FILE=$toolchain_file -DCMAKE_INSTALL_PREFIX=$install_dir -DCMAKE_BUILD_TYPE=Release $flink_eeros_source_dir
  make
  make install
  popd
fi

mkdir -p $laserscanner_build_dir
pushd $laserscanner_build_dir
cmake -DCMAKE_TOOLCHAIN_FILE=$toolchain_file -DCMAKE_INSTALL_PREFIX=$install_dir -#DCMAKE_BUILD_TYPE=Release $laserscanner_source_dir
make
popd

mkdir -p $omnimobot_build_dir
pushd $omnimobot_build_dir
cmake	-DCMAKE_TOOLCHAIN_FILE=$toolchain_file -DCMAKE_INSTALL_PREFIX=$install_dir -DCMAKE_BUILD_TYPE=Release $omnimobot_source_dir
make
popd
