#!/bin/bash
mkdir -p build_dbg
cd build_dbg
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j
cp -rf manips_wam* ../
