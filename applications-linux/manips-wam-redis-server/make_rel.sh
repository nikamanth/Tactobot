#!/bin/bash
mkdir -p build_rel 
cd build_rel 
cmake .. -DCMAKE_BUILD_TYPE=Release 
make 
cp -rf manips_wam* ../
