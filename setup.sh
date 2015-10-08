#!/bin/bash
git submodule init
git submodule sync
git submodule update
cd 3rdparty/scl.git/applications-linux/scl_lib
sh make_everything.sh
cd ../../../../
