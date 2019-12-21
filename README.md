# KFCmd

A library of kinematinc fit that developing for the CMD-3 experiment. 
The KFCmd is based on abstract classes from KFBase library (https://github.com/sergeigribanov/KFBase).

## Installing
1. git clone https://github.com/sergeigribanov/KFCmd.git
2. Create a directory to build the package in a suitable location and change the current directory to this one.
3. To build the package run the following commands:
    1. cmake  -DCMAKE_INSTALL_PREFIX=\<KFCmd installation prefix\> -DEIGEN3_INCLUDE_DIR=\<path to Eigen3 installation\> -DCCGO_DIR=\<path to CCGO installation\> -DKFBASE_DIR=\<path to KFBase installation\> \<path to KFCmd source code\>
    2. make
    3. make install
