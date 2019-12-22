# KFCmd

A library of kinematinc fit that developing for the CMD-3 experiment. 
The KFCmd is based on abstract classes from KFBase library (https://github.com/sergeigribanov/KFBase).

## Dependencies

1. Eigen3: http://eigen.tuxfamily.org/index.php?title=Main_Page
2. ROOT: https://root.cern.ch
3. CCGO: https://github.com/sergeigribanov/ccgo
4. KFBase: https://github.com/sergeigribanov/KFBase

## Installing
1. git clone https://github.com/sergeigribanov/KFCmd.git
2. Create a directory to build the package in a suitable location and change the current directory to this one.
3. Setup ROOT environment.
4. To build the package run the following commands:
    1. cmake  -DCMAKE_INSTALL_PREFIX=\<KFCmd installation prefix\> -DEIGEN3_INCLUDE_DIR=\<path to Eigen3 installation\> -DCCGO_DIR=\<path to CCGO installation\> -DKFBASE_DIR=\<path to KFBase installation\> \<path to KFCmd source code\>
    2. make
    3. make install
