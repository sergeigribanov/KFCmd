`Documentation in progress...`

## Overview
`KFCmd` is a detector-dependent part of the kinematic and vertex fitting software developed for the CMD-3 experiment.
## Dependencies
1. `Eigen 3` template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms: https://eigen.tuxfamily.org/index.php?title=Main_Page (successfully tested with version `3.4`).
2. `ROOT` data analysis framework used by high energy physics and others: https://root.cern (successfully tested with version `6.26`).
4. `KFBase` fitting package: https://github.com/sergeigribanov/KFBase.

## Installation
1. Get the source code:
```console
git clone https://github.com/sergeigribanov/KFCmd
```
2. Setup `ROOT` environment:
```console
source <path to ROOT installation>/bin/thisroot.sh
```
3. Create a build directory:
```console
mkdir <path to a build directory>
cd <path to a build directory>
```
4. Run CMake:
```console
cmake -DCMAKE_INSTALL_PREFIX=<installation prefix> <path to the source code directory>
```
To set `C++` standard, `-DCMAKE_CXX_STANDARD` option can be used. The same standard should be used that was used to build the `ROOT` frameworkand `KFBase` package.

5. Build the package:
```console
make
```
6. Install the package:
```console
make install
```
