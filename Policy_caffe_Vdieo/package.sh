#!/bin/bash
LAST_PATH=$PWD
SRC_PATH=`dirname $0`
echo $SRC_PATH
cd $SRC_PATH
if [ -n "$1"  ];then
	git pull
	git checkout -b $1 origin/$1
	git checkout $1
fi	
git pull

rm -rf build/
mkdir build

cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "CodeBlocks - Unix Makefiles" ..
make -j4 

cd ..

mkdir -p build/perception
#cp /opt/intel/mkl/lib/intel64/libmkl_intel_lp64.so build/perception/
#cp /opt/intel/mkl/lib/intel64/libmkl_intel_thread.so build/perception/
#cp /opt/intel/mkl/lib/intel64/libmkl_core.so build/perception/
#cp /opt/intel/mkl/lib/intel64/libmkl_avx2.so build/perception/
#cp /opt/intel/mkl/lib/intel64/libmkl_mc3.so build/perception/
#cp /opt/intel/mkl/lib/intel64/libmkl_def.so build/perception/
#
#cp /opt/intel/ipp/lib/intel64/libipp*9.so build/perception/


bash copy.sh build/PerceptionRun build/perception/


echo 'exit path'
cd $LAST_PATH
#

