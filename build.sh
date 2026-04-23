#!/bin/bash
# g2o
if [ $# -ge 1 ]; then
	tn="$1"
else
	tn="-j8"
fi 
echo "编译使用参数：$tn"
CMD_DIR=$(cd $(dirname $0); pwd;)
# minilzo
cd thirdparty/minilzo
if [ ! -d "build" ];then
    mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# sqlite_wrap
cd ../../sqlite_wrap
if [ ! -d "build" ];then
    mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

cd ../../..

# main project
if [ ! -d "build" ];then
    mkdir build
fi
cd build
cmake .. -DENABLE_TESTS=OFF -DCMAKE_BUILD_TYPE=Release
make $tn 
cd ..

