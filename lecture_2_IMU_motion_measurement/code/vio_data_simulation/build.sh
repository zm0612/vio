#!/bin/bash
echo "Configuring and building ..."
if [ ! -d "build" ]; then
    mkdir build
fi
cd build || exit # xx || xx 如果左边的命令不成功，则执行右边的命令
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
