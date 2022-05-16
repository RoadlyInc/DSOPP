#!/usr/bin/sh
find {src/,test/,pydsopp/} -name "*.?pp" | xargs clang-format-10 --style=file -i
find {src/,test/,pydsopp/} -name "CMakeLists.txt" | xargs cmake-format -i
cmake-format -i CMakeLists.txt
yapf -i -r test/pydsopp
yapf -i -r pydsopp
