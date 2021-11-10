mkdir dependency
cd public
REM It may need you with system priviledge
REM Note that I temporary use codeblocks MinGW generator

REM Buil GLFW and install it
cd glfw
mkdir build
cd build
REM Generate building GLFW necessary files and build it as dynamic shared library
cmake -DBUILD_SHARED_LIBS=ON -A Win32 ../
cmake --build . --config Release
cmake --install . --prefix ../../../dependency
cd ..
rm -rf build

REM Switch to previous directory
cd ..

REM Buil wxWidgets and install it
cd wxWidgets
mkdir build_bin
cd build_bin
REM Generate building GLFW necessary files and build it as dynamic shared library
cmake -DwxWidgets_ROOT_DIR:PATH=..\..\..\dependency -A Win32 ../
cmake --build . --config Release --target install
cmake --install . --prefix ../../../dependency
cd ..
rm -rf build_bin

REM Switch to previous directory
cd ..

REM Build wxWidgets and install it
cd OpenMesh-8.1
mkdir build
cd build
REM Generate building GLFW necessary files and build it as dynamic shared library
cmake -DCMAKE_BUILD_TYPE=Debug -A Win32 ../
cmake --build . --target install
cmake --install . --prefix ../../../dependency
cd ..
rm -rf build

cd ../../../