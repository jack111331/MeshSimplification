# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "D:\Program Files\JetBrains\CLion 2019.2.4\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "D:\Program Files\JetBrains\CLion 2019.2.4\bin\cmake\win\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test

# Include any dependencies generated for this target.
include src/CMakeFiles/example.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/example.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/example.dir/flags.make

src/cotire/example_CXX_prefix.hxx.gch: src/cotire/example_CXX_prefix.hxx
src/cotire/example_CXX_prefix.hxx.gch: D:/MinGW/bin/c++.exe
src/cotire/example_CXX_prefix.hxx.gch: src/cotire/example_CXX_prefix.hxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX precompiled header src/cotire/example_CXX_prefix.hxx.gch"
	cd /d D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src && "D:\Program Files\JetBrains\CLion 2019.2.4\bin\cmake\win\bin\cmake.exe" -DCOTIRE_BUILD_TYPE:STRING= -DCOTIRE_VERBOSE:BOOL=$(VERBOSE) -P D:/Code/Ray/public/wxWidgets/build/cmake/modules/cotire.cmake precompile D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/example_CXX_cotire.cmake D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/cotire/example_CXX_prefix.hxx D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/cotire/example_CXX_prefix.hxx.gch main.cpp

src/cotire/example_CXX_prefix.hxx: src/cotire/example_CXX_prefix.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating CXX prefix header src/cotire/example_CXX_prefix.hxx"
	"D:\Program Files\JetBrains\CLion 2019.2.4\bin\cmake\win\bin\cmake.exe" -DCOTIRE_BUILD_TYPE:STRING= -DCOTIRE_VERBOSE:BOOL=$(VERBOSE) -P D:/Code/Ray/public/wxWidgets/build/cmake/modules/cotire.cmake combine D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/example_CXX_cotire.cmake D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/cotire/example_CXX_prefix.hxx D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/cotire/example_CXX_prefix.cxx

src/cotire/example_CXX_prefix.cxx: src/cotire/example_CXX_unity.cxx
src/cotire/example_CXX_prefix.cxx: D:/MinGW/bin/c++.exe
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating CXX prefix source src/cotire/example_CXX_prefix.cxx"
	cd /d D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src && "D:\Program Files\JetBrains\CLion 2019.2.4\bin\cmake\win\bin\cmake.exe" -DCOTIRE_BUILD_TYPE:STRING= -DCOTIRE_VERBOSE:BOOL=$(VERBOSE) -P D:/Code/Ray/public/wxWidgets/build/cmake/modules/cotire.cmake prefix D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/example_CXX_cotire.cmake D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/cotire/example_CXX_prefix.cxx D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/cotire/example_CXX_unity.cxx

src/cotire/example_CXX_prefix.cxx.log: src/cotire/example_CXX_prefix.cxx
	@$(CMAKE_COMMAND) -E touch_nocreate src\cotire\example_CXX_prefix.cxx.log

src/cotire/example_CXX_unity.cxx: src/example_CXX_cotire.cmake
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating CXX unity source src/cotire/example_CXX_unity.cxx"
	cd /d D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src && "D:\Program Files\JetBrains\CLion 2019.2.4\bin\cmake\win\bin\cmake.exe" -DCOTIRE_BUILD_TYPE:STRING= -DCOTIRE_VERBOSE:BOOL=$(VERBOSE) -P D:/Code/Ray/public/wxWidgets/build/cmake/modules/cotire.cmake unity D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/example_CXX_cotire.cmake D:/Code/Ray/public/wxWidgets/cmake-build-debug/CMakeFiles/cotire_test/src/cotire/example_CXX_unity.cxx

src/CMakeFiles/example.dir/main.cpp.obj: src/CMakeFiles/example.dir/flags.make
src/CMakeFiles/example.dir/main.cpp.obj: D:/Code/Ray/public/wxWidgets/build/cmake/modules/cotire_test/src/main.cpp
src/CMakeFiles/example.dir/main.cpp.obj: src/cotire/example_CXX_prefix.hxx.gch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/example.dir/main.cpp.obj"
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\example.dir\main.cpp.obj -c D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\main.cpp

src/CMakeFiles/example.dir/main.cpp.i: cmake_force
	@echo Preprocessing CXX source to CMakeFiles/example.dir/main.cpp.i
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\main.cpp > CMakeFiles\example.dir\main.cpp.i

src/CMakeFiles/example.dir/main.cpp.s: cmake_force
	@echo Compiling CXX source to assembly CMakeFiles/example.dir/main.cpp.s
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\main.cpp -o CMakeFiles\example.dir\main.cpp.s

src/CMakeFiles/example.dir/example.cpp.obj: src/CMakeFiles/example.dir/flags.make
src/CMakeFiles/example.dir/example.cpp.obj: D:/Code/Ray/public/wxWidgets/build/cmake/modules/cotire_test/src/example.cpp
src/CMakeFiles/example.dir/example.cpp.obj: src/cotire/example_CXX_prefix.hxx.gch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/example.dir/example.cpp.obj"
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\example.dir\example.cpp.obj -c D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\example.cpp

src/CMakeFiles/example.dir/example.cpp.i: cmake_force
	@echo Preprocessing CXX source to CMakeFiles/example.dir/example.cpp.i
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\example.cpp > CMakeFiles\example.dir\example.cpp.i

src/CMakeFiles/example.dir/example.cpp.s: cmake_force
	@echo Compiling CXX source to assembly CMakeFiles/example.dir/example.cpp.s
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\example.cpp -o CMakeFiles\example.dir\example.cpp.s

src/CMakeFiles/example.dir/log.cpp.obj: src/CMakeFiles/example.dir/flags.make
src/CMakeFiles/example.dir/log.cpp.obj: D:/Code/Ray/public/wxWidgets/build/cmake/modules/cotire_test/src/log.cpp
src/CMakeFiles/example.dir/log.cpp.obj: src/cotire/example_CXX_prefix.hxx.gch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/example.dir/log.cpp.obj"
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\example.dir\log.cpp.obj -c D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\log.cpp

src/CMakeFiles/example.dir/log.cpp.i: cmake_force
	@echo Preprocessing CXX source to CMakeFiles/example.dir/log.cpp.i
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\log.cpp > CMakeFiles\example.dir\log.cpp.i

src/CMakeFiles/example.dir/log.cpp.s: cmake_force
	@echo Compiling CXX source to assembly CMakeFiles/example.dir/log.cpp.s
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && D:\MinGW\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src\log.cpp -o CMakeFiles\example.dir\log.cpp.s

# Object files for target example
example_OBJECTS = \
"CMakeFiles/example.dir/main.cpp.obj" \
"CMakeFiles/example.dir/example.cpp.obj" \
"CMakeFiles/example.dir/log.cpp.obj"

# External object files for target example
example_EXTERNAL_OBJECTS =

src/example.exe: src/CMakeFiles/example.dir/main.cpp.obj
src/example.exe: src/CMakeFiles/example.dir/example.cpp.obj
src/example.exe: src/CMakeFiles/example.dir/log.cpp.obj
src/example.exe: src/CMakeFiles/example.dir/build.make
src/example.exe: src/CMakeFiles/example.dir/linklibs.rsp
src/example.exe: src/CMakeFiles/example.dir/objects1.rsp
src/example.exe: src/CMakeFiles/example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable example.exe"
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\example.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/example.dir/build: src/example.exe

.PHONY : src/CMakeFiles/example.dir/build

src/CMakeFiles/example.dir/clean:
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && $(CMAKE_COMMAND) -P CMakeFiles\example.dir\cmake_clean.cmake
.PHONY : src/CMakeFiles/example.dir/clean

src/CMakeFiles/example.dir/depend: src/cotire/example_CXX_prefix.hxx.gch
src/CMakeFiles/example.dir/depend: src/cotire/example_CXX_prefix.hxx
src/CMakeFiles/example.dir/depend: src/cotire/example_CXX_prefix.cxx
src/CMakeFiles/example.dir/depend: src/cotire/example_CXX_prefix.cxx.log
src/CMakeFiles/example.dir/depend: src/cotire/example_CXX_unity.cxx
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src\CMakeFiles\example.dir\DependInfo.cmake
.PHONY : src/CMakeFiles/example.dir/depend
