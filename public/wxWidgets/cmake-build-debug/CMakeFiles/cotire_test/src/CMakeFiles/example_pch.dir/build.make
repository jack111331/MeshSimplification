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

# Utility rule file for example_pch.

# Include the progress variables for this target.
include src/CMakeFiles/example_pch.dir/progress.make

src/CMakeFiles/example_pch: src/cotire/example_CXX_prefix.hxx.gch


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

example_pch: src/CMakeFiles/example_pch
example_pch: src/cotire/example_CXX_prefix.hxx.gch
example_pch: src/cotire/example_CXX_prefix.hxx
example_pch: src/cotire/example_CXX_prefix.cxx
example_pch: src/cotire/example_CXX_prefix.cxx.log
example_pch: src/cotire/example_CXX_unity.cxx
example_pch: src/CMakeFiles/example_pch.dir/build.make

.PHONY : example_pch

# Rule to build all files generated by this target.
src/CMakeFiles/example_pch.dir/build: example_pch

.PHONY : src/CMakeFiles/example_pch.dir/build

src/CMakeFiles/example_pch.dir/clean:
	cd /d D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src && $(CMAKE_COMMAND) -P CMakeFiles\example_pch.dir\cmake_clean.cmake
.PHONY : src/CMakeFiles/example_pch.dir/clean

src/CMakeFiles/example_pch.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test D:\Code\Ray\public\wxWidgets\build\cmake\modules\cotire_test\src D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src D:\Code\Ray\public\wxWidgets\cmake-build-debug\CMakeFiles\cotire_test\src\CMakeFiles\example_pch.dir\DependInfo.cmake
.PHONY : src/CMakeFiles/example_pch.dir/depend

