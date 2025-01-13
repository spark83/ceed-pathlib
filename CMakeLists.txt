cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(ceedpath VERSION 1.0)

if(WIN32)
add_subdirectory(googletest)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd") 
else()

# Fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
FetchContent_MakeAvailable(googletest)
endif()

enable_testing()

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PATH_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(PATH_HEADER_FILES
    "include/cdAStar.hpp"
    "include/cdAStarMap.hpp"
    "include/cdGridMap.hpp"
    "include/cdHelperMethods.hpp"
    "include/cdHeuristics.hpp"
    "include/cdJumpStartMap.hpp"
    "include/FastDelegate.h"
    "include/FastDelegateBind.h")

set(PATH_SOURCE_FILES
    "src/cdGridMap.cpp"
    "src/cdJumpStartMap.cpp")

add_library(ceedpath ${PATH_SOURCE_FILES} ${PATH_HEADER_FILES})

target_include_directories(ceedpath PUBLIC ${PATH_INCLUDE_DIR})

install(TARGETS ceedpath DESTINATION lib)
install(FILES ${STREAMSIM_HEADER_FILES} DESTINATION include/ceedpath)

# Add the executable
add_subdirectory(tests)
