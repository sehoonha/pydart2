cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

################################################################################
# Setup the build mode
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "=====================================================")
message(STATUS "configuring build for Pydart2")
message(STATUS "=====================================================")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

IF(CMAKE_BUILD_TYPE MATCHES "Release")
  MESSAGE(STATUS "Set the release mode")
  ADD_DEFINITIONS(-DNDEBUG )
  ADD_DEFINITIONS(-DBOOST_UBLAS_NDEBUG )
ENDIF (CMAKE_BUILD_TYPE MATCHES "Release")

IF(APPLE)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flat_namespace -undefined suppress")
ELSEIF(UNIX)
  SET(CMAKE_CXX_FLAGS "-Wall -msse2 -fPIC")
  EXECUTE_PROCESS(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  IF(GCC_VERSION VERSION_LESS 4.8)
    MESSAGE(FATAL_ERROR "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ 4.8 or greater.")
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  ENDIF()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xlinker -rpath /usr/local/lib")
  SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  SET(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
ENDIF()
MESSAGE(STATUS "CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

################################################################################
# Find the required packages
### SWIG
find_package(SWIG REQUIRED)
message(STATUS "SWIG_FOUND = ${SWIG_FOUND}")
message(STATUS "SWIG_USE_FILE = ${SWIG_USE_FILE}")
include(${SWIG_USE_FILE})
### PythonLibs
find_package(PythonLibs REQUIRED)
message(STATUS "PYTHON_INCLUDE_DIR = ${PYTHON_INCLUDE_DIR}")
include_directories(${PYTHON_INCLUDE_DIR})
message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")

### PythonLibs
find_package(NumPy REQUIRED)
message(STATUS "NUMPY_INCLUDE_DIRS = ${NUMPY_INCLUDE_DIRS}")
include_directories(${NUMPY_INCLUDE_DIRS})

### Eigen
find_package(Eigen REQUIRED)
message(STATUS "Eigen_INCLUDE_DIRS = ${Eigen_INCLUDE_DIRS}")
include_directories(${Eigen_INCLUDE_DIRS})
### OpenGL
find_package(OpenGL)
if (OPENGL_FOUND)
  message(STATUS "OPENGL FOUND")
else (OPENGL_FOUND)
  message(STATUS "OPENGL NOT FOUND")
endif ()
### GLUT
find_package(GLUT)
if (GLUT_FOUND)
  message(STATUS "GLUT FOUND")
  message(STATUS "GLUT_LIBRARIES = ${GLUT_LIBRARIES}")
else (GLUT_FOUND)
  message(STATUS "GLUT NOT FOUND")
endif ()

find_package(Bullet)
if (BULLET_FOUND)
  message(STATUS "Bullet Found")
  message(STATUS "Bullet include directories = ${BULLET_INCLUDE_DIRS}")
  message(STATUS "Bullet Libraries = ${BULLET_LIBRARIES}")
  include_directories(${BULLET_INCLUDE_DIRS})
else (BULLET_FOUND)
  message(STATUS "Bullet NOT Found")
endif ()

find_library(LIB_DART dart)
message(STATUS "LIB_DART = ${LIB_DART}")
find_library(LIB_DART-GUI dart-gui)
message(STATUS "LIB_DART-GUI = ${LIB_DART-GUI}")
find_library(LIB_DART-OPTIMIZER-IPOPT dart-optimizer-ipopt)
message(STATUS "LIB_DART-OPTIMIZER-IPOPT = ${LIB_DART-OPTIMIZER-IPOPT}")
find_library(LIB_DART-OPTIMIZER-NLOPT dart-optimizer-nlopt)
message(STATUS "LIB_DART-OPTIMIZER-NLOPT = ${LIB_DART-OPTIMIZER-NLOPT}")
find_library(LIB_DART-PLANNING dart-planning)
message(STATUS "LIB_DART-PLANNING = ${LIB_DART-PLANNING}")
find_library(LIB_DART-UTILS dart-utils)
message(STATUS "LIB_DART-UTILS = ${LIB_DART-UTILS}")
find_library(LIB_DART-UTILS-URDF dart-utils-urdf)
message(STATUS "LIB_DART-UTILS-URDF = ${LIB_DART-UTILS-URDF}")

################################################################################
# Configure the project
#### Directories
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/pydart2_api")

include_directories(".")
include_directories("${CMAKE_SOURCE_DIR}")
include_directories("${CMAKE_SOURCE_DIR}/pydart2_api")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#### Projectfiles
file(GLOB pydart2_py "pydart2/*.py")
file(GLOB pydart2_srcs "pydart2/*.cpp")
file(GLOB pydart2_hdrs "pydart2/*.h")
file(GLOB pydart2_interfaces "pydart2/pydart2_api.i")
message(STATUS "pydart2_py = ${pydart2_py}")
message(STATUS "pydart2_hdrs = ${pydart2_hdrs}")
message(STATUS "pydart2_srcs = ${pydart2_srcs}")
#### SWIG configurations
set(CMAKE_SWIG_FLAGS "")
set_source_files_properties(${pydart2_interfaces} PROPERTIES CPLUSPLUS ON)
set_source_files_properties(${pydart2_interfaces} PROPERTIES SWIG_FLAGS "-includeall")
#### SWIG commands
set(CMAKE_SWIG_OUTDIR "${CMAKE_SOURCE_DIR}/pydart2")
swig_add_module(pydart2_api python ${pydart2_py} ${pydart2_srcs} ${pydart2_hdrs} ${pydart2_interfaces})
swig_link_libraries(pydart2_api ${PYTHON_LIBRARIES})
swig_link_libraries(pydart2_api ${LIB_DART} ${LIB_DART-GUI} ${LIB_DART-OPTIMIZER-IPOPT} ${LIB_DART-OPTIMIZER-NLOPT}
  ${LIB_DART-PLANNING} ${LIB_DART-UTILS} ${LIB_DART-UTILS-URDF})
swig_link_libraries(pydart2_api ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
swig_link_libraries(pydart2_api ${BULLET_LIBRARIES})

install(FILES "${CMAKE_BINARY_DIR}/_pydart2_api.so"
  DESTINATION "${CMAKE_SOURCE_DIR}/pydart2")
