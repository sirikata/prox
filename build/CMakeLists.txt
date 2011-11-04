#CMakeLists.txt
#Copyright (c) 2008, Ewen Cheslack-Postava
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#    * Neither the name of the libprox nor the names of its contributors
#      may be used to endorse or promote products derived from this software
#      without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
#ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#cmake options

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Default CACHE STRING
      "Build types include: Default Debug Release None RelWithDebInfo." FORCE
      )
ENDIF()
#-fno-rtti
SET(CMAKE_CXX_FLAGS "-Wall -Wno-unused-function -Wno-unused-variable -fvisibility=hidden")
#-fno-rtti
SET(CMAKE_CXX_FLAGS_DEFAULT "-O0 -g2 -Wall -Wno-unused-function -Wno-unused-variable -fvisibility=hidden" CACHE STRING
    "Built cxxflags with default settings."
    FORCE )
SET( CMAKE_C_FLAGS_DEFAULT "-g2 -Wall -Wno-unused-function -Wno-unused-variable -fvisibility=hidden" CACHE STRING
    "Built cflags with default settings."
    FORCE )
SET( CMAKE_EXE_LINKER_FLAGS_DEFAULT
    "" CACHE STRING
    "Linking binaries with default settings."
    FORCE )
SET( CMAKE_SHARED_LINKER_FLAGS_DEFAULT
    "" CACHE STRING
    "Building shared libraries with default settings."
    FORCE )
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_DEFAULT
    CMAKE_C_FLAGS_DEFAULT
    CMAKE_EXE_LINKER_FLAGS_DEFAULT
    CMAKE_SHARED_LINKER_FLAGS_DEFAULT )


#-fno-rtti
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -Wall -Wno-unused-function -Wno-unused-variable -fvisibility=hidden" CACHE STRING
    "Built cxxflags with default settings."
    FORCE )
SET( CMAKE_C_FLAGS_RELEASE "-g -Wall -Wno-unused-function -Wno-unused-variable -fvisibility=hidden" CACHE STRING
    "Built cflags with default settings."
    FORCE )
SET( CMAKE_EXE_LINKER_FLAGS_RELEASE
    "" CACHE STRING
    "Linking binaries with default settings."
    FORCE )
SET( CMAKE_SHARED_LINKER_FLAGS_RELEASE
    "" CACHE STRING
    "Building shared libraries with default settings."
    FORCE )
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_RELEASE
    CMAKE_C_FLAGS_RELEASE
    CMAKE_EXE_LINKER_FLAGS_RELEASE
    CMAKE_SHARED_LINKER_FLAGS_RELEASE )


#-fno-rtti
SET(CMAKE_CXX_FLAGS_OPROFILE "-O2 -g -Wall -Wno-unused-function -Wno-unused-variable -fvisibility=hidden" CACHE STRING
    "Built cxxflags with default settings."
    FORCE )
SET( CMAKE_C_FLAGS_OPROFILE "-O2 -g -Wall -Wno-unused-function -Wno-unused-variable -fvisibility=hidden" CACHE STRING
    "Built cflags with default settings."
    FORCE )
SET( CMAKE_EXE_LINKER_FLAGS_OPROFILE
    "" CACHE STRING
    "Linking binaries with default settings."
    FORCE )
SET( CMAKE_SHARED_LINKER_FLAGS_OPROFILE
    "" CACHE STRING
    "Building shared libraries with default settings."
    FORCE )
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_OPROFILE
    CMAKE_C_FLAGS_OPROFILE
    CMAKE_EXE_LINKER_FLAGS_OPROFILE
    CMAKE_SHARED_LINKER_FLAGS_OPROFILE )

#project global flags

PROJECT(libprox C CXX)

IF(NOT TOP_LEVEL)
  SET(TOP_LEVEL ${CMAKE_CURRENT_SOURCE_DIR}/..)
ENDIF()
GET_FILENAME_COMPONENT(TOP_LEVEL ${TOP_LEVEL} ABSOLUTE)

#add-ins

IF(NOT CMAKE_MODULE_PATH)
  SET(CMAKE_MODULE_PATH ${TOP_LEVEL}/build/modules)
ENDIF()
GET_FILENAME_COMPONENT(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ABSOLUTE)

MACRO(VERIFY_VERSION package_name major minor subminor)
  IF (${package_name}_MAJOR_VERSION LESS ${major})
    MESSAGE(FATAL_ERROR "${package_name} out of date: ${major}.${minor}.${subminor}")
  ELSEIF(${package_name}_MAJOR_VERSION EQUAL ${major})
    IF(${package_name}_MINOR_VERSION LESS ${minor})
       MESSAGE(FATAL_ERROR "${package_name} out of date: ${major}.${minor}.${subminor}")
    ELSEIF(${package_name}_MINOR_VERSION EQUAL ${major})
      IF(${package_name}_SUBMINOR_VERSION LESS ${subminor})
        MESSAGE(FATAL_ERROR "${package_name} out of date: ${major}.${minor}.${subminor}")
      ENDIF()
    ENDIF()
  ENDIF()
ENDMACRO(VERIFY_VERSION)


#libraries

#dependency: boost > 1.35
IF(NOT BOOST_ROOT)
  # FIXME This is gross, but should allow prox to find cbr's boost
  # without a problem
  SET(BOOST_ROOT ${TOP_LEVEL}/../sirikata/dependencies)
  GET_FILENAME_COMPONENT(BOOST_ROOT ${BOOST_ROOT} ABSOLUTE)
ENDIF()
SET(Boost_FIND_REQUIRED TRUE)
IF(WIN32)
  FIND_PACKAGE(Boost COMPONENTS thread date_time)
ELSE()
  FIND_PACKAGE(Boost COMPONENTS system)
  STRING(REPLACE "boost_system" "boost_thread" Boost_THREAD_LIBRARY "${Boost_SYSTEM_LIBRARY}")
  STRING(REPLACE "boost_system" "boost_date_time" Boost_DATE_TIME_LIBRARY "${Boost_SYSTEM_LIBRARY}")
ENDIF()
VERIFY_VERSION(Boost 1 35 0)

SET(BUILD_PROXSIM TRUE)

#dependency: glut
FIND_PACKAGE(OpenGL)
FIND_PACKAGE(GLUT)
IF(NOT OPENGL_FOUND OR NOT GLUT_FOUND)
  SET(BUILD_PROXSIM FALSE)
ENDIF(NOT OPENGL_FOUND OR NOT GLUT_FOUND)

#project locations
SET(LIBPROX_ROOT ${TOP_LEVEL}/libprox)
SET(PROXSIM_ROOT ${TOP_LEVEL}/proxsim)
SET(GEOM_PROXSIM_ROOT ${TOP_LEVEL}/proxsim/geom)
SET(MANUAL_PROXSIM_ROOT ${TOP_LEVEL}/proxsim/manual)

#include/source file location
SET(LIBPROX_INCLUDE_DIR ${LIBPROX_ROOT}/include)
SET(LIBPROX_SOURCE_DIR ${LIBPROX_ROOT}/src)

SET(LIBPROXSIMCORE_SOURCE_DIR ${PROXSIM_ROOT}/proxsimcore)
SET(GEOM_PROXSIM_SOURCE_DIR ${GEOM_PROXSIM_ROOT})
SET(MANUAL_PROXSIM_SOURCE_DIR ${MANUAL_PROXSIM_ROOT})

#cxx flags
INCLUDE_DIRECTORIES(BEFORE ${LIBPROX_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} )
IF(BUILD_PROXSIM)
  INCLUDE_DIRECTORIES(BEFORE ${PROXSIM_ROOT} ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
ENDIF(BUILD_PROXSIM)

#source files
SET(LIBPROX_SOURCES
  ${LIBPROX_SOURCE_DIR}/BoundingSphere.cpp
  ${LIBPROX_SOURCE_DIR}/DefaultSimulationTraits.cpp
  ${LIBPROX_SOURCE_DIR}/Duration.cpp
  ${LIBPROX_SOURCE_DIR}/SolidAngle.cpp
  ${LIBPROX_SOURCE_DIR}/Time.cpp
)


SET(LIBPROXSIMCORE_SOURCES
  ${LIBPROXSIMCORE_SOURCE_DIR}/Object.cpp
  ${LIBPROXSIMCORE_SOURCE_DIR}/ObjectLocationServiceCache.cpp
  ${LIBPROXSIMCORE_SOURCE_DIR}/Timer.cpp
  ${LIBPROXSIMCORE_SOURCE_DIR}/GLRendererBase.cpp
  ${LIBPROXSIMCORE_SOURCE_DIR}/SimulatorBase.cpp
  ${LIBPROXSIMCORE_SOURCE_DIR}/CSVLoader.cpp
)

SET(GEOM_PROXSIM_SOURCES
  ${GEOM_PROXSIM_SOURCE_DIR}/GLRenderer.cpp
  ${GEOM_PROXSIM_SOURCE_DIR}/Simulator.cpp
  ${GEOM_PROXSIM_SOURCE_DIR}/Querier.cpp
  ${GEOM_PROXSIM_SOURCE_DIR}/main.cpp
  ${GEOM_PROXSIM_SOURCE_DIR}/CSVLoader.cpp
)

SET(MANUAL_PROXSIM_SOURCES
  ${MANUAL_PROXSIM_SOURCE_DIR}/GLRenderer.cpp
  ${MANUAL_PROXSIM_SOURCE_DIR}/Simulator.cpp
  ${MANUAL_PROXSIM_SOURCE_DIR}/Querier.cpp
  ${MANUAL_PROXSIM_SOURCE_DIR}/CSVLoader.cpp
  ${MANUAL_PROXSIM_SOURCE_DIR}/main.cpp
)

#link flags
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

#libraries
ADD_LIBRARY(prox STATIC ${LIBPROX_SOURCES})
TARGET_LINK_LIBRARIES(prox ${LIBPROX_LIBRARIES} ${Boost_THREAD_LIBRARY})

ADD_LIBRARY(proxsimcore STATIC ${LIBPROXSIMCORE_SOURCES})

#binaries
IF(BUILD_PROXSIM)
  # Libs for any proxsim binaries
  SET(PROXSIM_BINARY_LIBS proxsimcore prox ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

  ADD_EXECUTABLE(proxsim ${GEOM_PROXSIM_SOURCES})
  TARGET_LINK_LIBRARIES(proxsim ${PROXSIM_BINARY_LIBS})

  ADD_EXECUTABLE(manual_proxsim ${MANUAL_PROXSIM_SOURCES})
  TARGET_LINK_LIBRARIES(manual_proxsim ${PROXSIM_BINARY_LIBS})
ENDIF(BUILD_PROXSIM)

GET_TARGET_PROPERTY(PROX_DEFS prox COMPILE_DEFINITIONS)
GET_TARGET_PROPERTY(PROXSIM_DEFS proxsim COMPILE_DEFINITIONS)
GET_TARGET_PROPERTY(MANUAL_PROXSIM_DEFS manual_proxsim COMPILE_DEFINITIONS)
SET(ALL_ADDITIONAL_DEFS)

# define debug preprocessor symbol
IF(${CMAKE_BUILD_TYPE} MATCHES "[Dd][Ee][Bb][Uu][Gg]")
  SET(PROX_DEFS ${PROX_DEFS} PROXDEBUG)
  SET(PROXSIM_DEFS ${PROXSIM_DEFS} PROXDEBUG)
  SET(MANUAL_PROXSIM_DEFS ${MANUAL_PROXSIM_DEFS} PROXDEBUG)
  SET(ALL_ADDITIONAL_DEFS ${ALL_ADDITIONAL_DEFS} -DPROXDEBUG)
ENDIF()

# RTree internal data type - just bounds vs. bounds + max size
SET(RTREE_DATA "MAXSIZE"
  CACHE STRING
  "The type of data to store in each node for constraint testing. Options: BOUNDS or MAXSIZE.")
SET(RTREE_DATA_DEFS
  RTREE_DATA_BOUNDS=1
  RTREE_DATA_MAXSIZE=2
  RTREE_DATA=RTREE_DATA_${RTREE_DATA}
  )
SET_TARGET_PROPERTIES(prox PROPERTIES COMPILE_DEFINITIONS "${RTREE_DATA_DEFS}")
SET_TARGET_PROPERTIES(proxsim PROPERTIES COMPILE_DEFINITIONS "${RTREE_DATA_DEFS}")
SET_TARGET_PROPERTIES(manual_proxsim PROPERTIES COMPILE_DEFINITIONS "${RTREE_DATA_DEFS}")
FOREACH(XDEF ${RTREE_DATA_DEFS})
  SET(ALL_ADDITIONAL_DEFS ${ALL_ADDITIONAL_DEFS} -D${XDEF})
ENDFOREACH()
ADD_DEFINITIONS(${ALL_ADDITIONAL_DEFS})

#installation information
INSTALL(TARGETS prox
        RUNTIME
          DESTINATION bin
        LIBRARY
          DESTINATION lib
        ARCHIVE
          DESTINATION lib
)

INSTALL(DIRECTORY
          # note: the trailing slashes avoid prepending the directory name
          # itself to the installed files
          ${LIBPROX_INCLUDE_DIR}/
        DESTINATION include/
        PATTERN "*.cpp" EXCLUDE
        PATTERN "*.cc" EXCLUDE
        PATTERN "*.hh" EXCLUDE
)
