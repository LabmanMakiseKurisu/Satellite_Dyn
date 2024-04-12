# Install script for directory: /home/amadeus/Satellite/deps/eigen/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/AdolcForward"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/AlignedVector3"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/ArpackSupport"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/AutoDiff"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/BVH"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/EulerAngles"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/FFT"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/IterativeSolvers"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/KroneckerProduct"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/LevenbergMarquardt"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/MatrixFunctions"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/MPRealSupport"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/NNLS"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/NonLinearOptimization"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/NumericalDiff"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/OpenGLSupport"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/Polynomials"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/SparseExtra"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/SpecialFunctions"
    "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/Splines"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/amadeus/Satellite/deps/eigen/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/amadeus/Satellite/build/deps/eigen/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

