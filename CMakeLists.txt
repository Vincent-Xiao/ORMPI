cmake_minimum_required (VERSION 2.8)

project (ORMP)

aux_source_directory(./3rdparty/half-1.12.0/include DIR_SRCS)
aux_source_directory(./src/include DIR_SRCS)
aux_source_directory(./src/cpp DIR_SRCS)
aux_source_directory(./src/ DIR_SRCS)

add_executable(ORMP ${DIR_SRCS})

set(CMAKE_CXX_FLAGS   "-std=c++11") 
