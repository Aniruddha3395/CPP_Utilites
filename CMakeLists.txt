cmake_minimum_required (VERSION 3.5)

# Executable name: 
##############################################################
SET(UT OUTFile)
##############################################################

project(${UT})

#including certain dirs
# requires eigen
include_directories(/usr/local/include/eigen3)

#requires libnabo
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/libnabo)
include_directories(${PROJECT_SOURCE_DIR}/data)

#requires gnuplot-iostream for compiling plot_utilities
include_directories(${PROJECT_SOURCE_DIR}/gnuplot-iostream-master)


#link
link_directories(/usr/local/lib)

# support for Libnabo  
SET(CMAKE_CXX_FLAGS "-std=c++11 -w -O3")

##############################################################

add_library(gen_utilities SHARED src/utilities.cpp
	src/NPAM_utilities.cpp 
	src/file_rw.cpp 
	src/transformation_utilities.cpp
	src/plot_utilities.cpp
	src/robot_comm.cpp)

# target libraries for using the libnabo
target_link_libraries(gen_utilities ${nlopt_ADDED_LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/include/libnabo/build/libnabo.a gomp)
