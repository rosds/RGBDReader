cmake_minimum_required (VERSION 2.8)

project (RGBDReader)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


############################################################################### 
#   Find packages                                                             #
############################################################################### 

#   Boost C++ Library
find_package (Boost 1.46 COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#   Point Cloud Library
find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories (${PCL_LIBRARY_DIRS})

#   OpenCV Library
find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})
link_directories (${OpenCV_LIBRARY_DIRS})

#   OpenMP Library
find_package (OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories (include)


############################################################################### 
#   RGBDReader Library build                                                  #
############################################################################### 

set (SRC src/RGBDReader.cpp)
add_library (RGBDReader SHARED ${SRC})
target_link_libraries (RGBDReader ${OpenCV_LIBRARIES})

install (TARGETS RGBDReader DESTINATION lib)
install (DIRECTORY include/RGBDReader DESTINATION include)

############################################################################### 
#   Optional sample programs                                                  #
############################################################################### 

option (WITH_SAMPLES "Generate sample binaries" OFF)

if (WITH_SAMPLES)

add_executable (display src/samples/display.cpp)
target_link_libraries (display RGBDReader 
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable (tum2pcd src/samples/tum2pcd.cpp)
target_link_libraries (tum2pcd RGBDReader 
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

endif (WITH_SAMPLES)


##############################################################################
#   Doxygen documentation                                                    #
##############################################################################

# adds custom command 'make doc' to generate the documentation.
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in
        ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)
