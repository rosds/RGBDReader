cmake_minimum_required (VERSION 2.8)

project (RGBDReader)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

find_package (Boost 1.46 COMPONENTS program_options REQUIRED)
find_package (PCL 1.7 REQUIRED)
find_package (OpenCV REQUIRED)
find_package (OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set (SRC src/RGBDReader_base.cpp)

include_directories (include 
    ${Boost_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

add_library (RGBDReader SHARED ${SRC})

add_executable (display EXCLUDE_FROM_ALL  src/main.cpp)
target_link_libraries (display RGBDReader 
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES})
