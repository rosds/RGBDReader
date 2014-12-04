cmake_minimum_required (VERSION 2.8)

project (RGBDReader)

find_package (Boost 1.46 COMPONENTS program_options REQUIRED)
find_package (PCL REQUIRED)

set (SRC src/RGBDReader_base.cpp)

include_directories (include ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable (display src/main.cpp ${SRC})
target_link_libraries (display ${Boost_LIBRARIES} ${PCL_LIBRARIES})
