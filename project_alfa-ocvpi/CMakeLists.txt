cmake_minimum_required(VERSION 2.8)
project( ocvpi )
find_package( OpenCV REQUIRED )
add_executable( ocvpi ocvpi.cpp )
target_link_libraries( ocvpi ${OpenCV_LIBS} )

include_directories(${robidouille/raspicam_cv})
