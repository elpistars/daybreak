cmake_minimum_required(VERSION 2.8)
project( OCVku )
find_package( OpenCV REQUIRED )
add_executable( OCVku OCVku.cpp )
target_link_libraries( OCVku ${OpenCV_LIBS} -ldxl -lm)

