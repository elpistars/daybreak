cmake_minimum_required(VERSION 2.8)
project( magnet )
add_executable( magnet code.cpp )
target_link_libraries( magnet -ldxl -lm -lpthread)
