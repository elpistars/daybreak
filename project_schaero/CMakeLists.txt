cmake_minimum_required(VERSION 2.8)
project( run )
add_executable( run wifi.cpp )
target_link_libraries( run -ldxl -lm )
