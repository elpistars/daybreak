cmake_minimum_required(VERSION 2.8)
project( AX12 )
add_executable( AX12 AX12.cpp )
target_link_libraries( AX12 -ldxl -lm )
