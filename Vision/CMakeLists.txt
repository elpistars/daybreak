INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES})

ADD_EXECUTABLE(run main.cpp )
target_link_libraries(run  ${PROJECT_NAME}_cv -ldxl -lm)
#target_link_libraries(run  ${PROJECT_NAME}_cv /opt/vc/lib/libmmal_core.so /opt/vc/lib/libmmal_util.so /opt/vc/lib/libmmal_vc_client.so /opt/vc/lib/libvcos.so /opt/vc/lib/libbcm_host.so /home/pi/pierre/libfacerec-0.04/libopencv_facerec.a ${OpenCV_LIBS})
INSTALL(TARGETS run   RUNTIME DESTINATION bin)
