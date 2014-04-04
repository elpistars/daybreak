INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES})

 
ADD_EXECUTABLE(raspicam_test raspicam_test.cpp )
INSTALL(TARGETS raspicam_test   RUNTIME DESTINATION bin)
 
IF (OpenCV_FOUND)
ADD_EXECUTABLE(raspicam_cv_test raspicam_cv_test.cpp )
target_link_libraries(raspicam_cv_test  ${PROJECT_NAME}_cv)
#target_link_libraries(raspicam_cv_test  ${PROJECT_NAME}_cv /opt/vc/lib/libmmal_core.so /opt/vc/lib/libmmal_util.so /opt/vc/lib/libmmal_vc_client.so /opt/vc/lib/libvcos.so /opt/vc/lib/libbcm_host.so /home/pi/pierre/libfacerec-0.04/libopencv_facerec.a ${OpenCV_LIBS})
INSTALL(TARGETS raspicam_cv_test   RUNTIME DESTINATION bin)

ENDIF()
