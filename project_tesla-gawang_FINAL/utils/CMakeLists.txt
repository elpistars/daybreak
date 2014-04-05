INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES})

IF (OpenCV_FOUND)
ADD_EXECUTABLE(raspicam_cv_test raspicam_cv_test.cpp )
target_link_libraries(raspicam_cv_test  ${PROJECT_NAME}_cv -ldxl)
INSTALL(TARGETS raspicam_cv_test   RUNTIME DESTINATION bin)

ENDIF()
