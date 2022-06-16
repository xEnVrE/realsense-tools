set(EXE_TARGET_NAME device)

# YARP
find_package(YARP REQUIRED COMPONENTS cv os sig)

# OpenCV
find_package(OpenCV REQUIRED)

# RealSense
find_package(realsense2 REQUIRED)

# Executable
add_executable(${EXE_TARGET_NAME} src/main.cpp)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      ${OpenCV_LIBS}
                      realsense2::realsense2
                      YARP::YARP_cv
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
