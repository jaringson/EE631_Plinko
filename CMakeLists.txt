cmake_minimum_required(VERSION 2.8)
project(Plinko)
find_package(OpenCV REQUIRED)
add_executable(Plinko Plinko.cpp)

target_link_libraries(Plinko ${OpenCV_LIBS})

add_executable(ball_detect ball_detection.cpp)
target_link_libraries(ball_detect ${OpenCV_LIBS})

add_executable(basic basic.cpp)
target_link_libraries(basic ${OpenCV_LIBS})

add_executable(video videoWrite.cpp)
target_link_libraries(video ${OpenCV_LIBS})
