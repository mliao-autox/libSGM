cmake_minimum_required(VERSION 2.8)

project(sgm_demo)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

message(STATUS ${CMAKE_MODULE_PATH})

FIND_PACKAGE(CUDA  REQUIRED)

#find_package(OpenCV REQUIRED core) # dammy 
#if(OpenCV_VERSION VERSION_LESS 3.0) 
#	find_package(OpenCV REQUIRED core highgui imgproc)  # 2.x
#else()
#	find_package(OpenCV REQUIRED core highgui imgproc imgcodecs videoio) # 3.x
#endif()
find_package(OpenCV 3.1 REQUIRED PATHS /usr/local/ NO_DEFAULT_PATH)

find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED)

find_package(GLFW REQUIRED)

include_directories(../../include)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLFW_INCLUDE_DIRS})

CUDA_ADD_EXECUTABLE(stereo_bf
stereosgm_bf.cpp
demo.cpp
renderer.cpp
cuda_gl.cu
bf.cpp
)

TARGET_LINK_LIBRARIES(stereo_bf sgm  ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${OpenCV_LIBS} ${GLEW_LIBRARIES} ${GLFW_LIBRARY} ${OPENGL_LIBRARY})



CUDA_ADD_EXECUTABLE(stereo_bf_rec
stereosgm_bf_rec.cpp
demo.cpp
renderer.cpp
cuda_gl.cu
bf.cpp
)

TARGET_LINK_LIBRARIES(stereo_bf_rec sgm  ${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${OpenCV_LIBS} ${GLEW_LIBRARIES} ${GLFW_LIBRARY} ${OPENGL_LIBRARY})
