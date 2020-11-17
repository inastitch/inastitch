cmake_minimum_required(VERSION 3.10)

project(tf_demo
    VERSION 0.1
    DESCRIPTION "tensorflow demo"
    LANGUAGES CXX
)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## OpenCV
set(OPENCV_STATIC_LIB_PATH "/home/pi/opencv/local/")
include_directories("${OPENCV_STATIC_LIB_PATH}/include/opencv4/")

file(GLOB OCV_STATIC_LIBS ${OPENCV_STATIC_LIB_PATH}/lib/*.a)
file(GLOB OCV_3RD_PARTY_STATIC_LIBS ${OPENCV_STATIC_LIB_PATH}/lib/opencv4/3rdparty/*.a)

## TensorFlow Lite
set(TFLITE_SRC_PATH "/home/pi/tensorflow/tensorflow/")
set(TFLITE_BUILD_PATH "/home/pi/tensorflow/build/")
# According to official documentation, it is the recommended way to include the git repo
# as "include path":
# | Currently, there is no straightforward way to extract all header files needed,
# | so you must include all header files in tensorflow/lite/ from the TensorFlow repository.
# -- https://www.tensorflow.org/lite/guide/build_arm64#c_library
include_directories(${TFLITE_SRC_PATH})
include_directories(${TFLITE_BUILD_PATH}/flatbuffers/include/)

## EdgeTPU
set(EDGETPU_RUNTIME_PATH "/home/pi/tensorflow/edgetpu_runtime/")
include_directories("${EDGETPU_RUNTIME_PATH}/libedgetpu/")

add_executable(tf_demo
    main.cpp
)

target_link_libraries(tf_demo
    # OpenCV static libraries
    # Note: multiples times because of dependencies
    ${OCV_STATIC_LIBS}
    ${OCV_STATIC_LIBS}
    ${OCV_3RD_PARTY_STATIC_LIBS}
    ${OCV_3RD_PARTY_STATIC_LIBS}
    ${OCV_STATIC_LIBS}

    # TensorFlow Lite static libraries
    ${TFLITE_BUILD_PATH}/libtensorflow-lite.a
    ${TFLITE_BUILD_PATH}/_deps/ruy-build/libruy.a
    ${TFLITE_BUILD_PATH}/_deps/flatbuffers-build/libflatbuffers.a
    ${TFLITE_BUILD_PATH}/_deps/farmhash-build/libfarmhash.a
    ${TFLITE_BUILD_PATH}/_deps/fft2d-build/libfft2d_fftsg.a

    # EdgeTPU
    # -L${EDGETPU_RUNTIME_PATH}/libedgetpu/direct/k8/
    -l:libedgetpu.so.1.0

    -lgtk-x11-2.0 -lgdk-x11-2.0 -lgdk_pixbuf-2.0 -lcairo -lgobject-2.0 -lglib-2.0
    -ljpeg -lz -pthread -ldl
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mobilenet_v1_300x300.tflite
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/coco_labels.txt
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
