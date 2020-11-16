## Build ``libtensorflow-lite.a``

    git clone https://github.com/tensorflow/tensorflow.git
    mkdir build && cd build/
    cmake -DCMAKE_BUILD_TYPE=Release ../tensorflow/tensorflow/lite/
    make -j4
