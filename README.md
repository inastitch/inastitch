# inastitch
## Build
Install build dependencies (Raspberry Pi):

    sudo apt install cmake git
    sudo apt install libboost-program-options-dev libturbojpeg0-dev libglfw3-dev libgles2-mesa-dev libglm-dev
    
Build ``inastitch``:

    git clone https://github.com/inastitch/inastitch.git
    mkdir build
    cd build/
    cmake ../inastitch/
    make

## Run
Make stitched video:

    inastitch --in-matrix data/inastitch_matrix.json --in-file0 data/stream0.mjpeg --in-file1 data/stream1.mjpeg --in-file2 data/stream2.mjpeg --out-file stitched.mjpeg

Play ``stitched.mjpeg`` with ``ffmpeg``:

    ffplay stitched.mjpeg
