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
Make stitched video with demo video:

    wget https://github.com/inastitch/inastitch/releases/download/v0.1/demo_video.tar.bz2
    tar xf demo_video.tar.bz2
    inastitch --in-matrix demo_video/inastitch_matrix.json --in-file0 demo_video/stream0.mjpeg --in-file1 demo_video/stream1.mjpeg --in-file2 demo_video/stream2.mjpeg --out-file stitched.mjpeg

Play ``stitched.mjpeg`` with ``ffmpeg``:

    ffplay stitched.mjpeg
