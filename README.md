# Inatech's open-source video stitcher: ``inastitch``
## Build
Install build dependencies (Raspberry Pi):

    sudo apt install cmake git
    sudo apt install libturbojpeg0-dev libglfw3-dev libgles2-mesa-dev libglm-dev

Build Boost and OpenCV as static libraries.
    
Build ``inastitch``:

    git clone https://github.com/inastitch/inastitch.git
    mkdir build
    cd build/
    cmake -DBOOST_INSTALL=/home/pi/boost/local/ -DOPENCV_STATIC_LIB_PATH=/home/pi/opencv/local/ ../inastitch
    make

## Run
Start V4L webcam streams:

    gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=10/1 ! jpegenc ! fdsink | ./inartpsend --local-ts --width 640 --height 480 --out-port 5000
    gst-launch-1.0 v4l2src device=/dev/video2 ! video/x-raw,width=640,height=480,framerate=10/1 ! jpegenc ! fdsink | ./inartpsend --local-ts --width 640 --height 480 --out-port 5001
    gst-launch-1.0 v4l2src device=/dev/video4 ! video/x-raw,width=640,height=480,framerate=10/1 ! jpegenc ! fdsink | ./inartpsend --local-ts --width 640 --height 480 --out-port 5002

**Note:** adapt V4L setup to the capabilities of your cameras.

Start stitching with default network ports (``5000`` center, ``5001`` left, ``5002`` right):

    ./inastitch

Press ``Enter`` to calibrate stitching.
