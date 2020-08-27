# MPP_LINUX_C++
a demo shows that how to use mpp decode uvc mjpeg 
stream and show on hdmi on linux

project architecture

├── build   --build directory  
├── CMakeLists.txt      --cmake script  
├── main.cpp            --main program  
├── mpp                 --mpp abstract interface  
├── README.md           --doc  
├── rkdrm               --drm interface(abount display)  
└── thread              --thread abstract interface(use posix)  
└── v4l2              --v4l2 driver 
## make & test
first please modify CMakeLists.txt to specified c and c++ compiler.  
just do that  
    makedir build
    cd build
    cmake ..
    make

cmake version >= 2.8 is required  

## how you will see
on your device screen,you will see that hdmi 
is displayed.  

## run example
./mpp_uvc_mjpeg_decode_hdmi_show -w 640 -h 480 -t 8 -f 0

