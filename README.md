#Create project source code
###
    git clone https://github.com/aesirteam/proxyRepeater.git
    cd ./proxyRepeater
    chmod +x genMakefiles
###
    Generic linux x86_64
    ./genMakefiles linux-x64
###
    Generic linux i386
    ./genMakefiles linux
###
    Openwrt from mips toolchain
    ./genMakefiles openwrt
###
    Arm from armhf toolchain (Raspbian or DeitPi)
    ./genMakefiles rpi
###
    NanoPi-NEO from friendlyarm (Ubuntu-core)
    ./genMakefiles neo
###
    make -j4
#Run Program
###
    ./objs/rtmpPusher-x64 -c objs/srs.conf
    or
    ./objs/streamCasterPusher-x64 -i objs/demo.sdp
#Clean project
###
    make clean
    or
    make distclean
