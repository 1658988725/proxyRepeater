#Create project source code
###
    git clone https://github.com/aesirteam/proxyRepeater.git
    cd ./proxyRepeater
    chmod +x genMakefiles
    ./genMakefiles linux-x64
    make -j4
#Run Program
###
    ./objs/rtmpPusher-x64 -c objs/srs.conf
    ./objs/streamCasterPusher-x64 -i objs/demo.sdp
#Clean project
###
    make clean
    or
    make distclean
#Create nodejs addon(v0.10.x or v0.12.x)
###
    ./genMakefiles linux-x64
    make node -j4
#Nodejs source sample
###
    var nvr = require('./build/Release/node_nvr_addon.node');
    
    var data = [{
        url: "rtsp://10.196.230.149/000100",
        endpoint: "rtmp://127.0.0.1:1935/srs",
        stream: "stream1",
        password: "publish2016"
    }, {
        url: "rtsp://10.196.230.138:8554/cam",
        endpoint: "rtmp://127.0.0.1:1935/srs",
        stream: "stream2",
        password: "publish2016"
    }]
    
    nvr.init(JSON.stringify(data), function(err, msg){
        if(err) {
            console.error(msg);
            return;
        } 
        nvr.start();
    })
