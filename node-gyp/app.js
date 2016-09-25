var nvr = require('./build/Release/node_nvr_addon.node');

var data = [{
        rtspUrl: "rtsp://10.196.230.149/000100",
        rtspUser: "admin",
        rtspPass: "12345",
        endpoint: "rtmp://127.0.0.1:1935/srs",
        stream: "stream1",
        password: "publish2016"
   }, {
	    rtspUrl: "rtsp://10.196.230.150/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif",
        rtspUser: "888888",
        rtspPass: "888888",
	    rtspUseTcp: 0,
        endpoint: "rtmp://127.0.0.1:1935/srs",
        stream: "stream2",
        password: "publish2016"
}];

nvr.init(JSON.stringify(data), function(err, msg){
	if(err) {
		console.error(msg);
		return;
	}
	nvr.start();
});